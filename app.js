'use strict';

const $ = (s) => document.querySelector(s);
const video = $('#camera');
const grid = $('#grid');
const gctx = grid.getContext('2d');
const vision = $('#vision');
const vctx = vision.getContext('2d', { willReadFrequently: true, alpha: false });

const ui = {
  startCard: $('#startCard'), start: $('#startBtn'), instruction: $('#instruction'), progress: $('#progress i'),
  status: $('#statusPill'), gridMode: $('#gridModePill'), stress: $('#stressBtn'), save: $('#saveBtn'), load: $('#loadBtn'), reset: $('#resetBtn'), diag: $('#diagBtn'),
  stressDialog: $('#stressDialog'), closeStress: $('#closeStress'), stressSummary: $('#stressSummary'), stressResults: $('#stressResults'), exportStress: $('#exportStressBtn'),
  dialog: $('#diagDialog'), closeDiag: $('#closeDiag'), export: $('#exportBtn'),
  x: $('#xVal'), y: $('#yVal'), z: $('#zVal'),
  dState: $('#dState'), dFov: $('#dFov'), dVisual: $('#dVisual'), dMotion: $('#dMotion'), dStill: $('#dStill'),
  dFps: $('#dFps'), dScale: $('#dScale'), dQuality: $('#dQuality'), dTracks: $('#dTracks'), dDrift: $('#dDrift'),
  dImuHz: $('#dImuHz'), dVideoHz: $('#dVideoHz'), dReason: $('#dReason'), dProjectionError: $('#dProjectionError'),
  dOriginQuality: $('#dOriginQuality'), dGridMode: $('#dGridMode'), dVisualStep: $('#dVisualStep'), dMoveGate: $('#dMoveGate'), dTiming:$('#dTiming'), dWorldBasis:$('#dWorldBasis'), dDirection:$('#dDirection'), stepLabel: $('#stepLabel'), stepDetail: $('#stepDetail'), stepTimer: $('#stepTimer')
};

const STORAGE_KEY = 'cruxtain.xyzAutoProfile.v3.1';
const LEGACY_STORAGE_KEYS = ['cruxtain.xyzBasis.v2.5','cruxtain.xyzBasis.v2.4','cruxtain.xyzBasis.v2.3'];
const PROFILE_SAVE_INTERVAL_MS = 4000;
const DEG = Math.PI / 180;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const lerp = (a, b, t) => a + (b - a) * t;
const median = (a) => {
  if (!a.length) return 0;
  const b = [...a].sort((x, y) => x - y);
  const m = b.length >> 1;
  return b.length & 1 ? b[m] : (b[m - 1] + b[m]) * 0.5;
};
const mad = (a, m = median(a)) => median(a.map(v => Math.abs(v - m))) || 1e-6;
const vecLength = (v) => Math.hypot(v.x, v.y, v.z);

function q(x = 0, y = 0, z = 0, w = 1) { return { x, y, z, w }; }
function qNorm(a) {
  const n = Math.hypot(a.x, a.y, a.z, a.w) || 1;
  return q(a.x / n, a.y / n, a.z / n, a.w / n);
}
function qMul(a, b) {
  return q(
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
  );
}
function qInv(a) { return q(-a.x, -a.y, -a.z, a.w); }
function qAxis(ax, ay, az, angle) {
  const h = angle * 0.5, s = Math.sin(h);
  return q(ax*s, ay*s, az*s, Math.cos(h));
}
function qRotate(a, v) {
  const ix = a.w*v.x + a.y*v.z - a.z*v.y;
  const iy = a.w*v.y + a.z*v.x - a.x*v.z;
  const iz = a.w*v.z + a.x*v.y - a.y*v.x;
  const iw = -a.x*v.x - a.y*v.y - a.z*v.z;
  return {
    x: ix*a.w + iw*-a.x + iy*-a.z - iz*-a.y,
    y: iy*a.w + iw*-a.y + iz*-a.x - ix*-a.z,
    z: iz*a.w + iw*-a.z + ix*-a.y - iy*-a.x
  };
}
function qFromEulerYXZ(beta, alpha, negGamma) {
  const c1=Math.cos(alpha/2), c2=Math.cos(beta/2), c3=Math.cos(negGamma/2);
  const s1=Math.sin(alpha/2), s2=Math.sin(beta/2), s3=Math.sin(negGamma/2);
  return q(
    s2*c1*c3 + c2*s1*s3,
    c2*s1*c3 - s2*c1*s3,
    c2*c1*s3 - s2*s1*c3,
    c2*c1*c3 + s2*s1*s3
  );
}
function deviceMotionQuaternion(alpha,beta,gamma){
  // Maps the DeviceMotion standard-orientation XYZ frame into our Y-up world.
  // Screen rotation is intentionally excluded: DeviceMotion axes do not rotate with UI orientation.
  let out=qFromEulerYXZ(beta*DEG,alpha*DEG,-gamma*DEG);out=qMul(out,qAxis(1,0,0,-Math.PI/2));return qNorm(out);
}
function deviceCameraQuaternion(alpha,beta,gamma){
  let out=deviceMotionQuaternion(alpha,beta,gamma);const screenAngle=((screen.orientation&&screen.orientation.angle)||window.orientation||0)*DEG;out=qMul(out,qAxis(0,0,1,-screenAngle));return qNorm(out);
}
function qDeltaVector(prev, curr) {
  let d = qNorm(qMul(qInv(prev), curr));
  if (d.w < 0) d = q(-d.x,-d.y,-d.z,-d.w);
  const s = Math.hypot(d.x,d.y,d.z);
  if (s < 1e-8) return {x:0,y:0,z:0};
  const angle = 2*Math.atan2(s, clamp(d.w,-1,1));
  return {x:d.x/s*angle, y:d.y/s*angle, z:d.z/s*angle};
}
function qAngle(a,b) { return vecLength(qDeltaVector(a,b)); }
function qAverage(samples) {
  if (!samples.length) return q();
  const ref = samples[0];
  let x=0,y=0,z=0,w=0;
  for (let s of samples) {
    if (s.x*ref.x+s.y*ref.y+s.z*ref.z+s.w*ref.w < 0) s=q(-s.x,-s.y,-s.z,-s.w);
    x+=s.x; y+=s.y; z+=s.z; w+=s.w;
  }
  return qNorm(q(x,y,z,w));
}
function qSlerp(a,b,t) {
  a=qNorm(a); b=qNorm(b);
  let dot=a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
  if(dot<0){b=q(-b.x,-b.y,-b.z,-b.w);dot=-dot;}
  if(dot>0.9995)return qNorm(q(lerp(a.x,b.x,t),lerp(a.y,b.y,t),lerp(a.z,b.z,t),lerp(a.w,b.w,t)));
  const th=Math.acos(clamp(dot,-1,1)),s=Math.sin(th)||1;
  const wa=Math.sin((1-t)*th)/s,wb=Math.sin(t*th)/s;
  return qNorm(q(a.x*wa+b.x*wb,a.y*wa+b.y*wb,a.z*wa+b.z*wb,a.w*wa+b.w*wb));
}
function cameraHeadingAngle(camQ) {
  const f=qRotate(camQ,{x:0,y:0,z:-1}),h=Math.hypot(f.x,f.z);
  if(h>0.08)return -Math.atan2(f.x,-f.z);
  const r=qRotate(camQ,{x:1,y:0,z:0});
  return Math.atan2(-r.z,r.x);
}
function rawRelativeQ(absQ) {
  if(!state.baseQ)return q();
  return qNorm(qMul(qInv(state.baseQ),absQ));
}
function relativeQ(absQ) {
  const raw=rawRelativeQ(absQ);
  return state.orientationCorrection ? qNorm(qMul(state.orientationCorrection,raw)) : raw;
}
function relativeCameraQ() { return relativeQ(state.orientationQ); }
function relativeMotionQ(){if(!state.baseQ)return state.motionQ;const raw=qNorm(qMul(qInv(state.baseQ),state.motionQ));return state.orientationCorrection?qNorm(qMul(state.orientationCorrection,raw)):raw;}

const state = {
  stage: 'idle', stream: null, trackSettings: {},
  orientationQ: q(), motionQ:q(), baseQ: null, orientationCorrection:q(), orientationHoldQ:null, previousFrameQ:null, previousFrameTime:0, orientationSamples:[], lastOrientationAt:0, orientationRate:0,
  gyro:{x:0,y:0,z:0}, accelDevice:{x:0,y:0,z:0}, accelGravityDevice:{x:0,y:0,z:0}, accelWorld:{x:0,y:0,z:0}, frameAccelWorld:{x:0,y:0,z:0}, accelSamples:[], accelBiasDevice:{x:0,y:0,z:0}, gravityWorld:{x:0,y:9.80665,z:0}, hasLinearAcceleration:false, hasGravityAcceleration:false,
  position:{x:0,y:0,z:0}, velocity:{x:0,y:0,z:0},
  fovX:62, fovY:48, fovSamples:[], focalConfidence:0, projectionError:Infinity, videoImuLagMs:85, timingSamples:[], timingConfidence:0,
  scale:1, scaleStability:0, scaleSamples:[], scaleLocked:false,
  visualConfidence:0, motionConfidence:0, stationary:false, stationaryScore:0, stillSince:0, stillScore:0, originQuality:0,
  originCaptured:false, biasSamples:[], gravitySamples:[],
  lastMotionAt:0, lastFrameAt:0, lastSetupAt:performance.now(), lastProfileSaveAt:0,
  imuCount:0, imuHz:0, imuStamp:performance.now(), videoCount:0, videoHz:0, videoStamp:performance.now(),
  processedFps:0, processCount:0, processStamp:performance.now(),
  frame:null, previousFrame:null, tracks:[], validTracks:0, flowMagnitude:0,
  translationSignal:{x:0,y:0,z:0,confidence:0,rawMagnitude:0}, visualStepMagnitude:0, movementGate:'initializing', lastMoveAt:0, driftRate:0, lastPositionForDrift:{x:0,y:0,z:0},
  poseReason:'Waiting for permissions', loopStarted:false, lastProcessAt:0, basisSaved:false, stageEnteredAt:performance.now(),
  gridMode:'off', worldRevision:0, translationDirectionConfidence:0,
  metric:{scaleIntervals:[],scaleHistory:[],lastScaleSolve:null,lastImuAt:0,lastVisualAt:0,automaticUpdates:0},
  map:{keyframes:[],landmarks:[],nextKeyframeId:1,nextLandmarkId:1,lastKeyframeId:null,sinceKeyframeVisual:0,poseInliers:0,reprojectionError:Infinity,confidence:0,relocalizations:0,loopClosures:0,lastRelocalizeAt:0,lastMapBuildAt:0,lastPoseAt:0,lastCorrection:0,gaugeReady:false,confirmed:0,lastAuthoritativeAt:0,lastAuthoritativePos:null,lastSolvedPos:null,relocalizationCandidates:[],bridgeAge:Infinity,suppressRenderUntil:0},
  stress:{active:false,complete:false,index:-1,startedAt:0,stageStartedAt:0,testStartPos:null,testStartQ:null,lastPos:null,lastYaw:0,yawTravel:0,pathLength:0,maxDisplacement:0,maxAxis:0,maxOffAxis:0,maxDriftRate:0,stableSince:0,movementSeen:false,results:[],overall:'not run',manual:false}
};

function setStage(stage, text, progress=100, pill='calibrating') {
  state.stage=stage;state.stageEnteredAt=performance.now();
  ui.instruction.textContent=text;ui.progress.style.width=`${clamp(progress,0,100)}%`;
  ui.status.textContent=stage==='tracking'?'AUTO TRACKING':stage.replaceAll('_',' ').toUpperCase();ui.status.dataset.state=pill;
  ui.stepLabel.textContent=stage==='tracking'?'ZERO-SETUP WORLD LOCK':stage.toUpperCase();ui.stepDetail.textContent=text;ui.stepTimer.textContent=stage==='tracking'?'LIVE':'—';
  state.gridMode=stage==='tracking'?'self-initializing':'off';
}

async function requestPermissions() {
  ui.start.disabled = true;
  try {
    if (!navigator.mediaDevices?.getUserMedia) throw new Error('This browser does not expose camera capture.');
    // Start every sensor permission request inside the original tap. On browsers that
    // require transient user activation (notably iOS), awaiting one prompt before
    // requesting the next can lose the activation and make the second request fail.
    const permissionRequests=[];
    if (typeof window.DeviceMotionEvent?.requestPermission === 'function')
      permissionRequests.push(window.DeviceMotionEvent.requestPermission().then(r=>['motion',r]));
    if (typeof window.DeviceOrientationEvent?.requestPermission === 'function')
      permissionRequests.push(window.DeviceOrientationEvent.requestPermission().then(r=>['orientation',r]));
    const permissionResults=await Promise.all(permissionRequests);
    for(const [kind,result] of permissionResults)if(result!=='granted')throw new Error(`${kind==='motion'?'Motion':'Orientation'} permission was not granted.`);

    state.stream = await navigator.mediaDevices.getUserMedia({
      video: { facingMode:{exact:'environment'}, width:{ideal:1280}, height:{ideal:720}, frameRate:{ideal:30,min:24,max:30} }, audio:false
    }).catch(() => navigator.mediaDevices.getUserMedia({
      video: { facingMode:{ideal:'environment'}, width:{ideal:1280}, height:{ideal:720}, frameRate:{ideal:30,max:30} }, audio:false
    }));

    const track = state.stream.getVideoTracks()[0];
    state.trackSettings = track.getSettings?.() || {};
    video.srcObject = state.stream;
    await video.play();

    addEventListener('deviceorientation', onOrientation, true);
    addEventListener('devicemotion', onMotion, true);
    document.addEventListener('visibilitychange', onVisibility);

    ui.startCard.hidden = true;
    ui.reset.disabled = false;
    if(ui.load)ui.load.disabled=true;
    resize();
    beginTracking();
    startVideoLoop();
    if (!state.loopStarted) { state.loopStarted = true; requestAnimationFrame(renderLoop); }
  } catch (err) {
    ui.start.disabled = false;
    ui.start.textContent = 'Try Again';
    ui.instruction.textContent = err?.message || String(err);
  }
}

function onVisibility() {
  if(document.hidden){state.velocity={x:0,y:0,z:0};state.poseReason='Paused while page is hidden';return;}
  state.previousFrame=null;state.previousFrameTime=0;state.metric.lastImuAt=0;
  state.poseReason='Live streams resumed; map relocalization will correct the pose automatically';
}

function onOrientation(e) {
  if (e.alpha == null || e.beta == null || e.gamma == null) return;
  const now=performance.now();
  const motionNext=deviceMotionQuaternion(e.alpha,e.beta,e.gamma),next=deviceCameraQuaternion(e.alpha,e.beta,e.gamma);
  if(state.lastOrientationAt){
    const dt=Math.max((now-state.lastOrientationAt)/1000,1e-3);
    state.orientationRate=qAngle(state.orientationQ,next)/dt;
  }
  state.motionQ=motionNext;state.orientationQ=next; state.lastOrientationAt=now;
  state.orientationSamples.push({t:now,q:next});
  while(state.orientationSamples.length>240||(state.orientationSamples[0]&&now-state.orientationSamples[0].t>5000))state.orientationSamples.shift();
  if(state.stage==='tracking'&&!state.originCaptured)initializeWorldFromCurrentOrientation();
}

function onMotion(e) {
  const now=performance.now(),rr=e.rotationRate||{};
  state.gyro={x:(rr.alpha||0)*DEG,y:(rr.beta||0)*DEG,z:(rr.gamma||0)*DEG};
  const a=e.acceleration||{},ag=e.accelerationIncludingGravity||{};
  const hasLinear=[a.x,a.y,a.z].every(Number.isFinite),hasGravity=[ag.x,ag.y,ag.z].every(Number.isFinite);
  state.hasLinearAcceleration=state.hasLinearAcceleration||hasLinear;state.hasGravityAcceleration=state.hasGravityAcceleration||hasGravity;
  if(hasGravity)state.accelGravityDevice={x:ag.x,y:ag.y,z:ag.z};
  if(hasLinear)state.accelDevice={x:a.x,y:a.y,z:a.z};
  const rot=state.baseQ?relativeMotionQ():state.motionQ;
  if(hasLinear){
    const corrected={x:state.accelDevice.x-state.accelBiasDevice.x,y:state.accelDevice.y-state.accelBiasDevice.y,z:state.accelDevice.z-state.accelBiasDevice.z};
    state.accelWorld=qRotate(rot,corrected);
  }else if(hasGravity&&state.originCaptured){
    const worldRaw=qRotate(rot,state.accelGravityDevice);
    state.accelWorld={x:worldRaw.x-state.gravityWorld.x,y:worldRaw.y-state.gravityWorld.y,z:worldRaw.z-state.gravityWorld.z};
  }else state.accelWorld={x:0,y:0,z:0};
  state.accelSamples.push({t:now,a:{...state.accelWorld}});while(state.accelSamples.length>360||(state.accelSamples[0]&&now-state.accelSamples[0].t>6000))state.accelSamples.shift();

  // Acceleration samples are retained for keyframe-to-keyframe preintegration.
  // They never independently free-run XYZ.
  state.metric.lastImuAt=now;
  state.lastMotionAt=now;state.imuCount++;
  if(now-state.imuStamp>=1000){state.imuHz=state.imuCount*1000/(now-state.imuStamp);state.imuCount=0;state.imuStamp=now;}
}

function loadAutomaticProfile(){
  try{
    const p=JSON.parse(localStorage.getItem(STORAGE_KEY)||'null');if(!p||Number(p.version)<3.1)return;
    // Camera geometry/timing are device properties and may seed a later session.
    // Metric scale is NOT reusable: every fresh monocular map owns a fresh internal gauge.
    if(Number.isFinite(p.fovX))state.fovX=clamp(p.fovX,34,105);
    if(Number.isFinite(p.videoImuLagMs))state.videoImuLagMs=clamp(p.videoImuLagMs,0,220);
    state.focalConfidence=clamp(p.focalConfidence||0.25,0,1);state.timingConfidence=clamp(p.timingConfidence||0.20,0,1);
  }catch{}
  state.scale=1;state.scaleStability=0;state.scaleLocked=false;
}
function saveAutomaticProfile(now=performance.now()){
  if(now-state.lastProfileSaveAt<PROFILE_SAVE_INTERVAL_MS)return;state.lastProfileSaveAt=now;
  try{localStorage.setItem(STORAGE_KEY,JSON.stringify({version:3.1,savedAt:new Date().toISOString(),fovX:state.fovX,videoImuLagMs:state.videoImuLagMs,lastSolvedScale:state.scaleLocked?state.scale:null,focalConfidence:state.focalConfidence,timingConfidence:state.timingConfidence}));}catch{}
}
function initializeWorldFromCurrentOrientation(){
  if(state.originCaptured||!state.lastOrientationAt)return false;
  state.baseQ=qAxis(0,1,0,cameraHeadingAngle(state.orientationQ));state.orientationCorrection=q();state.orientationHoldQ=null;state.originCaptured=true;state.worldRevision++;
  state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};
  state.originQuality=1;state.poseReason='World initialized automatically; learning map, timing, bias, and scale during normal movement';
  return true;
}
function beginTracking(){
  state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.baseQ=null;state.orientationCorrection=q();state.orientationHoldQ=null;state.previousFrameQ=null;state.previousFrameTime=0;state.previousFrame=null;state.frame=null;state.tracks=[];
  state.visualConfidence=0;state.motionConfidence=0;state.stationary=false;state.stationaryScore=0;state.stillSince=0;state.originCaptured=false;state.originQuality=0;state.biasSamples=[];state.gravitySamples=[];state.accelSamples=[];state.frameAccelWorld={x:0,y:0,z:0};state.validTracks=0;state.flowMagnitude=0;state.driftRate=0;state.lastPositionForDrift={x:0,y:0,z:0};state.translationDirectionConfidence=0;
  state.scale=1;state.scaleStability=0;state.scaleLocked=false;
  state.metric={scaleIntervals:[],scaleHistory:[],lastScaleSolve:null,lastImuAt:0,lastVisualAt:0,automaticUpdates:0};
  state.map={keyframes:[],landmarks:[],nextKeyframeId:1,nextLandmarkId:1,lastKeyframeId:null,sinceKeyframeVisual:0,poseInliers:0,reprojectionError:Infinity,confidence:0,relocalizations:0,loopClosures:0,lastRelocalizeAt:0,lastMapBuildAt:0,lastPoseAt:0,lastCorrection:0,gaugeReady:false,confirmed:0,lastAuthoritativeAt:0,lastAuthoritativePos:null,lastSolvedPos:null,relocalizationCandidates:[],bridgeAge:Infinity,suppressRenderUntil:0};
  loadAutomaticProfile();resetStress();if(ui.save)ui.save.disabled=true;if(ui.stress)ui.stress.disabled=false;
  setStage('tracking','Move naturally. The world lock self-initializes and continuously corrects itself—no calibration motions or measured distances.',100,'calibrating');
  initializeWorldFromCurrentOrientation();
}

function nearestOrientation(time) {
  const a=state.orientationSamples;
  if(!a.length)return state.orientationQ;
  if(time<=a[0].t)return a[0].q;
  const last=a[a.length-1]; if(time>=last.t)return last.q;
  for(let i=1;i<a.length;i++){
    if(a[i].t>=time){
      const p=a[i-1],n=a[i],t=clamp((time-p.t)/Math.max(1,n.t-p.t),0,1);
      return qSlerp(p.q,n.q,t);
    }
  }
  return last.q;
}

function startVideoLoop() {
  const cb=(now,meta)=>{
    state.videoCount++;
    if(now-state.videoStamp>=1000){state.videoHz=state.videoCount*1000/(now-state.videoStamp);state.videoCount=0;state.videoStamp=now;}
    processVideoFrame(now,meta);
    if(video.requestVideoFrameCallback)video.requestVideoFrameCallback(cb);
  };
  if(video.requestVideoFrameCallback)video.requestVideoFrameCallback(cb);
  else requestAnimationFrame(function fallback(now){processVideoFrame(now,{mediaTime:video.currentTime});requestAnimationFrame(fallback);});
}

function updateVisionCanvasSize() {
  const r=grid.getBoundingClientRect();
  const aspect=Math.max(0.35,Math.min(2.4,r.width/Math.max(1,r.height)));
  let w,h;
  if(aspect>=1){w=256;h=Math.max(112,Math.round(w/aspect));}
  else {h=256;w=Math.max(112,Math.round(h*aspect));}
  if(vision.width!==w||vision.height!==h){vision.width=w;vision.height=h;state.previousFrame=null;}
}

function captureGray() {
  if(video.readyState<2||!video.videoWidth)return null;
  updateVisionCanvasSize();
  const w=vision.width,h=vision.height;
  const srcAspect=video.videoWidth/video.videoHeight,dstAspect=w/h;
  let sx=0,sy=0,sw=video.videoWidth,sh=video.videoHeight;
  if(srcAspect>dstAspect){sw=video.videoHeight*dstAspect;sx=(video.videoWidth-sw)/2;}
  else{sh=video.videoWidth/dstAspect;sy=(video.videoHeight-sh)/2;}
  vctx.drawImage(video,sx,sy,sw,sh,0,0,w,h);
  const rgba=vctx.getImageData(0,0,w,h).data;
  const gray=new Uint8Array(w*h);
  for(let i=0,j=0;i<rgba.length;i+=4,j++)gray[j]=(rgba[i]*77+rgba[i+1]*150+rgba[i+2]*29)>>8;
  return {gray,w,h};
}

function cornerScore(img,w,h,x,y) {
  let a=0,b=0,c=0;
  for(let yy=-2;yy<=2;yy++)for(let xx=-2;xx<=2;xx++){
    const i=(y+yy)*w+x+xx,gx=img[i+1]-img[i-1],gy=img[i+w]-img[i-w];
    a+=gx*gx;b+=gx*gy;c+=gy*gy;
  }
  const tr=a+c,det=a*c-b*b;
  return det/(tr+1e-6);
}
function selectCorners(frame,maxPoints=90) {
  const {gray,w,h}=frame,pts=[];
  const cell=Math.max(18,Math.round(Math.min(w,h)/9)),border=12;
  for(let cy=border;cy<h-border;cy+=cell)for(let cx=border;cx<w-border;cx+=cell){
    let best=null;
    const yEnd=Math.min(cy+cell,h-border),xEnd=Math.min(cx+cell,w-border);
    for(let y=cy;y<yEnd;y+=3)for(let x=cx;x<xEnd;x+=3){const score=cornerScore(gray,w,h,x,y);if(!best||score>best.score)best={x,y,score};}
    if(best&&best.score>1800)pts.push(best);
  }
  return pts.sort((a,b)=>b.score-a.score).slice(0,maxPoints);
}
function patchSSD(a,b,w,h,x1,y1,x2,y2,r=3) {
  let s=0,n=0;x1=Math.round(x1);y1=Math.round(y1);x2=Math.round(x2);y2=Math.round(y2);
  if(x1-r<0||x1+r>=w||y1-r<0||y1+r>=h||x2-r<0||x2+r>=w||y2-r<0||y2+r>=h)return Infinity;
  for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++){const d=a[(y1+yy)*w+x1+xx]-b[(y2+yy)*w+x2+xx];s+=d*d;n++;}
  return s/n;
}
function grayBilinear(img,w,h,x,y){
  if(x<0||y<0||x>w-1||y>h-1)return NaN;const x0=Math.floor(x),y0=Math.floor(y),x1=Math.min(w-1,x0+1),y1=Math.min(h-1,y0+1),tx=x-x0,ty=y-y0;
  const a=img[y0*w+x0]*(1-tx)+img[y0*w+x1]*tx,b=img[y1*w+x0]*(1-tx)+img[y1*w+x1]*tx;return a*(1-ty)+b*ty;
}
function patchSSDSubpixel(a,b,w,h,x1,y1,x2,y2,r=3){
  if(x1-r<1||x1+r>w-2||y1-r<1||y1+r>h-2||x2-r<1||x2+r>w-2||y2-r<1||y2+r>h-2)return Infinity;let ss=0,n=0;
  for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++){const av=grayBilinear(a,w,h,x1+xx,y1+yy),bv=grayBilinear(b,w,h,x2+xx,y2+yy);const d=av-bv;ss+=d*d;n++;}return ss/Math.max(1,n);
}
function refineSubpixel(prev,curr,w,h,p,best){
  let out={...best};for(const step of [0.5,0.25]){const base={...out};for(let dy=-step;dy<=step+1e-9;dy+=step)for(let dx=-step;dx<=step+1e-9;dx+=step){const x=base.x+dx,y=base.y+dy,score=patchSSDSubpixel(prev,curr,w,h,p.x,p.y,x,y,3);if(score<out.score)out={score,x,y};}}return out;
}
function reverseTrack(from,to,w,h,p,target,search) {
  let best={score:Infinity,x:target.x,y:target.y};
  for(let dy=-search;dy<=search;dy++)for(let dx=-search;dx<=search;dx++){
    const x=target.x+dx,y=target.y+dy,score=patchSSD(from,to,w,h,p.x,p.y,x,y,3);
    if(score<best.score)best={score,x,y};
  }
  return best;
}
function trackPoint(prev,curr,w,h,p,search=12) {
  let best={score:Infinity,x:p.x,y:p.y},second=Infinity;
  for(let dy=-search;dy<=search;dy+=2)for(let dx=-search;dx<=search;dx+=2){
    const score=patchSSD(prev,curr,w,h,p.x,p.y,p.x+dx,p.y+dy,3);
    if(score<best.score){second=best.score;best={score,x:p.x+dx,y:p.y+dy};}else if(score<second)second=score;
  }
  const coarse={...best};
  for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
    const score=patchSSD(prev,curr,w,h,p.x,p.y,coarse.x+dx,coarse.y+dy,3);
    if(score<best.score)best={score,x:coarse.x+dx,y:coarse.y+dy};
  }
  best=refineSubpixel(prev,curr,w,h,p,best);
  const reverse=reverseTrack(curr,prev,w,h,best,p,5),fb=Math.hypot(reverse.x-p.x,reverse.y-p.y);
  const uniqueness=clamp((second-best.score)/(second+1e-6),0,1);
  return {...best,fb,confidence:clamp(uniqueness*2.7,0,1)*clamp((2200-best.score)/1900,0,1)};
}
function predictedRotatedPixel(p,prevQ,currQ,fx,fy,cx,cy) {
  // Exact ray rotation rather than a small-angle optical-flow approximation.
  // This removes axis-sign assumptions and remains valid through full 360° turns.
  const rayPrev={x:(p.x-cx)/fx,y:-(p.y-cy)/fy,z:-1};
  const worldRay=qRotate(prevQ,rayPrev);
  const rayCurr=qRotate(qInv(currQ),worldRay);
  if(rayCurr.z>=-1e-5)return null;
  return {x:cx+fx*(rayCurr.x/-rayCurr.z),y:cy-fy*(rayCurr.y/-rayCurr.z)};
}
function qualityTracks(prevFrame,currFrame) {
  const {w,h}=currFrame,corners=selectCorners(prevFrame,95),raw=[];
  for(const p of corners){
    const t=trackPoint(prevFrame.gray,currFrame.gray,w,h,p,12);
    if(t.confidence<0.10||t.fb>2.8||!Number.isFinite(t.score))continue;
    raw.push({p,q:{x:t.x,y:t.y},observed:{x:t.x-p.x,y:t.y-p.y},confidence:t.confidence});
  }
  return raw;
}
function residualSolution(raw,prevQ,currQ,w,h,hfov=state.fovX) {
  const fx=0.5*w/Math.tan(hfov*DEG/2),vfov=2*Math.atan(Math.tan(hfov*DEG/2)*(h/w))/DEG,fy=0.5*h/Math.tan(vfov*DEG/2),cx=w/2,cy=h/2;
  const tracks=[];
  for(const t of raw){
    const predicted=predictedRotatedPixel(t.p,prevQ,currQ,fx,fy,cx,cy);
    if(!predicted)continue;
    const rot={x:predicted.x-t.p.x,y:predicted.y-t.p.y};
    tracks.push({...t,rot,residual:{x:t.observed.x-rot.x,y:t.observed.y-rot.y}});
  }
  if(tracks.length<6)return {raw,tracks,inliers:[],confidence:0,fx,fy};
  const rx=tracks.map(t=>t.residual.x),ry=tracks.map(t=>t.residual.y),mx=median(rx),my=median(ry),sx=mad(rx,mx),sy=mad(ry,my);
  const inliers=tracks.filter(t=>Math.abs(t.residual.x-mx)<Math.max(1.7,3.4*sx)&&Math.abs(t.residual.y-my)<Math.max(1.7,3.4*sy));
  const confidence=clamp(inliers.length/28,0,1)*clamp(inliers.length/Math.max(1,tracks.length),0,1);
  return {raw,tracks,inliers,confidence,fx,fy};
}

function rotationProjectionError(raw,prevQ,currQ,w,h,hfov=state.fovX) {
  const fx=0.5*w/Math.tan(hfov*DEG/2),vfov=2*Math.atan(Math.tan(hfov*DEG/2)*(h/w))/DEG,fy=0.5*h/Math.tan(vfov*DEG/2),cx=w/2,cy=h/2,errors=[];
  for(const t of raw){const pp=predictedRotatedPixel(t.p,prevQ,currQ,fx,fy,cx,cy);if(pp)errors.push(Math.hypot(t.q.x-pp.x,t.q.y-pp.y));}
  return errors.length>=10?median(errors):Infinity;
}
function estimateVideoImuTiming(raw,prevTime,currTime,w,h) {
  if(raw.length<12)return;
  let best=null;
  for(let lag=0;lag<=220;lag+=10){
    const pq=nearestOrientation(prevTime-lag),cq=nearestOrientation(currTime-lag),rot=qAngle(pq,cq);
    if(rot<0.0025||rot>0.18)continue;
    const error=rotationProjectionError(raw,pq,cq,w,h,state.fovX);
    if(Number.isFinite(error)&&(!best||error<best.error))best={lag,error};
  }
  if(!best||best.error>8)return;
  state.timingSamples.push(best);if(state.timingSamples.length>48)state.timingSamples.shift();
  const recent=state.timingSamples.slice(-30),m=median(recent.map(x=>x.lag)),spread=mad(recent.map(x=>x.lag),m),err=median(recent.map(x=>x.error));
  state.videoImuLagMs=lerp(state.videoImuLagMs,m,0.22);
  state.timingConfidence=clamp(recent.length/12,0,1)*clamp(1-spread/55,0,1)*clamp(1-err/8,0,1);
}

function estimateFovFromTracks(raw,prevQ,currQ,w,h) {
  const rotation=qAngle(prevQ,currQ),acc=vecLength(correctedAcceleration());
  if(raw.length<12||rotation<0.003||rotation>0.14||acc>1.6)return;
  let best=null;
  for(let hfov=34;hfov<=105;hfov+=1){
    const fx=0.5*w/Math.tan(hfov*DEG/2),vfov=2*Math.atan(Math.tan(hfov*DEG/2)*(h/w))/DEG,fy=0.5*h/Math.tan(vfov*DEG/2),cx=w/2,cy=h/2;
    const errors=[];
    for(const t of raw){
      const pp=predictedRotatedPixel(t.p,prevQ,currQ,fx,fy,cx,cy);
      if(!pp)continue;
      errors.push(Math.hypot(t.q.x-pp.x,t.q.y-pp.y));
    }
    if(errors.length<10)continue;
    const m=median(errors),spread=mad(errors,m),trim=errors.filter(e=>e<Math.max(2.3,m+2.6*spread));
    const score=median(trim);
    if(!best||score<best.score)best={hfov,score,count:trim.length};
  }
  if(!best||best.count<10||best.score>5.5)return;
  state.fovSamples.push({fov:best.hfov,error:best.score});
  if(state.fovSamples.length>70)state.fovSamples.shift();
  if(state.fovSamples.length>=7){
    const recent=state.fovSamples.slice(-40),values=recent.map(s=>s.fov),m=median(values),spread=mad(values,m),error=median(recent.map(s=>s.error));
    state.fovX=clamp(lerp(state.fovX,m,0.20),34,105);
    state.fovY=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w))/DEG;
    state.projectionError=error;
    state.focalConfidence=clamp(recent.length/24,0,1)*clamp(1-spread/7,0,1)*clamp(1-error/6,0,1);
  }
}

function correctedAcceleration(){return state.accelWorld;}
function averageAcceleration(t0,t1){const a=state.accelSamples.filter(s=>s.t>=t0-12&&s.t<=t1+12);if(!a.length)return state.accelWorld;return{x:a.reduce((n,s)=>n+s.a.x,0)/a.length,y:a.reduce((n,s)=>n+s.a.y,0)/a.length,z:a.reduce((n,s)=>n+s.a.z,0)/a.length};}
function cross3(a,b){return{x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x};}
function dot3(a,b){return a.x*b.x+a.y*b.y+a.z*b.z;}
function norm3(v){const n=vecLength(v)||1;return{x:v.x/n,y:v.y/n,z:v.z/n};}
function smallestEigenVectorSym3(m) {
  const a=[[m[0][0],m[0][1],m[0][2]],[m[1][0],m[1][1],m[1][2]],[m[2][0],m[2][1],m[2][2]]],v=[[1,0,0],[0,1,0],[0,0,1]];
  for(let iter=0;iter<14;iter++){
    let p=0,r=1,max=Math.abs(a[0][1]);
    for(const [i,j] of [[0,2],[1,2]])if(Math.abs(a[i][j])>max){max=Math.abs(a[i][j]);p=i;r=j;}
    if(max<1e-9)break;
    const phi=0.5*Math.atan2(2*a[p][r],a[r][r]-a[p][p]),c=Math.cos(phi),ss=Math.sin(phi);
    for(let k=0;k<3;k++){const apk=a[p][k],ark=a[r][k];a[p][k]=c*apk-ss*ark;a[r][k]=ss*apk+c*ark;}
    for(let k=0;k<3;k++){const akp=a[k][p],akr=a[k][r];a[k][p]=c*akp-ss*akr;a[k][r]=ss*akp+c*akr;}
    for(let k=0;k<3;k++){const vkp=v[k][p],vkr=v[k][r];v[k][p]=c*vkp-ss*vkr;v[k][r]=ss*vkp+c*vkr;}
  }
  const vals=[a[0][0],a[1][1],a[2][2]],order=[0,1,2].sort((i,j)=>vals[i]-vals[j]),i=order[0],j=order[1];
  return {vector:norm3({x:v[0][i],y:v[1][i],z:v[2][i]}),confidence:clamp(1-Math.abs(vals[i])/(Math.abs(vals[j])+1e-8),0,1)};
}
function solveTranslationDirection(solution,w,h,prevQ,currQ) {
  const {fx,fy}=solution,cx=w/2,cy=h/2,normals=[];
  for(const t of solution.inliers){
    const b1=norm3({x:(t.p.x-cx)/fx,y:-(t.p.y-cy)/fy,z:-1});
    const world=qRotate(prevQ,b1),r1=norm3(qRotate(qInv(currQ),world));
    const b2=norm3({x:(t.q.x-cx)/fx,y:-(t.q.y-cy)/fy,z:-1}),n=cross3(b2,r1),nl=vecLength(n);
    if(nl>1e-6)normals.push({x:n.x/nl,y:n.y/nl,z:n.z/nl});
  }
  if(normals.length<8)return null;
  const fit=(arr)=>{const m=[[0,0,0],[0,0,0],[0,0,0]];for(const n of arr){m[0][0]+=n.x*n.x;m[0][1]+=n.x*n.y;m[0][2]+=n.x*n.z;m[1][0]+=n.y*n.x;m[1][1]+=n.y*n.y;m[1][2]+=n.y*n.z;m[2][0]+=n.z*n.x;m[2][1]+=n.z*n.y;m[2][2]+=n.z*n.z;}return smallestEigenVectorSym3(m);};
  let sol=fit(normals),errs=normals.map(n=>Math.abs(dot3(n,sol.vector))),em=median(errs),es=mad(errs,em),good=normals.filter((n,i)=>errs[i]<Math.max(0.012,em+2.8*es));
  if(good.length>=7)sol=fit(good);
  sol.confidence*=clamp(good.length/Math.max(1,normals.length),0,1);
  return sol;
}
function estimateTranslation(solution,w,h,dt,frameQ,prevQ) {
  if(solution.inliers.length<8||dt<=0)return {x:0,y:0,z:0,confidence:0,rawMagnitude:0};
  const {fx,fy}=solution,cx=w/2,cy=h/2;
  // A depth-agnostic epipolar fit determines translation DIRECTION after exact rotation removal.
  // The older median-flow estimate is retained only for arbitrary scale/magnitude and sign.
  const lateralX=median(solution.inliers.map(t=>-t.residual.x/fx));
  const lateralY=median(solution.inliers.map(t=> t.residual.y/fy));
  const zSamples=[];
  for(const t of solution.inliers){
    const nx=(t.p.x-cx)/fx,ny=-(t.p.y-cy)/fy,denom=nx*nx+ny*ny;if(denom<0.010)continue;
    const ux=t.residual.x/fx+lateralX,uy=-t.residual.y/fy+lateralY;zSamples.push((ux*nx+uy*ny)/denom);
  }
  const radial=zSamples.length?median(zSamples):0,guess={x:lateralX,y:lateralY,z:-radial},guessMag=vecLength(guess);
  const dirFit=solveTranslationDirection(solution,w,h,prevQ,frameQ);
  let localDelta=guess,dirConfidence=0.35;
  if(dirFit&&guessMag>1e-6){
    // Epipolar t points from camera 2 toward camera 1, so camera-center displacement is -t.
    let d={x:-dirFit.vector.x,y:-dirFit.vector.y,z:-dirFit.vector.z};
    if(dot3(d,guess)<0)d={x:-d.x,y:-d.y,z:-d.z};
    localDelta={x:d.x*guessMag,y:d.y*guessMag,z:d.z*guessMag};dirConfidence=dirFit.confidence;
  }
  const worldDelta=qRotate(relativeQ(frameQ),localDelta),rawMagnitude=vecLength(worldDelta);
  const spreadX=mad(solution.inliers.map(t=>t.residual.x)),spreadY=mad(solution.inliers.map(t=>t.residual.y));
  const parallaxPx=median(solution.inliers.map(t=>Math.hypot(t.residual.x,t.residual.y)));
  const geometric=clamp(1-(spreadX+spreadY)/(Math.max(w,h)*0.055),0.20,1),parallax=clamp((parallaxPx-0.10)/1.15,0,1);
  state.translationDirectionConfidence=lerp(state.translationDirectionConfidence,dirConfidence,0.24);
  return {...worldDelta,confidence:solution.confidence*geometric*(0.35+0.65*dirConfidence)*parallax,rawMagnitude};
}

function vAdd(a,b){return{x:a.x+b.x,y:a.y+b.y,z:a.z+b.z};}
function vSub(a,b){return{x:a.x-b.x,y:a.y-b.y,z:a.z-b.z};}
function vScale(a,k){return{x:a.x*k,y:a.y*k,z:a.z*k};}
function vLerp(a,b,t){return{x:lerp(a.x,b.x,t),y:lerp(a.y,b.y,t),z:lerp(a.z,b.z,t)};}
function rmsDescriptorDistance(a,b){if(!a||!b||a.length!==b.length)return Infinity;let s=0;for(let i=0;i<a.length;i++){const d=a[i]-b[i];s+=d*d;}return Math.sqrt(s/a.length);}
function descriptorAt(frame,x,y){
  const {gray,w,h}=frame;x=Math.round(x);y=Math.round(y);if(x<9||y<9||x>=w-9||y>=h-9)return null;const values=[];let sum=0;
  // 7x7 normalized appearance patch (49 dimensions). This is intentionally denser than
  // the old 5x5 signature so persistent landmark identity has much lower collision risk.
  for(let yy=-6;yy<=6;yy+=2)for(let xx=-6;xx<=6;xx+=2){const v=gray[(y+yy)*w+x+xx];values.push(v);sum+=v;}
  const mean=sum/values.length;let ss=0;for(const v of values){const d=v-mean;ss+=d*d;}const sd=Math.sqrt(ss/values.length)+5;return values.map(v=>(v-mean)/sd);
}
function solve3x3(A,b){
  const m=[[A[0][0],A[0][1],A[0][2],b[0]],[A[1][0],A[1][1],A[1][2],b[1]],[A[2][0],A[2][1],A[2][2],b[2]]];
  for(let c=0;c<3;c++){
    let r=c;for(let i=c+1;i<3;i++)if(Math.abs(m[i][c])>Math.abs(m[r][c]))r=i;
    if(Math.abs(m[r][c])<1e-9)return null;[m[c],m[r]]=[m[r],m[c]];
    const d=m[c][c];for(let j=c;j<4;j++)m[c][j]/=d;
    for(let i=0;i<3;i++)if(i!==c){const f=m[i][c];for(let j=c;j<4;j++)m[i][j]-=f*m[c][j];}
  }
  return{x:m[0][3],y:m[1][3],z:m[2][3]};
}
function rayWorldFromPixel(px,camQ,w,h,fx,fy){const c={x:w/2,y:h/2},r=norm3({x:(px.x-c.x)/fx,y:-(px.y-c.y)/fy,z:-1});return norm3(qRotate(camQ,r));}
function triangulateRays(c1,d1,c2,d2){
  const w0=vSub(c1,c2),a=dot3(d1,d1),b=dot3(d1,d2),c=dot3(d2,d2),d=dot3(d1,w0),e=dot3(d2,w0),den=a*c-b*b;
  if(Math.abs(den)<1e-5)return null;const t1=(b*e-c*d)/den,t2=(a*e-b*d)/den;if(t1<=0||t2<=0)return null;
  const p1=vAdd(c1,vScale(d1,t1)),p2=vAdd(c2,vScale(d2,t2)),gap=posDist(p1,p2),p=vScale(vAdd(p1,p2),0.5);
  const angle=Math.acos(clamp(dot3(d1,d2),-1,1));return{p,gap,angle,t1,t2};
}
function projectWorldPoint(P,camQ,camPos,w,h,fx,fy){const c=cameraPoint(P,camQ,camPos);if(c.z>=-0.06)return null;const p=projectCamera(c,fx,fy,w/2,h/2);return p.x>=0&&p.x<w&&p.y>=0&&p.y<h?p:null;}
function trackPointSeeded(prev,curr,w,h,p,seed,search=8){
  let best={score:Infinity,x:seed.x,y:seed.y},second=Infinity;
  for(let dy=-search;dy<=search;dy+=2)for(let dx=-search;dx<=search;dx+=2){const x=seed.x+dx,y=seed.y+dy,score=patchSSD(prev,curr,w,h,p.x,p.y,x,y,3);if(score<best.score){second=best.score;best={score,x,y};}else if(score<second)second=score;}
  const coarse={...best};for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const score=patchSSD(prev,curr,w,h,p.x,p.y,coarse.x+dx,coarse.y+dy,3);if(score<best.score)best={score,x:coarse.x+dx,y:coarse.y+dy};}
  best=refineSubpixel(prev,curr,w,h,p,best);
  let back={score:Infinity,x:p.x,y:p.y};for(let dy=-6;dy<=6;dy+=2)for(let dx=-6;dx<=6;dx+=2){const x=p.x+dx,y=p.y+dy,score=patchSSD(curr,prev,w,h,best.x,best.y,x,y,3);if(score<back.score)back={score,x,y};}const fb=Math.hypot(back.x-p.x,back.y-p.y),uniqueness=clamp((second-best.score)/(second+1e-6),0,1);
  return{...best,fb,confidence:clamp(uniqueness*2.8,0,1)*clamp((2400-best.score)/2100,0,1)};
}
function rescaleWorld(newScale){
  if(!Number.isFinite(newScale)||newScale<=0)return;const old=state.scale||1,f=newScale/old;if(Math.abs(f-1)<1e-5){state.scale=newScale;return;}
  state.position=vScale(state.position,f);state.velocity=vScale(state.velocity,f);state.lastPositionForDrift=vScale(state.lastPositionForDrift,f);
  for(const kf of state.map.keyframes)kf.pos=vScale(kf.pos,f);for(const lm of state.map.landmarks){lm.pos=vScale(lm.pos,f);if(lm.anchor)lm.anchor.pos=vScale(lm.anchor.pos,f);}
  const m=state.map;if(m.lastAuthoritativePos)m.lastAuthoritativePos=vScale(m.lastAuthoritativePos,f);if(m.lastSolvedPos)m.lastSolvedPos=vScale(m.lastSolvedPos,f);m.lastCorrection*=f;m.relocalizationCandidates=[];state.scale=newScale;
}
function preintegrateAcceleration(t0,t1){
  if(!(t1>t0))return null;const samples=state.accelSamples.filter(x=>x.t>=t0-35&&x.t<=t1+35).sort((a,b)=>a.t-b.t);if(samples.length<2)return null;
  let beta={x:0,y:0,z:0},alpha={x:0,y:0,z:0},elapsed=0;
  for(let i=1;i<samples.length;i++){
    const ta=Math.max(t0,samples[i-1].t),tb=Math.min(t1,samples[i].t);if(tb<=ta)continue;const dt=clamp((tb-ta)/1000,0,0.06);if(dt<=0)continue;
    const a=vScale(vAdd(samples[i-1].a,samples[i].a),0.5);alpha=vAdd(alpha,vAdd(vScale(beta,dt),vScale(a,0.5*dt*dt)));beta=vAdd(beta,vScale(a,dt));elapsed+=dt;
  }
  return elapsed>=0.08?{dt:elapsed,alpha,beta}:null;
}
function symmetricConditionEstimate(A,iters=120){
  const n=A.length,M=A.map(r=>r.slice());for(let it=0;it<iters;it++){let p=0,qx=1,best=0;for(let i=0;i<n;i++)for(let j=i+1;j<n;j++){const v=Math.abs(M[i][j]);if(v>best){best=v;p=i;qx=j;}}if(best<1e-10)break;const app=M[p][p],aqq=M[qx][qx],apq=M[p][qx],phi=.5*Math.atan2(2*apq,aqq-app),c=Math.cos(phi),ss=Math.sin(phi);for(let k=0;k<n;k++){const apk=M[p][k],aqk=M[qx][k];M[p][k]=c*apk-ss*aqk;M[qx][k]=ss*apk+c*aqk;}for(let k=0;k<n;k++){const akp=M[k][p],akq=M[k][qx];M[k][p]=c*akp-ss*akq;M[k][qx]=ss*akp+c*akq;}}
  const eig=Array.from({length:n},(_,i)=>Math.max(1e-12,Math.abs(M[i][i])));return Math.max(...eig)/Math.min(...eig);
}
function solveMetricScaleIntervals(){
  const qs=state.metric.scaleIntervals,N=qs.length;if(N<3)return null;const n=1+3*(N+1),rows=[],rhs=[];
  const addRow=(coef,b)=>{rows.push(coef);rhs.push(b);};
  for(let i=0;i<N;i++){const qv=qs[i],vi=1+3*i,vj=1+3*(i+1);for(let k=0;k<3;k++){
    let c=new Array(n).fill(0);c[0]=[qv.d.x,qv.d.y,qv.d.z][k];c[vi+k]=-qv.dt;addRow(c,[qv.alpha.x,qv.alpha.y,qv.alpha.z][k]);
    c=new Array(n).fill(0);c[vj+k]=1;c[vi+k]=-1;addRow(c,[qv.beta.x,qv.beta.y,qv.beta.z][k]);
  }}
  const H=Array.from({length:n},()=>Array(n).fill(0)),g=Array(n).fill(0);for(let r=0;r<rows.length;r++)for(let i=0;i<n;i++){g[i]+=rows[r][i]*rhs[r];for(let j=0;j<n;j++)H[i][j]+=rows[r][i]*rows[r][j];}
  for(let i=0;i<n;i++)H[i][i]+=1e-7;const x=solveLinearSystem(H,g);if(!x)return null;let se=0;for(let r=0;r<rows.length;r++){let y=0;for(let i=0;i<n;i++)y+=rows[r][i]*x[i];se+=(y-rhs[r])**2;}
  const scale=x[0],rms=Math.sqrt(se/Math.max(1,rows.length)),excitation=qs.reduce((z,qv)=>z+vecLength(qv.beta),0)/N,visualTravel=qs.reduce((z,qv)=>z+vecLength(qv.d),0),condition=symmetricConditionEstimate(H);
  return{scale,rms,excitation,visualTravel,condition};
}
function addMetricScaleInterval(dpVisual,pre){
  if(state.scaleLocked||!pre||pre.dt<0.08||pre.dt>3.0||vecLength(dpVisual)<0.035)return;const m=state.metric;
  m.scaleIntervals.push({d:{...dpVisual},dt:pre.dt,alpha:{...pre.alpha},beta:{...pre.beta}});if(m.scaleIntervals.length>8)m.scaleIntervals.shift();const sol=solveMetricScaleIntervals();m.lastScaleSolve=sol;if(!sol)return;
  const candidate=Number.isFinite(sol.scale)&&sol.scale>0.015&&sol.scale<8&&sol.rms<0.22&&sol.excitation>0.018&&sol.visualTravel>0.22&&Number.isFinite(sol.condition)&&sol.condition<1e10;
  if(!candidate){state.scaleStability=lerp(state.scaleStability,0,0.12);return;}m.scaleHistory.push(sol.scale);if(m.scaleHistory.length>6)m.scaleHistory.shift();
  const mean=m.scaleHistory.reduce((a,b)=>a+b,0)/m.scaleHistory.length,sd=Math.sqrt(m.scaleHistory.reduce((a,b)=>a+(b-mean)**2,0)/m.scaleHistory.length),cv=sd/Math.max(mean,1e-6);
  state.scaleStability=clamp((m.scaleHistory.length/4)*clamp(1-cv/0.22,0,1)*clamp(1-sol.rms/0.22,0,1),0,1);
  if(m.scaleHistory.length>=4&&cv<0.15&&state.scaleStability>0.72){rescaleWorld(mean);state.scaleLocked=true;state.scaleStability=1;m.automaticUpdates++;state.poseReason=`Metric scale self-locked from visual map + IMU (${mean.toFixed(3)} m/u)`;}
}
function recordScaleInterval(prevKf,nextKf){
  if(!prevKf||!nextKf||!state.map.gaugeReady||state.scaleLocked)return;const pre=preintegrateAcceleration(prevKf.imuT??(prevKf.t-state.videoImuLagMs),nextKf.imuT??(nextKf.t-state.videoImuLagMs));if(!pre)return;const dp=vSub(nextKf.gaugePos,prevKf.gaugePos);addMetricScaleInterval(dp,pre);
}
function addKeyframe(frame,camQ,now,posOverride=null){
  const m=state.map,prev=keyframeById(m.lastKeyframeId),pose=posOverride?{...posOverride}:{...state.position},kf={id:m.nextKeyframeId++,t:now,imuT:Number.isFinite(frame.imuTime)?frame.imuTime:now-state.videoImuLagMs,pos:pose,gaugePos:vScale(pose,1/Math.max(state.scale,1e-9)),q:{...camQ},frame:{gray:frame.gray.slice(),w:frame.w,h:frame.h},fovX:state.fovX};
  m.keyframes.push(kf);m.lastKeyframeId=kf.id;m.sinceKeyframeVisual=0;if(m.keyframes.length>28)m.keyframes.shift();if(prev&&m.gaugeReady)recordScaleInterval(prev,kf);return kf;
}
function keyframeById(id){return state.map.keyframes.find(k=>k.id===id)||null;}
function frameMapMatches(kf,frame,currQ){
  if(!kf||kf.frame.w!==frame.w||kf.frame.h!==frame.h)return[];const w=frame.w,h=frame.h,fx=.5*w/Math.tan(state.fovX*DEG/2),vfov=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vfov/2),out=[],seeds=[];
  // Re-track landmark observations that are already known in this keyframe first. This carries
  // physical identity across keyframes instead of hoping a fresh corner detector chooses them again.
  const known=state.map.landmarks.filter(l=>l.keyObs?.[kf.id]).sort((a,b)=>(b.observations+b.confidence)-(a.observations+a.confidence)).slice(0,58);for(const lm of known)seeds.push({p:{...lm.keyObs[kf.id]},landmarkHint:lm});
  for(const p of selectCorners(kf.frame,85)){if(seeds.some(s=>Math.hypot(s.p.x-p.x,s.p.y-p.y)<4))continue;seeds.push({p,landmarkHint:null});if(seeds.length>=95)break;}
  for(const seed of seeds){const p=seed.p,pred=predictedRotatedPixel(p,kf.q,currQ,fx,fy,w/2,h/2);if(!pred||pred.x<9||pred.y<9||pred.x>w-9||pred.y>h-9)continue;const t=trackPointSeeded(kf.frame.gray,frame.gray,w,h,p,pred,10);if(t.confidence<0.10||t.fb>3.2)continue;const d1=descriptorAt(kf.frame,p.x,p.y),d2=descriptorAt(frame,t.x,t.y);if(!d1||!d2||rmsDescriptorDistance(d1,d2)>0.72)continue;out.push({p,q:{x:t.x,y:t.y},descriptor:d2,confidence:t.confidence,landmarkHint:seed.landmarkHint});}
  return out;
}
function bootstrapTranslationDirection(kf,frame,currQ,matches){
  if(!kf||matches.length<10)return null;const raw=matches.map(mt=>({p:{...mt.p},q:{...mt.q},observed:{x:mt.q.x-mt.p.x,y:mt.q.y-mt.p.y},confidence:mt.confidence||1})),sol=residualSolution(raw,kf.q,currQ,frame.w,frame.h,state.fovX);
  if(sol.inliers.length<10)return null;const fit=solveTranslationDirection(sol,frame.w,frame.h,kf.q,currQ);if(!fit||fit.confidence<0.20)return null;const {fx,fy}=sol,cx=frame.w/2,cy=frame.h/2;
  const lateralX=median(sol.inliers.map(t=>-t.residual.x/fx)),lateralY=median(sol.inliers.map(t=>t.residual.y/fy)),zSamples=[];
  for(const t of sol.inliers){const nx=(t.p.x-cx)/fx,ny=-(t.p.y-cy)/fy,den=nx*nx+ny*ny;if(den<0.01)continue;const ux=t.residual.x/fx+lateralX,uy=-t.residual.y/fy+lateralY;zSamples.push((ux*nx+uy*ny)/den);}
  const guess={x:lateralX,y:lateralY,z:-(zSamples.length?median(zSamples):0)};let local={x:-fit.vector.x,y:-fit.vector.y,z:-fit.vector.z};if(vecLength(guess)>1e-5&&dot3(local,guess)<0)local=vScale(local,-1);
  const parallax=median(sol.inliers.map(t=>Math.hypot(t.residual.x,t.residual.y)));if(parallax<0.75)return null;return{direction:norm3(qRotate(currQ,local)),confidence:fit.confidence*clamp(sol.inliers.length/24,0,1)*clamp((parallax-.5)/2.0,0,1),parallax,inliers:sol.inliers.length};
}
function buildPersistentMap(frame,camQ,now){
  const m=state.map;if(!state.originCaptured||state.visualConfidence<0.07)return;let kf=keyframeById(m.lastKeyframeId);if(!kf){addKeyframe(frame,camQ,now);return;}
  const rot=qAngle(kf.q,camQ),age=now-kf.t;if(age<240)return;let matches=frameMapMatches(kf,frame,camQ);if(matches.length<10)return;
  if(!m.gaugeReady){
    const boot=bootstrapTranslationDirection(kf,frame,camQ,matches);if(!boot||boot.confidence<0.12)return;
    // The first baseline is exactly one INTERNAL VISUAL UNIT. It is not a physical distance,
    // is never shown to the user, and is later converted to metres by the visual/IMU scale solve.
    state.position=vAdd(kf.pos,boot.direction);state.velocity={x:0,y:0,z:0};m.gaugeReady=true;m.lastAuthoritativeAt=now;m.lastAuthoritativePos={...state.position};m.lastSolvedPos={...state.position};m.bridgeAge=0;m.confidence=Math.max(m.confidence,0.14);state.poseReason=`Visual map gauge initialized automatically (${boot.inliers} epipolar inliers)`;
  }else{
    if(now-m.lastAuthoritativeAt>180)return;const authoritativePos=m.lastSolvedPos||state.position,baselineGauge=posDist(kf.gaugePos,vScale(authoritativePos,1/Math.max(state.scale,1e-9)));if(baselineGauge<0.08&&rot<7*DEG&&m.sinceKeyframeVisual<0.012)return;
  }
  const currentPos=(m.lastSolvedPos&&now-m.lastAuthoritativeAt<180)?m.lastSolvedPos:state.position,baseline=posDist(kf.pos,currentPos),w=frame.w,h=frame.h,fx=.5*w/Math.tan(state.fovX*DEG/2),vfov=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vfov/2),created=[],currentKeyframeId=m.nextKeyframeId;
  if(baseline>Math.max(0.004,state.scale*0.004))for(const mt of matches){
    // First propagate identity through the shared keyframe pixel. This is much stronger than
    // rediscovering the same physical point from 3D proximity on every baseline.
    let associated=mt.landmarkHint||null,bestAssoc=Infinity;if(!associated)for(const lm of m.landmarks){const kp=lm.keyObs?.[kf.id];if(!kp)continue;const px=Math.hypot(kp.x-mt.p.x,kp.y-mt.p.y),dd=rmsDescriptorDistance(lm.descriptor,mt.descriptor);if(px<3.0&&dd<0.58&&px+dd*4<bestAssoc){bestAssoc=px+dd*4;associated=lm;}}
    const d1=rayWorldFromPixel(mt.p,kf.q,w,h,fx,fy),d2=rayWorldFromPixel(mt.q,camQ,w,h,fx,fy),tri=triangulateRays(kf.pos,d1,currentPos,d2);if(!tri)continue;if(tri.angle<1.0*DEG||tri.gap>Math.max(0.035,baseline*0.18))continue;const depthMax=Math.max(0.4,baseline*120);if(tri.t1>depthMax||tri.t2>depthMax)continue;
    if(associated){if(associated.lastConfirmedKeyframeId!==currentKeyframeId){associated.observations++;associated.lastConfirmedKeyframeId=currentKeyframeId;}associated.keyObs=associated.keyObs||{};associated.keyObs[currentKeyframeId]={...mt.q};associated.lastSeen=now;associated.confidence=clamp(associated.confidence+0.04,0,1);associated.pos=vLerp(associated.pos,tri.p,clamp(0.18/Math.sqrt(associated.observations),0.035,0.10));continue;}
    let duplicate=null;for(const lm of m.landmarks){if(posDist(lm.pos,tri.p)<Math.max(0.018,state.scale*0.012)&&rmsDescriptorDistance(lm.descriptor,mt.descriptor)<0.42){duplicate=lm;break;}}
    if(duplicate){if(duplicate.lastConfirmedKeyframeId!==currentKeyframeId){duplicate.observations++;duplicate.lastConfirmedKeyframeId=currentKeyframeId;}duplicate.keyObs=duplicate.keyObs||{};duplicate.keyObs[kf.id]=duplicate.keyObs[kf.id]||{...mt.p};duplicate.keyObs[currentKeyframeId]={...mt.q};duplicate.lastSeen=now;duplicate.confidence=clamp(duplicate.confidence+0.025,0,1);}
    else created.push({id:m.nextLandmarkId++,pos:tri.p,descriptor:mt.descriptor,observations:2,lastConfirmedKeyframeId:currentKeyframeId,createdAt:now,lastSeen:now,confidence:clamp(mt.confidence*(tri.angle/(4*DEG)),0.15,1),keyObs:{[kf.id]:{...mt.p},[currentKeyframeId]:{...mt.q}},anchor:{pos:{...kf.pos},q:{...kf.q},pixel:{...mt.p},w,h,fx,fy}});if(created.length>=28)break;
  }
  m.landmarks.push(...created);if(m.landmarks.length>520){m.landmarks.sort((a,b)=>(b.observations+2*b.confidence)-(a.observations+2*a.confidence));m.landmarks.length=420;}m.confirmed=m.landmarks.filter(l=>l.observations>=3).length;addKeyframe(frame,camQ,now,currentPos);m.lastMapBuildAt=now;
}
function currentFeatureSet(frame,maxPoints=95){return selectCorners(frame,maxPoints).map(p=>({p:{x:p.x,y:p.y},descriptor:descriptorAt(frame,p.x,p.y)})).filter(x=>x.descriptor);}
function localMapMatches(frame,camQ,predPos){
  const m=state.map;if(m.landmarks.length<6)return[];const w=frame.w,h=frame.h,fx=.5*w/Math.tan(state.fovX*DEG/2),vfov=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vfov/2),features=currentFeatureSet(frame,105),used=new Set(),out=[];
  const lms=[...m.landmarks].sort((a,b)=>(b.lastSeen+b.observations*100)-(a.lastSeen+a.observations*100)).slice(0,260);
  for(const lm of lms){const pp=projectWorldPoint(lm.pos,camQ,predPos,w,h,fx,fy);if(!pp)continue;let best=null,second=Infinity;
    for(let i=0;i<features.length;i++){if(used.has(i))continue;const f=features[i],distPx=Math.hypot(f.p.x-pp.x,f.p.y-pp.y),searchRadius=m.confirmed>=6?20:34;if(distPx>searchRadius)continue;const dd=rmsDescriptorDistance(lm.descriptor,f.descriptor);if(!best||dd<best.dd){second=best?best.dd:second;best={i,f,dd};}else if(dd<second)second=dd;}
    if(best&&best.dd<0.68&&(second===Infinity||best.dd<second*0.86)){used.add(best.i);out.push({landmark:lm,pixel:best.f.p,descriptorDistance:best.dd});}
  }return out;
}
function globalMapMatches(frame){
  const m=state.map;if(m.landmarks.length<12)return[];const features=currentFeatureSet(frame,82),lms=m.landmarks.filter(l=>l.observations>=3).sort((a,b)=>(b.observations+b.confidence)-(a.observations+a.confidence)).slice(0,280),bestForLm=new Map(),featureBest=[];
  for(let fi=0;fi<features.length;fi++){let best=null,second=null;for(const lm of lms){const dd=rmsDescriptorDistance(features[fi].descriptor,lm.descriptor),old=bestForLm.get(lm.id);if(!old||dd<old.dd)bestForLm.set(lm.id,{fi,dd});if(!best||dd<best.dd){second=best;best={lm,dd};}else if(!second||dd<second.dd)second={lm,dd};}featureBest.push({fi,best,second});}
  const candidates=[];for(const x of featureBest){const {fi,best,second}=x;if(!best||best.dd>=0.56||(second&&best.dd>=second.dd*0.76))continue;const mutual=bestForLm.get(best.lm.id);if(!mutual||mutual.fi!==fi)continue;candidates.push({landmark:best.lm,pixel:features[fi].p,descriptorDistance:best.dd,fi});}
  candidates.sort((a,b)=>a.descriptorDistance-b.descriptorDistance);return candidates.slice(0,28);
}
function solveCameraCenter(matches,camQ,w,h,fx,fy){
  if(matches.length<2)return null;const A=[[0,0,0],[0,0,0],[0,0,0]],b=[0,0,0];
  for(const mt of matches){const d=rayWorldFromPixel(mt.pixel,camQ,w,h,fx,fy),P=mt.landmark.pos,xx=1-d.x*d.x,xy=-d.x*d.y,xz=-d.x*d.z,yy=1-d.y*d.y,yz=-d.y*d.z,zz=1-d.z*d.z;
    A[0][0]+=xx;A[0][1]+=xy;A[0][2]+=xz;A[1][0]+=xy;A[1][1]+=yy;A[1][2]+=yz;A[2][0]+=xz;A[2][1]+=yz;A[2][2]+=zz;
    b[0]+=xx*P.x+xy*P.y+xz*P.z;b[1]+=xy*P.x+yy*P.y+yz*P.z;b[2]+=xz*P.x+yz*P.y+zz*P.z;
  }return solve3x3(A,b);
}
function solveLinearSystem(A,b){
  const n=b.length,m=A.map((r,i)=>[...r,b[i]]);
  for(let c=0;c<n;c++){let r=c;for(let i=c+1;i<n;i++)if(Math.abs(m[i][c])>Math.abs(m[r][c]))r=i;if(Math.abs(m[r][c])<1e-10)return null;[m[c],m[r]]=[m[r],m[c]];const d=m[c][c];for(let j=c;j<=n;j++)m[c][j]/=d;for(let i=0;i<n;i++)if(i!==c){const f=m[i][c];if(Math.abs(f)<1e-14)continue;for(let j=c;j<=n;j++)m[i][j]-=f*m[c][j];}}
  return m.map(r=>r[n]);
}
function refineMapPose(matches,frame,startQ,startPos){
  if(matches.length<4||!startPos)return null;
  const w=frame.w,h=frame.h,fx=.5*w/Math.tan(state.fovX*DEG/2),vfov=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vfov/2),epsP=Math.max(1e-4,state.scale*0.00035),epsR=0.00045;
  let qe=qNorm(startQ),pos={...startPos};
  for(let iter=0;iter<5;iter++){
    const A=Array.from({length:6},()=>Array(6).fill(0)),b=Array(6).fill(0);let used=0,totalErr=0;
    for(const mt of matches){
      const base=projectWorldPoint(mt.landmark.pos,qe,pos,w,h,fx,fy);if(!base)continue;const rx=mt.pixel.x-base.x,ry=mt.pixel.y-base.y,e=Math.hypot(rx,ry);if(e>18)continue;const weight=e<=4?1:4/Math.max(e,1e-6),jx=[],jy=[];
      for(let k=0;k<6;k++){
        let qp=qe,ppos={...pos};if(k<3){if(k===0)ppos.x+=epsP;else if(k===1)ppos.y+=epsP;else ppos.z+=epsP;}else{const ax=k===3?1:0,ay=k===4?1:0,az=k===5?1:0;qp=qNorm(qMul(qAxis(ax,ay,az,epsR),qe));}
        const p2=projectWorldPoint(mt.landmark.pos,qp,ppos,w,h,fx,fy);if(!p2){jx.push(0);jy.push(0);continue;}const ep=k<3?epsP:epsR;jx.push((p2.x-base.x)/ep);jy.push((p2.y-base.y)/ep);
      }
      for(let i=0;i<6;i++){b[i]+=weight*(jx[i]*rx+jy[i]*ry);for(let j=0;j<6;j++)A[i][j]+=weight*(jx[i]*jx[j]+jy[i]*jy[j]);}used++;totalErr+=e;
    }
    if(used<4)break;for(let i=0;i<6;i++)A[i][i]+=i<3?1e-3:3e-3;const d=solveLinearSystem(A,b);if(!d)break;
    let dp={x:d[0],y:d[1],z:d[2]},pm=vecLength(dp),rm=Math.hypot(d[3],d[4],d[5]);const pLimit=Math.max(0.08,state.scale*0.08);if(pm>pLimit)dp=vScale(dp,pLimit/pm);pos=vAdd(pos,dp);
    if(rm>1e-8){const rLimit=0.055,rr=Math.min(rm,rLimit),dq=qAxis(d[3]/rm,d[4]/rm,d[5]/rm,rr);qe=qNorm(qMul(dq,qe));}
    if(pm<epsP*0.7&&rm<epsR*0.7)break;
  }
  return{q:qe,pos};
}
function mapPoseSolution(matches,frame,camQ){
  if(matches.length<4)return null;const w=frame.w,h=frame.h,fx=.5*w/Math.tan(state.fovX*DEG/2),vfov=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vfov/2),pool=matches.slice(0,20);let best=null,attempts=0;
  // A wider first gate tolerates a few degrees of IMU orientation drift. The alternating
  // 3D-bearing refinement below then lets the visual map correct both position AND attitude.
  for(let i=0;i<pool.length&&attempts<55;i++)for(let j=i+1;j<pool.length&&attempts<55;j++,attempts++){
    const pos=solveCameraCenter([pool[i],pool[j]],camQ,w,h,fx,fy);if(!pos)continue;const scored=[];for(const mt of matches){const pp=projectWorldPoint(mt.landmark.pos,camQ,pos,w,h,fx,fy);if(!pp)continue;const e=Math.hypot(pp.x-mt.pixel.x,pp.y-mt.pixel.y);if(e<11.0)scored.push({mt,e});}
    if(scored.length<4)continue;const err=median(scored.map(x=>x.e));if(!best||scored.length>best.inliers.length||(scored.length===best.inliers.length&&err<best.error))best={pos,inliers:scored.map(x=>x.mt),error:err};
  }
  if(!best)return null;const refined=refineMapPose(best.inliers,frame,camQ,best.pos);if(refined){best.pos=refined.pos;best.q=refined.q;}else best.q=camQ;
  let final=[];for(const mt of matches){const pp=projectWorldPoint(mt.landmark.pos,best.q,best.pos,w,h,fx,fy);if(!pp)continue;const e=Math.hypot(pp.x-mt.pixel.x,pp.y-mt.pixel.y);if(e<5.5)final.push({mt,e});}
  if(final.length<4)return null;const second=refineMapPose(final.map(x=>x.mt),frame,best.q,best.pos);if(second){best.pos=second.pos;best.q=second.q;}
  final=[];for(const mt of matches){const pp=projectWorldPoint(mt.landmark.pos,best.q,best.pos,w,h,fx,fy);if(!pp)continue;const e=Math.hypot(pp.x-mt.pixel.x,pp.y-mt.pixel.y);if(e<4.5)final.push({mt,e});}
  if(final.length<4)return null;best.inliers=final.map(x=>x.mt);best.error=median(final.map(x=>x.e));best.confidence=clamp(final.length/14,0,1)*clamp(1-best.error/5,0,1);return best;
}
function qualifyMapMatches(matches,now){
  // Observation count is a KEYFRAME qualification count, not a frame counter.
  // Two-view landmarks may bootstrap local PnP; global relocalization remains confirmed-only.
  for(const mt of matches){mt.landmark.lastSeen=now;mt.landmark.confidence=clamp(mt.landmark.confidence+0.004,0,1);}
  const confirmed=state.map.landmarks.filter(l=>l.observations>=3).length;state.map.confirmed=confirmed;
  return matches.filter(mt=>mt.landmark.observations>=(confirmed>=6?3:2));
}
function refineMatchedLandmarks(sol,frame,camQ){
  if(!sol||!sol.inliers)return;const w=frame.w,h=frame.h,fx=.5*w/Math.tan(state.fovX*DEG/2),vfov=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vfov/2);
  for(const mt of sol.inliers){const lm=mt.landmark,a=lm.anchor;if(!a||lm.observations<3)continue;const d1=rayWorldFromPixel(a.pixel,a.q,a.w,a.h,a.fx,a.fy),d2=rayWorldFromPixel(mt.pixel,camQ,w,h,fx,fy),tri=triangulateRays(a.pos,d1,sol.pos,d2);if(!tri||tri.angle<1.1*DEG)continue;const baseline=posDist(a.pos,sol.pos);if(tri.gap>Math.max(0.025,baseline*0.12))continue;const gain=clamp(0.16/Math.sqrt(lm.observations),0.025,0.08);lm.pos=vLerp(lm.pos,tri.p,gain);}
}
function poseConsensus(candidates,posTolerance,rotTolerance){
  if(candidates.length<3)return null;for(let i=0;i<candidates.length;i++){const cluster=candidates.filter(c=>posDist(c.pos,candidates[i].pos)<=posTolerance&&qAngle(c.q,candidates[i].q)<=rotTolerance);if(cluster.length>=3){const pos=cluster.reduce((a,c)=>vAdd(a,c.pos),{x:0,y:0,z:0}),qAvg=qAverage(cluster.map(c=>c.q));return{pos:vScale(pos,1/cluster.length),q:qAvg,count:cluster.length};}}return null;
}
function applyMapPose(sol,global,now,frame,camQ){
  if(!sol||sol.inliers.length<4||sol.error>4.5)return false;const m=state.map,solQ=sol.q||camQ,correction=posDist(state.position,sol.pos),orientDelta=qNorm(qMul(solQ,qInv(camQ))),orientError=qAngle(q(),orientDelta);
  if(orientError>14*DEG)return false;
  const prevSolved=m.lastSolvedPos?{...m.lastSolvedPos}:null,prevAt=m.lastAuthoritativeAt||0,target={...sol.pos};
  // PnP is the authority. Ordinary corrections are bounded for display continuity. A confirmed
  // global loop closure is applied exactly, but the lattice is suppressed for one correction frame
  // so the mapper is never forced to triangulate from a deliberately lagged/fake pose.
  if(global){state.position={...target};m.suppressRenderUntil=now+110;state.velocity=vScale(state.velocity,0.18);}else{let dp=vSub(target,state.position),pm=vecLength(dp),maxStep=state.scaleLocked?0.055:0.09;if(pm>maxStep)dp=vScale(dp,maxStep/pm);state.position=vAdd(state.position,dp);}
  if(orientError>0.00015){const maxRot=(global?2.0:1.2)*DEG,corrQ=qSlerp(q(),orientDelta,Math.min(1,maxRot/Math.max(orientError,1e-9)));state.orientationCorrection=qNorm(qMul(corrQ,state.orientationCorrection));}
  if(!global&&prevSolved&&prevAt&&now>prevAt&&correction<(state.scaleLocked?0.22:0.35)){const measured=vScale(vSub(target,prevSolved),1/Math.max((now-prevAt)/1000,1e-3)),speed=vecLength(measured),bounded=speed>5?vScale(measured,5/speed):measured;state.velocity=vLerp(state.velocity,bounded,0.55);}m.lastSolvedPos=target;m.lastAuthoritativeAt=now;m.lastAuthoritativePos=target;m.bridgeAge=0;
  m.lastCorrection=correction;m.poseInliers=sol.inliers.length;m.reprojectionError=sol.error;m.confidence=lerp(m.confidence,sol.confidence,0.35);m.relocalizations++;const loopThreshold=state.scaleLocked?0.12:0.18;if(global&&(correction>loopThreshold||orientError>1.5*DEG))m.loopClosures++;
  for(const mt of sol.inliers){mt.landmark.lastSeen=now;mt.landmark.confidence=clamp(mt.landmark.confidence+0.02,0,1);}refineMatchedLandmarks(sol,frame,solQ);m.confirmed=m.landmarks.filter(l=>l.observations>=3).length;
  state.poseReason=`Map-authoritative pose: ${sol.inliers.length} landmark inliers, ${sol.error.toFixed(1)} px${global?' • loop closure confirmed':''}`;return true;
}
function relocalizeAgainstMap(frame,camQ,now){
  const m=state.map;if(!m.gaugeReady||m.landmarks.length<6||now-(m.lastPoseAt||0)<75)return false;m.lastPoseAt=now;
  let matches=qualifyMapMatches(localMapMatches(frame,camQ,state.position),now),sol=mapPoseSolution(matches,frame,camQ),ok=applyMapPose(sol,false,now,frame,camQ);
  const shouldGlobal=(!ok||m.confidence<0.25||now-m.lastRelocalizeAt>1500)&&now-m.lastRelocalizeAt>420;
  if(shouldGlobal){m.lastRelocalizeAt=now;matches=globalMapMatches(frame);sol=mapPoseSolution(matches,frame,camQ);if(sol&&sol.inliers.length>=5&&sol.error<4.2){m.relocalizationCandidates.push({t:now,pos:{...sol.pos},q:{...(sol.q||camQ)},sol});m.relocalizationCandidates=m.relocalizationCandidates.filter(c=>now-c.t<1100).slice(-6);const consensus=poseConsensus(m.relocalizationCandidates,state.scaleLocked?0.18:0.24,7*DEG);if(consensus){const accepted={...sol,pos:consensus.pos,q:consensus.q};if(applyMapPose(accepted,true,now,frame,camQ)){ok=true;m.relocalizationCandidates=[];}}}}
  if(!ok){m.poseInliers=0;m.confidence*=0.96;m.reprojectionError=Infinity;}return ok;
}
function learnQuietBiasAndGravity(){
  if(!state.originCaptured||!state.stationary||Math.max(vecLength(state.gyro),state.orientationRate)>0.04)return;
  if(state.hasLinearAcceleration){const a=state.accelDevice;state.accelBiasDevice={x:lerp(state.accelBiasDevice.x,a.x,0.025),y:lerp(state.accelBiasDevice.y,a.y,0.025),z:lerp(state.accelBiasDevice.z,a.z,0.025)};}
  if(state.hasGravityAcceleration){const g=qRotate(relativeMotionQ(),state.accelGravityDevice);state.gravityWorld={x:lerp(state.gravityWorld.x,g.x,0.012),y:lerp(state.gravityWorld.y,g.y,0.012),z:lerp(state.gravityWorld.z,g.z,0.012)};}
}
function updatePose(rawVisualDelta,dt,now) {
  const a=correctedAcceleration(),gyroMag=vecLength(state.gyro),accMag=vecLength(a),visualSpeed=(rawVisualDelta.rawMagnitude||0)/Math.max(dt,1e-3),motionFresh=now-state.lastMotionAt<350,m=state.map;
  state.visualStepMagnitude=lerp(state.visualStepMagnitude,rawVisualDelta.rawMagnitude||0,0.28);if(rawVisualDelta.confidence>0.06)m.sinceKeyframeVisual+=rawVisualDelta.rawMagnitude||0;
  const mapSpeed=vecLength(state.velocity),deliberateMap=m.gaugeReady&&mapSpeed>(state.scaleLocked?0.035:0.045),deliberateVisual=rawVisualDelta.confidence>0.10&&visualSpeed>0.022,deliberateInertial=motionFresh&&accMag>0.24,deliberateAngular=Math.max(gyroMag,state.orientationRate)>0.055,deliberate=deliberateMap||deliberateVisual||deliberateInertial||deliberateAngular;
  if(deliberate){state.stationaryScore=Math.min(state.stationaryScore,0.12);state.stationary=false;state.stillSince=0;state.lastMoveAt=now;state.movementGate='MOVING';}
  else{const vs=state.validTracks>=6?1-clamp((visualSpeed-0.008)/0.10,0,1):0.56,rs=1-clamp((Math.max(gyroMag,state.orientationRate)-0.018)/0.24,0,1),as=1-clamp((accMag-0.05)/0.70,0,1),qq=(motionFresh||now-state.lastOrientationAt<350)?0.50*vs+0.29*rs+0.21*as:0;state.stationaryScore=clamp(state.stationaryScore+dt*(qq>0.58?qq*2.0:-1.8),0,1);state.stationary=state.stationaryScore>0.62;state.movementGate=state.stationary?'STILL':'FREE';}
  if(state.stationary){if(!state.stillSince)state.stillSince=now;state.velocity={x:0,y:0,z:0};}else state.stillSince=0;learnQuietBiasAndGravity();
  // XYZ is never integrated from optical-flow magnitude. After map bootstrap, only a short
  // bounded prediction bridges camera gaps; persistent landmarks/PnP remain long-term authority.
  if(m.gaugeReady&&!state.stationary){const age=now-(m.lastAuthoritativeAt||0);m.bridgeAge=age;if(age<=320){let d=vScale(state.velocity,dt);if(state.scaleLocked&&motionFresh){d=vAdd(d,vScale(a,0.5*dt*dt));state.velocity=vAdd(state.velocity,vScale(a,dt));}const maxStep=state.scaleLocked?0.09:0.14,dm=vecLength(d);if(dm>maxStep)d=vScale(d,maxStep/dm);state.position=vAdd(state.position,d);state.movementGate='MAP BRIDGE';}else{state.velocity=vScale(state.velocity,0.72);state.movementGate='RELOCALIZING';}}
  const speed=vecLength(state.velocity);if(speed>5)state.velocity=vScale(state.velocity,5/speed);const sourceAgreement=clamp(1-Math.abs(visualSpeed-accMag*0.10)/(visualSpeed+0.30),0,1);state.motionConfidence=clamp(0.48*state.visualConfidence+0.15*(motionFresh?1:0.55)+0.12*sourceAgreement+0.25*m.confidence,0,1);
  const p=state.position,lp=state.lastPositionForDrift;if(state.stationary){const drift=Math.hypot(p.x-lp.x,p.y-lp.y,p.z-lp.z)/Math.max(dt,1e-3);state.driftRate=lerp(state.driftRate,drift,0.10);}else state.driftRate*=0.96;state.lastPositionForDrift={...p};
}

function processVideoFrame(now,meta={}) {
  if(now-state.lastProcessAt<42)return;const dt=clamp((now-(state.lastProcessAt||now))/1000,0.01,0.12);state.lastProcessAt=now;
  const frame=captureGray();if(!frame)return;const frameTime=Number.isFinite(meta.presentationTime)?meta.presentationTime:(Number.isFinite(meta.expectedDisplayTime)?meta.expectedDisplayTime:now);frame.time=frameTime;frame.imuTime=frameTime-state.videoImuLagMs;
  if(!state.originCaptured)initializeWorldFromCurrentOrientation();
  if(state.previousFrame&&state.previousFrameTime){
    const raw=qualityTracks(state.previousFrame,frame),rawFlow=raw.length?median(raw.map(t=>Math.hypot(t.observed.x,t.observed.y))):0;state.flowMagnitude=lerp(state.flowMagnitude,rawFlow,0.25);
    // Timing/intrinsics self-refine only before the map gauge is committed. Once landmarks
    // define the world, changing camera geometry underneath them would itself create drift.
    if(!state.map.gaugeReady)estimateVideoImuTiming(raw,state.previousFrameTime,frameTime,frame.w,frame.h);
    const imuT0=state.previousFrameTime-state.videoImuLagMs,imuT1=frameTime-state.videoImuLagMs,prevQ=nearestOrientation(imuT0),frameQ=nearestOrientation(imuT1);state.frameAccelWorld=averageAcceleration(imuT0,imuT1);if(!state.map.gaugeReady)estimateFovFromTracks(raw,prevQ,frameQ,frame.w,frame.h);
    const solution=residualSolution(raw,prevQ,frameQ,frame.w,frame.h,state.fovX);state.tracks=solution.inliers;state.validTracks=solution.inliers.length;state.visualConfidence=lerp(state.visualConfidence,solution.confidence,0.26);
    const vv=estimateTranslation(solution,frame.w,frame.h,dt,frameQ,prevQ);state.translationSignal=vv;updatePose(vv,dt,now);
    const mapCorrected=relocalizeAgainstMap(frame,relativeQ(frameQ),now);buildPersistentMap(frame,relativeQ(frameQ),now);
    if(!mapCorrected&&state.visualConfidence>0.08)state.poseReason=`Visual-inertial tracking • building persistent map (${state.map.landmarks.length} landmarks)`;
    state.previousFrameQ=frameQ;
  }else if(state.originCaptured){addKeyframe(frame,relativeCameraQ(),now);}
  state.previousFrame=frame;state.previousFrameTime=frameTime;state.frame=frame;state.processCount++;
  if(now-state.processStamp>=1000){state.processedFps=state.processCount*1000/(now-state.processStamp);state.processCount=0;state.processStamp=now;}
}

function maintainWorldOrientationLock() {
  if(!state.originCaptured||!state.baseQ)return;const raw=rawRelativeQ(state.orientationQ),corrected=qNorm(qMul(state.orientationCorrection,raw)),angular=Math.max(vecLength(state.gyro),state.orientationRate);
  if(state.stationary&&state.stillSince&&performance.now()-state.stillSince>900&&angular<0.007){if(!state.orientationHoldQ)state.orientationHoldQ=corrected;else state.orientationCorrection=qNorm(qMul(state.orientationHoldQ,qInv(raw)));}else state.orientationHoldQ=null;
}
function worldMetricAuthoritative(now=performance.now()){
  const m=state.map;return !!(state.scaleLocked&&m.gaugeReady&&m.confirmed>=6&&m.confidence>0.26&&(m.poseInliers>=5||now-m.lastAuthoritativeAt<420));
}
function trackingMachine(now){
  maintainWorldOrientationLock();if(state.stage!=='tracking')return;if(!state.originCaptured)initializeWorldFromCurrentOrientation();saveAutomaticProfile(now);const m=state.map,locked=worldMetricAuthoritative(now);
  const profileQuality=0.16*state.visualConfidence+0.14*state.motionConfidence+0.30*state.scaleStability+0.40*m.confidence,quality=clamp(profileQuality,0,1);ui.progress.style.width=`${Math.round(quality*100)}%`;
  if(!state.originCaptured){ui.status.textContent='SENSORS STARTING';ui.status.dataset.state='calibrating';ui.stepTimer.textContent='AUTO';ui.stepDetail.textContent='Waiting for the first orientation sample; no action is required.';state.poseReason='Waiting for first orientation sample';state.gridMode='self-initializing';return;}
  if(locked){ui.status.textContent='WORLD LOCKED';ui.status.dataset.state='locked';state.gridMode='metric world';}
  else if(m.gaugeReady&&now-m.lastAuthoritativeAt>420){ui.status.textContent='RELOCALIZING';ui.status.dataset.state='lost';state.gridMode=state.scaleLocked?'world held':'self-initializing';}
  else{ui.status.textContent='AUTO INITIALIZING';ui.status.dataset.state='calibrating';state.gridMode='self-initializing';}
  ui.stepLabel.textContent='ZERO-SETUP WORLD LOCK';ui.stepTimer.textContent='LIVE';ui.stepDetail.textContent=`${m.landmarks.length} landmarks • ${m.confirmed} confirmed • ${m.poseInliers} PnP inliers • metric ${Math.round(state.scaleStability*100)}% • loops ${m.loopClosures}`;
  ui.instruction.textContent='Use the phone normally. World mapping, metric scale, drift repair, and relocalization happen automatically; no calibration motion is required.';
}

function cameraPoint(world,camQ,camPos) {
  const rel={x:world.x-camPos.x,y:world.y-camPos.y,z:world.z-camPos.z};
  return qRotate(qInv(camQ),rel);
}
function projectCamera(c,fx,fy,cx,cy) { return {x:cx+fx*(c.x/-c.z),y:cy-fy*(c.y/-c.z),z:-c.z}; }
function drawWorldSegment(a,b,camQ,camPos,fx,fy,cx,cy,stroke,width=1,near=0.08) {
  let ca=cameraPoint(a,camQ,camPos),cb=cameraPoint(b,camQ,camPos);
  const aVisible=ca.z<-near,bVisible=cb.z<-near;
  if(!aVisible&&!bVisible)return;
  if(aVisible!==bVisible){
    const t=(-near-ca.z)/(cb.z-ca.z);
    const clip={x:ca.x+(cb.x-ca.x)*t,y:ca.y+(cb.y-ca.y)*t,z:-near};
    if(!aVisible)ca=clip;else cb=clip;
  }
  const pa=projectCamera(ca,fx,fy,cx,cy),pb=projectCamera(cb,fx,fy,cx,cy),W=cx*2,H=cy*2;
  if((pa.x<-900&&pb.x<-900)||(pa.x>W+900&&pb.x>W+900)||(pa.y<-900&&pb.y<-900)||(pa.y>H+900&&pb.y>H+900))return;
  gctx.strokeStyle=stroke;gctx.lineWidth=width;gctx.beginPath();gctx.moveTo(pa.x,pa.y);gctx.lineTo(pb.x,pb.y);gctx.stroke();
}
function drawOriginReticle(W,H) {
  gctx.strokeStyle='rgba(255,255,255,.42)';gctx.lineWidth=1;gctx.beginPath();gctx.arc(W/2,H/2,Math.min(W,H)*.18,0,Math.PI*2);gctx.stroke();
  const p=clamp(state.stillScore,0,1);gctx.strokeStyle='rgba(104,247,171,.95)';gctx.lineWidth=4;gctx.beginPath();gctx.arc(W/2,H/2,Math.min(W,H)*.18,-Math.PI/2,-Math.PI/2+Math.PI*2*p);gctx.stroke();
}
function drawProjectionTunnel(camQ,camPos,fx,fy,cx,cy) {
  for(const z of [-3,-5,-8,-12]){
    const s=Math.abs(z)*0.42,alpha=z===-3?.85:.42;
    const c=[{x:-s,y:-s*.7,z},{x:s,y:-s*.7,z},{x:s,y:s*.7,z},{x:-s,y:s*.7,z}];
    for(let i=0;i<4;i++)drawWorldSegment(c[i],c[(i+1)%4],camQ,camPos,fx,fy,cx,cy,`rgba(255,211,107,${alpha})`,z===-3?2:1);
  }
  for(const sx of [-1,1])for(const sy of [-1,1])drawWorldSegment({x:sx*1.26,y:sy*.88,z:-3},{x:sx*5.04,y:sy*3.53,z:-12},camQ,camPos,fx,fy,cx,cy,'rgba(255,211,107,.35)',1);
}
function drawMarkerCube(center,size,camQ,camPos,fx,fy,cx,cy,stroke){
  const s=size/2,pts=[];
  for(const dx of [-s,s])for(const dy of [-s,s])for(const dz of [-s,s])pts.push({x:center.x+dx,y:center.y+dy,z:center.z+dz});
  const idx=(ix,iy,iz)=>(ix*4+iy*2+iz);
  for(let iy=0;iy<2;iy++)for(let iz=0;iz<2;iz++)drawWorldSegment(pts[idx(0,iy,iz)],pts[idx(1,iy,iz)],camQ,camPos,fx,fy,cx,cy,stroke,2);
  for(let ix=0;ix<2;ix++)for(let iz=0;iz<2;iz++)drawWorldSegment(pts[idx(ix,0,iz)],pts[idx(ix,1,iz)],camQ,camPos,fx,fy,cx,cy,stroke,2);
  for(let ix=0;ix<2;ix++)for(let iy=0;iy<2;iy++)drawWorldSegment(pts[idx(ix,iy,0)],pts[idx(ix,iy,1)],camQ,camPos,fx,fy,cx,cy,stroke,2);
}
function drawLattice(camQ,camPos,fx,fy,cx,cy,full) {
  // True world-centered 3D lattice.  The startup camera is INSIDE it, so
  // turning 90°, 180° or 360° still reveals fixed virtual structure.
  const lim=full?12:8, yLim=full?8:6, step=full?2:3;
  const minor='rgba(91,226,255,.18)',major='rgba(91,226,255,.34)';
  for(let x=-lim;x<=lim;x+=step)for(let y=-yLim;y<=yLim;y+=step){const c=(x===0||y===0)?major:minor;drawWorldSegment({x,y,z:-lim},{x,y,z:lim},camQ,camPos,fx,fy,cx,cy,c,(x===0||y===0)?1.15:.72);}
  for(let x=-lim;x<=lim;x+=step)for(let z=-lim;z<=lim;z+=step){const c=(x===0||z===0)?major:minor;drawWorldSegment({x,y:-yLim,z},{x,y:yLim,z},camQ,camPos,fx,fy,cx,cy,c,(x===0||z===0)?1.15:.72);}
  for(let y=-yLim;y<=yLim;y+=step)for(let z=-lim;z<=lim;z+=step){const c=(y===0||z===0)?major:minor;drawWorldSegment({x:-lim,y,z},{x:lim,y,z},camQ,camPos,fx,fy,cx,cy,c,(y===0||z===0)?1.15:.72);}

  // Six fixed direction beacons make 180°/360° tracking visually undeniable.
  drawMarkerCube({x:5,y:0,z:0},0.7,camQ,camPos,fx,fy,cx,cy,'rgba(255,90,100,.95)');
  drawMarkerCube({x:-5,y:0,z:0},0.7,camQ,camPos,fx,fy,cx,cy,'rgba(170,55,65,.92)');
  drawMarkerCube({x:0,y:5,z:0},0.7,camQ,camPos,fx,fy,cx,cy,'rgba(90,255,145,.95)');
  drawMarkerCube({x:0,y:-5,z:0},0.7,camQ,camPos,fx,fy,cx,cy,'rgba(55,165,95,.92)');
  drawMarkerCube({x:0,y:0,z:-5},0.7,camQ,camPos,fx,fy,cx,cy,'rgba(90,145,255,.98)');
  drawMarkerCube({x:0,y:0,z:5},0.7,camQ,camPos,fx,fy,cx,cy,'rgba(170,105,255,.96)');
}

function drawGrid() {
  const dpr=Math.min(devicePixelRatio||1,2),rect=grid.getBoundingClientRect();if(grid.width!==Math.round(rect.width*dpr)||grid.height!==Math.round(rect.height*dpr))resize();
  const W=rect.width,H=rect.height;gctx.setTransform(dpr,0,0,dpr,0,0);gctx.clearRect(0,0,W,H);const fx=.5*W/Math.tan(state.fovX*DEG/2),fy=.5*H/Math.tan(state.fovY*DEG/2),cx=W/2,cy=H/2,camQ=relativeCameraQ(),camPos=state.position;
  if(state.stage==='tracking'&&state.originCaptured&&worldMetricAuthoritative()&&performance.now()>=state.map.suppressRenderUntil)drawLattice(camQ,camPos,fx,fy,cx,cy,true);else drawOriginReticle(W,H);
  gctx.fillStyle='rgba(255,255,255,.78)';gctx.font='11px system-ui';gctx.fillText(`HFOV ${state.fovX.toFixed(1)}° • landmarks ${state.map.landmarks.length} • map inliers ${state.map.poseInliers}`,12,H-14);
}

function updateUI() {
  const metricReady=state.scaleLocked&&state.map.gaugeReady;ui.x.textContent=metricReady?state.position.x.toFixed(3):'—';ui.y.textContent=metricReady?state.position.y.toFixed(3):'—';ui.z.textContent=metricReady?state.position.z.toFixed(3):'—';ui.gridMode.textContent=state.gridMode.toUpperCase();
  ui.dState.textContent=state.stage;ui.dFov.textContent=`${state.fovX.toFixed(1)}° × ${state.fovY.toFixed(1)}°`;ui.dVisual.textContent=`${Math.round(state.visualConfidence*100)}%`;ui.dMotion.textContent=`${Math.round(state.motionConfidence*100)}%`;
  ui.dStill.textContent=state.stationary?`yes ${Math.round(state.stationaryScore*100)}%`:`no ${Math.round(state.stationaryScore*100)}%`;ui.dFps.textContent=state.processedFps.toFixed(1);ui.dScale.textContent=state.scaleLocked?`${state.scale.toFixed(3)} m/u • LOCKED`:`AUTO • ${Math.round(state.scaleStability*100)}%`;
  ui.dQuality.textContent=worldMetricAuthoritative()?'metric map locked':state.map.gaugeReady?'visual map initializing':state.visualConfidence>.12?'visual bootstrap':'weak';ui.dTracks.textContent=String(state.validTracks);ui.dDrift.textContent=`${state.driftRate.toFixed(4)} m/s`;
  ui.dImuHz.textContent=state.imuHz.toFixed(1);ui.dVideoHz.textContent=state.videoHz.toFixed(1);ui.dReason.textContent=state.poseReason;ui.dProjectionError.textContent=Number.isFinite(state.map.reprojectionError)?`${state.map.reprojectionError.toFixed(2)} px map`:Number.isFinite(state.projectionError)?`${state.projectionError.toFixed(2)} px rotation`:'—';
  ui.dOriginQuality.textContent=state.originCaptured?'automatic':'waiting';ui.dGridMode.textContent=state.gridMode;if(ui.dVisualStep)ui.dVisualStep.textContent=state.visualStepMagnitude.toFixed(5);if(ui.dMoveGate)ui.dMoveGate.textContent=state.movementGate;
  if(ui.dTiming)ui.dTiming.textContent=`${state.videoImuLagMs.toFixed(0)} ms (${Math.round(state.timingConfidence*100)}%)`;if(ui.dWorldBasis)ui.dWorldBasis.textContent=`gravity + start yaw • ${state.map.landmarks.length} landmarks • ${state.map.loopClosures} loop closures`;if(ui.dDirection)ui.dDirection.textContent=`${Math.round(state.translationDirectionConfidence*100)}%`;
}

const STRESS_TESTS = [
  {id:'stationary',name:'Stationary creep',instruction:'Hold the phone normally still for 4 seconds.',kind:'stationary'},
  {id:'yaw360',name:'360° rotation',instruction:'Turn slowly through one complete circle and finish facing your starting direction.',kind:'yaw'},
  {id:'xReturn',name:'X out-and-back',instruction:'Face the startup direction. Step or lean sideways, return to your starting spot, then stop.',kind:'axis',axis:'x',threshold:0.10},
  {id:'zReturn',name:'Z out-and-back',instruction:'Face the startup direction. Move forward/back, return to your starting spot, then stop.',kind:'axis',axis:'z',threshold:0.10},
  {id:'yReturn',name:'Y up-and-down',instruction:'Raise/lower the phone clearly, return to the starting height, then hold still.',kind:'axis',axis:'y',threshold:0.07},
  {id:'mixedLoop',name:'Mixed motion loop',instruction:'Walk or lean through a small loop while turning, return near the start, then stop.',kind:'mixed'}
];
function clonePos(p){return{x:p.x,y:p.y,z:p.z};}
function posDelta(a,b){return{x:a.x-b.x,y:a.y-b.y,z:a.z-b.z};}
function posDist(a,b){const d=posDelta(a,b);return Math.hypot(d.x,d.y,d.z);}
function wrapPi(a){while(a>Math.PI)a-=Math.PI*2;while(a<-Math.PI)a+=Math.PI*2;return a;}
function cameraYaw(camQ){const f=qRotate(camQ,{x:0,y:0,z:-1});return Math.atan2(f.x,-f.z);}
function resetStress(){
  state.stress={active:false,complete:false,index:-1,startedAt:0,stageStartedAt:0,testStartPos:null,testStartQ:null,lastPos:null,lastYaw:0,yawTravel:0,pathLength:0,maxDisplacement:0,maxAxis:0,maxOffAxis:0,maxDriftRate:0,stableSince:0,movementSeen:false,results:[],overall:'not run',manual:false};
  if(ui.stress){ui.stress.textContent='Stress Test';ui.stress.disabled=state.stage!=='tracking';}
}
function startStressTest(){
  if(state.stage!=='tracking')return;
  resetStress();state.stress.active=true;state.stress.startedAt=performance.now();state.stress.index=0;if(ui.save)ui.save.disabled=true;ui.stress.textContent='Next Test';enterStressTest(0);
}
function enterStressTest(index){
  const s=state.stress,t=STRESS_TESTS[index],now=performance.now();
  s.index=index;s.stageStartedAt=now;s.testStartPos=clonePos(state.position);s.testStartQ=relativeCameraQ();s.lastPos=clonePos(state.position);s.lastYaw=cameraYaw(s.testStartQ);s.yawTravel=0;s.pathLength=0;s.maxDisplacement=0;s.maxAxis=0;s.maxOffAxis=0;s.maxDriftRate=0;s.stableSince=0;s.movementSeen=false;s.manual=false;
  ui.stepLabel.textContent=`STRESS ${index+1} OF ${STRESS_TESTS.length}`;ui.stepDetail.textContent=t.instruction;ui.instruction.textContent=t.instruction;ui.stepTimer.textContent='0%';ui.status.textContent='STRESS TEST';ui.status.dataset.state='calibrating';state.poseReason=`Stress test: ${t.name}`;
}
function stressCommonSample(){
  const s=state.stress,p=state.position,d=posDelta(p,s.testStartPos),dist=Math.hypot(d.x,d.y,d.z);s.pathLength+=posDist(p,s.lastPos);s.lastPos=clonePos(p);s.maxDisplacement=Math.max(s.maxDisplacement,dist);s.maxDriftRate=Math.max(s.maxDriftRate,state.driftRate);return{d,dist};
}
function finishStressResult(pass,metrics,note,manual=false){
  const s=state.stress,t=STRESS_TESTS[s.index];s.results.push({id:t.id,name:t.name,pass:!!pass,manual:!!manual,metrics,note});
  const next=s.index+1;if(next<STRESS_TESTS.length){enterStressTest(next);}else finishStressTest();
}
function finishStressTest(){
  const s=state.stress;s.active=false;s.complete=true;s.overall=s.results.every(r=>r.pass&&!r.manual)?'PASS':(s.results.some(r=>r.pass)?'MIXED':'FAIL');
  ui.stress.textContent='View Stress';if(ui.save)ui.save.disabled=false;ui.status.textContent=s.overall==='PASS'?'STRESS PASS':'STRESS REVIEW';ui.status.dataset.state=s.overall==='PASS'?'locked':'calibrating';ui.stepLabel.textContent='STRESS COMPLETE';ui.stepTimer.textContent=s.overall;ui.stepDetail.textContent=`${s.results.filter(r=>r.pass).length}/${s.results.length} tests passed.`;ui.instruction.textContent='Stress qualification complete. The tracker continues running and refining its map automatically.';state.poseReason=`Stress ${s.overall}: ${s.results.filter(r=>r.pass).length}/${s.results.length} passed`;showStressResults();
}
function manualAdvanceStress(){
  if(!state.stress.active)return;const s=state.stress,t=STRESS_TESTS[s.index];const {d,dist}=stressCommonSample();
  let metrics={manualAdvance:true,closure:dist,maxDisplacement:s.maxDisplacement,pathLength:s.pathLength};
  if(t.kind==='axis'){const axis=Math.abs(d[t.axis]);const off=Math.hypot(...['x','y','z'].filter(a=>a!==t.axis).map(a=>d[a]));metrics={...metrics,maxAxis:s.maxAxis,maxOffAxis:s.maxOffAxis,axisPurity:s.maxAxis?s.maxOffAxis/s.maxAxis:Infinity};}
  finishStressResult(false,metrics,'Advanced manually before automatic completion.',true);
}
function updateStress(now){
  const s=state.stress;if(!s.active||state.stage!=='tracking')return;const t=STRESS_TESTS[s.index],elapsed=(now-s.stageStartedAt)/1000,{d,dist}=stressCommonSample();
  if(t.kind==='stationary'){
    const pct=clamp(elapsed/4,0,1);ui.stepTimer.textContent=`${Math.round(pct*100)}%`;ui.stepDetail.textContent=`Hold still • max displacement ${s.maxDisplacement.toFixed(4)} u • drift ${s.maxDriftRate.toFixed(4)} u/s`;
    if(elapsed>=4){const pass=s.maxDisplacement<0.035&&s.maxDriftRate<0.025;finishStressResult(pass,{duration:elapsed,maxDisplacement:s.maxDisplacement,maxDriftRate:s.maxDriftRate,finalDisplacement:dist},pass?'No significant stationary creep detected.':'Stationary pose moved beyond the qualification threshold.');}
    return;
  }
  if(t.kind==='yaw'){
    const yaw=cameraYaw(relativeCameraQ()),dy=Math.abs(wrapPi(yaw-s.lastYaw));s.lastYaw=yaw;if(dy<0.45)s.yawTravel+=dy;const deg=s.yawTravel/DEG,closureDeg=qAngle(s.testStartQ,relativeCameraQ())/DEG;ui.stepTimer.textContent=`${Math.min(360,Math.round(deg))}°`;ui.stepDetail.textContent=`Complete 360° • position leak ${s.maxDisplacement.toFixed(3)} u • heading closure ${closureDeg.toFixed(1)}°`;
    if(deg>=330&&closureDeg<18&&state.orientationRate<0.18){const pass=closureDeg<12&&s.maxDisplacement<0.12;finishStressResult(pass,{yawTravelDeg:deg,orientationClosureDeg:closureDeg,maxPositionLeak:s.maxDisplacement,finalPositionLeak:dist},pass?'Rotation stayed separated from XYZ and closed near its starting attitude.':'360° rotation produced excessive orientation closure error or false translation.');}else if(elapsed>28&&deg>250){const pass=closureDeg<15&&s.maxDisplacement<0.12;finishStressResult(pass,{yawTravelDeg:deg,orientationClosureDeg:closureDeg,maxPositionLeak:s.maxDisplacement,finalPositionLeak:dist},'Timed completion after a near-full rotation.');}
    return;
  }
  if(t.kind==='axis'){
    const axis=Math.abs(d[t.axis]),off=Math.hypot(...['x','y','z'].filter(a=>a!==t.axis).map(a=>d[a]));s.maxAxis=Math.max(s.maxAxis,axis);s.maxOffAxis=Math.max(s.maxOffAxis,off);if(s.maxAxis>t.threshold)s.movementSeen=true;const closure=dist,returnThreshold=Math.max(0.055,s.maxAxis*0.30);if(s.movementSeen&&closure<returnThreshold&&state.stationary){if(!s.stableSince)s.stableSince=now;}else s.stableSince=0;const purity=s.maxAxis? s.maxOffAxis/s.maxAxis:Infinity;ui.stepTimer.textContent=`${Math.round(clamp(s.maxAxis/t.threshold,0,1)*100)}%`;ui.stepDetail.textContent=`${t.axis.toUpperCase()} excursion ${s.maxAxis.toFixed(3)} u • off-axis ratio ${Number.isFinite(purity)?purity.toFixed(2):'—'} • closure ${closure.toFixed(3)} u`;
    if(s.stableSince&&now-s.stableSince>700){const closureRatio=s.maxAxis?closure/s.maxAxis:Infinity,pass=s.maxAxis>t.threshold&&purity<0.65&&closureRatio<0.35;finishStressResult(pass,{axis:t.axis,maxAxis:s.maxAxis,maxOffAxis:s.maxOffAxis,axisPurity:purity,closure,closureRatio,pathLength:s.pathLength},pass?'Dominant motion stayed on the expected world axis and returned near the start.':'Axis cross-talk or return closure exceeded the qualification threshold.');}
    return;
  }
  if(t.kind==='mixed'){
    if(s.maxDisplacement>0.15&&s.pathLength>0.45)s.movementSeen=true;const closure=dist,closureRatio=s.maxDisplacement?closure/s.maxDisplacement:Infinity,rotClosure=qAngle(s.testStartQ,relativeCameraQ())/DEG;if(s.movementSeen&&closure<Math.max(0.08,s.maxDisplacement*0.35)&&state.stationary){if(!s.stableSince)s.stableSince=now;}else s.stableSince=0;ui.stepTimer.textContent=`${s.movementSeen?'RETURN':'MOVE'}`;ui.stepDetail.textContent=`Path ${s.pathLength.toFixed(2)} u • excursion ${s.maxDisplacement.toFixed(2)} u • closure ${closure.toFixed(3)} u`;
    if(s.stableSince&&now-s.stableSince>800){const pass=closureRatio<0.35&&s.pathLength>0.45;finishStressResult(pass,{pathLength:s.pathLength,maxDisplacement:s.maxDisplacement,closure,closureRatio,orientationClosureDeg:rotClosure},pass?'Combined translation/rotation returned close to the starting transform.':'Mixed-motion loop accumulated excessive closure error.');}
  }
}
function stressReportObject(){
  const s=state.stress;return{version:1,completed:s.complete,overall:s.overall,startedAt:s.startedAt?new Date(Date.now()-(performance.now()-s.startedAt)).toISOString():null,results:s.results,finalDiagnostics:{position:clonePos(state.position),fovX:state.fovX,fovY:state.fovY,scale:state.scale,visualConfidence:state.visualConfidence,motionConfidence:state.motionConfidence,projectionResidualPx:state.projectionError,imuHz:state.imuHz,videoHz:state.videoHz,tracks:state.validTracks}};
}
function renderStressResults(){
  const r=stressReportObject(),passed=r.results.filter(x=>x.pass).length;ui.stressSummary.textContent=r.completed?`${r.overall} — ${passed}/${r.results.length} tests passed`:'Stress test has not completed.';ui.stressResults.innerHTML='';
  for(const item of r.results){const card=document.createElement('div');card.className='stressResult';card.dataset.pass=String(item.pass);const metricText=Object.entries(item.metrics||{}).map(([k,v])=>`${k}: ${typeof v==='number'&&Number.isFinite(v)?Number(v.toFixed(4)):v}`).join(' • ');card.innerHTML=`<header><b>${item.name}</b><strong>${item.pass?'PASS':'REVIEW'}</strong></header><small>${item.note||''}</small><small>${metricText}</small>`;ui.stressResults.appendChild(card);}
}
function showStressResults(){renderStressResults();if(!ui.stressDialog.open)ui.stressDialog.showModal();}
function exportStressReport(){const blob=new Blob([JSON.stringify(stressReportObject(),null,2)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='cruxtain-xyz-stress-report-v3-1.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}

function renderLoop(now){trackingMachine(now);updateStress(now);drawGrid();updateUI();requestAnimationFrame(renderLoop);}
function resize(){const r=grid.getBoundingClientRect(),d=Math.min(devicePixelRatio||1,2);grid.width=Math.max(1,Math.round(r.width*d));grid.height=Math.max(1,Math.round(r.height*d));if(state.fovX)state.fovY=2*Math.atan(Math.tan(state.fovX*DEG/2)*(r.height/Math.max(1,r.width)))/DEG;updateVisionCanvasSize();}

function basisObject(){return{version:3.1,savedAt:new Date().toISOString(),fovX:state.fovX,fovY:state.fovY,scale:state.scale,axisConvention:{x:'right from startup heading',y:'gravity up',z:'backward from startup heading; camera looks toward -Z'},cameraSettings:state.trackSettings,automatic:{focalConfidence:state.focalConfidence,timingConfidence:state.timingConfidence,videoImuLagMs:state.videoImuLagMs,scaleStability:state.scaleStability,scaleUpdates:state.metric.automaticUpdates,scaleSolve:state.metric.lastScaleSolve,mapGaugeReady:state.map.gaugeReady,confirmedLandmarks:state.map.confirmed,mapLandmarks:state.map.landmarks.length,mapInliers:state.map.poseInliers,mapConfidence:state.map.confidence,relocalizations:state.map.relocalizations,loopClosures:state.map.loopClosures,mapReprojectionErrorPx:state.map.reprojectionError},stress:stressReportObject(),note:'Zero-setup map-authoritative tracker: startup origin is automatic; a private monocular visual gauge bootstraps itself; metric scale is solved per session from visual keyframes plus IMU preintegration; persistent landmarks/PnP and consensus loop closure repair drift.'};}
function saveBasis(){saveAutomaticProfile(1e12);state.basisSaved=true;ui.instruction.textContent='Automatic device profile saved. No calibration sequence is required.';}
function loadBasis(){loadAutomaticProfile();ui.instruction.textContent='Automatic device profile refreshed. Tracking and the current map continue without a setup sequence.';}
function exportBasis(){const blob=new Blob([JSON.stringify(basisObject(),null,2)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='cruxtain-definitive-xyz-auto-v3-1.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}

ui.start.addEventListener('click',requestPermissions);
ui.stress.addEventListener('click',()=>{if(state.stress.active)manualAdvanceStress();else if(state.stress.complete)showStressResults();else startStressTest();});
if(ui.save)ui.save.addEventListener('click',saveBasis);if(ui.load)ui.load.addEventListener('click',loadBasis);ui.reset.addEventListener('click',beginTracking);
ui.diag.addEventListener('click',()=>ui.dialog.showModal());ui.closeDiag.addEventListener('click',()=>ui.dialog.close());ui.export.addEventListener('click',exportBasis);
ui.closeStress.addEventListener('click',()=>ui.stressDialog.close());ui.exportStress.addEventListener('click',exportStressReport);
addEventListener('resize',resize);
addEventListener('orientationchange',()=>setTimeout(()=>{resize();state.previousFrame=null;state.previousFrameTime=0;state.poseReason='Screen orientation changed; live map relocalization continues automatically';},250));
if('serviceWorker'in navigator)navigator.serviceWorker.register('./service-worker.js').catch(()=>{});
