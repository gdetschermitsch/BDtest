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
  dOriginQuality: $('#dOriginQuality'), dGridMode: $('#dGridMode'), dVisualStep: $('#dVisualStep'), dMoveGate: $('#dMoveGate'),
  dSensorOffset: $('#dSensorOffset'), dParallax: $('#dParallax'), dCoherence: $('#dCoherence'), dAnchors: $('#dAnchors'), dRelocalize: $('#dRelocalize'), dLandmarks: $('#dLandmarks'), dMapPose: $('#dMapPose'),
  stepLabel: $('#stepLabel'), stepDetail: $('#stepDetail'), stepTimer: $('#stepTimer')
};

const STORAGE_KEY = 'cruxtain.xyzBasis.v2.5';
const LEGACY_STORAGE_KEYS = ['cruxtain.xyzBasis.v2.4','cruxtain.xyzBasis.v2.3'];
const CALIBRATION_DISTANCE_M = 0.30;
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
function vDot(a,b){return a.x*b.x+a.y*b.y+a.z*b.z;}
function vCross(a,b){return{x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x};}
function vNorm(a){const n=vecLength(a)||1;return{x:a.x/n,y:a.y/n,z:a.z/n};}
function smallestEigenVectorSymmetric(m){
  const a=[[m[0][0],m[0][1],m[0][2]],[m[1][0],m[1][1],m[1][2]],[m[2][0],m[2][1],m[2][2]]];
  const v=[[1,0,0],[0,1,0],[0,0,1]];
  for(let it=0;it<14;it++){
    let p=0,qx=1,max=Math.abs(a[0][1]);
    for(const [i,j] of [[0,2],[1,2]]){const x=Math.abs(a[i][j]);if(x>max){max=x;p=i;qx=j;}}
    if(max<1e-10)break;
    const phi=.5*Math.atan2(2*a[p][qx],a[qx][qx]-a[p][p]),c=Math.cos(phi),sn=Math.sin(phi);
    const app=c*c*a[p][p]-2*sn*c*a[p][qx]+sn*sn*a[qx][qx];
    const aqq=sn*sn*a[p][p]+2*sn*c*a[p][qx]+c*c*a[qx][qx];
    for(let k=0;k<3;k++)if(k!==p&&k!==qx){const akp=a[k][p],akq=a[k][qx];a[k][p]=a[p][k]=c*akp-sn*akq;a[k][qx]=a[qx][k]=sn*akp+c*akq;}
    a[p][p]=app;a[qx][qx]=aqq;a[p][qx]=a[qx][p]=0;
    for(let k=0;k<3;k++){const vkp=v[k][p],vkq=v[k][qx];v[k][p]=c*vkp-sn*vkq;v[k][qx]=sn*vkp+c*vkq;}
  }
  let idx=0;if(a[1][1]<a[idx][idx])idx=1;if(a[2][2]<a[idx][idx])idx=2;
  return vNorm({x:v[0][idx],y:v[1][idx],z:v[2][idx]});
}

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
function qDot(a,b){return a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;}
function qSlerp(a,b,t){
  let bb=b,cos=qDot(a,b);
  if(cos<0){bb=q(-b.x,-b.y,-b.z,-b.w);cos=-cos;}
  if(cos>0.9995)return qNorm(q(lerp(a.x,bb.x,t),lerp(a.y,bb.y,t),lerp(a.z,bb.z,t),lerp(a.w,bb.w,t)));
  const ang=Math.acos(clamp(cos,-1,1)),s=Math.sin(ang);
  return qNorm(q(
    (Math.sin((1-t)*ang)*a.x+Math.sin(t*ang)*bb.x)/s,
    (Math.sin((1-t)*ang)*a.y+Math.sin(t*ang)*bb.y)/s,
    (Math.sin((1-t)*ang)*a.z+Math.sin(t*ang)*bb.z)/s,
    (Math.sin((1-t)*ang)*a.w+Math.sin(t*ang)*bb.w)/s
  ));
}
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
function deviceBodyQuaternion(alpha,beta,gamma){
  let out=qFromEulerYXZ(beta*DEG,alpha*DEG,-gamma*DEG);out=qMul(out,qAxis(1,0,0,-Math.PI/2));return qNorm(out);
}
function deviceCameraQuaternion(alpha, beta, gamma) {
  let out=deviceBodyQuaternion(alpha,beta,gamma);
  const screenAngle = ((screen.orientation && screen.orientation.angle) || window.orientation || 0) * DEG;
  out = qMul(out, qAxis(0,0,1,-screenAngle));
  return qNorm(out);
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
function rawRelativeQ(absQ) {
  if (!state.baseQ) return q();
  return qNorm(qMul(qInv(state.baseQ), absQ));
}
function relativeQ(absQ) {
  return qNorm(qMul(state.orientationCorrection||q(),rawRelativeQ(absQ)));
}
function relativeCameraQ() { return relativeQ(state.orientationQ); }

const state = {
  stage: 'idle', stream: null, trackSettings: {},
  orientationQ: q(), bodyOrientationQ:q(), baseQ: null, orientationCorrection:q(), previousFrameQ: null, orientationSamples: [], lastOrientationAt: 0, orientationRate: 0,
  sensorVideoOffsetMs:-45, sensorOffsetQuality:0, lastOffsetSolveAt:0, previousFrameTime:0,
  gyro: {x:0,y:0,z:0}, accelDevice: {x:0,y:0,z:0}, accelWorld: {x:0,y:0,z:0}, accelBiasDevice: {x:0,y:0,z:0},
  position: {x:0,y:0,z:0}, velocity: {x:0,y:0,z:0},
  fovX: 62, fovY: 48, fovSamples: [], focalConfidence: 0, projectionError: Infinity,
  scale: 1, scaleStability: 0, scaleSamples: [], scaleLocked: false,
  visualConfidence: 0, motionConfidence: 0, stationary: false, stationaryScore: 0, stillSince: 0, stillScore: 0, originQuality: 0,
  originQuaternionSamples: [], originAccelSamples: [], originCaptured: false,
  lastMotionAt: 0, lastFrameAt: 0, calibrationStartedAt: 0, lastSetupAt: performance.now(),
  imuCount: 0, imuHz: 0, imuStamp: performance.now(), videoCount: 0, videoHz: 0, videoStamp: performance.now(),
  processedFps: 0, processCount: 0, processStamp: performance.now(),
  frame: null, previousFrame: null, tracks: [], validTracks: 0, flowMagnitude: 0,
  translationSignal: {x:0,y:0,z:0,confidence:0,rawMagnitude:0,coherence:0,parallaxPx:0}, visualStepMagnitude:0, movementGate:'still', lastMoveAt:0, lastVisualTranslationAt:0, driftRate: 0, lastPositionForDrift: {x:0,y:0,z:0},
  keyframes:[], lastKeyframeAt:0, lastRelocalizeAt:0, relocalization:{count:0,last:'none',error:Infinity},
  landmarks:[], mapKeyframe:null, mapPoseConfidence:0, landmarkMatches:0, lastMapSolveAt:0, lastMapExpandAt:0,
  poseReason: 'Not started', loopStarted: false, lastProcessAt: 0, basisSaved: false, stageEnteredAt: performance.now(),
  calib: { visualPath:0, visualNet:{x:0,y:0,z:0}, inertialPath:0, inertialVelocity:{x:0,y:0,z:0}, lastPosition:{x:0,y:0,z:0}, motionSeen:false, startFrame:null, startQ:null },
  gridMode: 'off', worldRevision: 0,
  stress: {active:false,complete:false,index:-1,startedAt:0,stageStartedAt:0,testStartPos:null,testStartQ:null,lastPos:null,lastYaw:0,yawTravel:0,pathLength:0,maxDisplacement:0,maxAxis:0,maxOffAxis:0,maxDriftRate:0,stableSince:0,movementSeen:false,results:[],overall:'not run',manual:false}
};

function setStage(stage, text, progress, pill = 'calibrating') {
  state.stage = stage;
  state.stageEnteredAt = performance.now();
  ui.instruction.textContent = text;
  ui.progress.style.width = `${progress}%`;
  ui.status.textContent = stage.replaceAll('_', ' ').toUpperCase();
  ui.status.dataset.state = pill;
  const labels={hold_still:'STEP 1 OF 4',fov_sync:'STEP 2 OF 4',xyz_lock:'STEP 3 OF 4',settle_check:'STEP 4 OF 4',locked:'3D TEST',revalidating:'REVALIDATING',idle:'READY'};
  ui.stepLabel.textContent=labels[stage]||stage.toUpperCase();
  ui.stepDetail.textContent=text;
  state.gridMode = stage==='locked' ? '3D lattice' : stage==='settle_check' ? '3D preview' : stage==='fov_sync' ? 'projection tunnel' : stage==='xyz_lock' ? 'translation cage' : 'origin reticle';
}

async function requestPermissions() {
  ui.start.disabled = true;
  try {
    if (!navigator.mediaDevices?.getUserMedia) throw new Error('This browser does not expose camera capture.');
    if (typeof window.DeviceMotionEvent?.requestPermission === 'function') {
      const r = await window.DeviceMotionEvent.requestPermission();
      if (r !== 'granted') throw new Error('Motion permission was not granted.');
    }
    if (typeof window.DeviceOrientationEvent?.requestPermission === 'function') {
      const r = await window.DeviceOrientationEvent.requestPermission();
      if (r !== 'granted') throw new Error('Orientation permission was not granted.');
    }

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
    ui.load.disabled = !([STORAGE_KEY,...LEGACY_STORAGE_KEYS].some(k=>localStorage.getItem(k)));
    resize();
    beginSetup();
    startVideoLoop();
    if (!state.loopStarted) { state.loopStarted = true; requestAnimationFrame(renderLoop); }
  } catch (err) {
    ui.start.disabled = false;
    ui.start.textContent = 'Try Again';
    ui.instruction.textContent = err?.message || String(err);
  }
}

function onVisibility() {
  if (document.hidden) {
    state.velocity={x:0,y:0,z:0};
    state.poseReason='Paused while page is hidden';
  } else if (state.stage==='locked') {
    setStage('revalidating','Hold normally for a moment while the live sensor timing is revalidated…',94);
    state.stillSince=0; state.stillScore=0;
  }
}

function onOrientation(e) {
  if (e.alpha == null || e.beta == null || e.gamma == null) return;
  const now=performance.now();
  const next=deviceCameraQuaternion(e.alpha,e.beta,e.gamma);state.bodyOrientationQ=deviceBodyQuaternion(e.alpha,e.beta,e.gamma);
  if(state.lastOrientationAt){
    const dt=Math.max((now-state.lastOrientationAt)/1000,1e-3);
    state.orientationRate=qAngle(state.orientationQ,next)/dt;
  }
  state.orientationQ=next; state.lastOrientationAt=now;
  state.orientationSamples.push({t:now,q:next});
  while(state.orientationSamples.length>180||(state.orientationSamples[0]&&now-state.orientationSamples[0].t>3500))state.orientationSamples.shift();
}

function bodyToBasisQ(){
  if(!state.baseQ)return state.bodyOrientationQ;
  return qNorm(qMul(state.orientationCorrection||q(),qMul(qInv(state.baseQ),state.bodyOrientationQ)));
}
function basisGravityVector(){
  const absoluteGravity={x:0,y:-9.80665,z:0};if(!state.baseQ)return absoluteGravity;
  return qRotate(state.orientationCorrection||q(),qRotate(qInv(state.baseQ),absoluteGravity));
}
function onMotion(e) {
  const now=performance.now(),rr=e.rotationRate||{};
  // DeviceMotion alpha/beta/gamma are rotations about device Z/X/Y respectively.
  state.gyro={x:(rr.beta||0)*DEG,y:(rr.gamma||0)*DEG,z:(rr.alpha||0)*DEG};
  const a=e.acceleration||{},hasLinear=[a.x,a.y,a.z].every(Number.isFinite),rot=bodyToBasisQ();
  if(hasLinear){
    state.accelDevice={x:a.x,y:a.y,z:a.z};
    const corrected={x:state.accelDevice.x-state.accelBiasDevice.x,y:state.accelDevice.y-state.accelBiasDevice.y,z:state.accelDevice.z-state.accelBiasDevice.z};state.accelWorld=qRotate(rot,corrected);
  } else {
    const ag=e.accelerationIncludingGravity||{};
    if([ag.x,ag.y,ag.z].every(Number.isFinite)){
      const worldRaw=qRotate(rot,{x:ag.x,y:ag.y,z:ag.z}),g=basisGravityVector();state.accelWorld={x:worldRaw.x-g.x,y:worldRaw.y-g.y,z:worldRaw.z-g.z};
    }
  }
  state.lastMotionAt=now;state.imuCount++;if(now-state.imuStamp>=1000){state.imuHz=state.imuCount*1000/(now-state.imuStamp);state.imuCount=0;state.imuStamp=now;}
}

function beginSetup() {
  state.position={x:0,y:0,z:0}; state.velocity={x:0,y:0,z:0};
  state.baseQ=null; state.orientationCorrection=q(); state.previousFrameQ=null; state.previousFrame=null; state.previousFrameTime=0; state.frame=null; state.tracks=[];
  state.sensorVideoOffsetMs=-45; state.sensorOffsetQuality=0; state.lastOffsetSolveAt=0;
  state.keyframes=[];state.lastKeyframeAt=0;state.lastRelocalizeAt=0;state.relocalization={count:0,last:'none',error:Infinity};
  state.landmarks=[];state.mapKeyframe=null;state.mapPoseConfidence=0;state.landmarkMatches=0;state.lastMapSolveAt=0;state.lastMapExpandAt=0;
  state.fovSamples=[]; state.scaleSamples=[]; state.scale=1; state.scaleLocked=false;
  state.focalConfidence=0; state.projectionError=Infinity; state.visualConfidence=0; state.motionConfidence=0; state.scaleStability=0;
  state.validTracks=0; state.flowMagnitude=0; state.stationaryScore=0; state.stillSince=0; state.stillScore=0; state.originQuality=0;
  state.originQuaternionSamples=[]; state.originAccelSamples=[]; state.originCaptured=false;
  state.accelBiasDevice={x:0,y:0,z:0}; state.calibrationStartedAt=performance.now(); state.lastSetupAt=performance.now();
  state.calib={visualPath:0,visualNet:{x:0,y:0,z:0},inertialPath:0,inertialVelocity:{x:0,y:0,z:0},lastPosition:{x:0,y:0,z:0},motionSeen:false,startFrame:null,startQ:null};
  state.poseReason='Collecting a tolerant averaged origin; natural hand tremor is allowed';
  resetStress();
  ui.save.disabled=true; ui.stress.disabled=true;
  setStage('hold_still','Hold the phone normally. The origin uses a rolling average and does not require tripod-level stillness.',10);
}

function orientationAt(time,requireBounds=false) {
  const a=state.orientationSamples;if(!a.length)return requireBounds?null:state.orientationQ;
  if(time<=a[0].t)return requireBounds?null:a[0].q;
  if(time>=a[a.length-1].t)return requireBounds?null:a[a.length-1].q;
  let lo=0,hi=a.length-1;
  while(hi-lo>1){const m=(lo+hi)>>1;if(a[m].t<=time)lo=m;else hi=m;}
  const span=Math.max(1e-3,a[hi].t-a[lo].t),t=clamp((time-a[lo].t)/span,0,1);
  return qSlerp(a[lo].q,a[hi].q,t);
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
  const {w,h}=currFrame,corners=selectCorners(prevFrame,100),raw=[];
  for(const p of corners){
    const t=trackPoint(prevFrame.gray,currFrame.gray,w,h,p,16);
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
  // Do not force translation flow to be a constant 2D vector. Real Z motion
  // is radial and scene depth changes the magnitude point-by-point. Reject only
  // extreme residuals here; epipolar RANSAC below handles geometric outliers.
  const mags=tracks.map(t=>Math.hypot(t.residual.x,t.residual.y)),mm=median(mags),sm=mad(mags,mm),limit=Math.min(Math.max(w,h)*0.18,Math.max(4.0,mm+3.8*sm));
  const inliers=tracks.filter((t,i)=>mags[i]<limit&&t.confidence>0.10);
  const avgTrack=inliers.length?inliers.reduce((a,t)=>a+t.confidence,0)/inliers.length:0;
  const confidence=clamp(inliers.length/30,0,1)*clamp(inliers.length/Math.max(1,tracks.length),0,1)*(0.55+0.45*avgTrack);
  return {raw,tracks,inliers,confidence,fx,fy};
}

function rotationalFitError(raw,prevQ,currQ,w,h,hfov=state.fovX){
  const fx=.5*w/Math.tan(hfov*DEG/2),vfov=2*Math.atan(Math.tan(hfov*DEG/2)*(h/w))/DEG,fy=.5*h/Math.tan(vfov*DEG/2),cx=w/2,cy=h/2,errors=[];
  for(const t of raw){const pp=predictedRotatedPixel(t.p,prevQ,currQ,fx,fy,cx,cy);if(pp)errors.push(Math.hypot(t.q.x-pp.x,t.q.y-pp.y));}
  if(errors.length<10)return Infinity;const m=median(errors),sp=mad(errors,m),trim=errors.filter(e=>e<Math.max(2.2,m+2.4*sp));return trim.length>=8?median(trim):Infinity;
}
function updateSensorVideoOffset(raw,prevTime,currTime,w,h,now){
  if(now-state.lastOffsetSolveAt<420||raw.length<12)return;
  if(state.stage!=='fov_sync'&&(state.orientationRate<0.08||vecLength(correctedAcceleration())>0.65))return;
  state.lastOffsetSolveAt=now;let best=null,second=Infinity;
  for(let off=-140;off<=20;off+=10){
    const q1=orientationAt(prevTime+off,true),q2=orientationAt(currTime+off,true);if(!q1||!q2)continue;
    const rot=qAngle(q1,q2);if(rot<0.002||rot>0.20)continue;
    const score=rotationalFitError(raw,q1,q2,w,h,state.fovX);if(!Number.isFinite(score))continue;
    if(!best||score<best.score){second=best?best.score:second;best={off,score};}else if(score<second)second=score;
  }
  if(best&&best.score<7){
    const separation=Number.isFinite(second)?clamp((second-best.score)/(second+1e-6),0,1):0;
    const quality=clamp(1-best.score/5.5,0,1)*(0.65+0.35*separation);
    state.sensorVideoOffsetMs=lerp(state.sensorVideoOffsetMs,best.off,0.24);
    state.sensorOffsetQuality=lerp(state.sensorOffsetQuality,quality,0.18);
  }
}
function bearingFromPixel(p,fx,fy,cx,cy){return vNorm({x:(p.x-cx)/fx,y:-(p.y-cy)/fy,z:-1});}
function robustTranslationDirection(solution,prevQ,currQ,w,h){
  const {fx,fy}=solution,cx=w/2,cy=h/2,R=qMul(qInv(currQ),prevQ),items=[];
  for(const t of solution.inliers){
    const r1=qRotate(R,bearingFromPixel(t.p,fx,fy,cx,cy)),r2=bearingFromPixel(t.q,fx,fy,cx,cy),n0=vCross(r1,r2),nl=vecLength(n0);
    if(nl<1e-5)continue;items.push({n:{x:n0.x/nl,y:n0.y/nl,z:n0.z/nl},r1:vNorm(r1),r2});
  }
  if(items.length<7)return null;
  let best=null;
  for(let i=0;i<items.length;i++)for(let j=i+1;j<items.length;j+=Math.max(1,Math.floor(items.length/14))){
    const c=vCross(items[i].n,items[j].n),cl=vecLength(c);if(cl<0.08)continue;const dir={x:c.x/cl,y:c.y/cl,z:c.z/cl};
    const errs=items.map(it=>Math.abs(vDot(it.n,dir))),inliers=errs.filter(e=>e<0.018),score=median(errs);
    if(!best||inliers.length>best.count||(inliers.length===best.count&&score<best.score))best={dir,count:inliers.length,score};
  }
  if(!best||best.count<6)return null;
  const chosen=items.filter(it=>Math.abs(vDot(it.n,best.dir))<0.022),M=[[0,0,0],[0,0,0],[0,0,0]];
  for(const it of chosen){const n=it.n;M[0][0]+=n.x*n.x;M[0][1]+=n.x*n.y;M[0][2]+=n.x*n.z;M[1][0]+=n.y*n.x;M[1][1]+=n.y*n.y;M[1][2]+=n.y*n.z;M[2][0]+=n.z*n.x;M[2][1]+=n.z*n.y;M[2][2]+=n.z*n.z;}
  let dir=smallestEigenVectorSymmetric(M);if(vDot(dir,best.dir)<0)dir={x:-dir.x,y:-dir.y,z:-dir.z};
  const epi=median(chosen.map(it=>Math.abs(vDot(it.n,dir)))),ratio=chosen.length/items.length;
  const parallax=median(chosen.map(it=>Math.acos(clamp(vDot(it.r1,it.r2),-1,1))));
  return {dir,coherence:clamp(ratio*(1-epi/0.026),0,1),parallax,epipolarError:epi,count:chosen.length};
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

function correctedAcceleration() { return state.accelWorld; }
function estimateTranslation(solution,w,h,dt,prevQ,frameQ) {
  if(solution.inliers.length<8||dt<=0)return {x:0,y:0,z:0,confidence:0,rawMagnitude:0,coherence:0,parallaxPx:0};
  const {fx,fy}=solution,cx=w/2,cy=h/2;
  // Legacy residual vector is retained ONLY to resolve the sign ambiguity of
  // the epipolar translation direction. Its depth-dependent magnitude is not
  // allowed to choose the XYZ direction anymore.
  const lateralX=median(solution.inliers.map(t=>-t.residual.x/fx));
  const lateralY=median(solution.inliers.map(t=> t.residual.y/fy));
  const zSamples=[];
  for(const t of solution.inliers){
    const nx=(t.p.x-cx)/fx,ny=-(t.p.y-cy)/fy,denom=nx*nx+ny*ny;if(denom<0.010)continue;
    const ux=t.residual.x/fx+lateralX,uy=-t.residual.y/fy+lateralY;zSamples.push((ux*nx+uy*ny)/denom);
  }
  const legacy={x:lateralX,y:lateralY,z:-(zSamples.length?median(zSamples):0)};
  const geom=robustTranslationDirection(solution,prevQ,frameQ,w,h);
  if(!geom)return {x:0,y:0,z:0,confidence:0,rawMagnitude:0,coherence:0,parallaxPx:0};
  let dir=geom.dir;if(vDot(dir,legacy)<0)dir={x:-dir.x,y:-dir.y,z:-dir.z};
  const normalizedResiduals=solution.inliers.map(t=>Math.hypot(t.residual.x/fx,t.residual.y/fy));
  const proxy=median(normalizedResiduals),parallaxPx=geom.parallax*((fx+fy)*.5);
  const localDelta={x:dir.x*proxy,y:dir.y*proxy,z:dir.z*proxy},worldDelta=qRotate(relativeQ(frameQ),localDelta);
  const spread=mad(normalizedResiduals,proxy),geometric=clamp(1-spread/(proxy*1.8+0.0025),0.15,1);
  const parallaxEvidence=clamp((parallaxPx-0.12)/1.15,0,1);
  const confidence=solution.confidence*geom.coherence*geometric*(0.35+0.65*parallaxEvidence);
  return {...worldDelta,confidence,rawMagnitude:proxy,coherence:geom.coherence,parallaxPx};
}

function updateScaleCalibration(rawVisualDelta,dt) {
  if(state.scaleLocked||(state.stage!=='xyz_lock'&&state.stage!=='settle_check'))return;
  const a=correctedAcceleration(),aMag=vecLength(a),goodVisual=rawVisualDelta.confidence>0.10&&rawVisualDelta.coherence>0.42&&rawVisualDelta.parallaxPx>0.16;
  if(aMag>0.12||goodVisual)state.calib.motionSeen=true;
  state.calib.inertialVelocity.x+=a.x*dt;state.calib.inertialVelocity.y+=a.y*dt;state.calib.inertialVelocity.z+=a.z*dt;
  const inertialSpeed=vecLength(state.calib.inertialVelocity),inertialStep=inertialSpeed*dt;
  if(inertialStep<0.10)state.calib.inertialPath+=inertialStep;
  if(goodVisual&&rawVisualDelta.rawMagnitude<0.20){
    state.calib.visualPath+=rawVisualDelta.rawMagnitude;
    state.calib.visualNet.x+=rawVisualDelta.x;state.calib.visualNet.y+=rawVisualDelta.y;state.calib.visualNet.z+=rawVisualDelta.z;
  }
  if(state.stationary)state.calib.inertialVelocity={x:0,y:0,z:0};
  const net=vecLength(state.calib.visualNet),straightness=state.calib.visualPath?clamp(net/state.calib.visualPath,0,1):0;
  if(state.stationary&&state.calib.motionSeen&&net>0.012&&state.calib.visualPath>0.016&&straightness>0.58){
    const candidate=clamp(CALIBRATION_DISTANCE_M/net,0.20,30);
    state.scale=candidate;
    state.scaleSamples=[candidate];
    state.scaleStability=clamp(0.30+0.44*straightness+0.26*state.visualConfidence,0,1);
  } else state.scaleStability=Math.max(state.scaleStability,clamp((net/0.04)*straightness*0.28,0,0.28));
}

function qualityTracksWide(prevFrame,currFrame,search=22){
  if(!prevFrame||!currFrame||prevFrame.w!==currFrame.w||prevFrame.h!==currFrame.h)return [];
  const {w,h}=currFrame,corners=selectCorners(prevFrame,82),raw=[];
  for(const p of corners){const t=trackPoint(prevFrame.gray,currFrame.gray,w,h,p,search);if(t.confidence<0.14||t.fb>2.5||!Number.isFinite(t.score))continue;raw.push({p,q:{x:t.x,y:t.y},observed:{x:t.x-p.x,y:t.y-p.y},confidence:t.confidence,score:t.score});}
  return raw;
}
function storeKeyframe(frame,frameQ,now){
  if(!frame||now-state.lastKeyframeAt<900)return;const rel=relativeQ(frameQ);
  const distinct=state.keyframes.every(k=>posDist(k.position,state.position)>0.22||qAngle(k.q,rel)>18*DEG);if(!distinct)return;
  state.keyframes.push({frame:{gray:frame.gray.slice(),w:frame.w,h:frame.h},q:rel,position:{...state.position},t:now});
  if(state.keyframes.length>12)state.keyframes.splice(1,1);state.lastKeyframeAt=now;
}
function maybeRelocalize(frame,frameQ,now){
  if(state.stage!=='locked'||!state.stationary||now-state.lastRelocalizeAt<480)return;state.lastRelocalizeAt=now;
  if(!state.keyframes.length){storeKeyframe(frame,frameQ,now);return;}
  let best=null;const currentRel=relativeQ(frameQ);
  for(const k of state.keyframes){
    if(k.frame.w!==frame.w||k.frame.h!==frame.h)continue;const sensorAngle=qAngle(k.q,currentRel)/DEG;if(sensorAngle>10)continue;
    const raw=qualityTracksWide(k.frame,frame,20);if(raw.length<14)continue;const shifts=raw.map(t=>Math.hypot(t.observed.x,t.observed.y)),m=median(shifts),sp=mad(shifts,m),consistent=raw.filter((t,i)=>shifts[i]<Math.max(1.3,m+2.5*sp));
    const confidence=consistent.length/Math.max(1,raw.length);if(confidence<0.68)continue;const score=m+sensorAngle*0.055;
    if(!best||score<best.score)best={k,score,pixel:m,count:consistent.length,sensorAngle};
  }
  if(best&&best.pixel<1.25&&best.count>=14){
    const distance=posDist(state.position,best.k.position);if(distance<4.0){
      const strength=best.pixel<0.48?0.82:best.pixel<0.8?0.54:0.28;
      state.position.x=lerp(state.position.x,best.k.position.x,strength);state.position.y=lerp(state.position.y,best.k.position.y,strength);state.position.z=lerp(state.position.z,best.k.position.z,strength);state.velocity={x:0,y:0,z:0};
      // A near-identical camera image is also a visual attitude anchor. Correct
      // slow compass/device-orientation creep without touching the physical origin.
      if(best.pixel<0.72&&best.sensorAngle<7){const target=qNorm(qMul(best.k.q,qInv(rawRelativeQ(frameQ))));state.orientationCorrection=qSlerp(state.orientationCorrection,target,0.24);}
      state.relocalization.count++;state.relocalization.error=best.pixel;state.relocalization.last=`anchor ${state.keyframes.indexOf(best.k)+1} • ${best.pixel.toFixed(2)} px`;return;
    }
  }
  state.relocalization.last=best?`near ${best.pixel.toFixed(2)} px`:'no match';
  storeKeyframe(frame,frameQ,now);
}

function cloneFrame(frame){return frame?{gray:frame.gray.slice(),w:frame.w,h:frame.h}:null;}
function descriptorAt(frame,x,y,r=2){
  x=Math.round(x);y=Math.round(y);if(!frame||x-r<0||x+r>=frame.w||y-r<0||y+r>=frame.h)return null;
  const vals=[],img=frame.gray,w=frame.w;let mean=0;
  for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++){const v=img[(y+yy)*w+x+xx];vals.push(v);mean+=v;}mean/=vals.length;
  let norm=0;for(let i=0;i<vals.length;i++){vals[i]-=mean;norm+=vals[i]*vals[i];}norm=Math.sqrt(norm)||1;
  const out=new Float32Array(vals.length);for(let i=0;i<vals.length;i++)out[i]=vals[i]/norm;return out;
}
function descriptorDistanceAt(desc,frame,x,y,r=2){
  x=Math.round(x);y=Math.round(y);if(!desc||x-r<0||x+r>=frame.w||y-r<0||y+r>=frame.h)return Infinity;
  const img=frame.gray,w=frame.w,n=(r*2+1)**2;let mean=0,k=0;
  for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++)mean+=img[(y+yy)*w+x+xx];mean/=n;
  let norm=0;for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++){const d=img[(y+yy)*w+x+xx]-mean;norm+=d*d;}norm=Math.sqrt(norm)||1;
  let dot=0;for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++){dot+=desc[k++]*(img[(y+yy)*w+x+xx]-mean)/norm;}return 1-clamp(dot,-1,1);
}
function searchDescriptor(desc,frame,pred,search=13){
  let best={d:Infinity,x:pred.x,y:pred.y},second=Infinity;
  for(let dy=-search;dy<=search;dy+=2)for(let dx=-search;dx<=search;dx+=2){const d=descriptorDistanceAt(desc,frame,pred.x+dx,pred.y+dy);if(d<best.d){second=best.d;best={d,x:pred.x+dx,y:pred.y+dy};}else if(d<second)second=d;}
  const coarse={...best};for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const d=descriptorDistanceAt(desc,frame,coarse.x+dx,coarse.y+dy);if(d<best.d)best={d,x:coarse.x+dx,y:coarse.y+dy};}
  const uniqueness=Number.isFinite(second)?clamp((second-best.d)/(second+1e-6),0,1):0;return {...best,uniqueness};
}
function solve3x3(A,b){
  const m=[[A[0][0],A[0][1],A[0][2],b[0]],[A[1][0],A[1][1],A[1][2],b[1]],[A[2][0],A[2][1],A[2][2],b[2]]];
  for(let c=0;c<3;c++){let pivot=c;for(let r=c+1;r<3;r++)if(Math.abs(m[r][c])>Math.abs(m[pivot][c]))pivot=r;if(Math.abs(m[pivot][c])<1e-8)return null;[m[c],m[pivot]]=[m[pivot],m[c]];const d=m[c][c];for(let j=c;j<4;j++)m[c][j]/=d;for(let r=0;r<3;r++)if(r!==c){const f=m[r][c];for(let j=c;j<4;j++)m[r][j]-=f*m[c][j];}}
  return{x:m[0][3],y:m[1][3],z:m[2][3]};
}
function triangulateWorldPoint(C0,d0,C1,d1){
  d0=vNorm(d0);d1=vNorm(d1);const b=vDot(d0,d1),den=1-b*b;if(den<1e-5)return null;const w0={x:C0.x-C1.x,y:C0.y-C1.y,z:C0.z-C1.z},d=vDot(d0,w0),e=vDot(d1,w0),s=(b*e-d)/den,t=(e-b*d)/den;if(s<0.20||t<0.20||s>35||t>35)return null;
  const p0={x:C0.x+s*d0.x,y:C0.y+s*d0.y,z:C0.z+s*d0.z},p1={x:C1.x+t*d1.x,y:C1.y+t*d1.y,z:C1.z+t*d1.z},gap=posDist(p0,p1),depth=(s+t)*.5;if(gap>Math.max(0.045,depth*0.018))return null;
  return{point:{x:(p0.x+p1.x)*.5,y:(p0.y+p1.y)*.5,z:(p0.z+p1.z)*.5},gap,depth};
}
function addLandmarksFromPair(kf,frame,frameQ,camPos,search=30,maxAdd=48){
  if(!kf?.frame||!frame)return 0;const raw=qualityTracksWide(kf.frame,frame,search);if(raw.length<10)return 0;
  const w=frame.w,h=frame.h,fx=.5*w/Math.tan(state.fovX*DEG/2),vf=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vf/2),cx=w/2,cy=h/2,currQ=relativeQ(frameQ);let added=0;
  for(const t of raw){
    const d0=qRotate(kf.q,bearingFromPixel(t.p,fx,fy,cx,cy)),d1=qRotate(currQ,bearingFromPixel(t.q,fx,fy,cx,cy)),par=Math.acos(clamp(vDot(vNorm(d0),vNorm(d1)),-1,1));if(par<0.009)continue;
    const tri=triangulateWorldPoint(kf.position,d0,camPos,d1);if(!tri)continue;const desc=descriptorAt(frame,t.q.x,t.q.y,2);if(!desc)continue;
    if(state.landmarks.some(l=>posDist(l.position,tri.point)<0.075))continue;
    state.landmarks.push({position:tri.point,descriptor:desc,lastSeen:performance.now(),quality:clamp(t.confidence*(1-tri.gap/0.12),0,1)});added++;if(added>=maxAdd)break;
  }
  if(state.landmarks.length>240)state.landmarks.splice(0,state.landmarks.length-240);return added;
}
function bootstrapMetricMap(frame,frameQ,now){
  const c=state.calib;if(!c.startFrame||!c.startQ||!frame)return 0;state.landmarks=[];
  const kf={frame:c.startFrame,q:c.startQ,position:{x:0,y:0,z:0}},added=addLandmarksFromPair(kf,frame,frameQ,state.position,40,80);
  if(added>=8){state.mapKeyframe={frame:cloneFrame(frame),q:relativeQ(frameQ),position:{...state.position},t:now};state.mapPoseConfidence=clamp(added/45,0,0.8);}return added;
}
function projectWorldPoint(P,camQ,camPos,fx,fy,cx,cy){const c=cameraPoint(P,camQ,camPos);if(c.z>=-0.08)return null;return projectCamera(c,fx,fy,cx,cy);}
function solvePositionFromMatches(matches,camQ,w,h){
  if(matches.length<5)return null;const fx=.5*w/Math.tan(state.fovX*DEG/2),vf=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vf/2),cx=w/2,cy=h/2;
  const solve=(arr)=>{const A=[[0,0,0],[0,0,0],[0,0,0]],b=[0,0,0];for(const m of arr){const d=vNorm(qRotate(camQ,bearingFromPixel(m.pixel,fx,fy,cx,cy))),P=m.lm.position,M=[[1-d.x*d.x,-d.x*d.y,-d.x*d.z],[-d.y*d.x,1-d.y*d.y,-d.y*d.z],[-d.z*d.x,-d.z*d.y,1-d.z*d.z]];for(let i=0;i<3;i++){for(let j=0;j<3;j++)A[i][j]+=M[i][j];b[i]+=M[i][0]*P.x+M[i][1]*P.y+M[i][2]*P.z;}}return solve3x3(A,b);};
  let pos=solve(matches);if(!pos)return null;let scored=matches.map(m=>{const pp=projectWorldPoint(m.lm.position,camQ,pos,fx,fy,cx,cy);return{m,e:pp?Math.hypot(pp.x-m.pixel.x,pp.y-m.pixel.y):Infinity};}),errs=scored.map(x=>x.e).filter(Number.isFinite);if(errs.length<5)return null;const me=median(errs),sp=mad(errs,me),lim=Math.max(2.0,me+2.8*sp),inliers=scored.filter(x=>x.e<lim).map(x=>x.m);if(inliers.length>=5){const refined=solve(inliers);if(refined)pos=refined;}
  const finalErrs=inliers.map(m=>{const pp=projectWorldPoint(m.lm.position,camQ,pos,fx,fy,cx,cy);return pp?Math.hypot(pp.x-m.pixel.x,pp.y-m.pixel.y):99;}),error=median(finalErrs),ratio=inliers.length/matches.length,confidence=clamp(inliers.length/16,0,1)*ratio*clamp(1-error/5.0,0,1);return{position:pos,error,confidence,inliers};
}
function updateMetricMap(frame,frameQ,now){
  if(state.stage!=='locked'||state.landmarks.length<6||now-state.lastMapSolveAt<100)return;state.lastMapSolveAt=now;
  const camQ=relativeQ(frameQ),w=frame.w,h=frame.h,fx=.5*w/Math.tan(state.fovX*DEG/2),vf=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w)),fy=.5*h/Math.tan(vf/2),cx=w/2,cy=h/2,matches=[];
  const candidates=[...state.landmarks].sort((a,b)=>(b.lastSeen||0)-(a.lastSeen||0)).slice(0,80);
  for(const lm of candidates){const pp=projectWorldPoint(lm.position,camQ,state.position,fx,fy,cx,cy);if(!pp||pp.x<16||pp.x>w-16||pp.y<16||pp.y>h-16)continue;const m=searchDescriptor(lm.descriptor,frame,pp,13);if(m.d<0.46&&m.uniqueness>0.045)matches.push({lm,pixel:{x:m.x,y:m.y},descriptorError:m.d});}
  state.landmarkMatches=matches.length;const solved=solvePositionFromMatches(matches,camQ,w,h);
  if(!solved){state.mapPoseConfidence*=0.90;return;}state.mapPoseConfidence=lerp(state.mapPoseConfidence,solved.confidence,0.30);
  if(solved.confidence>0.24&&solved.error<3.6){
    const jump=posDist(state.position,solved.position),maxJump=solved.confidence>0.62?2.5:0.85;if(jump<maxJump){const k=clamp(0.30+0.52*solved.confidence,0.30,0.82);state.position.x=lerp(state.position.x,solved.position.x,k);state.position.y=lerp(state.position.y,solved.position.y,k);state.position.z=lerp(state.position.z,solved.position.z,k);for(const m of solved.inliers){m.lm.lastSeen=now;const d=descriptorAt(frame,m.pixel.x,m.pixel.y,2);if(d&&m.descriptorError<0.24)m.lm.descriptor=d;}}
  }
  if(state.mapKeyframe&&state.mapPoseConfidence>0.42&&now-state.lastMapExpandAt>700){const baseline=posDist(state.mapKeyframe.position,state.position),rot=qAngle(state.mapKeyframe.q,camQ)/DEG;if(baseline>0.16&&baseline<1.4&&rot<48){const added=addLandmarksFromPair(state.mapKeyframe,frame,frameQ,state.position,30,36);state.lastMapExpandAt=now;if(added>=4||baseline>0.45)state.mapKeyframe={frame:cloneFrame(frame),q:camQ,position:{...state.position},t:now};}}
}


function updatePose(rawVisualDelta,dt,now) {
  const a=correctedAcceleration(),gyroMag=vecLength(state.gyro),accMag=vecLength(a),visualSpeed=(rawVisualDelta.rawMagnitude||0)/Math.max(dt,1e-3),motionFresh=now-state.lastMotionAt<350;
  state.visualStepMagnitude=lerp(state.visualStepMagnitude,rawVisualDelta.rawMagnitude||0,0.28);

  const visualTranslation=rawVisualDelta.confidence>0.115&&rawVisualDelta.coherence>0.46&&rawVisualDelta.parallaxPx>0.20&&visualSpeed>0.004;
  const inertialTranslation=motionFresh&&accMag>0.24;
  const fastRotation=Math.max(gyroMag,state.orientationRate)>0.42;
  // Rotation by itself must never open the XYZ integration gate. During a fast
  // turn we demand either inertial translation or very strong visual parallax.
  const translationActive=visualTranslation&&(!fastRotation||inertialTranslation||rawVisualDelta.parallaxPx>1.15);
  const deliberate=translationActive||inertialTranslation;
  if(deliberate){
    state.stationaryScore=Math.min(state.stationaryScore,0.12);state.stationary=false;state.stillSince=0;state.lastMoveAt=now;
    state.movementGate=translationActive?'TRANSLATING':'MOTION';
  } else {
    const visualStillScore=state.validTracks>=6?1-clamp((visualSpeed-0.008)/0.095,0,1):0.56;
    const angularStillScore=1-clamp((Math.max(gyroMag,state.orientationRate)-0.018)/0.30,0,1);
    const accelStillScore=1-clamp((accMag-0.055)/0.78,0,1);
    const stationaryQuality=(motionFresh||now-state.lastOrientationAt<350)?(0.54*visualStillScore+0.24*angularStillScore+0.22*accelStillScore):0;
    state.stationaryScore=clamp(state.stationaryScore+dt*(stationaryQuality>0.57?stationaryQuality*1.9:-1.55),0,1);
    state.stationary=state.stationaryScore>0.60;
    state.movementGate=state.stationary?'STILL':fastRotation?'ROTATING':'FREE';
  }

  if(state.stationary){if(!state.stillSince)state.stillSince=now;state.velocity={x:0,y:0,z:0};}else state.stillSince=0;
  updateScaleCalibration(rawVisualDelta,dt);
  const active=(state.stage==='xyz_lock'||state.stage==='settle_check'||state.stage==='locked'||state.stage==='revalidating');
  if(active&&!state.stationary){
    if(translationActive){
      const gain=clamp(0.80+0.20*rawVisualDelta.confidence,0.80,1),dx=rawVisualDelta.x*state.scale*gain,dy=rawVisualDelta.y*state.scale*gain,dz=rawVisualDelta.z*state.scale*gain;
      state.position.x+=dx;state.position.y+=dy;state.position.z+=dz;state.lastVisualTranslationAt=now;
      const measured={x:dx/dt,y:dy/dt,z:dz/dt};state.velocity.x=lerp(state.velocity.x,measured.x,0.58);state.velocity.y=lerp(state.velocity.y,measured.y,0.58);state.velocity.z=lerp(state.velocity.z,measured.z,0.58);
    } else if(inertialTranslation&&now-state.lastVisualTranslationAt<120&&!fastRotation){
      // Bridge a single dropped visual frame, but never free-run IMU position.
      state.velocity.x+=a.x*dt*0.10;state.velocity.y+=a.y*dt*0.10;state.velocity.z+=a.z*dt*0.10;
      state.position.x+=state.velocity.x*dt;state.position.y+=state.velocity.y*dt;state.position.z+=state.velocity.z*dt;
    } else {state.velocity.x*=0.68;state.velocity.y*=0.68;state.velocity.z*=0.68;}
  }

  const maxSpeed=4.0,speed=vecLength(state.velocity);if(speed>maxSpeed){const k=maxSpeed/speed;state.velocity.x*=k;state.velocity.y*=k;state.velocity.z*=k;}
  if(!translationActive&&!state.stationary){state.velocity.x*=0.82;state.velocity.y*=0.82;state.velocity.z*=0.82;}
  const sourceAgreement=visualTranslation?clamp(0.45+0.35*rawVisualDelta.coherence+0.20*(inertialTranslation?1:0.5),0,1):0.35;
  state.motionConfidence=clamp(0.54*state.visualConfidence+0.20*sourceAgreement+0.16*(motionFresh?1:0.55)+0.10*(state.stationary?1:0.75),0,1);
  const p=state.position,lp=state.lastPositionForDrift;if(state.stationary){const drift=Math.hypot(p.x-lp.x,p.y-lp.y,p.z-lp.z)/Math.max(dt,1e-3);state.driftRate=lerp(state.driftRate,drift,0.10);}else state.driftRate*=0.96;state.lastPositionForDrift={...p};
}

function processVideoFrame(now) {
  if(now-state.lastProcessAt<45)return;
  const dt=clamp((now-(state.lastProcessAt||now))/1000,0.01,0.12);state.lastProcessAt=now;
  const frame=captureGray();if(!frame)return;const frameTime=now;
  if(state.previousFrame&&state.previousFrameTime){
    const raw=qualityTracks(state.previousFrame,frame),rawFlow=raw.length?median(raw.map(t=>Math.hypot(t.observed.x,t.observed.y))):0;state.flowMagnitude=lerp(state.flowMagnitude,rawFlow,0.25);
    updateSensorVideoOffset(raw,state.previousFrameTime,frameTime,frame.w,frame.h,now);
    const prevQ=orientationAt(state.previousFrameTime+state.sensorVideoOffsetMs)||state.orientationQ,frameQ=orientationAt(frameTime+state.sensorVideoOffsetMs)||state.orientationQ;
    if(state.stage==='fov_sync')estimateFovFromTracks(raw,prevQ,frameQ,frame.w,frame.h);
    const solution=residualSolution(raw,prevQ,frameQ,frame.w,frame.h,state.fovX);state.tracks=solution.inliers;state.validTracks=solution.inliers.length;state.visualConfidence=lerp(state.visualConfidence,solution.confidence,0.26);
    const vv=estimateTranslation(solution,frame.w,frame.h,dt,prevQ,frameQ);state.translationSignal=vv;updatePose(vv,dt,now);
    updateMetricMap(frame,frameQ,now);maybeRelocalize(frame,frameQ,now);
  }
  state.previousFrame=frame;state.previousFrameTime=frameTime;state.previousFrameQ=orientationAt(frameTime+state.sensorVideoOffsetMs)||state.orientationQ;state.frame=frame;
  state.processCount++;if(now-state.processStamp>=1000){state.processedFps=state.processCount*1000/(now-state.processStamp);state.processCount=0;state.processStamp=now;}
}

function originStability(now,dt) {
  const motionFresh=now-state.lastMotionAt<500,orientationFresh=now-state.lastOrientationAt<500;
  const gyroMag=vecLength(state.gyro),accMag=vecLength(state.accelWorld);
  const gyroScore=1-clamp((Math.max(gyroMag,state.orientationRate)-0.015)/0.22,0,1);
  const accScore=1-clamp((accMag-0.08)/1.15,0,1);
  const visualScore=state.validTracks>=5?1-clamp((state.flowMagnitude-0.15)/3.8,0,1):0.62;
  const freshness=(motionFresh||orientationFresh)?1:0;
  const quality=freshness*(0.43*gyroScore+0.34*accScore+0.23*visualScore);
  state.originQuality=lerp(state.originQuality,quality,0.12);
  const gain=quality>0.34?quality*0.78:-0.08;
  state.stillScore=clamp(state.stillScore+gain*dt,0,1.05);
  if(quality>0.38){
    state.originQuaternionSamples.push(state.orientationQ);
    state.originAccelSamples.push({...state.accelDevice});
    if(state.originQuaternionSamples.length>120)state.originQuaternionSamples.shift();
    if(state.originAccelSamples.length>120)state.originAccelSamples.shift();
  }
  return quality;
}
function captureOrigin() {
  const qSamples=state.originQuaternionSamples.slice(-80),aSamples=state.originAccelSamples.slice(-80);
  state.baseQ=qAverage(qSamples.length?qSamples:[state.orientationQ]);state.orientationCorrection=q();
  if(aSamples.length){
    state.accelBiasDevice={x:median(aSamples.map(a=>a.x)),y:median(aSamples.map(a=>a.y)),z:median(aSamples.map(a=>a.z))};
  }
  state.previousFrameQ=state.orientationQ;state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.originCaptured=true;state.worldRevision++;
}

function updateSetupGuidance(now,originQuality=0) {
  if(state.stage==='hold_still'){
    ui.stepTimer.textContent=`${Math.round(state.stillScore*100)}%`;
    ui.stepDetail.textContent=state.stillScore>0.65?'Origin is averaging now—keep holding normally.':'Natural hand tremor is accepted; avoid deliberate movement.';
  } else if(state.stage==='fov_sync'){
    ui.stepTimer.textContent=`${Math.round(state.focalConfidence*100)}%`;
    ui.stepDetail.textContent=`Turn left and right slowly. Projection residual ${Number.isFinite(state.projectionError)?state.projectionError.toFixed(1)+' px':'—'}.`;
  } else if(state.stage==='xyz_lock'){
    ui.stepTimer.textContent='30 CM';
    ui.stepDetail.textContent='Move the phone about 30 cm / 12 in sideways in one smooth straight move, then stop.';
  } else if(state.stage==='settle_check'){
    const elapsed=state.stillSince?now-state.stillSince:0,remaining=Math.max(0,0.75-elapsed/1000);
    ui.stepTimer.textContent=state.stationary?`${remaining.toFixed(1)}s`:'WAITING';
    ui.stepDetail.textContent=state.stationary?'Position is frozen while stationary—keep holding.':'Stop naturally; final drift verification begins automatically.';
  } else if(state.stage==='locked'){
    if(state.stress.complete){ui.stepTimer.textContent=state.stress.overall;ui.stepDetail.textContent=`Stress complete: ${state.stress.results.filter(r=>r.pass).length}/${state.stress.results.length} tests passed. Review results, then save.`;}
    else {ui.stepTimer.textContent='TEST';ui.stepDetail.textContent='360° lattice is active. Run Stress Test before saving the basis.';}
  }
}

function setupMachine(now) {
  const dt=clamp((now-state.lastSetupAt)/1000,0,0.1);state.lastSetupAt=now;
  const motionFresh=now-state.lastMotionAt<500;
  if(state.stage==='hold_still'){
    const quality=originStability(now,dt);updateSetupGuidance(now,quality);
    state.poseReason=`Origin averaging ${Math.round(state.stillScore*100)}%; quality ${Math.round(state.originQuality*100)}%`;
    const softTimeout=now-state.calibrationStartedAt>3500&&state.originQuality>0.28;
    if((state.stillScore>=0.72||softTimeout)&&state.originQuaternionSamples.length>=10){
      captureOrigin();state.calibrationStartedAt=now;
      setStage('fov_sync','Origin captured at 0,0,0. Slowly look left and right so image motion can solve the visible camera FOV.',34);
      state.poseReason='Origin captured; solving the exact visible crop projection from rotation and image displacement';
    }
  } else if(state.stage==='fov_sync'){
    updateSetupGuidance(now);
    if(state.focalConfidence>0.52&&state.fovSamples.length>=10&&state.projectionError<4.2){
      state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.calibrationStartedAt=now;
      state.calib={visualPath:0,visualNet:{x:0,y:0,z:0},inertialPath:0,inertialVelocity:{x:0,y:0,z:0},lastPosition:{x:0,y:0,z:0},motionSeen:false,startFrame:cloneFrame(state.frame),startQ:relativeQ(state.previousFrameQ||state.orientationQ)};
      setStage('xyz_lock','Projection is synchronized. Move the phone about 30 cm / 12 in sideways in one smooth straight move, then stop.',64);
      state.poseReason='Projection synchronized; solving a known-length translation baseline and camera-to-world axis conversion';
    } else if(now-state.calibrationStartedAt>15000)state.poseReason='Projection needs slower rotation and visible contrast; continue left/right without translating';
  } else if(state.stage==='xyz_lock'){
    updateSetupGuidance(now);
    const moved=state.calib.visualPath>0.016||vecLength(state.position)>0.012;
    const qualified=moved&&state.visualConfidence>0.28&&state.validTracks>=8&&state.scaleStability>0.22&&state.motionConfidence>0.34;
    if(qualified&&state.stationary){
      // The calibration gesture defines 0.30 m in the same world frame used by the lattice.
      state.position={x:state.calib.visualNet.x*state.scale,y:state.calib.visualNet.y*state.scale,z:state.calib.visualNet.z*state.scale};
      state.lastPositionForDrift={...state.position};const mapAdded=bootstrapMetricMap(state.frame,state.previousFrameQ||state.orientationQ,now);state.scaleLocked=true;state.calibrationStartedAt=now;state.stillSince=now;
      setStage('settle_check','The 30 cm translation baseline is locked. Hold naturally for the final no-creep check.',88);
      state.poseReason=`Scale frozen; ${mapAdded} metric landmarks triangulated for drift-resistant position solving`;
    }
  } else if(state.stage==='settle_check'){
    updateSetupGuidance(now);
    if(state.stationary&&state.stillSince&&now-state.stillSince>750&&state.driftRate<0.018){
      state.velocity={x:0,y:0,z:0};
      setStage('locked','Synchronization basis qualified. A world-centered 360° 3D lattice is active; walk/lean to verify XYZ translation before saving.',100,'locked');
      ui.status.textContent='POSE LOCKED';ui.save.disabled=true;ui.stress.disabled=false;
      state.poseReason='Origin, visible FOV, world-axis translation, scale freeze, and no-creep gates passed';
    }
  } else if(state.stage==='revalidating'){
    updateSetupGuidance(now);
    if(state.stationary&&state.stillSince&&now-state.stillSince>550){state.velocity={x:0,y:0,z:0};const resumeStress=state.stress.active?state.stress.index:-1;setStage('locked','Saved projection basis revalidated. Full 3D lattice restored at the current physical origin.',100,'locked');ui.stress.disabled=false;ui.save.disabled=!state.stress.complete;if(resumeStress>=0)enterStressTest(resumeStress);}
  }

  if(state.stage==='locked'){
    updateSetupGuidance(now);
    if((!motionFresh&&now-state.lastOrientationAt>500)||state.validTracks<4||state.visualConfidence<0.055){ui.status.dataset.state='lost';ui.status.textContent='POSE WEAK';state.poseReason=!motionFresh?'Motion stream stale':'Too few reliable visual tracks';}
    else{ui.status.dataset.state='locked';ui.status.textContent=state.stress.complete?(state.stress.overall==='PASS'?'STRESS PASS':'STRESS REVIEW'):'POSE LOCKED';}
  }
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
  const dpr=Math.min(devicePixelRatio||1,2),rect=grid.getBoundingClientRect();
  if(grid.width!==Math.round(rect.width*dpr)||grid.height!==Math.round(rect.height*dpr))resize();
  const W=rect.width,H=rect.height;gctx.setTransform(dpr,0,0,dpr,0,0);gctx.clearRect(0,0,W,H);
  const fx=.5*W/Math.tan(state.fovX*DEG/2),fy=.5*H/Math.tan(state.fovY*DEG/2),cx=W/2,cy=H/2,camQ=relativeCameraQ(),camPos=state.position;
  if(state.stage==='hold_still'||state.stage==='idle')drawOriginReticle(W,H);
  else if(state.stage==='fov_sync')drawProjectionTunnel(camQ,camPos,fx,fy,cx,cy);
  else if(state.stage==='xyz_lock')drawLattice(camQ,camPos,fx,fy,cx,cy,false);
  else if(state.stage==='settle_check'||state.stage==='locked'||state.stage==='revalidating')drawLattice(camQ,camPos,fx,fy,cx,cy,true);
  gctx.fillStyle='rgba(255,255,255,.78)';gctx.font='11px system-ui';gctx.fillText(`HFOV ${state.fovX.toFixed(1)}° • ${state.gridMode} • tracks ${state.validTracks}`,12,H-14);
}

function updateUI() {
  ui.x.textContent=state.position.x.toFixed(3);ui.y.textContent=state.position.y.toFixed(3);ui.z.textContent=state.position.z.toFixed(3);
  ui.gridMode.textContent=state.gridMode.toUpperCase();
  ui.dState.textContent=state.stage;ui.dFov.textContent=`${state.fovX.toFixed(1)}° × ${state.fovY.toFixed(1)}°`;
  ui.dVisual.textContent=`${Math.round(state.visualConfidence*100)}%`;ui.dMotion.textContent=`${Math.round(state.motionConfidence*100)}%`;
  ui.dStill.textContent=state.stationary?`locked ${Math.round(state.stationaryScore*100)}%`:`${Math.round(state.stationaryScore*100)}%`;ui.dFps.textContent=state.processedFps.toFixed(1);
  ui.dScale.textContent=`${state.scale.toFixed(3)} (${Math.round(state.scaleStability*100)}%)${state.scaleLocked?' locked':''}`;
  ui.dQuality.textContent=state.stage==='locked'?(state.visualConfidence>.30?'qualified':'limited'):'unqualified';
  ui.dTracks.textContent=String(state.validTracks);ui.dDrift.textContent=`${state.driftRate.toFixed(4)} m/s`;
  ui.dImuHz.textContent=state.imuHz.toFixed(1);ui.dVideoHz.textContent=state.videoHz.toFixed(1);ui.dReason.textContent=state.poseReason;
  ui.dProjectionError.textContent=Number.isFinite(state.projectionError)?`${state.projectionError.toFixed(2)} px`:'—';
  ui.dOriginQuality.textContent=`${Math.round(state.originQuality*100)}%`;ui.dGridMode.textContent=state.gridMode;
  if(ui.dVisualStep)ui.dVisualStep.textContent=state.visualStepMagnitude.toFixed(5);if(ui.dMoveGate)ui.dMoveGate.textContent=state.movementGate;
  if(ui.dSensorOffset)ui.dSensorOffset.textContent=`${state.sensorVideoOffsetMs.toFixed(0)} ms (${Math.round(state.sensorOffsetQuality*100)}%)`;
  if(ui.dParallax)ui.dParallax.textContent=`${(state.translationSignal.parallaxPx||0).toFixed(2)} px`;
  if(ui.dCoherence)ui.dCoherence.textContent=`${Math.round((state.translationSignal.coherence||0)*100)}%`;
  if(ui.dAnchors)ui.dAnchors.textContent=String(state.keyframes.length);
  if(ui.dRelocalize)ui.dRelocalize.textContent=`${state.relocalization.last} • ${state.relocalization.count}`;
  if(ui.dLandmarks)ui.dLandmarks.textContent=`${state.landmarks.length} (${state.landmarkMatches} matched)`;
  if(ui.dMapPose)ui.dMapPose.textContent=`${Math.round(state.mapPoseConfidence*100)}%`;
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
  if(ui.stress){ui.stress.textContent='Stress Test';ui.stress.disabled=state.stage!=='locked';}
}
function startStressTest(){
  if(state.stage!=='locked')return;
  resetStress();state.stress.active=true;state.stress.startedAt=performance.now();state.stress.index=0;ui.save.disabled=true;ui.stress.textContent='Next Test';enterStressTest(0);
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
  ui.stress.textContent='View Stress';ui.save.disabled=false;ui.status.textContent=s.overall==='PASS'?'STRESS PASS':'STRESS REVIEW';ui.status.dataset.state=s.overall==='PASS'?'locked':'calibrating';ui.stepLabel.textContent='STRESS COMPLETE';ui.stepTimer.textContent=s.overall;ui.stepDetail.textContent=`${s.results.filter(r=>r.pass).length}/${s.results.length} tests passed. Review the report before saving.`;ui.instruction.textContent='Stress qualification complete. Inspect closure and axis-purity measurements, then save the basis if the behavior is acceptable.';state.poseReason=`Stress ${s.overall}: ${s.results.filter(r=>r.pass).length}/${s.results.length} passed`;showStressResults();
}
function manualAdvanceStress(){
  if(!state.stress.active)return;const s=state.stress,t=STRESS_TESTS[s.index];const {d,dist}=stressCommonSample();
  let metrics={manualAdvance:true,closure:dist,maxDisplacement:s.maxDisplacement,pathLength:s.pathLength};
  if(t.kind==='axis'){const axis=Math.abs(d[t.axis]);const off=Math.hypot(...['x','y','z'].filter(a=>a!==t.axis).map(a=>d[a]));metrics={...metrics,maxAxis:s.maxAxis,maxOffAxis:s.maxOffAxis,axisPurity:s.maxAxis?s.maxOffAxis/s.maxAxis:Infinity};}
  finishStressResult(false,metrics,'Advanced manually before automatic completion.',true);
}
function updateStress(now){
  const s=state.stress;if(!s.active||state.stage!=='locked')return;const t=STRESS_TESTS[s.index],elapsed=(now-s.stageStartedAt)/1000,{d,dist}=stressCommonSample();
  if(t.kind==='stationary'){
    const pct=clamp(elapsed/4,0,1);ui.stepTimer.textContent=`${Math.round(pct*100)}%`;ui.stepDetail.textContent=`Hold still • max displacement ${s.maxDisplacement.toFixed(4)} m • drift ${s.maxDriftRate.toFixed(4)} m/s`;
    if(elapsed>=4){const pass=s.maxDisplacement<0.035&&s.maxDriftRate<0.025;finishStressResult(pass,{duration:elapsed,maxDisplacement:s.maxDisplacement,maxDriftRate:s.maxDriftRate,finalDisplacement:dist},pass?'No significant stationary creep detected.':'Stationary pose moved beyond the qualification threshold.');}
    return;
  }
  if(t.kind==='yaw'){
    const yaw=cameraYaw(relativeCameraQ()),dy=Math.abs(wrapPi(yaw-s.lastYaw));s.lastYaw=yaw;if(dy<0.45)s.yawTravel+=dy;const deg=s.yawTravel/DEG,closureDeg=qAngle(s.testStartQ,relativeCameraQ())/DEG;ui.stepTimer.textContent=`${Math.min(360,Math.round(deg))}°`;ui.stepDetail.textContent=`Complete 360° • position leak ${s.maxDisplacement.toFixed(3)} m • heading closure ${closureDeg.toFixed(1)}°`;
    if(deg>=330&&closureDeg<18&&state.orientationRate<0.18){const pass=closureDeg<12&&s.maxDisplacement<0.12;finishStressResult(pass,{yawTravelDeg:deg,orientationClosureDeg:closureDeg,maxPositionLeak:s.maxDisplacement,finalPositionLeak:dist},pass?'Rotation stayed separated from XYZ and closed near its starting attitude.':'360° rotation produced excessive orientation closure error or false translation.');}else if(elapsed>28&&deg>250){const pass=closureDeg<15&&s.maxDisplacement<0.12;finishStressResult(pass,{yawTravelDeg:deg,orientationClosureDeg:closureDeg,maxPositionLeak:s.maxDisplacement,finalPositionLeak:dist},'Timed completion after a near-full rotation.');}
    return;
  }
  if(t.kind==='axis'){
    const axis=Math.abs(d[t.axis]),off=Math.hypot(...['x','y','z'].filter(a=>a!==t.axis).map(a=>d[a]));s.maxAxis=Math.max(s.maxAxis,axis);s.maxOffAxis=Math.max(s.maxOffAxis,off);if(s.maxAxis>t.threshold)s.movementSeen=true;const closure=dist,returnThreshold=Math.max(0.055,s.maxAxis*0.30);if(s.movementSeen&&closure<returnThreshold&&state.stationary){if(!s.stableSince)s.stableSince=now;}else s.stableSince=0;const purity=s.maxAxis? s.maxOffAxis/s.maxAxis:Infinity;ui.stepTimer.textContent=`${Math.round(clamp(s.maxAxis/t.threshold,0,1)*100)}%`;ui.stepDetail.textContent=`${t.axis.toUpperCase()} excursion ${s.maxAxis.toFixed(3)} m • off-axis ratio ${Number.isFinite(purity)?purity.toFixed(2):'—'} • closure ${closure.toFixed(3)} m`;
    if(s.stableSince&&now-s.stableSince>700){const closureRatio=s.maxAxis?closure/s.maxAxis:Infinity,pass=s.maxAxis>t.threshold&&purity<0.65&&closureRatio<0.35;finishStressResult(pass,{axis:t.axis,maxAxis:s.maxAxis,maxOffAxis:s.maxOffAxis,axisPurity:purity,closure,closureRatio,pathLength:s.pathLength},pass?'Dominant motion stayed on the expected world axis and returned near the start.':'Axis cross-talk or return closure exceeded the qualification threshold.');}
    return;
  }
  if(t.kind==='mixed'){
    if(s.maxDisplacement>0.15&&s.pathLength>0.45)s.movementSeen=true;const closure=dist,closureRatio=s.maxDisplacement?closure/s.maxDisplacement:Infinity,rotClosure=qAngle(s.testStartQ,relativeCameraQ())/DEG;if(s.movementSeen&&closure<Math.max(0.08,s.maxDisplacement*0.35)&&state.stationary){if(!s.stableSince)s.stableSince=now;}else s.stableSince=0;ui.stepTimer.textContent=`${s.movementSeen?'RETURN':'MOVE'}`;ui.stepDetail.textContent=`Path ${s.pathLength.toFixed(2)} m • excursion ${s.maxDisplacement.toFixed(2)} m • closure ${closure.toFixed(3)} m`;
    if(s.stableSince&&now-s.stableSince>800){const pass=closureRatio<0.35&&s.pathLength>0.45;finishStressResult(pass,{pathLength:s.pathLength,maxDisplacement:s.maxDisplacement,closure,closureRatio,orientationClosureDeg:rotClosure},pass?'Combined translation/rotation returned close to the starting transform.':'Mixed-motion loop accumulated excessive closure error.');}
  }
}
function stressReportObject(){
  const s=state.stress;return{version:1,completed:s.complete,overall:s.overall,startedAt:s.startedAt?new Date(Date.now()-(performance.now()-s.startedAt)).toISOString():null,results:s.results,finalDiagnostics:{position:clonePos(state.position),fovX:state.fovX,fovY:state.fovY,scale:state.scale,visualConfidence:state.visualConfidence,motionConfidence:state.motionConfidence,projectionResidualPx:state.projectionError,sensorVideoOffsetMs:state.sensorVideoOffsetMs,translationParallaxPx:state.translationSignal.parallaxPx||0,translationCoherence:state.translationSignal.coherence||0,visualAnchors:state.keyframes.length,relocalizations:state.relocalization.count,metricLandmarks:state.landmarks.length,metricLandmarkMatches:state.landmarkMatches,mapPoseConfidence:state.mapPoseConfidence,imuHz:state.imuHz,videoHz:state.videoHz,tracks:state.validTracks}};
}
function renderStressResults(){
  const r=stressReportObject(),passed=r.results.filter(x=>x.pass).length;ui.stressSummary.textContent=r.completed?`${r.overall} — ${passed}/${r.results.length} tests passed`:'Stress test has not completed.';ui.stressResults.innerHTML='';
  for(const item of r.results){const card=document.createElement('div');card.className='stressResult';card.dataset.pass=String(item.pass);const metricText=Object.entries(item.metrics||{}).map(([k,v])=>`${k}: ${typeof v==='number'&&Number.isFinite(v)?Number(v.toFixed(4)):v}`).join(' • ');card.innerHTML=`<header><b>${item.name}</b><strong>${item.pass?'PASS':'REVIEW'}</strong></header><small>${item.note||''}</small><small>${metricText}</small>`;ui.stressResults.appendChild(card);}
}
function showStressResults(){renderStressResults();if(!ui.stressDialog.open)ui.stressDialog.showModal();}
function exportStressReport(){const blob=new Blob([JSON.stringify(stressReportObject(),null,2)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='cruxtain-xyz-stress-report-v2-5.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}

function renderLoop(now){setupMachine(now);updateStress(now);drawGrid();updateUI();requestAnimationFrame(renderLoop);}
function resize(){const r=grid.getBoundingClientRect(),d=Math.min(devicePixelRatio||1,2);grid.width=Math.max(1,Math.round(r.width*d));grid.height=Math.max(1,Math.round(r.height*d));if(state.fovX)state.fovY=2*Math.atan(Math.tan(state.fovX*DEG/2)*(r.height/Math.max(1,r.width)))/DEG;updateVisionCanvasSize();}

function basisObject() {
  return {
    version:2.5,savedAt:new Date().toISOString(),fovX:state.fovX,fovY:state.fovY,scale:state.scale,calibrationDistanceM:CALIBRATION_DISTANCE_M,sensorVideoOffsetMs:state.sensorVideoOffsetMs,
    axisConvention:{x:'right',y:'up',z:'backward; camera looks toward -Z',units:'meters after 30 cm baseline'},cameraSettings:state.trackSettings,
    qualification:{visualConfidence:state.visualConfidence,motionConfidence:state.motionConfidence,scaleStability:state.scaleStability,stationaryDrift:state.driftRate,projectionResidualPx:state.projectionError,originQuality:state.originQuality,sensorVideoOffsetMs:state.sensorVideoOffsetMs,metricLandmarks:state.landmarks.length,mapPoseConfidence:state.mapPoseConfidence,imuHz:state.imuHz,videoHz:state.videoHz,stress:stressReportObject()},
    note:'Reload restores the calibrated visible-camera projection, known-length scale, and camera/IMU timing offset. The current physical pose becomes the reloaded origin; visual anchors are rebuilt during the live session.'
  };
}
function saveBasis(){localStorage.setItem(STORAGE_KEY,JSON.stringify(basisObject()));state.basisSaved=true;ui.load.disabled=false;ui.instruction.textContent='Basis saved locally after the 3D walk-around test. Reload restores projection and scale, then establishes the current physical pose as origin.';}
function loadBasis(){
  const raw=[STORAGE_KEY,...LEGACY_STORAGE_KEYS].map(k=>localStorage.getItem(k)).find(Boolean);if(!raw)return;
  try{
    resetStress(); ui.stress.disabled=true;
    const b=JSON.parse(raw);if(Number(b.version)<2.5)throw new Error('This older basis used the previous depth-dependent scale solver. Run the v2.5 synchronization once for the 30 cm baseline.');
    state.fovX=clamp(b.fovX||62,34,105);state.fovY=clamp(b.fovY||48,20,105);state.scale=clamp(b.scale||1,.1,30);state.scaleLocked=true;state.sensorVideoOffsetMs=clamp(b.sensorVideoOffsetMs??-45,-160,40);state.orientationCorrection=q();state.keyframes=[];state.landmarks=[];state.mapKeyframe=null;state.mapPoseConfidence=0;state.landmarkMatches=0;
    state.scaleStability=b.qualification?.scaleStability||.5;state.projectionError=b.qualification?.projectionResidualPx??Infinity;
    state.baseQ=state.orientationQ;state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.stillScore=0;state.stillSince=0;
    setStage('revalidating','Saved projection and scale loaded. Hold normally while the current camera pose becomes the new origin.',94);
    ui.save.disabled=false;state.poseReason='Revalidating saved basis against current live camera and motion streams';
  }catch(err){ui.instruction.textContent=`Saved basis could not be loaded: ${err.message}`;}
}
function exportBasis(){const blob=new Blob([JSON.stringify(basisObject(),null,2)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='cruxtain-definitive-xyz-basis-v2-5.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}

ui.start.addEventListener('click',requestPermissions);
ui.stress.addEventListener('click',()=>{if(state.stress.active)manualAdvanceStress();else if(state.stress.complete)showStressResults();else startStressTest();});
ui.save.addEventListener('click',saveBasis);ui.load.addEventListener('click',loadBasis);ui.reset.addEventListener('click',beginSetup);
ui.diag.addEventListener('click',()=>ui.dialog.showModal());ui.closeDiag.addEventListener('click',()=>ui.dialog.close());ui.export.addEventListener('click',exportBasis);
ui.closeStress.addEventListener('click',()=>ui.stressDialog.close());ui.exportStress.addEventListener('click',exportStressReport);
addEventListener('resize',resize);
addEventListener('orientationchange',()=>setTimeout(()=>{resize();if(state.stage==='locked')setStage('revalidating','Screen orientation changed. Hold normally while axes and projection revalidate.',94);},250));
if('serviceWorker'in navigator)navigator.serviceWorker.register('./service-worker.js').catch(()=>{});
