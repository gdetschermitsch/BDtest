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

const STORAGE_KEY = 'cruxtain.xyzBasis.v2.5';
const LEGACY_STORAGE_KEYS = ['cruxtain.xyzBasis.v2.4','cruxtain.xyzBasis.v2.3'];
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
function deviceCameraQuaternion(alpha, beta, gamma) {
  let out = qFromEulerYXZ(beta*DEG, alpha*DEG, -gamma*DEG);
  out = qMul(out, qAxis(1,0,0,-Math.PI/2));
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

const state = {
  stage: 'idle', stream: null, trackSettings: {},
  orientationQ: q(), baseQ: null, orientationCorrection:q(), orientationHoldQ:null, previousFrameQ: null, previousFrameTime:0, orientationSamples: [], lastOrientationAt: 0, orientationRate: 0,
  gyro: {x:0,y:0,z:0}, accelDevice: {x:0,y:0,z:0}, accelGravityDevice:{x:0,y:0,z:0}, accelWorld: {x:0,y:0,z:0}, accelBiasDevice: {x:0,y:0,z:0}, gravityWorld:{x:0,y:-9.80665,z:0}, hasLinearAcceleration:false, hasGravityAcceleration:false,
  position: {x:0,y:0,z:0}, velocity: {x:0,y:0,z:0},
  fovX: 62, fovY: 48, fovSamples: [], focalConfidence: 0, projectionError: Infinity, videoImuLagMs:70, timingSamples:[], timingConfidence:0,
  scale: 1, scaleStability: 0, scaleSamples: [], scaleLocked: false,
  visualConfidence: 0, motionConfidence: 0, stationary: false, stationaryScore: 0, stillSince: 0, stillScore: 0, originQuality: 0,
  originQuaternionSamples: [], originAccelSamples: [], originGravitySamples:[], originCaptured: false,
  lastMotionAt: 0, lastFrameAt: 0, calibrationStartedAt: 0, lastSetupAt: performance.now(),
  imuCount: 0, imuHz: 0, imuStamp: performance.now(), videoCount: 0, videoHz: 0, videoStamp: performance.now(),
  processedFps: 0, processCount: 0, processStamp: performance.now(),
  frame: null, previousFrame: null, tracks: [], validTracks: 0, flowMagnitude: 0,
  translationSignal: {x:0,y:0,z:0,confidence:0,rawMagnitude:0}, visualStepMagnitude:0, movementGate:'still', lastMoveAt:0, driftRate: 0, lastPositionForDrift: {x:0,y:0,z:0},
  poseReason: 'Not started', loopStarted: false, lastProcessAt: 0, basisSaved: false, stageEnteredAt: performance.now(),
  calib: { visualPath:0, inertialPath:0, inertialVelocity:{x:0,y:0,z:0}, rawVisualPosition:{x:0,y:0,z:0}, lastPosition:{x:0,y:0,z:0}, motionSeen:false, metricCaptured:false },
  gridMode: 'off', worldRevision: 0, translationDirectionConfidence:0,
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
    ui.load.disabled = !(localStorage.getItem(STORAGE_KEY) || LEGACY_STORAGE_KEYS.some(k=>localStorage.getItem(k)));
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
  const next=deviceCameraQuaternion(e.alpha,e.beta,e.gamma);
  if(state.lastOrientationAt){
    const dt=Math.max((now-state.lastOrientationAt)/1000,1e-3);
    state.orientationRate=qAngle(state.orientationQ,next)/dt;
  }
  state.orientationQ=next; state.lastOrientationAt=now;
  state.orientationSamples.push({t:now,q:next});
  while(state.orientationSamples.length>180||(state.orientationSamples[0]&&now-state.orientationSamples[0].t>3500))state.orientationSamples.shift();
}

function onMotion(e) {
  const now=performance.now();
  const rr=e.rotationRate||{};
  // DeviceMotion rotation-rate axes share the physical phone frame used by the camera.
  state.gyro={x:(rr.alpha||0)*DEG,y:(rr.beta||0)*DEG,z:(rr.gamma||0)*DEG};
  const a=e.acceleration||{},ag=e.accelerationIncludingGravity||{};
  const hasLinear=[a.x,a.y,a.z].every(Number.isFinite),hasGravity=[ag.x,ag.y,ag.z].every(Number.isFinite);
  state.hasLinearAcceleration=state.hasLinearAcceleration||hasLinear;state.hasGravityAcceleration=state.hasGravityAcceleration||hasGravity;
  if(hasGravity)state.accelGravityDevice={x:ag.x,y:ag.y,z:ag.z};
  const rot=state.baseQ?relativeCameraQ():state.orientationQ;
  if(hasLinear){
    state.accelDevice={x:a.x,y:a.y,z:a.z};
    const corrected={x:state.accelDevice.x-state.accelBiasDevice.x,y:state.accelDevice.y-state.accelBiasDevice.y,z:state.accelDevice.z-state.accelBiasDevice.z};
    state.accelWorld=qRotate(rot,corrected);
  } else if(hasGravity){
    if(!state.baseQ){
      // Before the gravity vector is calibrated, do not let gravity masquerade as motion.
      state.accelWorld={x:0,y:0,z:0};
    } else {
      const worldRaw=qRotate(rot,state.accelGravityDevice);
      // The gravity vector is measured at origin in the actual browser sign convention.
      state.accelWorld={x:worldRaw.x-state.gravityWorld.x,y:worldRaw.y-state.gravityWorld.y,z:worldRaw.z-state.gravityWorld.z};
    }
  }
  state.lastMotionAt=now;
  state.imuCount++;
  if(now-state.imuStamp>=1000){state.imuHz=state.imuCount*1000/(now-state.imuStamp);state.imuCount=0;state.imuStamp=now;}
}

function beginSetup() {
  state.position={x:0,y:0,z:0}; state.velocity={x:0,y:0,z:0};
  state.baseQ=null; state.orientationCorrection=q(); state.orientationHoldQ=null; state.previousFrameQ=null; state.previousFrameTime=0; state.previousFrame=null; state.frame=null; state.tracks=[];
  state.fovSamples=[]; state.timingSamples=[]; state.videoImuLagMs=70; state.timingConfidence=0; state.scaleSamples=[]; state.scale=1; state.scaleLocked=false;
  state.focalConfidence=0; state.projectionError=Infinity; state.visualConfidence=0; state.motionConfidence=0; state.scaleStability=0;
  state.validTracks=0; state.flowMagnitude=0; state.stationaryScore=0; state.stillSince=0; state.stillScore=0; state.originQuality=0; state.driftRate=0; state.lastPositionForDrift={x:0,y:0,z:0}; state.translationDirectionConfidence=0;
  state.originQuaternionSamples=[]; state.originAccelSamples=[]; state.originGravitySamples=[]; state.originCaptured=false;
  state.accelBiasDevice={x:0,y:0,z:0}; state.gravityWorld={x:0,y:-9.80665,z:0}; state.hasLinearAcceleration=false; state.hasGravityAcceleration=false; state.calibrationStartedAt=performance.now(); state.lastSetupAt=performance.now();
  state.calib={visualPath:0,inertialPath:0,inertialVelocity:{x:0,y:0,z:0},rawVisualPosition:{x:0,y:0,z:0},lastPosition:{x:0,y:0,z:0},motionSeen:false,metricCaptured:false};
  state.poseReason='Collecting a gravity-level origin; natural hand tremor is allowed';
  resetStress();
  ui.save.disabled=true; ui.stress.disabled=true;
  setStage('hold_still','Hold the phone normally. The origin uses a rolling average and does not require tripod-level stillness.',10);
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

function correctedAcceleration() { return state.accelWorld; }
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

function updateScaleCalibration(rawVisualDelta,dt) {
  if(state.scaleLocked||(state.stage!=='xyz_lock'&&state.stage!=='settle_check'))return;
  const c=state.calib,a=correctedAcceleration(),aMag=vecLength(a);
  if(aMag>0.045)c.motionSeen=true;
  c.inertialVelocity.x+=a.x*dt;c.inertialVelocity.y+=a.y*dt;c.inertialVelocity.z+=a.z*dt;
  const inertialStep=vecLength(c.inertialVelocity)*dt,visualStep=rawVisualDelta.rawMagnitude;
  if(rawVisualDelta.confidence>0.10&&visualStep<0.20){
    c.rawVisualPosition.x+=rawVisualDelta.x;c.rawVisualPosition.y+=rawVisualDelta.y;c.rawVisualPosition.z+=rawVisualDelta.z;c.visualPath+=visualStep;
  }
  if(inertialStep<0.10)c.inertialPath+=inertialStep;
  if(state.stationary)c.inertialVelocity={x:0,y:0,z:0};
  const rx=c.rawVisualPosition.x,off=Math.hypot(c.rawVisualPosition.y,c.rawVisualPosition.z),axisPurity=Math.abs(rx)>1e-5?off/Math.abs(rx):Infinity;
  // Step 3 is now a real metric reference: the user moves exactly 0.50 m to world-right.
  // This removes the old arbitrary 3.2 fallback and gives the lattice a physical scale.
  if(!c.metricCaptured&&state.stationary&&Math.abs(rx)>0.022&&axisPurity<0.85){
    const candidate=clamp(0.50/Math.abs(rx),0.12,20);
    state.scaleSamples.push(candidate);if(state.scaleSamples.length>24)state.scaleSamples.shift();
    const m=median(state.scaleSamples),spread=mad(state.scaleSamples,m);state.scale=m;
    state.scaleStability=clamp(0.55+0.45*(1-axisPurity),0,1)*clamp(1-spread/(m*0.35+1e-3),0.45,1);
    c.metricCaptured=true;
    state.position={x:c.rawVisualPosition.x*state.scale,y:c.rawVisualPosition.y*state.scale,z:c.rawVisualPosition.z*state.scale};
    state.velocity={x:0,y:0,z:0};
  } else if(!c.metricCaptured){
    state.scaleStability=clamp(Math.abs(rx)/0.07,0,0.48)*clamp(1-axisPurity/1.4,0.15,1);
  }
}

function updatePose(rawVisualDelta,dt,now) {
  const a=correctedAcceleration(),gyroMag=vecLength(state.gyro),accMag=vecLength(a);
  const visualSpeed=(rawVisualDelta.rawMagnitude||0)/Math.max(dt,1e-3);
  const motionFresh=now-state.lastMotionAt<350;
  state.visualStepMagnitude=lerp(state.visualStepMagnitude,rawVisualDelta.rawMagnitude||0,0.28);

  // Hard release: once deliberate translation is observed, the old stillness
  // latch cannot suppress the beginning of the user's movement.
  const deliberateVisual=rawVisualDelta.confidence>0.11&&visualSpeed>0.028;
  const deliberateInertial=motionFresh&&accMag>0.30;
  const deliberateAngular=Math.max(gyroMag,state.orientationRate)>0.060;
  const deliberate=deliberateVisual||deliberateInertial||deliberateAngular;
  if(deliberate){
    state.stationaryScore=Math.min(state.stationaryScore,0.12);
    state.stationary=false;state.stillSince=0;state.lastMoveAt=now;state.movementGate='MOVING';
  } else {
    const visualStillScore=state.validTracks>=6?1-clamp((visualSpeed-0.010)/0.12,0,1):0.52;
    const angularStillScore=1-clamp((Math.max(gyroMag,state.orientationRate)-0.020)/0.28,0,1);
    const accelStillScore=1-clamp((accMag-0.06)/0.85,0,1);
    const stationaryQuality=(motionFresh||now-state.lastOrientationAt<350)?(0.50*visualStillScore+0.28*angularStillScore+0.22*accelStillScore):0;
    state.stationaryScore=clamp(state.stationaryScore+dt*(stationaryQuality>0.55?stationaryQuality*1.8:-1.7),0,1);
    state.stationary=state.stationaryScore>0.58;
    state.movementGate=state.stationary?'STILL':'FREE';
  }

  if(state.stationary){
    if(!state.stillSince)state.stillSince=now;
    state.velocity={x:0,y:0,z:0};
  } else state.stillSince=0;

  updateScaleCalibration(rawVisualDelta,dt);
  const active=(state.stage==='xyz_lock'||state.stage==='settle_check'||state.stage==='locked'||state.stage==='revalidating');
  if(active&&!state.stationary){
    if(rawVisualDelta.confidence>0.08&&rawVisualDelta.rawMagnitude>0.00015){
      const gain=clamp(0.78+0.22*rawVisualDelta.confidence,0.78,1);
      const dx=rawVisualDelta.x*state.scale*gain,dy=rawVisualDelta.y*state.scale*gain,dz=rawVisualDelta.z*state.scale*gain;
      state.position.x+=dx;state.position.y+=dy;state.position.z+=dz;
      const measured={x:dx/dt,y:dy/dt,z:dz/dt};
      state.velocity.x=lerp(state.velocity.x,measured.x,0.62);
      state.velocity.y=lerp(state.velocity.y,measured.y,0.62);
      state.velocity.z=lerp(state.velocity.z,measured.z,0.62);
    } else {
      // Never invent position from browser acceleration when vision is weak.
      // Holding the last visual position is far safer than an inertial free-run that becomes drift.
      state.velocity.x*=0.55;state.velocity.y*=0.55;state.velocity.z*=0.55;
    }
  }

  const maxSpeed=5.0,speed=vecLength(state.velocity);
  if(speed>maxSpeed){const k=maxSpeed/speed;state.velocity.x*=k;state.velocity.y*=k;state.velocity.z*=k;}
  if(!deliberate&&!state.stationary){state.velocity.x*=0.88;state.velocity.y*=0.88;state.velocity.z*=0.88;}

  const sourceAgreement=clamp(1-Math.abs(visualSpeed-accMag*0.10)/(visualSpeed+0.30),0,1);
  state.motionConfidence=clamp(0.56*state.visualConfidence+0.16*(motionFresh?1:0.55)+0.18*sourceAgreement+0.10*(state.stationary?1:0.8),0,1);
  const p=state.position,lp=state.lastPositionForDrift;
  if(state.stationary){const drift=Math.hypot(p.x-lp.x,p.y-lp.y,p.z-lp.z)/Math.max(dt,1e-3);state.driftRate=lerp(state.driftRate,drift,0.10);}else state.driftRate*=0.96;
  state.lastPositionForDrift={...p};
}

function processVideoFrame(now,meta={}) {
  if(now-state.lastProcessAt<45)return;
  const dt=clamp((now-(state.lastProcessAt||now))/1000,0.01,0.12);state.lastProcessAt=now;
  const frame=captureGray();if(!frame)return;
  const frameTime=Number.isFinite(meta.presentationTime)?meta.presentationTime:(Number.isFinite(meta.expectedDisplayTime)?meta.expectedDisplayTime:now);
  if(state.previousFrame&&state.previousFrameTime){
    const raw=qualityTracks(state.previousFrame,frame),rawFlow=raw.length?median(raw.map(t=>Math.hypot(t.observed.x,t.observed.y))):0;
    state.flowMagnitude=lerp(state.flowMagnitude,rawFlow,0.25);
    if(state.stage==='fov_sync')estimateVideoImuTiming(raw,state.previousFrameTime,frameTime,frame.w,frame.h);
    const prevQ=nearestOrientation(state.previousFrameTime-state.videoImuLagMs),frameQ=nearestOrientation(frameTime-state.videoImuLagMs);
    if(state.stage==='fov_sync')estimateFovFromTracks(raw,prevQ,frameQ,frame.w,frame.h);
    const solution=residualSolution(raw,prevQ,frameQ,frame.w,frame.h,state.fovX);
    state.tracks=solution.inliers;state.validTracks=solution.inliers.length;state.visualConfidence=lerp(state.visualConfidence,solution.confidence,0.26);
    const vv=estimateTranslation(solution,frame.w,frame.h,dt,frameQ,prevQ);state.translationSignal=vv;updatePose(vv,dt,now);
    state.previousFrameQ=frameQ;
  }
  state.previousFrame=frame;state.previousFrameTime=frameTime;state.frame=frame;
  state.processCount++;
  if(now-state.processStamp>=1000){state.processedFps=state.processCount*1000/(now-state.processStamp);state.processCount=0;state.processStamp=now;}
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
    if(state.hasGravityAcceleration)state.originGravitySamples.push({q:{...state.orientationQ},a:{...state.accelGravityDevice}});
    if(state.originQuaternionSamples.length>120)state.originQuaternionSamples.shift();
    if(state.originAccelSamples.length>120)state.originAccelSamples.shift();
    if(state.originGravitySamples.length>120)state.originGravitySamples.shift();
  }
  return quality;
}
function captureOrigin() {
  const qSamples=state.originQuaternionSamples.slice(-80),aSamples=state.originAccelSamples.slice(-80),gSamples=state.originGravitySamples.slice(-80);
  const averaged=qAverage(qSamples.length?qSamples:[state.orientationQ]);
  // World Y is gravity-up; only the initial horizontal heading is zeroed.
  // Pitch/roll are intentionally NOT baked into the world basis.
  state.baseQ=qAxis(0,1,0,cameraHeadingAngle(averaged));
  state.orientationCorrection=q();state.orientationHoldQ=null;
  if(aSamples.length&&state.hasLinearAcceleration){state.accelBiasDevice={x:median(aSamples.map(a=>a.x)),y:median(aSamples.map(a=>a.y)),z:median(aSamples.map(a=>a.z))};}
  if(gSamples.length){
    const worldG=gSamples.map(s=>qRotate(rawRelativeQ(s.q),s.a));
    state.gravityWorld={x:median(worldG.map(v=>v.x)),y:median(worldG.map(v=>v.y)),z:median(worldG.map(v=>v.z))};
  }
  state.previousFrameQ=state.orientationQ;state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.originCaptured=true;state.worldRevision++;
}
function maintainWorldOrientationLock() {
  if(!state.originCaptured||!state.baseQ)return;
  const raw=rawRelativeQ(state.orientationQ),corrected=qNorm(qMul(state.orientationCorrection,raw));
  const angular=Math.max(vecLength(state.gyro),state.orientationRate);
  if(state.stationary&&angular<0.028){
    if(!state.orientationHoldQ)state.orientationHoldQ=corrected;
    else state.orientationCorrection=qNorm(qMul(state.orientationHoldQ,qInv(raw)));
  } else state.orientationHoldQ=null;
}

function updateSetupGuidance(now,originQuality=0) {
  if(state.stage==='hold_still'){
    ui.stepTimer.textContent=`${Math.round(state.stillScore*100)}%`;
    ui.stepDetail.textContent=state.stillScore>0.65?'Origin is averaging now—keep holding normally.':'Natural hand tremor is accepted; avoid deliberate movement.';
  } else if(state.stage==='fov_sync'){
    ui.stepTimer.textContent=`${Math.round(state.focalConfidence*100)}%`;
    ui.stepDetail.textContent=`Turn left/right slowly. Camera↔IMU lag ${state.videoImuLagMs.toFixed(0)} ms • residual ${Number.isFinite(state.projectionError)?state.projectionError.toFixed(1)+' px':'—'}.`;
  } else if(state.stage==='xyz_lock'){
    ui.stepTimer.textContent=state.calib.metricCaptured?'SCALE LOCK':'0.50 m';
    ui.stepDetail.textContent='Keep the same heading. Move the phone exactly 0.50 m (19.7 in) to your RIGHT in one smooth motion, then stop.';
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
  const dt=clamp((now-state.lastSetupAt)/1000,0,0.1);state.lastSetupAt=now;maintainWorldOrientationLock();
  const motionFresh=now-state.lastMotionAt<500;
  if(state.stage==='hold_still'){
    const quality=originStability(now,dt);updateSetupGuidance(now,quality);
    state.poseReason=`Origin averaging ${Math.round(state.stillScore*100)}%; quality ${Math.round(state.originQuality*100)}%`;
    const softTimeout=now-state.calibrationStartedAt>3500&&state.originQuality>0.28;
    if((state.stillScore>=0.72||softTimeout)&&state.originQuaternionSamples.length>=10){
      captureOrigin();state.calibrationStartedAt=now;
      setStage('fov_sync','Origin captured at 0,0,0. Slowly look left and right so image motion can solve the visible camera FOV.',34);
      state.poseReason='Gravity-level origin captured; solving camera projection and camera-to-IMU timing offset';
    }
  } else if(state.stage==='fov_sync'){
    updateSetupGuidance(now);
    if(state.focalConfidence>0.50&&state.timingConfidence>0.28&&state.fovSamples.length>=9&&state.projectionError<4.2){
      state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.calibrationStartedAt=now;
      state.calib={visualPath:0,inertialPath:0,inertialVelocity:{x:0,y:0,z:0},rawVisualPosition:{x:0,y:0,z:0},lastPosition:{x:0,y:0,z:0},motionSeen:false,metricCaptured:false};
      setStage('xyz_lock','Projection and sensor timing are synchronized. Keep this heading, move the phone exactly 0.50 m (19.7 in) to your RIGHT, then stop.',64);
      state.poseReason='Projection/timing synchronized; calibrating 0.50 m metric translation on gravity-level world X';
    } else if(now-state.calibrationStartedAt>15000)state.poseReason='Projection needs slower rotation and visible contrast; continue left/right without translating';
  } else if(state.stage==='xyz_lock'){
    updateSetupGuidance(now);
    const moved=state.calib.metricCaptured;
    const qualified=moved&&state.visualConfidence>0.26&&state.validTracks>=8&&state.scaleStability>0.42&&state.motionConfidence>0.30;
    if(qualified&&state.stationary){
      state.scaleLocked=true;state.calibrationStartedAt=now;state.stillSince=now;
      setStage('settle_check','Translation has been connected to the same world transform. Hold naturally for the final no-creep check.',88);
      state.poseReason='Scale frozen; proving stationary position freeze before enabling the full lattice';
    }
  } else if(state.stage==='settle_check'){
    updateSetupGuidance(now);
    if(state.stationary&&state.stillSince&&now-state.stillSince>750&&state.driftRate<0.018){
      state.velocity={x:0,y:0,z:0};
      setStage('locked','Synchronization basis qualified. A world-centered 360° 3D lattice is active; walk/lean to verify XYZ translation before saving.',100,'locked');
      ui.status.textContent='POSE LOCKED';ui.save.disabled=true;ui.stress.disabled=false;
      state.poseReason='Gravity-level yaw basis, camera/IMU timing, epipolar translation direction, metric scale, and no-creep gates passed';
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
  ui.dTracks.textContent=String(state.validTracks);ui.dDrift.textContent=`${state.driftRate.toFixed(4)} u/s`;
  ui.dImuHz.textContent=state.imuHz.toFixed(1);ui.dVideoHz.textContent=state.videoHz.toFixed(1);ui.dReason.textContent=state.poseReason;
  ui.dProjectionError.textContent=Number.isFinite(state.projectionError)?`${state.projectionError.toFixed(2)} px`:'—';
  ui.dOriginQuality.textContent=`${Math.round(state.originQuality*100)}%`;ui.dGridMode.textContent=state.gridMode;
  if(ui.dVisualStep)ui.dVisualStep.textContent=state.visualStepMagnitude.toFixed(5);if(ui.dMoveGate)ui.dMoveGate.textContent=state.movementGate;
  if(ui.dTiming)ui.dTiming.textContent=`${state.videoImuLagMs.toFixed(0)} ms (${Math.round(state.timingConfidence*100)}%)`;
  if(ui.dWorldBasis)ui.dWorldBasis.textContent='gravity-level + start yaw';
  if(ui.dDirection)ui.dDirection.textContent=`${Math.round(state.translationDirectionConfidence*100)}%`;
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
function exportStressReport(){const blob=new Blob([JSON.stringify(stressReportObject(),null,2)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='cruxtain-xyz-stress-report-v2-5.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}

function renderLoop(now){setupMachine(now);updateStress(now);drawGrid();updateUI();requestAnimationFrame(renderLoop);}
function resize(){const r=grid.getBoundingClientRect(),d=Math.min(devicePixelRatio||1,2);grid.width=Math.max(1,Math.round(r.width*d));grid.height=Math.max(1,Math.round(r.height*d));if(state.fovX)state.fovY=2*Math.atan(Math.tan(state.fovX*DEG/2)*(r.height/Math.max(1,r.width)))/DEG;updateVisionCanvasSize();}

function basisObject() {
  return {
    version:2.5,savedAt:new Date().toISOString(),fovX:state.fovX,fovY:state.fovY,scale:state.scale,
    axisConvention:{x:'right from starting heading',y:'gravity up',z:'backward from starting heading; camera looks toward -Z'},cameraSettings:state.trackSettings,
    qualification:{visualConfidence:state.visualConfidence,motionConfidence:state.motionConfidence,scaleStability:state.scaleStability,stationaryDrift:state.driftRate,projectionResidualPx:state.projectionError,originQuality:state.originQuality,videoImuLagMs:state.videoImuLagMs,timingConfidence:state.timingConfidence,translationDirectionConfidence:state.translationDirectionConfidence,imuHz:state.imuHz,videoHz:state.videoHz,stress:stressReportObject()},
    note:'World Y is gravity-level, horizontal yaw is zeroed from the starting heading, and scale is calibrated from the 0.50 m translation step. Reload restores projection/timing/scale but, without persistent visual anchors, the current physical location becomes the reloaded origin.'
  };
}
function saveBasis(){localStorage.setItem(STORAGE_KEY,JSON.stringify(basisObject()));state.basisSaved=true;ui.load.disabled=false;ui.instruction.textContent='Basis saved locally after the 3D walk-around test. Reload restores projection, timing, and metric scale, then establishes the current physical location as origin.';}
function loadBasis(){
  const raw=localStorage.getItem(STORAGE_KEY)||LEGACY_STORAGE_KEYS.map(k=>localStorage.getItem(k)).find(Boolean);if(!raw)return;
  try{
    resetStress(); ui.stress.disabled=true;
    const b=JSON.parse(raw);if(Number(b.version)<2.5)throw new Error('This basis predates the gravity-level/metric tracking repair. Run the synchronization setup once to create a v2.5 basis.');
    state.fovX=clamp(b.fovX||62,34,100);state.fovY=clamp(b.fovY||48,20,100);state.scale=clamp(b.scale||1,.1,20);state.scaleLocked=true;
    state.scaleStability=b.qualification?.scaleStability||.5;state.projectionError=b.qualification?.projectionResidualPx??Infinity;state.videoImuLagMs=clamp(b.qualification?.videoImuLagMs??70,0,220);state.timingConfidence=b.qualification?.timingConfidence||0;
    state.baseQ=qAxis(0,1,0,cameraHeadingAngle(state.orientationQ));state.orientationCorrection=q();state.orientationHoldQ=null;state.originCaptured=true;state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.stillScore=0;state.stillSince=0;
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
