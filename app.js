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
  dFps: $('#dFps'), dScale: $('#dScale'), dQuality: $('#dQuality'), dTracks: $('#dTracks'), dMap: $('#dMap'), dPoseResidual: $('#dPoseResidual'), dDrift: $('#dDrift'),
  dImuHz: $('#dImuHz'), dVideoHz: $('#dVideoHz'), dReason: $('#dReason'), dProjectionError: $('#dProjectionError'),
  dOriginQuality: $('#dOriginQuality'), dGridMode: $('#dGridMode'), dVisualStep: $('#dVisualStep'), dMoveGate: $('#dMoveGate'), stepLabel: $('#stepLabel'), stepDetail: $('#stepDetail'), stepTimer: $('#stepTimer')
};

const STORAGE_KEY = 'cruxtain.xyzBasis.v2.5';
const LEGACY_STORAGE_KEY = 'cruxtain.xyzBasis.v2.4';
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
function relativeQ(absQ) {
  if (!state.baseQ) return q();
  return qNorm(qMul(qInv(state.baseQ), absQ));
}
function relativeCameraQ() { return relativeQ(state.orientationQ); }
function worldCameraQ() { return state.map?.initialized ? state.worldQ : relativeCameraQ(); }

function vAdd(a,b){return{x:a.x+b.x,y:a.y+b.y,z:a.z+b.z};}
function vSub(a,b){return{x:a.x-b.x,y:a.y-b.y,z:a.z-b.z};}
function vScale(a,s){return{x:a.x*s,y:a.y*s,z:a.z*s};}
function vDot(a,b){return a.x*b.x+a.y*b.y+a.z*b.z;}
function vCross(a,b){return{x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x};}
function vNorm(a){const n=vecLength(a)||1;return{x:a.x/n,y:a.y/n,z:a.z/n};}
function qFromRotVec(v){
  const a=vecLength(v);
  if(a<1e-10)return q(v.x*.5,v.y*.5,v.z*.5,1);
  const s=Math.sin(a*.5)/a;return qNorm(q(v.x*s,v.y*s,v.z*s,Math.cos(a*.5)));
}
function solveLinear(A,b){
  const n=b.length,M=A.map((r,i)=>[...r,b[i]]);
  for(let c=0;c<n;c++){
    let pivot=c;for(let r=c+1;r<n;r++)if(Math.abs(M[r][c])>Math.abs(M[pivot][c]))pivot=r;
    if(Math.abs(M[pivot][c])<1e-10)return null;
    [M[c],M[pivot]]=[M[pivot],M[c]];
    const d=M[c][c];for(let j=c;j<=n;j++)M[c][j]/=d;
    for(let r=0;r<n;r++){if(r===c)continue;const f=M[r][c];if(!f)continue;for(let j=c;j<=n;j++)M[r][j]-=f*M[c][j];}
  }
  return M.map(r=>r[n]);
}
function smallestEigenVector3(A){
  const M=A.map(r=>r.slice()),V=[[1,0,0],[0,1,0],[0,0,1]];
  for(let iter=0;iter<18;iter++){
    let p=0,qc=1,max=Math.abs(M[0][1]);
    for(const [i,j] of [[0,2],[1,2]]){const x=Math.abs(M[i][j]);if(x>max){max=x;p=i;qc=j;}}
    if(max<1e-10)break;
    const phi=.5*Math.atan2(2*M[p][qc],M[qc][qc]-M[p][p]),c=Math.cos(phi),sn=Math.sin(phi);
    const app=M[p][p],aqq=M[qc][qc],apq=M[p][qc];
    M[p][p]=c*c*app-2*sn*c*apq+sn*sn*aqq;M[qc][qc]=sn*sn*app+2*sn*c*apq+c*c*aqq;M[p][qc]=M[qc][p]=0;
    for(let k=0;k<3;k++)if(k!==p&&k!==qc){const mkp=M[k][p],mkq=M[k][qc];M[k][p]=M[p][k]=c*mkp-sn*mkq;M[k][qc]=M[qc][k]=sn*mkp+c*mkq;}
    for(let k=0;k<3;k++){const vkp=V[k][p],vkq=V[k][qc];V[k][p]=c*vkp-sn*vkq;V[k][qc]=sn*vkp+c*vkq;}
  }
  let idx=0;if(M[1][1]<M[idx][idx])idx=1;if(M[2][2]<M[idx][idx])idx=2;
  return vNorm({x:V[0][idx],y:V[1][idx],z:V[2][idx]});
}

const METRIC_BASELINE_M=0.3048; // 12 in / 30.48 cm. User-measured baseline fixes monocular scale.

const state = {
  stage: 'idle', stream: null, trackSettings: {},
  orientationQ: q(), baseQ: null, previousFrameQ: null, orientationSamples: [], lastOrientationAt: 0, orientationRate: 0,
  gyro: {x:0,y:0,z:0}, accelDevice: {x:0,y:0,z:0}, accelWorld: {x:0,y:0,z:0}, accelBiasDevice: {x:0,y:0,z:0},
  position: {x:0,y:0,z:0}, velocity: {x:0,y:0,z:0}, worldQ:q(), poseResidual:Infinity,
  fovX: 62, fovY: 48, fovSamples: [], focalConfidence: 0, projectionError: Infinity,
  scale: 1, scaleStability: 0, scaleSamples: [], scaleLocked: false,
  visualConfidence: 0, motionConfidence: 0, stationary: false, stationaryScore: 0, stillSince: 0, stillScore: 0, originQuality: 0,
  originQuaternionSamples: [], originAccelSamples: [], originCaptured: false,
  lastMotionAt: 0, lastFrameAt: 0, calibrationStartedAt: 0, lastSetupAt: performance.now(),
  imuCount: 0, imuHz: 0, imuStamp: performance.now(), videoCount: 0, videoHz: 0, videoStamp: performance.now(),
  processedFps: 0, processCount: 0, processStamp: performance.now(),
  frame: null, previousFrame: null, tracks: [], validTracks: 0, flowMagnitude: 0,
  translationSignal: {x:0,y:0,z:0,confidence:0,rawMagnitude:0}, visualStepMagnitude:0, movementGate:'still', lastMoveAt:0, driftRate: 0, lastPositionForDrift: {x:0,y:0,z:0},
  poseReason: 'Not started', loopStarted: false, lastProcessAt: 0, lastMediaTime: null, basisSaved: false, stageEnteredAt: performance.now(),
  calib: { visualPath:0, inertialPath:0, inertialVelocity:{x:0,y:0,z:0}, lastPosition:{x:0,y:0,z:0}, motionSeen:false },
  gridMode: 'off', worldRevision: 0,
  map:{initialized:false,frameSeq:0,landmarks:[],pending:[],calib:null,lastSensorQ:null,lastGoodAt:0,reprojection:Infinity,mapConfidence:0,nextId:1,trackingGood:false},
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
    ui.load.disabled = !(localStorage.getItem(STORAGE_KEY) || localStorage.getItem(LEGACY_STORAGE_KEY));
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
    state.velocity={x:0,y:0,z:0};state.previousFrame=null;state.lastMediaTime=null;
    state.poseReason='Paused while page is hidden';
  } else if (state.stage==='locked') {
    if(state.map.initialized){
      const sensorNow=relativeCameraQ();if(state.map.lastSensorQ){const d=qNorm(qMul(qInv(state.map.lastSensorQ),sensorNow));state.worldQ=qNorm(qMul(state.worldQ,d));}state.map.lastSensorQ=sensorNow;state.map.pending=[];for(const lm of state.map.landmarks)lm.lastSeenSeq=-1;
    }
    state.previousFrame=null;state.lastMediaTime=null;setStage('revalidating','Hold normally while the visual world map reacquires scene landmarks…',94);state.stillSince=0;state.stillScore=0;
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
  // Current Device Motion spec: alpha/beta/gamma are rates about device X/Y/Z.
  state.gyro={x:(rr.alpha||0)*DEG,y:(rr.beta||0)*DEG,z:(rr.gamma||0)*DEG};
  const a=e.acceleration||{};
  const hasLinear=[a.x,a.y,a.z].every(Number.isFinite);
  if(hasLinear){
    state.accelDevice={x:a.x,y:a.y,z:a.z};
    const rot=state.baseQ?relativeCameraQ():state.orientationQ;
    const corrected={x:state.accelDevice.x-state.accelBiasDevice.x,y:state.accelDevice.y-state.accelBiasDevice.y,z:state.accelDevice.z-state.accelBiasDevice.z};
    state.accelWorld=qRotate(rot,corrected);
  } else {
    const ag=e.accelerationIncludingGravity||{};
    if([ag.x,ag.y,ag.z].every(Number.isFinite)){
      const rot=state.baseQ?relativeCameraQ():state.orientationQ;
      const worldRaw=qRotate(rot,{x:ag.x,y:ag.y,z:ag.z});
      // Conservative gravity removal fallback. It is used as weak evidence only.
      worldRaw.y+=9.80665;
      state.accelWorld=worldRaw;
    }
  }
  state.lastMotionAt=now;
  state.imuCount++;
  if(now-state.imuStamp>=1000){state.imuHz=state.imuCount*1000/(now-state.imuStamp);state.imuCount=0;state.imuStamp=now;}
}

function beginSetup() {
  state.position={x:0,y:0,z:0}; state.velocity={x:0,y:0,z:0}; resetWorldMap();
  state.baseQ=null; state.previousFrameQ=null; state.previousFrame=null; state.frame=null; state.tracks=[];
  state.fovSamples=[]; state.scaleSamples=[]; state.scale=1; state.scaleLocked=false;
  state.focalConfidence=0; state.projectionError=Infinity; state.visualConfidence=0; state.motionConfidence=0; state.scaleStability=0;
  state.validTracks=0; state.flowMagnitude=0; state.stationaryScore=0; state.stillSince=0; state.stillScore=0; state.originQuality=0;
  state.originQuaternionSamples=[]; state.originAccelSamples=[]; state.originCaptured=false;
  state.accelBiasDevice={x:0,y:0,z:0}; state.calibrationStartedAt=performance.now(); state.lastSetupAt=performance.now();state.lastMediaTime=null;
  state.calib={visualPath:0,inertialPath:0,inertialVelocity:{x:0,y:0,z:0},lastPosition:{x:0,y:0,z:0},motionSeen:false};
  state.poseReason='Collecting a tolerant averaged origin; natural hand tremor is allowed';
  resetStress();
  ui.save.disabled=true; ui.stress.disabled=true;
  setStage('hold_still','Hold the phone normally. The origin uses a rolling average and does not require tripod-level stillness.',10);
}

function nearestOrientation(time) {
  if(!state.orientationSamples.length)return state.orientationQ;
  let best=state.orientationSamples[0],bestDt=Math.abs(best.t-time);
  for(let i=1;i<state.orientationSamples.length;i++){
    const d=Math.abs(state.orientationSamples[i].t-time);
    if(d<bestDt){best=state.orientationSamples[i];bestDt=d;}
  }
  return best.q;
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
function estimateTranslation(solution,w,h,dt,frameQ) {
  if(solution.inliers.length<8||dt<=0)return {x:0,y:0,z:0,confidence:0,rawMagnitude:0};
  const {fx,fy}=solution,cx=w/2,cy=h/2;
  // Residual flow after exact rotation compensation.  The result is an
  // arbitrary-scale camera displacement for THIS frame, not a damped velocity.
  const lateralX=median(solution.inliers.map(t=>-t.residual.x/fx));
  const lateralY=median(solution.inliers.map(t=> t.residual.y/fy));
  const zSamples=[];
  for(const t of solution.inliers){
    const nx=(t.p.x-cx)/fx,ny=-(t.p.y-cy)/fy,denom=nx*nx+ny*ny;
    if(denom<0.010)continue;
    const ux=t.residual.x/fx+lateralX;
    const uy=-t.residual.y/fy+lateralY;
    zSamples.push((ux*nx+uy*ny)/denom);
  }
  const radial=zSamples.length?median(zSamples):0;
  const localDelta={x:lateralX,y:lateralY,z:-radial};
  const worldDelta=qRotate(relativeQ(frameQ),localDelta);
  const rawMagnitude=vecLength(worldDelta);
  const spreadX=mad(solution.inliers.map(t=>t.residual.x));
  const spreadY=mad(solution.inliers.map(t=>t.residual.y));
  const geometric=clamp(1-(spreadX+spreadY)/(Math.max(w,h)*0.055),0.25,1);
  return {...worldDelta,confidence:solution.confidence*geometric,rawMagnitude};
}


function intrinsicsFor(w,h){
  const fx=.5*w/Math.tan(state.fovX*DEG/2);
  const vfov=2*Math.atan(Math.tan(state.fovX*DEG/2)*(h/w));
  const fy=.5*h/Math.tan(vfov/2);
  return {fx,fy,cx:w/2,cy:h/2};
}
function cameraRayWorld(pixel,camQ,w,h){
  const {fx,fy,cx,cy}=intrinsicsFor(w,h);
  return vNorm(qRotate(camQ,{x:(pixel.x-cx)/fx,y:-(pixel.y-cy)/fy,z:-1}));
}
function projectWorldPixel(world,camQ,camPos,w,h){
  const {fx,fy,cx,cy}=intrinsicsFor(w,h),c=cameraPoint(world,camQ,camPos);
  if(c.z>=-0.04)return null;
  return {x:cx+fx*(c.x/-c.z),y:cy-fy*(c.y/-c.z),depth:-c.z};
}
function triangulateRays(c0,q0,p0,c1,q1,p1,w,h){
  const d0=cameraRayWorld(p0,q0,w,h),d1=cameraRayWorld(p1,q1,w,h),w0=vSub(c0,c1);
  const a=vDot(d0,d0),b=vDot(d0,d1),c=vDot(d1,d1),d=vDot(d0,w0),e=vDot(d1,w0),den=a*c-b*b;
  if(Math.abs(den)<1e-7)return null;
  const s=(b*e-c*d)/den,t=(a*e-b*d)/den,pA=vAdd(c0,vScale(d0,s)),pB=vAdd(c1,vScale(d1,t)),world=vScale(vAdd(pA,pB),.5);
  return {world,s,t,gap:vecLength(vSub(pA,pB)),angle:Math.acos(clamp(vDot(d0,d1),-1,1))};
}
function estimateBaselineDirection(tracks,startQ,endQ,w,h){
  const normals=[];
  for(const t of tracks){const d0=cameraRayWorld(t.start,startQ,w,h),d1=cameraRayWorld(t.p,endQ,w,h),n=vCross(d0,d1),nl=vecLength(n);if(nl>1e-5)normals.push(vScale(n,1/nl));}
  if(normals.length<8)return null;
  const solve=(set)=>{const A=[[0,0,0],[0,0,0],[0,0,0]];for(const u of set){const v=[u.x,u.y,u.z];for(let r=0;r<3;r++)for(let c=0;c<3;c++)A[r][c]+=v[r]*v[c];}return smallestEigenVector3(A);};
  let dir=solve(normals),errs=normals.map(n=>Math.abs(vDot(n,dir))),m=median(errs),spread=mad(errs,m),limit=Math.max(.012,m+2.6*spread),inliers=normals.filter((n,i)=>errs[i]<=limit);if(inliers.length>=8)dir=solve(inliers);return dir;
}

function extractTemplate(frame,p,r=3){
  const {gray,w,h}=frame,x=Math.round(p.x),y=Math.round(p.y);if(x-r<0||x+r>=w||y-r<0||y+r>=h)return null;
  const data=new Uint8Array((2*r+1)*(2*r+1));let k=0,sum=0;
  for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++){const v=gray[(y+yy)*w+x+xx];data[k++]=v;sum+=v;}
  return {data,r,mean:sum/data.length};
}
function templateSSD(template,frame,x,y){
  if(!template)return Infinity;const {gray,w,h}=frame,r=template.r,xi=Math.round(x),yi=Math.round(y);if(xi-r<0||xi+r>=w||yi-r<0||yi+r>=h)return Infinity;
  let sum=0,k=0,mean=0,n=template.data.length;for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++)mean+=gray[(yi+yy)*w+xi+xx];mean/=n;
  for(let yy=-r;yy<=r;yy++)for(let xx=-r;xx<=r;xx++){const a=template.data[k++]-template.mean,b=gray[(yi+yy)*w+xi+xx]-mean,d=a-b;sum+=d*d;}
  return sum/n;
}
function searchTemplate(template,frame,center,search=8){
  let best={score:Infinity,x:center.x,y:center.y},second=Infinity;
  for(let dy=-search;dy<=search;dy+=2)for(let dx=-search;dx<=search;dx+=2){const score=templateSSD(template,frame,center.x+dx,center.y+dy);if(score<best.score){second=best.score;best={score,x:center.x+dx,y:center.y+dy};}else if(score<second)second=score;}
  if(!Number.isFinite(best.score))return null;const uniqueness=clamp((second-best.score)/(second+1e-6),0,1);return {...best,confidence:clamp(uniqueness*2.5,0,1)*clamp((1700-best.score)/1500,0,1)};
}
function qBlend(a,b,t){
  let bx=b.x,by=b.y,bz=b.z,bw=b.w,d=a.x*bx+a.y*by+a.z*bz+a.w*bw;if(d<0){bx=-bx;by=-by;bz=-bz;bw=-bw;d=-d;}
  if(d>.9995)return qNorm(q(lerp(a.x,bx,t),lerp(a.y,by,t),lerp(a.z,bz,t),lerp(a.w,bw,t)));
  const th=Math.acos(clamp(d,-1,1)),sn=Math.sin(th),wa=Math.sin((1-t)*th)/sn,wb=Math.sin(t*th)/sn;return qNorm(q(a.x*wa+bx*wb,a.y*wa+by*wb,a.z*wa+bz*wb,a.w*wa+bw*wb));
}
function refinePose(initialPos,initialQ,observations,w,h,iterations=5){
  let pos={...initialPos},camQ=qNorm(initialQ);const EPSP=.001,EPSR=.001;
  const project=(world,p,qv)=>projectWorldPixel(world,qv,p,w,h);
  for(let iter=0;iter<iterations;iter++){
    const H=Array.from({length:6},()=>Array(6).fill(0)),g=Array(6).fill(0);let used=0;
    for(const o of observations){
      const base=project(o.world,pos,camQ);if(!base)continue;const rx=o.pixel.x-base.x,ry=o.pixel.y-base.y,err=Math.hypot(rx,ry),robust=err<=4?1:4/Math.max(err,1e-3),wt=Math.max(.08,o.confidence||1)*robust,J=Array.from({length:2},()=>Array(6).fill(0));
      for(let j=0;j<6;j++){
        let pp,pq=camQ,eps=j<3?EPSP:EPSR;
        if(j<3){pp={...pos};if(j===0)pp.x+=eps;if(j===1)pp.y+=eps;if(j===2)pp.z+=eps;}
        else{pp=pos;const rv={x:j===3?eps:0,y:j===4?eps:0,z:j===5?eps:0};pq=qNorm(qMul(camQ,qFromRotVec(rv)));}
        const p2=project(o.world,pp,pq);if(!p2)continue;J[0][j]=(p2.x-base.x)/eps;J[1][j]=(p2.y-base.y)/eps;
      }
      for(let r=0;r<6;r++){g[r]+=wt*(J[0][r]*rx+J[1][r]*ry);for(let c=0;c<6;c++)H[r][c]+=wt*(J[0][r]*J[0][c]+J[1][r]*J[1][c]);}
      used++;
    }
    if(used<6)return null;for(let i=0;i<6;i++)H[i][i]+=1e-5;const d=solveLinear(H,g);if(!d)return null;
    let dp={x:d[0],y:d[1],z:d[2]},dr={x:d[3],y:d[4],z:d[5]},pl=vecLength(dp),rl=vecLength(dr);if(pl>.18)dp=vScale(dp,.18/pl);if(rl>.10)dr=vScale(dr,.10/rl);
    pos=vAdd(pos,dp);camQ=qNorm(qMul(camQ,qFromRotVec(dr)));if(vecLength(dp)<.00025&&vecLength(dr)<.00015)break;
  }
  const scored=[];for(const o of observations){const p=project(o.world,pos,camQ);if(p)scored.push({...o,error:Math.hypot(o.pixel.x-p.x,o.pixel.y-p.y)});}
  if(scored.length<6)return null;const med=median(scored.map(o=>o.error)),spread=mad(scored.map(o=>o.error),med),limit=Math.max(2.8,Math.min(9,med+3*spread)),inliers=scored.filter(o=>o.error<=limit);
  if(inliers.length<6)return null;const residual=median(inliers.map(o=>o.error));return {pos,camQ,inliers,residual,confidence:clamp(inliers.length/32,0,1)*clamp(1-residual/8,0,1)};
}
function resetWorldMap(){
  state.map={initialized:false,frameSeq:0,landmarks:[],pending:[],calib:null,lastSensorQ:null,lastGoodAt:0,reprojection:Infinity,mapConfidence:0,nextId:1,trackingGood:false};state.worldQ=q();state.poseResidual=Infinity;
}
function beginMetricCalibration(){
  resetWorldMap();const frame=state.frame;if(!frame)return false;const startQ=state.previousFrameQ?relativeQ(state.previousFrameQ):relativeCameraQ(),pts=selectCorners(frame,125).filter(p=>p.x>8&&p.y>8&&p.x<frame.w-8&&p.y<frame.h-8);
  state.map.calib={startQ,startAt:performance.now(),tracks:pts.map(p=>({id:state.map.nextId++,start:{x:p.x,y:p.y},p:{x:p.x,y:p.y},confidence:1})),parallax:0,moved:false,lastQ:startQ};state.worldQ=startQ;state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.scale=1;state.scaleLocked=false;state.scaleStability=0;return pts.length>=16;
}
function updateMetricCalibration(prevFrame,currFrame,currSensorQ){
  const cal=state.map.calib;if(!cal)return;const next=[];
  for(const t of cal.tracks){const m=trackPoint(prevFrame.gray,currFrame.gray,currFrame.w,currFrame.h,t.p,14);if(m.confidence<.08||m.fb>3.0||!Number.isFinite(m.score))continue;next.push({...t,p:{x:m.x,y:m.y},confidence:Math.min(t.confidence,m.confidence)});}
  cal.tracks=next;cal.lastQ=currSensorQ;
  const {fx,fy,cx,cy}=intrinsicsFor(currFrame.w,currFrame.h),res=[];
  for(const t of next){const pr=predictedRotatedPixel(t.start,cal.startQ,currSensorQ,fx,fy,cx,cy);if(pr)res.push(Math.hypot(t.p.x-pr.x,t.p.y-pr.y));}
  cal.parallax=res.length?median(res):0;if(cal.parallax>5)cal.moved=true;state.scaleStability=clamp(next.length/45,0,.55)*clamp(cal.parallax/10,0,1);
}
function finalizeMetricCalibration(frame){
  const cal=state.map.calib;if(!cal||cal.tracks.length<14)return false;let dir=estimateBaselineDirection(cal.tracks,cal.startQ,cal.lastQ,frame.w,frame.h);if(!dir)return false;
  const evaluate=(sgn)=>{const end=vScale(dir,METRIC_BASELINE_M*sgn),landmarks=[];for(const t of cal.tracks){const tri=triangulateRays({x:0,y:0,z:0},cal.startQ,t.start,end,cal.lastQ,t.p,frame.w,frame.h);if(!tri)continue;const depth=Math.min(tri.s,tri.t);if(tri.s<=.2||tri.t<=.2||depth>35||tri.angle<.006||tri.gap>Math.max(.08,depth*.018))continue;landmarks.push({id:t.id,world:tri.world,p:{...t.p},template:extractTemplate(frame,t.p),lastSeenSeq:0,misses:0,quality:clamp(1-tri.gap/.12,0,1)});}return{end,landmarks};};
  const a=evaluate(1),b=evaluate(-1),best=a.landmarks.length>=b.landmarks.length?a:b;if(best.landmarks.length<12)return false;
  state.map.initialized=true;state.map.landmarks=best.landmarks;state.map.pending=[];state.map.frameSeq=0;state.map.lastSensorQ=cal.lastQ;state.map.lastGoodAt=performance.now();state.map.mapConfidence=clamp(best.landmarks.length/45,0,1);state.map.reprojection=0;state.map.trackingGood=true;state.position={...best.end};state.lastPositionForDrift={...best.end};state.driftRate=0;state.worldQ=cal.lastQ;state.velocity={x:0,y:0,z:0};state.scale=1;state.scaleLocked=true;state.scaleStability=clamp(.45+.55*state.map.mapConfidence,0,1);state.poseResidual=0;state.worldRevision++;return true;
}
function addPendingFeatures(frame){
  const map=state.map;if(!map.initialized||map.mapConfidence<.18||map.pending.length>70)return;const occupied=[];for(const lm of map.landmarks)if(lm.p&&lm.misses<2)occupied.push(lm.p);for(const p of map.pending)occupied.push(p.p);
  const candidates=selectCorners(frame,80).filter(p=>p.x>9&&p.y>9&&p.x<frame.w-9&&p.y<frame.h-9).filter(p=>occupied.every(o=>Math.hypot(p.x-o.x,p.y-o.y)>11)).slice(0,20);
  for(const p of candidates)map.pending.push({id:map.nextId++,anchorPos:{...state.position},anchorQ:state.worldQ,anchorPixel:{x:p.x,y:p.y},p:{x:p.x,y:p.y},lastSeenSeq:map.frameSeq,age:0});
}
function updatePendingFeatures(prevFrame,currFrame){
  const map=state.map,next=[];for(const f of map.pending){if(f.lastSeenSeq!==map.frameSeq-1)continue;const m=trackPoint(prevFrame.gray,currFrame.gray,currFrame.w,currFrame.h,f.p,12);if(m.confidence<.09||m.fb>3)continue;f.p={x:m.x,y:m.y};f.lastSeenSeq=map.frameSeq;f.age++;const baseline=vecLength(vSub(state.position,f.anchorPos));if(baseline>.065&&f.age>=2){const tri=triangulateRays(f.anchorPos,f.anchorQ,f.anchorPixel,state.position,state.worldQ,f.p,currFrame.w,currFrame.h);if(tri&&tri.s>.2&&tri.t>.2&&Math.min(tri.s,tri.t)<35&&tri.angle>.0045&&tri.gap<Math.max(.09,Math.min(tri.s,tri.t)*.02)){map.landmarks.push({id:f.id,world:tri.world,p:{...f.p},template:extractTemplate(currFrame,f.p),lastSeenSeq:map.frameSeq,misses:0,quality:clamp(1-tri.gap/.13,0,1)});continue;}}next.push(f);}map.pending=next.slice(-75);
}
function updateWorldMap(prevFrame,currFrame,currSensorQ,dt,now){
  const map=state.map;if(!map.initialized)return false;map.frameSeq++;const seq=map.frameSeq;
  if(map.lastSensorQ){const sd=qNorm(qMul(qInv(map.lastSensorQ),currSensorQ)),ang=qAngle(q(),sd);if(ang<.35)state.worldQ=qNorm(qMul(state.worldQ,sd));}map.lastSensorQ=currSensorQ;
  const predictedPos={x:state.position.x+state.velocity.x*dt,y:state.position.y+state.velocity.y*dt,z:state.position.z+state.velocity.z*dt},observations=[];
  for(const lm of map.landmarks){
    let pixel=null,conf=.2;
    if(lm.p&&lm.lastSeenSeq===seq-1){const t=trackPoint(prevFrame.gray,currFrame.gray,currFrame.w,currFrame.h,lm.p,11);if(t.confidence>.08&&t.fb<3){pixel={x:t.x,y:t.y};conf=t.confidence;}}
    if(!pixel&&lm.template){const pred=projectWorldPixel(lm.world,state.worldQ,predictedPos,currFrame.w,currFrame.h);if(pred&&pred.x>8&&pred.y>8&&pred.x<currFrame.w-8&&pred.y<currFrame.h-8){const searchRadius=Math.round(clamp(9+(now-map.lastGoodAt)/90,9,28)),m=searchTemplate(lm.template,currFrame,pred,searchRadius);if(m&&m.confidence>.10&&m.score<1500){pixel={x:m.x,y:m.y};conf=m.confidence*.7;}}}
    if(pixel){const pred=projectWorldPixel(lm.world,state.worldQ,predictedPos,currFrame.w,currFrame.h);if(!pred||Math.hypot(pixel.x-pred.x,pixel.y-pred.y)<55)observations.push({id:lm.id,world:lm.world,pixel,confidence:conf});}
  }
  const solved=observations.length>=7?refinePose(predictedPos,state.worldQ,observations,currFrame.w,currFrame.h,5):null;
  if(solved&&solved.inliers.length>=7&&solved.residual<8.5&&vecLength(vSub(solved.pos,predictedPos))<0.35&&qAngle(state.worldQ,solved.camQ)<0.24){
    const prevPos={...state.position},blend=solved.residual<3.5?.94:.82;state.position={x:lerp(state.position.x,solved.pos.x,blend),y:lerp(state.position.y,solved.pos.y,blend),z:lerp(state.position.z,solved.pos.z,blend)};state.worldQ=qBlend(state.worldQ,solved.camQ,blend);const dp=vSub(state.position,prevPos);state.velocity={x:dp.x/Math.max(dt,.01),y:dp.y/Math.max(dt,.01),z:dp.z/Math.max(dt,.01)};
    const ids=new Map(solved.inliers.map(o=>[o.id,o]));for(const lm of map.landmarks){const o=ids.get(lm.id);if(o){lm.p={...o.pixel};lm.lastSeenSeq=seq;lm.misses=0;if(seq%18===0)lm.template=extractTemplate(currFrame,lm.p)||lm.template;}else lm.misses=(lm.misses||0)+1;}
    map.reprojection=solved.residual;map.mapConfidence=lerp(map.mapConfidence,solved.confidence,.28);map.lastGoodAt=now;map.trackingGood=true;state.poseResidual=solved.residual;state.visualConfidence=lerp(state.visualConfidence,Math.max(solved.confidence,.15),.32);state.validTracks=solved.inliers.length;state.translationSignal={...dp,confidence:solved.confidence,rawMagnitude:vecLength(dp)};state.visualStepMagnitude=lerp(state.visualStepMagnitude,vecLength(dp),.3);
    updatePendingFeatures(prevFrame,currFrame);if(seq%8===0)addPendingFeatures(currFrame);if(map.landmarks.length>220)map.landmarks=map.landmarks.sort((a,b)=>(a.misses||0)-(b.misses||0)).slice(0,220);return true;
  }
  map.mapConfidence*=.90;map.trackingGood=false;state.poseResidual=Infinity;state.validTracks=observations.length;state.visualConfidence*=.92;if(now-map.lastGoodAt<220){state.position.x+=state.velocity.x*dt;state.position.y+=state.velocity.y*dt;state.position.z+=state.velocity.z*dt;}state.velocity.x*=.35;state.velocity.y*=.35;state.velocity.z*=.35;return false;
}

function updateScaleCalibration(rawVisualVelocity,dt) {
  if(state.stage!=='xyz_lock'||state.map.initialized)return;
  const visualStep=rawVisualVelocity.rawMagnitude||0;
  if(visualStep<0.20)state.calib.visualPath+=visualStep;
  state.scale=1; // Metric scale is established only by the measured 12-inch baseline.
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
  const deliberate=deliberateVisual||deliberateInertial;
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
  // v2.5 never integrates monocular flow or acceleration into authoritative XYZ.
  // Position changes only after the measured baseline creates a 3D map and PnP re-solves the pose.

  const maxSpeed=5.0,speed=vecLength(state.velocity);
  if(speed>maxSpeed){const k=maxSpeed/speed;state.velocity.x*=k;state.velocity.y*=k;state.velocity.z*=k;}
  if(!deliberate&&!state.stationary){state.velocity.x*=0.88;state.velocity.y*=0.88;state.velocity.z*=0.88;}

  const mapEvidence=state.map.initialized?state.map.mapConfidence:0;
  state.motionConfidence=clamp(0.58*state.visualConfidence+0.16*(motionFresh?1:0.55)+0.16*mapEvidence+0.10*(state.stationary?1:0.8),0,1);
  const p=state.position,lp=state.lastPositionForDrift;
  if(state.stationary){const drift=Math.hypot(p.x-lp.x,p.y-lp.y,p.z-lp.z)/Math.max(dt,1e-3);state.driftRate=lerp(state.driftRate,drift,0.10);}else state.driftRate*=0.96;
  state.lastPositionForDrift={...p};
}

function processVideoFrame(now,meta={}) {
  if(now-state.lastProcessAt<45)return;
  const mediaTime=Number.isFinite(meta.mediaTime)?meta.mediaTime:null,dt=clamp(mediaTime!=null&&state.lastMediaTime!=null?mediaTime-state.lastMediaTime:(now-(state.lastProcessAt||now))/1000,0.01,0.12);state.lastProcessAt=now;if(mediaTime!=null)state.lastMediaTime=mediaTime;
  const frame=captureGray();if(!frame)return;
  const poseTime=Number.isFinite(meta.captureTime)?meta.captureTime:(Number.isFinite(meta.presentationTime)?meta.presentationTime:now),frameQ=nearestOrientation(poseTime),sensorRelQ=state.baseQ?relativeQ(frameQ):q();
  if(state.previousFrame&&state.previousFrameQ){
    const raw=qualityTracks(state.previousFrame,frame),rawFlow=raw.length?median(raw.map(t=>Math.hypot(t.observed.x,t.observed.y))):0;
    state.flowMagnitude=lerp(state.flowMagnitude,rawFlow,0.25);
    if(state.stage==='fov_sync')estimateFovFromTracks(raw,state.previousFrameQ,frameQ,frame.w,frame.h);
    const solution=residualSolution(raw,state.previousFrameQ,frameQ,frame.w,frame.h,state.fovX);
    state.tracks=solution.inliers;
    if(!state.map.initialized){state.validTracks=solution.inliers.length;state.visualConfidence=lerp(state.visualConfidence,solution.confidence,0.26);}
    const vv=estimateTranslation(solution,frame.w,frame.h,dt,frameQ);state.translationSignal=vv;
    if(state.map.initialized&&(state.stage==='settle_check'||state.stage==='locked'||state.stage==='revalidating')){
      updateWorldMap(state.previousFrame,frame,sensorRelQ,dt,now);
    }
    updatePose(vv,dt,now);
    if(state.stage==='xyz_lock'&&state.map.calib&&!state.map.initialized){
      updateMetricCalibration(state.previousFrame,frame,sensorRelQ);
    }
  }
  state.previousFrame=frame;state.previousFrameQ=frameQ;state.frame=frame;
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
    if(state.originQuaternionSamples.length>120)state.originQuaternionSamples.shift();
    if(state.originAccelSamples.length>120)state.originAccelSamples.shift();
  }
  return quality;
}
function captureOrigin() {
  const qSamples=state.originQuaternionSamples.slice(-80),aSamples=state.originAccelSamples.slice(-80);
  state.baseQ=qAverage(qSamples.length?qSamples:[state.orientationQ]);
  if(aSamples.length){
    state.accelBiasDevice={x:median(aSamples.map(a=>a.x)),y:median(aSamples.map(a=>a.y)),z:median(aSamples.map(a=>a.z))};
  }
  state.previousFrame=null;state.frame=null;state.previousFrameQ=null;state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.worldQ=q();resetWorldMap();state.originCaptured=true;state.worldRevision++;
}

function updateSetupGuidance(now,originQuality=0) {
  if(state.stage==='hold_still'){
    ui.stepTimer.textContent=`${Math.round(state.stillScore*100)}%`;
    ui.stepDetail.textContent=state.stillScore>0.65?'Origin is averaging now—keep holding normally.':'Natural hand tremor is accepted; avoid deliberate movement.';
  } else if(state.stage==='fov_sync'){
    ui.stepTimer.textContent=`${Math.round(state.focalConfidence*100)}%`;
    ui.stepDetail.textContent=`Turn left and right slowly. Projection residual ${Number.isFinite(state.projectionError)?state.projectionError.toFixed(1)+' px':'—'}.`;
  } else if(state.stage==='xyz_lock'){
    const cal=state.map.calib;ui.stepTimer.textContent=cal?`${Math.round(cal.parallax||0)} px`:'ARMING';
    ui.stepDetail.textContent=cal?.moved?`Now stop and hold. ${cal.tracks.length} baseline features remain.`:'Move the PHONE sideways exactly 12 in / 30.48 cm, keeping the same scene visible, then stop.';
  } else if(state.stage==='settle_check'){
    const elapsed=state.stillSince?now-state.stillSince:0,remaining=Math.max(0,0.75-elapsed/1000);
    ui.stepTimer.textContent=state.stationary?`${remaining.toFixed(1)}s`:'WAITING';
    ui.stepDetail.textContent=state.stationary?`World map is holding ${state.validTracks} landmarks; keep holding.`:'Stop naturally; the landmark-anchored no-creep check begins automatically.';
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
      state.calib={visualPath:0,inertialPath:0,inertialVelocity:{x:0,y:0,z:0},lastPosition:{x:0,y:0,z:0},motionSeen:false};
      beginMetricCalibration();
      setStage('xyz_lock','Metric world lock: move the PHONE sideways exactly 12 in / 30.48 cm while keeping the same scene visible, then stop.',64);
      state.poseReason='Projection synchronized; collecting a known physical baseline to triangulate persistent 3D landmarks';
    } else if(now-state.calibrationStartedAt>15000)state.poseReason='Projection needs slower rotation and visible contrast; continue left/right without translating';
  } else if(state.stage==='xyz_lock'){
    if(!state.map.calib&&state.frame)beginMetricCalibration();
    updateSetupGuidance(now);
    const cal=state.map.calib;
    if(cal&&cal.moved&&cal.tracks.length>=14&&cal.parallax>5&&state.stationary&&state.stillSince&&now-state.stillSince>450){
      if(finalizeMetricCalibration(state.frame)){
        state.calibrationStartedAt=now;state.stillSince=now;
        setStage('settle_check','Metric scale and 3D landmarks are locked. Hold naturally for the final landmark-anchored no-creep check.',88);
        state.poseReason=`Metric baseline accepted; ${state.map.landmarks.length} world landmarks triangulated at real scale`;
      } else state.poseReason='Baseline was visible but triangulation was weak. Keep the scene in view and repeat the 12-inch sideways move.';
    } else if(cal&&cal.tracks.length<10){
      state.poseReason='Too many baseline features were lost. Return near the start view and hold so the 12-inch calibration can re-arm.';
      if(state.stationary&&now-cal.startAt>1200)beginMetricCalibration();
    }
  } else if(state.stage==='settle_check'){
    updateSetupGuidance(now);
    if(state.stationary&&state.stillSince&&now-state.stillSince>750&&state.driftRate<0.018){
      state.velocity={x:0,y:0,z:0};
      setStage('locked','Metric world lock qualified. The 360° lattice is anchored to triangulated scene landmarks; walk around and verify real-to-virtual motion before saving.',100,'locked');
      ui.status.textContent='POSE LOCKED';ui.save.disabled=true;ui.stress.disabled=false;
      state.poseReason=`Metric world lock passed: ${state.map.landmarks.length} mapped landmarks, ${state.validTracks} currently observed`;
    }
  } else if(state.stage==='revalidating'){
    updateSetupGuidance(now);
    if(state.map.initialized&&state.map.trackingGood&&state.validTracks>=6&&state.stationary&&state.stillSince&&now-state.stillSince>550){state.velocity={x:0,y:0,z:0};const resumeStress=state.stress.active?state.stress.index:-1;setStage('locked','Visual world map reacquired. The existing metric origin and lattice remain unchanged.',100,'locked');ui.stress.disabled=false;ui.save.disabled=!state.stress.complete;if(resumeStress>=0)enterStressTest(resumeStress);}
  }

  if(state.stage==='locked'){
    updateSetupGuidance(now);
    if((!motionFresh&&now-state.lastOrientationAt>500)||state.validTracks<6||state.map.mapConfidence<0.10){ui.status.dataset.state='lost';ui.status.textContent='POSE WEAK';state.poseReason=!motionFresh?'Motion stream stale':`World-map lock weak: ${state.validTracks} landmarks visible`;}
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
  const fx=.5*W/Math.tan(state.fovX*DEG/2),fy=.5*H/Math.tan(state.fovY*DEG/2),cx=W/2,cy=H/2,camQ=worldCameraQ(),camPos=state.position;
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
  ui.dScale.textContent=state.map.initialized?`metric • 12 in baseline (${Math.round(state.scaleStability*100)}%)`:`awaiting 12 in baseline`;
  ui.dQuality.textContent=state.stage==='locked'?(state.map.mapConfidence>.24&&state.validTracks>=8?'world-locked':'limited'):'unqualified';
  ui.dTracks.textContent=String(state.validTracks);if(ui.dMap)ui.dMap.textContent=state.map.initialized?`${state.map.landmarks.length} mapped / ${Math.round(state.map.mapConfidence*100)}%`:'not built';if(ui.dPoseResidual)ui.dPoseResidual.textContent=Number.isFinite(state.poseResidual)?`${state.poseResidual.toFixed(2)} px`:'—';ui.dDrift.textContent=`${state.driftRate.toFixed(4)} m/s`;
  ui.dImuHz.textContent=state.imuHz.toFixed(1);ui.dVideoHz.textContent=state.videoHz.toFixed(1);ui.dReason.textContent=state.poseReason;
  ui.dProjectionError.textContent=Number.isFinite(state.projectionError)?`${state.projectionError.toFixed(2)} px`:'—';
  ui.dOriginQuality.textContent=`${Math.round(state.originQuality*100)}%`;ui.dGridMode.textContent=state.gridMode;
  if(ui.dVisualStep)ui.dVisualStep.textContent=state.visualStepMagnitude.toFixed(5);if(ui.dMoveGate)ui.dMoveGate.textContent=state.movementGate;
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
  s.index=index;s.stageStartedAt=now;s.testStartPos=clonePos(state.position);s.testStartQ=worldCameraQ();s.lastPos=clonePos(state.position);s.lastYaw=cameraYaw(s.testStartQ);s.yawTravel=0;s.pathLength=0;s.maxDisplacement=0;s.maxAxis=0;s.maxOffAxis=0;s.maxDriftRate=0;s.stableSince=0;s.movementSeen=false;s.manual=false;
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
    const yaw=cameraYaw(worldCameraQ()),dy=Math.abs(wrapPi(yaw-s.lastYaw));s.lastYaw=yaw;if(dy<0.45)s.yawTravel+=dy;const deg=s.yawTravel/DEG,closureDeg=qAngle(s.testStartQ,worldCameraQ())/DEG;ui.stepTimer.textContent=`${Math.min(360,Math.round(deg))}°`;ui.stepDetail.textContent=`Complete 360° • position leak ${s.maxDisplacement.toFixed(3)} m • heading closure ${closureDeg.toFixed(1)}°`;
    if(deg>=330&&closureDeg<18&&state.orientationRate<0.18){const pass=closureDeg<12&&s.maxDisplacement<0.12;finishStressResult(pass,{yawTravelDeg:deg,orientationClosureDeg:closureDeg,maxPositionLeak:s.maxDisplacement,finalPositionLeak:dist},pass?'Rotation stayed separated from XYZ and closed near its starting attitude.':'360° rotation produced excessive orientation closure error or false translation.');}else if(elapsed>28&&deg>250){const pass=closureDeg<15&&s.maxDisplacement<0.12;finishStressResult(pass,{yawTravelDeg:deg,orientationClosureDeg:closureDeg,maxPositionLeak:s.maxDisplacement,finalPositionLeak:dist},'Timed completion after a near-full rotation.');}
    return;
  }
  if(t.kind==='axis'){
    const axis=Math.abs(d[t.axis]),off=Math.hypot(...['x','y','z'].filter(a=>a!==t.axis).map(a=>d[a]));s.maxAxis=Math.max(s.maxAxis,axis);s.maxOffAxis=Math.max(s.maxOffAxis,off);if(s.maxAxis>t.threshold)s.movementSeen=true;const closure=dist,returnThreshold=Math.max(0.055,s.maxAxis*0.30);if(s.movementSeen&&closure<returnThreshold&&state.stationary){if(!s.stableSince)s.stableSince=now;}else s.stableSince=0;const purity=s.maxAxis? s.maxOffAxis/s.maxAxis:Infinity;ui.stepTimer.textContent=`${Math.round(clamp(s.maxAxis/t.threshold,0,1)*100)}%`;ui.stepDetail.textContent=`${t.axis.toUpperCase()} excursion ${s.maxAxis.toFixed(3)} m • off-axis ratio ${Number.isFinite(purity)?purity.toFixed(2):'—'} • closure ${closure.toFixed(3)} m`;
    if(s.stableSince&&now-s.stableSince>700){const closureRatio=s.maxAxis?closure/s.maxAxis:Infinity,pass=s.maxAxis>t.threshold&&purity<0.65&&closureRatio<0.35;finishStressResult(pass,{axis:t.axis,maxAxis:s.maxAxis,maxOffAxis:s.maxOffAxis,axisPurity:purity,closure,closureRatio,pathLength:s.pathLength},pass?'Dominant motion stayed on the expected world axis and returned near the start.':'Axis cross-talk or return closure exceeded the qualification threshold.');}
    return;
  }
  if(t.kind==='mixed'){
    if(s.maxDisplacement>0.15&&s.pathLength>0.45)s.movementSeen=true;const closure=dist,closureRatio=s.maxDisplacement?closure/s.maxDisplacement:Infinity,rotClosure=qAngle(s.testStartQ,worldCameraQ())/DEG;if(s.movementSeen&&closure<Math.max(0.08,s.maxDisplacement*0.35)&&state.stationary){if(!s.stableSince)s.stableSince=now;}else s.stableSince=0;ui.stepTimer.textContent=`${s.movementSeen?'RETURN':'MOVE'}`;ui.stepDetail.textContent=`Path ${s.pathLength.toFixed(2)} m • excursion ${s.maxDisplacement.toFixed(2)} m • closure ${closure.toFixed(3)} m`;
    if(s.stableSince&&now-s.stableSince>800){const pass=closureRatio<0.35&&s.pathLength>0.45;finishStressResult(pass,{pathLength:s.pathLength,maxDisplacement:s.maxDisplacement,closure,closureRatio,orientationClosureDeg:rotClosure},pass?'Combined translation/rotation returned close to the starting transform.':'Mixed-motion loop accumulated excessive closure error.');}
  }
}
function stressReportObject(){
  const s=state.stress;return{version:1,completed:s.complete,overall:s.overall,startedAt:s.startedAt?new Date(Date.now()-(performance.now()-s.startedAt)).toISOString():null,results:s.results,finalDiagnostics:{position:clonePos(state.position),fovX:state.fovX,fovY:state.fovY,scale:state.scale,visualConfidence:state.visualConfidence,motionConfidence:state.motionConfidence,projectionResidualPx:state.projectionError,poseResidualPx:state.poseResidual,mapLandmarks:state.map.landmarks.length,mapConfidence:state.map.mapConfidence,imuHz:state.imuHz,videoHz:state.videoHz,tracks:state.validTracks}};
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
    axisConvention:{x:'right',y:'up',z:'backward; camera looks toward -Z'},cameraSettings:state.trackSettings,
    qualification:{visualConfidence:state.visualConfidence,motionConfidence:state.motionConfidence,scaleStability:state.scaleStability,stationaryDrift:state.driftRate,projectionResidualPx:state.projectionError,poseResidualPx:state.poseResidual,mapLandmarks:state.map.landmarks.length,mapConfidence:state.map.mapConfidence,originQuality:state.originQuality,imuHz:state.imuHz,videoHz:state.videoHz,stress:stressReportObject()},
    metricBaselineMeters:METRIC_BASELINE_M,
    note:'Reload restores the visible-camera projection. A fresh 12-inch baseline is still required each camera session because live visual landmarks belong to the current physical scene.'
  };
}
function saveBasis(){localStorage.setItem(STORAGE_KEY,JSON.stringify(basisObject()));state.basisSaved=true;ui.load.disabled=false;ui.instruction.textContent='Basis saved locally. Reload can reuse the camera projection, but the live 12-inch landmark baseline must be rebuilt for a new camera session.';}
function loadBasis(){
  const raw=localStorage.getItem(STORAGE_KEY)||localStorage.getItem(LEGACY_STORAGE_KEY);if(!raw)return;
  try{
    resetStress();ui.stress.disabled=true;const b=JSON.parse(raw);if(Number(b.version)<2.3)throw new Error('Older basis format; run the synchronization setup once.');
    state.fovX=clamp(b.fovX||62,34,105);state.fovY=clamp(b.fovY||48,20,105);state.projectionError=b.qualification?.projectionResidualPx??Infinity;state.baseQ=state.orientationQ;state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.stillScore=0;state.stillSince=0;state.scale=1;state.scaleLocked=false;state.scaleStability=0;resetWorldMap();state.previousFrame=null;state.frame=null;state.calibrationStartedAt=performance.now();
    setStage('xyz_lock','Saved camera projection loaded. Rebuild the live metric world: move the PHONE sideways exactly 12 in / 30.48 cm, then stop.',64);
    ui.save.disabled=true;state.poseReason='Projection restored; waiting for a fresh measured baseline so this physical scene gets its own 3D landmark map';
  }catch(err){ui.instruction.textContent=`Saved basis could not be loaded: ${err.message}`;}
}

function exportBasis(){const blob=new Blob([JSON.stringify(basisObject(),null,2)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='cruxtain-definitive-xyz-basis-v2-5.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}

ui.start.addEventListener('click',requestPermissions);
ui.stress.addEventListener('click',()=>{if(state.stress.active)manualAdvanceStress();else if(state.stress.complete)showStressResults();else startStressTest();});
ui.save.addEventListener('click',saveBasis);ui.load.addEventListener('click',loadBasis);ui.reset.addEventListener('click',beginSetup);
ui.diag.addEventListener('click',()=>ui.dialog.showModal());ui.closeDiag.addEventListener('click',()=>ui.dialog.close());ui.export.addEventListener('click',exportBasis);
ui.closeStress.addEventListener('click',()=>ui.stressDialog.close());ui.exportStress.addEventListener('click',exportStressReport);
addEventListener('resize',resize);
addEventListener('orientationchange',()=>setTimeout(()=>{resize();if(state.stage==='locked'){if(state.map.initialized){state.map.pending=[];state.map.lastSensorQ=relativeCameraQ();for(const lm of state.map.landmarks)lm.lastSeenSeq=-1;}state.previousFrame=null;state.lastMediaTime=null;setStage('revalidating','Screen orientation changed. Hold normally while the visual map reacquires with the new viewport.',94);}},250));
if('serviceWorker'in navigator)navigator.serviceWorker.register('./service-worker.js').catch(()=>{});
