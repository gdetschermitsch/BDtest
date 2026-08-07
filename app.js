'use strict';

const $ = (s) => document.querySelector(s);
const video = $('#camera');
const grid = $('#grid');
const gctx = grid.getContext('2d');
const vision = $('#vision');
const vctx = vision.getContext('2d', { willReadFrequently: true, alpha: false });

const ui = {
  startCard: $('#startCard'), start: $('#startBtn'), instruction: $('#instruction'), progress: $('#progress i'),
  status: $('#statusPill'), gridMode: $('#gridModePill'), save: $('#saveBtn'), load: $('#loadBtn'), reset: $('#resetBtn'), diag: $('#diagBtn'),
  dialog: $('#diagDialog'), closeDiag: $('#closeDiag'), export: $('#exportBtn'),
  x: $('#xVal'), y: $('#yVal'), z: $('#zVal'),
  dState: $('#dState'), dFov: $('#dFov'), dVisual: $('#dVisual'), dMotion: $('#dMotion'), dStill: $('#dStill'),
  dFps: $('#dFps'), dScale: $('#dScale'), dQuality: $('#dQuality'), dTracks: $('#dTracks'), dDrift: $('#dDrift'),
  dImuHz: $('#dImuHz'), dVideoHz: $('#dVideoHz'), dReason: $('#dReason'), dProjectionError: $('#dProjectionError'),
  dOriginQuality: $('#dOriginQuality'), dGridMode: $('#dGridMode'), dVisualStep: $('#dVisualStep'), dMoveGate: $('#dMoveGate'), stepLabel: $('#stepLabel'), stepDetail: $('#stepDetail'), stepTimer: $('#stepTimer')
};

const STORAGE_KEY = 'cruxtain.xyzBasis.v2.3';
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

const state = {
  stage: 'idle', stream: null, trackSettings: {},
  orientationQ: q(), baseQ: null, previousFrameQ: null, orientationSamples: [], lastOrientationAt: 0, orientationRate: 0,
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
  translationSignal: {x:0,y:0,z:0,confidence:0,rawMagnitude:0}, visualStepMagnitude:0, movementGate:'still', lastMoveAt:0, driftRate: 0, lastPositionForDrift: {x:0,y:0,z:0},
  poseReason: 'Not started', loopStarted: false, lastProcessAt: 0, basisSaved: false, stageEnteredAt: performance.now(),
  calib: { visualPath:0, inertialPath:0, inertialVelocity:{x:0,y:0,z:0}, lastPosition:{x:0,y:0,z:0}, motionSeen:false },
  gridMode: 'off', worldRevision: 0
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
    ui.load.disabled = !localStorage.getItem(STORAGE_KEY);
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
  state.position={x:0,y:0,z:0}; state.velocity={x:0,y:0,z:0};
  state.baseQ=null; state.previousFrameQ=null; state.previousFrame=null; state.frame=null; state.tracks=[];
  state.fovSamples=[]; state.scaleSamples=[]; state.scale=1; state.scaleLocked=false;
  state.focalConfidence=0; state.projectionError=Infinity; state.visualConfidence=0; state.motionConfidence=0; state.scaleStability=0;
  state.validTracks=0; state.flowMagnitude=0; state.stationaryScore=0; state.stillSince=0; state.stillScore=0; state.originQuality=0;
  state.originQuaternionSamples=[]; state.originAccelSamples=[]; state.originCaptured=false;
  state.accelBiasDevice={x:0,y:0,z:0}; state.calibrationStartedAt=performance.now(); state.lastSetupAt=performance.now();
  state.calib={visualPath:0,inertialPath:0,inertialVelocity:{x:0,y:0,z:0},lastPosition:{x:0,y:0,z:0},motionSeen:false};
  state.poseReason='Collecting a tolerant averaged origin; natural hand tremor is allowed';
  ui.save.disabled=true;
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

function updateScaleCalibration(rawVisualVelocity,dt) {
  if(state.scaleLocked||(state.stage!=='xyz_lock'&&state.stage!=='settle_check'))return;
  const a=correctedAcceleration(),aMag=vecLength(a);
  if(aMag>0.045)state.calib.motionSeen=true;
  state.calib.inertialVelocity.x+=a.x*dt;
  state.calib.inertialVelocity.y+=a.y*dt;
  state.calib.inertialVelocity.z+=a.z*dt;
  const inertialSpeed=vecLength(state.calib.inertialVelocity);
  const visualStep=rawVisualVelocity.rawMagnitude;
  const inertialStep=inertialSpeed*dt;
  if(visualStep<0.20)state.calib.visualPath+=visualStep;
  if(inertialStep<0.10)state.calib.inertialPath+=inertialStep;
  if(state.stationary){
    state.calib.inertialVelocity={x:0,y:0,z:0};
  }
  if(state.calib.visualPath>0.015&&state.calib.inertialPath>0.004){
    const candidate=clamp(state.calib.inertialPath/state.calib.visualPath,0.12,12);
    state.scaleSamples.push(candidate);
    if(state.scaleSamples.length>60)state.scaleSamples.shift();
    const m=median(state.scaleSamples),spread=mad(state.scaleSamples,m);
    state.scale=lerp(state.scale,m,0.06);
    state.scaleStability=clamp(state.scaleSamples.length/18,0,1)*clamp(1-spread/(m*0.75+1e-3),0,1);
  } else if(state.calib.visualPath>0.04){
    // Arbitrary scale is allowed. This deterministic fallback remains fixed and never breathes.
    const fallback=3.2;
    state.scale=lerp(state.scale,fallback,0.035);
    state.scaleStability=clamp(state.calib.visualPath/0.16,0,0.55);
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
    } else if(motionFresh&&now-state.lastMoveAt<180){
      // Very short inertial bridge only; never let acceleration free-run into drift.
      state.velocity.x+=a.x*dt*0.14;state.velocity.y+=a.y*dt*0.14;state.velocity.z+=a.z*dt*0.14;
      state.position.x+=state.velocity.x*dt;state.position.y+=state.velocity.y*dt;state.position.z+=state.velocity.z*dt;
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

function processVideoFrame(now) {
  if(now-state.lastProcessAt<45)return;
  const dt=clamp((now-(state.lastProcessAt||now))/1000,0.01,0.12);state.lastProcessAt=now;
  const frame=captureGray();if(!frame)return;
  const frameQ=nearestOrientation(now);
  if(state.previousFrame&&state.previousFrameQ){
    const raw=qualityTracks(state.previousFrame,frame),rawFlow=raw.length?median(raw.map(t=>Math.hypot(t.observed.x,t.observed.y))):0;
    state.flowMagnitude=lerp(state.flowMagnitude,rawFlow,0.25);
    if(state.stage==='fov_sync')estimateFovFromTracks(raw,state.previousFrameQ,frameQ,frame.w,frame.h);
    const solution=residualSolution(raw,state.previousFrameQ,frameQ,frame.w,frame.h,state.fovX);
    state.tracks=solution.inliers;state.validTracks=solution.inliers.length;state.visualConfidence=lerp(state.visualConfidence,solution.confidence,0.26);
    const vv=estimateTranslation(solution,frame.w,frame.h,dt,frameQ);state.translationSignal=vv;updatePose(vv,dt,now);
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
    ui.stepTimer.textContent=`${Math.round(state.scaleStability*100)}%`;
    ui.stepDetail.textContent='Move in one small sideways arc, then stop. The camera-axis movement is converted into world XYZ.';
  } else if(state.stage==='settle_check'){
    const elapsed=state.stillSince?now-state.stillSince:0,remaining=Math.max(0,0.75-elapsed/1000);
    ui.stepTimer.textContent=state.stationary?`${remaining.toFixed(1)}s`:'WAITING';
    ui.stepDetail.textContent=state.stationary?'Position is frozen while stationary—keep holding.':'Stop naturally; final drift verification begins automatically.';
  } else if(state.stage==='locked'){
    ui.stepTimer.textContent='TEST';
    ui.stepDetail.textContent='360° lattice is active. Walk/lean now: XYZ should change immediately; turn completely around and the lattice must still exist.';
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
      setStage('xyz_lock','Projection is now driving the overlay. Make one small sideways arc, then stop, to qualify proportional XYZ movement.',64);
      state.poseReason='Projection synchronized; qualifying translation scale and camera-to-world axis conversion';
    } else if(now-state.calibrationStartedAt>15000)state.poseReason='Projection needs slower rotation and visible contrast; continue left/right without translating';
  } else if(state.stage==='xyz_lock'){
    updateSetupGuidance(now);
    const moved=state.calib.visualPath>0.035||vecLength(state.position)>0.025;
    const qualified=moved&&state.visualConfidence>0.28&&state.validTracks>=8&&state.scaleStability>0.22&&state.motionConfidence>0.34;
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
      ui.status.textContent='POSE LOCKED';ui.save.disabled=false;
      state.poseReason='Origin, visible FOV, world-axis translation, scale freeze, and no-creep gates passed';
    }
  } else if(state.stage==='revalidating'){
    updateSetupGuidance(now);
    if(state.stationary&&state.stillSince&&now-state.stillSince>550){state.velocity={x:0,y:0,z:0};setStage('locked','Saved projection basis revalidated. Full 3D lattice restored at the current physical origin.',100,'locked');ui.save.disabled=false;}
  }

  if(state.stage==='locked'){
    updateSetupGuidance(now);
    if((!motionFresh&&now-state.lastOrientationAt>500)||state.validTracks<4||state.visualConfidence<0.055){ui.status.dataset.state='lost';ui.status.textContent='POSE WEAK';state.poseReason=!motionFresh?'Motion stream stale':'Too few reliable visual tracks';}
    else{ui.status.dataset.state='locked';ui.status.textContent='POSE LOCKED';}
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
}
function renderLoop(now){setupMachine(now);drawGrid();updateUI();requestAnimationFrame(renderLoop);}
function resize(){const r=grid.getBoundingClientRect(),d=Math.min(devicePixelRatio||1,2);grid.width=Math.max(1,Math.round(r.width*d));grid.height=Math.max(1,Math.round(r.height*d));if(state.fovX)state.fovY=2*Math.atan(Math.tan(state.fovX*DEG/2)*(r.height/Math.max(1,r.width)))/DEG;updateVisionCanvasSize();}

function basisObject() {
  return {
    version:2.3,savedAt:new Date().toISOString(),fovX:state.fovX,fovY:state.fovY,scale:state.scale,
    axisConvention:{x:'right',y:'up',z:'backward; camera looks toward -Z'},cameraSettings:state.trackSettings,
    qualification:{visualConfidence:state.visualConfidence,motionConfidence:state.motionConfidence,scaleStability:state.scaleStability,stationaryDrift:state.driftRate,projectionResidualPx:state.projectionError,originQuality:state.originQuality,imuHz:state.imuHz,videoHz:state.videoHz},
    note:'Reload restores the calibrated visible-camera projection and proportional scale. Without persistent real-world anchors, the current physical pose becomes the reloaded origin.'
  };
}
function saveBasis(){localStorage.setItem(STORAGE_KEY,JSON.stringify(basisObject()));state.basisSaved=true;ui.load.disabled=false;ui.instruction.textContent='Basis saved locally after the 3D walk-around test. Reload restores projection and scale, then establishes the current physical pose as origin.';}
function loadBasis(){
  const raw=localStorage.getItem(STORAGE_KEY);if(!raw)return;
  try{
    const b=JSON.parse(raw);if(Number(b.version)<2.3)throw new Error('Older basis format; run the new synchronization setup once.');
    state.fovX=clamp(b.fovX||62,34,100);state.fovY=clamp(b.fovY||48,20,100);state.scale=clamp(b.scale||1,.1,12);state.scaleLocked=true;
    state.scaleStability=b.qualification?.scaleStability||.5;state.projectionError=b.qualification?.projectionResidualPx??Infinity;
    state.baseQ=state.orientationQ;state.position={x:0,y:0,z:0};state.velocity={x:0,y:0,z:0};state.stillScore=0;state.stillSince=0;
    setStage('revalidating','Saved projection and scale loaded. Hold normally while the current camera pose becomes the new origin.',94);
    ui.save.disabled=false;state.poseReason='Revalidating saved basis against current live camera and motion streams';
  }catch(err){ui.instruction.textContent=`Saved basis could not be loaded: ${err.message}`;}
}
function exportBasis(){const blob=new Blob([JSON.stringify(basisObject(),null,2)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='cruxtain-definitive-xyz-basis-v2-3.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}

ui.start.addEventListener('click',requestPermissions);
ui.save.addEventListener('click',saveBasis);ui.load.addEventListener('click',loadBasis);ui.reset.addEventListener('click',beginSetup);
ui.diag.addEventListener('click',()=>ui.dialog.showModal());ui.closeDiag.addEventListener('click',()=>ui.dialog.close());ui.export.addEventListener('click',exportBasis);
addEventListener('resize',resize);
addEventListener('orientationchange',()=>setTimeout(()=>{resize();if(state.stage==='locked')setStage('revalidating','Screen orientation changed. Hold normally while axes and projection revalidate.',94);},250));
if('serviceWorker'in navigator)navigator.serviceWorker.register('./service-worker.js').catch(()=>{});
