import{FeatureTracker}from'./features.js';import{Sensors}from'./sensors.js';import{Slam}from'./slam.js';import{SubstrateRecorder}from'./substrate_recorder.js';
const $=x=>document.getElementById(x),video=$('cam'),overlay=$('overlay'),ctx=overlay.getContext('2d'),S=new Sensors(),REC=new SubstrateRecorder();
let F,SL,K,W,H,stream,running=false,capturing=false,busy=false,lastFrame=null,fps=0,motionCount=0,motionT=performance.now(),sensorHz=0,startT=0,timer=null;
const protocol=[['STILL',0,3000,'HOLD STILL'],['ROTATE',3000,6500,'ROTATE LEFT / RIGHT IN PLACE'],['FORWARD',6500,10500,'WALK FORWARD NORMALLY'],['LATERAL',10500,14000,'MOVE SIDEWAYS NORMALLY'],['STOP',14000,17000,'STOP AND HOLD STILL']];
function status(t,c='warn'){$('state').textContent=t;$('state').className='state '+c}
function phase(){const t=performance.now()-startT,p=protocol.find(q=>t>=q[1]&&t<q[2]);if(!p){finish();return}REC.setPhase(p[0]);$('phase').textContent=`${p[3]} · ${((p[2]-t)/1000).toFixed(1)} s`}
function norm3(a){const n=Math.hypot(...a)||1;return a.map(v=>v/n)}
function objectiveSnapshot(){
 const g=norm3(S.gravity),up=g.map(v=>-v),h=S.heading;
 return {worldUpDevice:up.map(v=>+v.toFixed(6)),headingDeg:h==null?null:+h.toFixed(5),gyroRateDegPerSec:S.rate.map(v=>+v.toFixed(5)),linearAccelMS2:S.accel.map(v=>+v.toFixed(5)),gravityMagnitude:+Math.hypot(...S.gravity).toFixed(6)}
}
async function init(){
 await S.start();stream=await navigator.mediaDevices.getUserMedia({video:{facingMode:{ideal:'environment'},width:{ideal:1280},height:{ideal:720},frameRate:{ideal:30}},audio:false});video.srcObject=stream;await video.play();await new Promise(r=>setTimeout(r,150));
 const tr=stream.getVideoTracks()[0],set=tr.getSettings?tr.getSettings():{},cap=tr.getCapabilities?tr.getCapabilities():{},vw=video.videoWidth||set.width||1280,vh=video.videoHeight||set.height||720;W=320;H=Math.max(120,Math.round(W*vh/vw));const f0=.88*W;K={fx:f0,fy:f0,cx:W/2,cy:H/2};F=new FeatureTracker(W,H);SL=new Slam(K);$('camera').textContent=`${vw}×${vh}`;$('procsize').textContent=`${W}×${H}`;
 return{tr,set,cap,vw,vh}
}
async function start(){
 try{
  const c=running?null:await init();running=true;F.reset();SL.reset();
  REC.start({userAgent:navigator.userAgent,devicePixelRatio,screen:{w:screen.width,h:screen.height,orientation:screen.orientation?.type||null},camera:c?{videoWidth:c.vw,videoHeight:c.vh,settings:c.set,capabilities:c.cap}:null,processing:{width:W,height:H,initialKHypothesis:K},protocol:protocol.map(p=>({name:p[0],durationMs:p[2]-p[1]}))});
  S.onMotion=s=>{REC.motionSample(s);motionCount++;const now=performance.now();if(now-motionT>=1000){sensorHz=motionCount*1000/(now-motionT);motionCount=0;motionT=now;$('shz').textContent=sensorHz.toFixed(1)}};
  S.onOrientation=s=>REC.orientationSample(s);
  capturing=true;startT=performance.now();$('start').disabled=true;$('export').disabled=true;status('SUBSTRATE RECORDING','good');timer=setInterval(phase,70);schedule()
 }catch(e){status('START FAILED: '+e.message,'bad');$('debug').textContent=e.stack||e}
}
function finish(){if(!capturing)return;capturing=false;clearInterval(timer);REC.stop();$('phase').textContent='SUBSTRATE COMPLETE — Export the data';$('start').disabled=false;$('start').textContent='Repeat';$('export').disabled=false;status('SUBSTRATE COMPLETE','good')}
function schedule(){if(!running)return;if('requestVideoFrameCallback'in video)video.requestVideoFrameCallback(process);else requestAnimationFrame(t=>process(t,{}))}
function process(now,meta={}){
 if(busy){schedule();return}busy=true;
 try{
  const pt=performance.now();if(lastFrame!=null){const f=1000/Math.max(1,pt-lastFrame);fps=fps?.9*fps+.1*f:f;$('fps').textContent=fps.toFixed(1)}lastFrame=pt;
  const r=F.process(video),o=r.tracks.length?SL.update(r,S):{stage:'need tracks',p:SL.p,ein:0,tri:0,pnpUsed:0,reproj:null,map:SL.map.size};
  const obj=objectiveSnapshot();
  $('up').textContent=obj.worldUpDevice.map(v=>v.toFixed(2)).join(',');$('head').textContent=obj.headingDeg==null?'—':obj.headingDeg.toFixed(1)+'°';$('gyro').textContent=Math.hypot(...S.rate).toFixed(2)+'°/s';$('accel').textContent=Math.hypot(...S.accel).toFixed(2)+' m/s²';
  $('features').textContent=r.detected;$('tracks').textContent=r.tracks.length;$('motion').textContent=(r.motionPx||0).toFixed(2)+' px';$('einliers').textContent=o.ein||0;$('tri').textContent=o.tri||0;$('map').textContent=o.map||SL.map.size;$('pnp').textContent=o.pnpUsed||0;$('reproj').textContent=o.reproj==null?'—':Number.isFinite(o.reproj)?o.reproj.toFixed(2)+' px':'reject';
  const auth=(o.pnpUsed||0)>=8&&Number.isFinite(o.reproj)?'MAP/PnP':(o.ein||0)>=8?'RELATIVE GEOMETRY':'NONE';$('authority').textContent=auth;
  const p=o.p||SL.p;['x','y','z'].forEach((k,i)=>$(k).textContent=p[i].toFixed(3));
  if(capturing){
   REC.frame({video:{callbackNow:+(Number(now)||0).toFixed(3),mediaTime:Number.isFinite(meta.mediaTime)?+meta.mediaTime.toFixed(6):null,presentedFrames:meta.presentedFrames??null,expectedDisplayTime:Number.isFinite(meta.expectedDisplayTime)?+meta.expectedDisplayTime.toFixed(3):null},objective:obj,feature:{detected:r.detected,tracked:r.tracks.length,seeded:r.seeded||0,motionPx:+(r.motionPx||0).toFixed(4),tracks:F.telemetry(r.tracks,120)},geometry:{stage:o.stage,essentialInliers:o.ein||0,essentialIds:o.essentialIds||[],triangulated:o.tri||0,triEvidence:o.triEvidence||[],relative:o.relative||null,pnpUsed:o.pnpUsed||0,pnpIds:o.pnpIds||[],reprojection:o.reproj,mapSize:o.map||SL.map.size,relativePosition:p.map(v=>+v.toFixed(6))}});
   REC.derive({globalLock:{upDevice:obj.worldUpDevice,headingDeg:obj.headingDeg},rotationEvidence:{gyro:S.rate,visualStage:o.stage},translationEvidence:{essentialInliers:o.ein||0,pnpUsed:o.pnpUsed||0,reprojection:o.reproj},scale:{state:'UNRESOLVED',reason:'raw metric acceleration recorded for coupled visual-inertial scale solve; no arbitrary visual translation constant may be called meters'},authority:auth})
  }
  draw(r.tracks,o.essentialIds||[]);$('debug').textContent=JSON.stringify({phase:REC.phase,objective:obj,stage:o.stage,authority:auth,features:r.detected,tracks:r.tracks.length,map:SL.map.size,relativeXYZ:p},null,2)
 }catch(e){status('SUBSTRATE ERROR: '+e.message,'bad');REC.event('runtime-error',{message:e.message,stack:e.stack||null})}finally{busy=false;schedule()}
}
function draw(tracks,ids){overlay.width=innerWidth*devicePixelRatio;overlay.height=innerHeight*devicePixelRatio;ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);ctx.clearRect(0,0,innerWidth,innerHeight);const sx=innerWidth/W,sy=innerHeight/H,set=new Set(ids);for(const q of tracks){ctx.strokeStyle=set.has(q.id)?'rgba(80,255,145,.9)':'rgba(255,210,90,.55)';ctx.beginPath();ctx.moveTo(q.prevX*sx,q.prevY*sy);ctx.lineTo(q.x*sx,q.y*sy);ctx.stroke()}}
$('start').onclick=start;$('export').onclick=()=>REC.download({finalMapSize:SL?.map?.size||0,frameCount:REC.frames.length,motionSamples:REC.motion.length,orientationSamples:REC.orientation.length});$('dbg').onclick=()=>$('hud').classList.toggle('debug');
