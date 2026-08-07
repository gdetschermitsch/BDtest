import{FeatureTracker}from'./features.js';
import{Sensors}from'./sensors.js';
import{Slam}from'./slam.js';
import{EvidenceRecorder}from'./recorder.js';

const $=id=>document.getElementById(id),video=$('cam'),overlay=$('overlay'),ctx=overlay.getContext('2d');
let W=320,H=180,K=null,F=null,SL=null;
const S=new Sensors(),REC=new EvidenceRecorder();
let running=false,capturing=false,stream,busy=false,frameCount=0,lastFramePerf=null,fpsEMA=0,captureTimer=null,captureStart=0;

const protocol=[
 {name:'STILL',from:0,to:3000,text:'HOLD STILL'},
 {name:'ROTATE',from:3000,to:6000,text:'ROTATE LEFT / RIGHT IN PLACE'},
 {name:'FORWARD',from:6000,to:10000,text:'WALK FORWARD NORMALLY'},
 {name:'STOP',from:10000,to:13000,text:'STOP AND HOLD STILL'}
];

function status(s,c='warn'){$('state').textContent=s;$('state').className='state '+c}
function phaseFor(ms){return protocol.find(p=>ms>=p.from&&ms<p.to)||null}
function updatePhase(){
 if(!capturing)return;
 const ms=performance.now()-captureStart,p=phaseFor(ms);
 if(p){$('phase').textContent=`${p.text} · ${Math.max(0,(p.to-ms)/1000).toFixed(1)} s`;if(REC.phase!==p.name)REC.setPhase(p.name)}
 else finishCapture();
}
function computeProcessingSize(vw,vh){
 const targetW=320, aspect=vh/vw;
 return [targetW,Math.max(120,Math.round(targetW*aspect))];
}
async function initialize(){
 await S.start();
 stream=await navigator.mediaDevices.getUserMedia({video:{facingMode:{ideal:'environment'},width:{ideal:1280},height:{ideal:720},frameRate:{ideal:30}},audio:false});
 video.srcObject=stream;await video.play();
 await new Promise(r=>setTimeout(r,150));
 const track=stream.getVideoTracks()[0],settings=track.getSettings?track.getSettings():{};
 const vw=video.videoWidth||settings.width||1280,vh=video.videoHeight||settings.height||720;
 [W,H]=computeProcessingSize(vw,vh);
 // Initial focal hypothesis is recorded, not trusted. Offline replay can sweep focal length.
 const f0=0.88*W;K={fx:f0,fy:f0,cx:W/2,cy:H/2};
 F=new FeatureTracker(W,H);SL=new Slam(K);
 $('camera').textContent=`${vw}×${vh}`;
 $('procsize').textContent=`${W}×${H}`;
 return {track,settings,vw,vh};
}
async function startCapture(){
 if(capturing)return;
 try{
   if(!running){
     const cam=await initialize();running=true;status('CAMERA + SENSORS READY','good');
     const caps=cam.track.getCapabilities?cam.track.getCapabilities():{};
     REC.start({
       userAgent:navigator.userAgent,
       platform:navigator.platform,
       devicePixelRatio,
       screen:{w:screen.width,h:screen.height,orientation:screen.orientation?.type||null},
       camera:{videoWidth:cam.vw,videoHeight:cam.vh,settings:cam.settings,capabilities:caps},
       processing:{width:W,height:H,K0:K},
       protocol:protocol.map(p=>({name:p.name,durationMs:p.to-p.from}))
     });
     S.onMotion=s=>REC.motionSample(s);S.onOrientation=s=>REC.orientationSample(s);
     schedule();
   }else{
     REC.start({...REC.meta, restarted:true});
   }
   F.reset();SL.reset();frameCount=0;lastFramePerf=null;fpsEMA=0;
   captureStart=performance.now();capturing=true;$('export').disabled=true;$('start').disabled=true;
   REC.setPhase('STILL');status('RECORDING EVIDENCE','good');$('phase').textContent='HOLD STILL · 3.0 s';
   captureTimer=setInterval(updatePhase,80);
 }catch(e){status('START FAILED: '+e.message,'bad');$('debug').textContent=e.stack||String(e)}
}
function finishCapture(){
 if(!capturing)return;
 capturing=false;clearInterval(captureTimer);captureTimer=null;REC.stop();
 $('phase').textContent='CAPTURE COMPLETE — tap Export Test Data';
 $('export').disabled=false;$('start').disabled=false;$('start').textContent='Repeat Capture';
 status('CAPTURE COMPLETE','good')
}
function schedule(){if(!running)return;if('requestVideoFrameCallback'in video)video.requestVideoFrameCallback(process);else requestAnimationFrame(t=>process(t,{}))}
function process(now,meta={}){
 if(busy){schedule();return}busy=true;
 try{
   const perf=performance.now();
   if(lastFramePerf!=null){const inst=1000/Math.max(1,perf-lastFramePerf);fpsEMA=fpsEMA?0.9*fpsEMA+0.1*inst:inst;$('fps').textContent=fpsEMA.toFixed(1)}
   lastFramePerf=perf; frameCount++;
   const r=F.process(video);
   $('features').textContent=r.detected;$('tracks').textContent=r.tracks.length;$('motion').textContent=(r.motionPx||0).toFixed(2)+' px';
   let o={ok:false,stage:r.tracks.length?'not solved':'need tracks',p:SL.p||[0,0,0],ein:0,tri:0,pnpUsed:0,reproj:null,gyroMag:Math.hypot(...S.rate)};
   if(r.tracks.length)o=SL.update(r,S);
   $('einliers').textContent=o.ein||0;$('triangulated').textContent=o.tri||0;$('pnp').textContent=o.pnpUsed||0;
   $('reproj').textContent=o.reproj==null?'—':Number.isFinite(o.reproj)?o.reproj.toFixed(2)+' px':'REJECT';
   $('geometry').textContent=o.stage;
   $('sensors').textContent=(S.heading==null?'heading —':S.heading.toFixed(0)+'°')+' / gyro '+Math.hypot(...S.rate).toFixed(2);
   const p=o.p||SL.p;['x','y','z'].forEach((k,i)=>$(k).textContent=p[i].toFixed(3));
   if(capturing){
     const t=+(performance.now()-REC.startedAt).toFixed(3);
     REC.frame({
       t,
       video:{now:+(Number(now)||0).toFixed(3),mediaTime:Number.isFinite(meta.mediaTime)?+meta.mediaTime.toFixed(6):null,presentedFrames:meta.presentedFrames??null,expectedDisplayTime:Number.isFinite(meta.expectedDisplayTime)?+meta.expectedDisplayTime.toFixed(3):null},
       feature:{detected:r.detected,tracked:r.tracks.length,seeded:r.seeded||0,motionPx:+(r.motionPx||0).toFixed(4),tracks:F.telemetry(r.tracks,100)},
       sensorSnapshot:{heading:S.heading==null?null:+S.heading.toFixed(4),gravity:S.gravity.map(v=>+v.toFixed(5)),accel:S.accel.map(v=>+v.toFixed(5)),rate:S.rate.map(v=>+v.toFixed(5))},
       geometry:{
         stage:o.stage,ok:!!o.ok,gyroMag:+(o.gyroMag||0).toFixed(5),
         essentialInliers:o.ein||0,essentialIds:o.essentialIds||[],
         triangulated:o.tri||0,triEvidence:o.triEvidence||[],
         pnpUsed:o.pnpUsed||0,pnpIds:o.pnpIds||[],
         reprojection:o.reproj==null?null:+o.reproj,
         relative:o.relative||null,
         mapSize:o.map||SL.map.size,
         p:(p||[0,0,0]).map(v=>+v.toFixed(6))
       }
     });
   }
   draw(r.tracks,o.essentialIds||[]);
   $('debug').textContent=JSON.stringify({phase:REC.phase,frameCount,fps:fpsEMA,stage:o.stage,features:r.detected,tracks:r.tracks.length,motionPx:r.motionPx,essential:o.ein||0,tri:o.tri||0,pnp:o.pnpUsed||0,reproj:o.reproj,map:SL.map.size,position:p},null,2);
 }catch(e){status('VISION ERROR: '+e.message,'bad');$('debug').textContent=e.stack||String(e);if(capturing)REC.event('runtime-error',{message:e.message,stack:e.stack||null})}
 finally{busy=false;schedule()}
}
function draw(tracks,inlierIds){
 overlay.width=innerWidth*devicePixelRatio;overlay.height=innerHeight*devicePixelRatio;ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);ctx.clearRect(0,0,innerWidth,innerHeight);
 const sx=innerWidth/W,sy=innerHeight/H,set=new Set(inlierIds);
 for(const q of tracks){ctx.strokeStyle=set.has(q.id)?'rgba(80,255,145,.9)':'rgba(255,210,90,.55)';ctx.beginPath();ctx.moveTo(q.prevX*sx,q.prevY*sy);ctx.lineTo(q.x*sx,q.y*sy);ctx.stroke();ctx.fillStyle='rgba(255,255,255,.75)';ctx.fillRect(q.x*sx-1,q.y*sy-1,2,2)}
}
$('start').onclick=startCapture;
$('export').onclick=()=>REC.download({finalMapSize:SL?.map?.size||0,framesRecorded:REC.frames.length,motionSamples:REC.motion.length,orientationSamples:REC.orientation.length});
$('dbg').onclick=()=>$('hud').classList.toggle('debug');
