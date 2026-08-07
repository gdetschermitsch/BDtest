
import {SensorHub} from './sensors.js';
import {VisionGeometry} from './vision.js';

const $=id=>document.getElementById(id);
const video=$('cam'),overlay=$('overlay'),ctx=overlay.getContext('2d');
const sensors=new SensorHub();
let vision=null,stream=null,running=false,lastProcess=0,lastGood=[0,0,0];

function setState(text,kind='warn'){ $('state').textContent=text; $('state').className='state '+kind; }
function stage(n,text){
  $('stageTitle').textContent=`Stage ${n}`;
  $('stageText').textContent=text;
}
function waitCV(timeout=20000){
  return new Promise((resolve,reject)=>{
    const ready=()=>window.cv&&cv.Mat&&cv.goodFeaturesToTrack;
    if(ready())return resolve();
    const timer=setTimeout(()=>reject(new Error('OpenCV.js did not initialize')),timeout);
    addEventListener('opencv-ready',()=>{clearTimeout(timer);resolve()},{once:true});
  });
}
async function start(){
  try{
    $('runtime').textContent='loading';
    await waitCV();
    $('runtime').textContent='OpenCV.js ready';
    await sensors.start();
    stream=await navigator.mediaDevices.getUserMedia({video:{facingMode:{ideal:'environment'},width:{ideal:1280},height:{ideal:720}},audio:false});
    video.srcObject=stream; await video.play();
    vision=new VisionGeometry(320,240);
    running=true; lastProcess=0;
    setState('INITIALIZING FEATURE MAP','warn');
    stage('1 · Feature tracks','Move naturally. Visible dots must appear before geometric pose is allowed.');
    requestAnimationFrame(loop);
  }catch(e){setState('START FAILED: '+e.message,'bad');$('log').textContent=e.stack||e.message}
}
function loop(t){
  if(!running)return;
  if(t-lastProcess>80 && video.readyState>=2){
    lastProcess=t;
    try{
      const r=vision.process(video);
      render(r);
    }catch(e){
      setState('VISION ERROR — HOLDING POSE','bad');
      $('log').textContent=(e.stack||e.message)+'\n'+$('log').textContent;
    }
  }
  requestAnimationFrame(loop);
}
function render(r){
  $('features').textContent=r.features?.length||0;
  $('tracked').textContent=r.tracked?.length||0;
  $('inliers').textContent=r.inliers?.length||0;
  $('keyframes').textContent=r.keyframes||0;
  $('landmarks').textContent=r.landmarks?.length||0;
  $('heading').textContent=sensors.heading==null?'unavailable':sensors.heading.toFixed(1)+'°';
  $('gravity').textContent=Math.hypot(...sensors.gravity).toFixed(2);
  $('gyro').textContent=sensors.live?'live':'waiting';

  if(r.pose){
    lastGood=r.pose.t;
    $('poseSource').textContent='essential geometry';
    setState('GEOMETRIC POSE ACCEPTED · MAP SCALE','good');
    stage('3 · Essential geometry + triangulation','A multi-view pose was accepted and static landmark candidates were triangulated. Metric scale remains unlocked.');
  }else if((r.inliers?.length||0)>=8){
    $('poseSource').textContent='held';
    setState('TRACKS LIVE · WAITING FOR PARALLAX','warn');
    stage('2 · Multi-view geometry','Features are tracked, but the frame is not geometrically strong enough to update pose.');
  }else{
    $('poseSource').textContent='held';
    setState((r.features?.length||0)>0?'FEATURES LIVE · BUILDING TRACKS':'NO FEATURES — HOLDING POSE','warn');
    stage('1 · Feature tracks','The virtual pose remains fixed until sufficient real camera tracks exist.');
  }
  ['x','y','z'].forEach((k,i)=>$(k).textContent=lastGood[i].toFixed(3)+' map');
  $('reprojection').textContent=r.geometry?.medianParallax?`${r.geometry.medianParallax.toFixed(2)} px parallax`:'—';
  draw(r);
}
function draw(r){
  const dpr=devicePixelRatio||1;overlay.width=innerWidth*dpr;overlay.height=innerHeight*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,innerWidth,innerHeight);
  const sx=innerWidth/320,sy=innerHeight/240;
  ctx.lineWidth=1;
  for(const tr of (r.tracked||[]).slice(0,300)){
    ctx.strokeStyle='rgba(255,209,102,.55)';ctx.beginPath();ctx.moveTo(tr.p.x*sx,tr.p.y*sy);ctx.lineTo(tr.q.x*sx,tr.q.y*sy);ctx.stroke();
  }
  ctx.fillStyle='rgba(112,255,155,.95)';
  for(const tr of (r.inliers||[]).slice(0,300)){ctx.beginPath();ctx.arc(tr.q.x*sx,tr.q.y*sy,2.2,0,Math.PI*2);ctx.fill()}
  if(!(r.inliers||[]).length){
    ctx.fillStyle='rgba(255,255,255,.8)';
    for(const p of (r.features||[]).slice(0,300)){ctx.beginPath();ctx.arc(p.x*sx,p.y*sy,1.8,0,Math.PI*2);ctx.fill()}
  }
}
$('start').onclick=()=>{if(!running)start()};
$('reset').onclick=()=>{vision?.reset();lastGood=[0,0,0];setState('WORLD RESET','warn')};
$('debug').onclick=()=>$('hud').classList.toggle('debug');
