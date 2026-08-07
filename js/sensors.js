export class Sensors{
constructor(){this.heading=null;this.alpha=null;this.beta=null;this.gamma=null;this.absolute=false;this.gravity=[0,0,-9.81];this.rate=[0,0,0];this.accel=[0,0,0];this.ok=false;this.onMotion=null;this.onOrientation=null}
async start(){
 if(typeof DeviceMotionEvent!=="undefined"&&typeof DeviceMotionEvent.requestPermission==="function"){if(await DeviceMotionEvent.requestPermission()!=="granted")throw Error("Motion permission denied")}
 if(typeof DeviceOrientationEvent!=="undefined"&&typeof DeviceOrientationEvent.requestPermission==="function"){try{await DeviceOrientationEvent.requestPermission()}catch{}}
 addEventListener("devicemotion",e=>{const t=performance.now(),g=e.accelerationIncludingGravity||{},a=e.acceleration||{},r=e.rotationRate||{};
   this.gravity=[g.x||0,g.y||0,g.z||0];this.accel=[a.x||0,a.y||0,a.z||0];this.rate=[r.alpha||0,r.beta||0,r.gamma||0];this.ok=true;
   this.onMotion&&this.onMotion(this,t)
 },{passive:true});
 addEventListener("deviceorientation",e=>{const t=performance.now(),h=e.webkitCompassHeading;
   this.heading=Number.isFinite(h)?h:(Number.isFinite(e.alpha)?(360-e.alpha)%360:null);
   this.alpha=Number.isFinite(e.alpha)?e.alpha:null;this.beta=Number.isFinite(e.beta)?e.beta:null;this.gamma=Number.isFinite(e.gamma)?e.gamma:null;this.absolute=!!e.absolute;
   this.onOrientation&&this.onOrientation(this,t)
 },{passive:true});
}}