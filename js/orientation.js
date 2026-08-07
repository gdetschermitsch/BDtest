import{I3,mm,tr,mv,norm,normalize,rodrigues,orthonormalize,rotX,rotY,rotZ}from'./linalg.js';

const D2C=[1,0,0, 0,-1,0, 0,0,-1]; // camera x right, y down, z forward -> device x right, y top, z toward user
const E2G=[1,0,0, 0,0,1, 0,-1,0]; // Earth E,N,U -> global X=east,Y=up,Z=south

function eulerDeviceToEarth(alpha,beta,gamma){
 const d=Math.PI/180;
 // W3C intrinsic Z-X'-Y'' convention.
 return mm(mm(rotZ((alpha||0)*d),rotX((beta||0)*d),3,3,3),rotY((gamma||0)*d),3,3,3);
}
export class OrientationResolver{
 constructor(){this.reset()}
 reset(){this.Rwc=I3();this.initialized=false;this.lastMotionT=null;this.heading0=null;this.mode='waiting';this.bias=[0,0,0]}
 setGyroBias(b){this.bias=b.slice()}
 initializeFromOrientation(s){
   if(s.alpha==null||s.beta==null||s.gamma==null)return false;
   const Rde=eulerDeviceToEarth(s.alpha,s.beta,s.gamma);
   this.Rwc=orthonormalize(mm(mm(E2G,Rde,3,3,3),D2C,3,3,3));
   this.heading0=s.heading;this.initialized=true;this.mode='global-lock';return true
 }
 observeAbsolute(s){
   if(!this.initialized)return this.initializeFromOrientation(s);
   return true
 }
 integrateGyro(rateDeg,tMs){
   if(!this.initialized){this.lastMotionT=tMs;return}
   if(this.lastMotionT==null){this.lastMotionT=tMs;return}
   const dt=Math.max(0,Math.min(.05,(tMs-this.lastMotionT)/1000));this.lastMotionT=tMs;if(!dt)return;
   // Empirical browser gyro -> visual camera small-rotation mapping recovered from substrate.
   const d=Math.PI/180;
   const w=[-(rateDeg[0]-this.bias[0])*d,(rateDeg[1]-this.bias[1])*d,(rateDeg[2]-this.bias[2])*d];
   this.Rwc=orthonormalize(mm(this.Rwc,rodrigues(w.map(v=>v*dt)),3,3,3));
   this.mode='gyro-predicted'
 }
 cameraToWorld(){return this.Rwc}
 worldToCamera(){return tr(this.Rwc,3,3)}
 relativeR21(Rwc1,Rwc2){return mm(tr(Rwc2,3,3),Rwc1,3,3,3)}
 accelDeviceToWorld(aDevice){
   // device acceleration -> camera via inverse D2C (= D2C), then camera -> world.
   const aCam=mv(D2C,aDevice,3,3);return mv(this.Rwc,aCam,3,3)
 }
}