import{norm}from'./linalg.js';
export class IMUBuffer{
 constructor(orientation,timeOffsetMs=-85){this.O=orientation;this.timeOffsetMs=timeOffsetMs;this.samples=[];this.bias=[0,0,0];this.gyroBias=[0,0,0];this.quiet=[];this.calibrated=false}
 add(sensor,t){
   this.O.integrateGyro(sensor.rate,t);
   if(!this.calibrated){const gm=Math.hypot(...sensor.rate),am=Math.hypot(...sensor.accel);if(gm<5&&am<.55)this.quiet.push({a:sensor.accel.slice(),g:sensor.rate.slice()});if(this.quiet.length>=70)this.finishBias()}
   const ab=sensor.accel.map((v,i)=>v-this.bias[i]),aw=this.O.accelDeviceToWorld(ab);this.samples.push({t,a:aw,rate:sensor.rate.slice()});if(this.samples.length>5000)this.samples.splice(0,1000)
 }
 finishBias(){const med=k=>{const a=this.quiet.map(q=>q[k]).sort((x,y)=>x-y),m=a.length>>1;return a.length&1?a[m]:(a[m-1]+a[m])/2};for(let i=0;i<3;i++){this.bias[i]=med('a'+i)}}
 // above generic med isn't suitable for vectors; explicit:
 finalize(){
   if(!this.quiet.length)return;
   const median=a=>{const b=[...a].sort((x,y)=>x-y),m=b.length>>1;return b.length&1?b[m]:(b[m-1]+b[m])/2};
   this.bias=[0,1,2].map(i=>median(this.quiet.map(q=>q.a[i])));
   this.gyroBias=[0,1,2].map(i=>median(this.quiet.map(q=>q.g[i])));
   this.O.setGyroBias(this.gyroBias);this.calibrated=true
 }
 finishBias(){this.finalize()}
 integrate(frameT0,frameT1){
   const t0=frameT0+this.timeOffsetMs,t1=frameT1+this.timeOffsetMs,arr=this.samples.filter(s=>s.t>=t0&&s.t<=t1);
   if(arr.length<2)return null;let dv=[0,0,0],dp=[0,0,0],last=arr[0];
   for(let i=1;i<arr.length;i++){const s=arr[i],dt=Math.max(0,Math.min(.05,(s.t-last.t)/1000)),a=last.a.map((v,k)=>(v+s.a[k])*.5);for(let k=0;k<3;k++){dp[k]+=dv[k]*dt+.5*a[k]*dt*dt;dv[k]+=a[k]*dt}last=s}
   return{dt:(t1-t0)/1000,alpha:dp,beta:dv,samples:arr.length}
 }
}