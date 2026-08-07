import{I3,tr,mm,mv,scale,add,sub,norm}from'./linalg.js';
import{OrientationResolver}from'./orientation.js';import{IMUBuffer}from'./imu.js';import{ScaleEstimator}from'./scale.js';
import{derotatedResiduals,knownRTranslationRansac,chooseTranslationSign}from'./known_r_geometry.js';
import{matchDescriptors}from'./matcher.js';import{LandmarkMap}from'./landmarks.js';import{pnpTranslationRansac}from'./pnp.js';

function median(a){if(!a.length)return 0;const b=[...a].sort((x,y)=>x-y),m=b.length>>1;return b.length&1?b[m]:(b[m-1]+b[m])/2}
function tcwFrom(Rwc,p){const Rcw=tr(Rwc,3,3);return{Rcw,t:scale(mv(Rcw,p,3,3),-1)}}
function cameraRayWorld(obs,K,Rwc){const r=[(obs.x-K.cx)/K.fx,(obs.y-K.cy)/K.fy,1],n=Math.hypot(...r);return mv(Rwc,r.map(v=>v/n),3,3)}
function triangulateWorld(a,b,p1,R1,p2,R2,K){
 const v1=cameraRayWorld(a,K,R1),v2=cameraRayWorld(b,K,R2),w0=sub(p1,p2),aa=1,bb=v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2],cc=1,d=v1[0]*w0[0]+v1[1]*w0[1]+v1[2]*w0[2],e=v2[0]*w0[0]+v2[1]*w0[1]+v2[2]*w0[2],den=aa*cc-bb*bb;if(Math.abs(den)<1e-7)return null;const s=(bb*e-cc*d)/den,u=(aa*e-bb*d)/den;if(s<=0||u<=0)return null;const A=add(p1,scale(v1,s)),B=add(p2,scale(v2,u)),P=scale(add(A,B),.5),err=norm(sub(A,B)),pa=Math.acos(Math.max(-1,Math.min(1,bb)))*180/Math.PI;return{P,error:err,parallaxDeg:pa}
}
export class ObjectiveTranslator{
 constructor(K){this.K=K;this.O=new OrientationResolver();this.IMU=new IMUBuffer(this.O,-85);this.S=new ScaleEstimator();this.map=new LandmarkMap();this.reset()}
 reset(){this.pVis=[0,0,0];this.metricP=[0,0,0];this.kf=null;this.initialized=false;this.poseAuthority='NONE';this.lastFrameT=null;this.lastPnp=null;this.visualScaleDefined=false;this.keyframes=0;this.pendingObs=[];this.map?.reset();this.S?.reset()}
 onOrientation(sensor){this.O.observeAbsolute(sensor)}
 onMotion(sensor,t){this.IMU.add(sensor,t)}
 frame(observations,frameT){
   const Rwc=this.O.cameraToWorld(),Rcw=tr(Rwc,3,3);if(!this.O.initialized)return this.out('WAITING ORIENTATION');
   if(!this.IMU.calibrated)return this.out('CALIBRATING IMU');
   if(!this.initialized){this.initialized=true;this.kf={t:frameT,p:[0,0,0],Rwc:Rwc.slice(),obs:observations};this.keyframes=1;return this.out('WORLD LOCKED')}
   const seed=tcwFrom(Rwc,this.pVis),corr=this.map.correspondences(observations,Rcw,seed.t,this.K);
   if(corr.length>=8){
     const pnp=pnpTranslationRansac(corr,Rcw,seed.t,this.K,90,3.2);
     if(pnp&&pnp.used>=7&&pnp.rms<3.5&&norm(sub(pnp.p,this.pVis))<1.5){this.pVis=pnp.p;this.lastPnp=pnp;this.map.confirm(pnp.inliers);this.poseAuthority='MAP/PnP'}
   }
   // Keyframe correspondences use descriptors, not tracker ID identity.
   const matches=matchDescriptors(this.kf.obs,observations,{maxDist:.72,ratio:.82});
   if(matches.length<18)return this.out(this.poseAuthority==='MAP/PnP'?'MAP/PnP':'NEED MATCHES',{matches:matches.length,corr:corr.length});
   const R21=this.O.relativeR21(this.kf.Rwc,Rwc),m=matches.map(z=>({x1:z.a.x,y1:z.a.y,x2:z.b.x,y2:z.b.y,a:z.a,b:z.b,d:z.d})),dr=derotatedResiduals(m,R21,this.K),parallax=median(dr.map(q=>q.r));
   if(parallax<1.1)return this.out(this.poseAuthority==='MAP/PnP'?'MAP/PnP + PARALLAX HOLD':'PARALLAX HOLD',{matches:m.length,parallax,corr:corr.length});
   const kr=knownRTranslationRansac(m,R21,this.K,180,.004);if(!kr)return this.out('TRANSLATION REJECTED',{matches:m.length,parallax});
   const signed=chooseTranslationSign(kr.inliers,R21,kr.t,this.K);if(signed.points.length<10)return this.out('TRIANGULATION REJECTED',{matches:m.length,parallax,inliers:kr.inliers.length});
   let newP=this.pVis.slice();
   if(!this.visualScaleDefined){
     // One unavoidable monocular gauge choice: define the FIRST visual baseline as 1 arbitrary unit.
     // Every later visual position must be solved against that map; no repeated fixed step is permitted.
     const C2cam1=scale(mv(tr(R21,3,3),signed.t,3,3),-1);
     newP=add(this.kf.p,mv(this.kf.Rwc,C2cam1,3,3));
     this.visualScaleDefined=true;
   } else if(this.poseAuthority!=='MAP/PnP'){
     return this.out('MAP ACQUISITION HOLD',{matches:m.length,parallax,inliers:kr.inliers.length,triangulated:signed.points.length,corr:corr.length});
   }
   // triangulate/qualify new world points between keyframes using descriptors and the accepted visual poses
   let added=0;
   for(const z of matches){if(this.map.trackLink.has(z.b.id))continue;const tw=triangulateWorld(z.a,z.b,this.kf.p,this.kf.Rwc,newP,Rwc,this.K);if(!tw||tw.parallaxDeg<.5||tw.error>.12)continue;this.map.add(tw.P,z.b,{parallaxDeg:tw.parallaxDeg,error:tw.error});added++}
   const pre=this.IMU.integrate(this.kf.t,frameT),dp=sub(newP,this.kf.p),scaleResult=this.S.addInterval(dp,pre);
   this.pVis=newP;this.kf={t:frameT,p:newP.slice(),Rwc:Rwc.slice(),obs:observations};this.keyframes++;this.map.prune();
   if(this.S.state==='METRIC LOCKED'&&this.S.scale){this.metricP=this.pVis.map(v=>v*this.S.scale);this.poseAuthority=this.poseAuthority==='MAP/PnP'?'METRIC MAP/PnP':'METRIC VISUAL'}
   return this.out(this.poseAuthority,{matches:m.length,parallax,inliers:kr.inliers.length,triangulated:signed.points.length,added,scaleResult,corr:corr.length})
 }
 out(stage,extra={}){
   const s=this.S.scale;return{stage,authority:this.poseAuthority,globalLock:this.O.initialized,imuCalibrated:this.IMU.calibrated,Rwc:this.O.cameraToWorld().slice(),visualPosition:this.pVis.slice(),metricPosition:s?this.pVis.map(v=>v*s):null,scale:s,scaleState:this.S.state,scaleResidual:this.S.residual,mapSize:this.map.items.size,qualifiedLandmarks:this.map.qualifiedCount(),keyframes:this.keyframes,pnpUsed:this.lastPnp?.used||0,pnpRms:this.lastPnp?.rms??null,...extra}
 }
}