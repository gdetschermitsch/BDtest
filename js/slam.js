import{essentialRansac,recoverPose,optimizePose}from'./geometry.js';import{I3,mm,mv,tr,add,scale,norm}from'./linalg.js';
export class Slam{constructor(K){this.K=K;this.reset()}reset(){this.Rwc=I3();this.p=[0,0,0];this.map=new Map();this.key=null;this.relativeScale=0.12;this.frame=0;this.lastAccepted=null}
update(result){this.frame++;const tracks=result.tracks;if(tracks.length<20)return{ok:false,stage:'need tracks'};const er=essentialRansac(tracks,this.K);if(!er)return{ok:false,stage:'essential rejected',ein:0};const pose=recoverPose(er.E,er.inliers,this.K);if(!pose)return{ok:false,stage:'triangulation rejected',ein:er.inliers.length};let R=pose.R,t=scale(pose.t,this.relativeScale);const corr=[];for(const item of pose.points){const id=item.track.id;if(this.map.has(id))corr.push({P:this.map.get(id),u:item.track.x,v:item.track.y})}
let reproj=null;if(corr.length>=10){const opt=optimizePose(corr,R,t,this.K);if(opt&&opt.rms<5.5){R=opt.R;t=opt.t;reproj=opt.rms}}
// current camera center in previous/world frame, compose with existing world pose
const C=scale(mv(tr(R,3,3),t,3,3),-1),worldDelta=mv(this.Rwc,C,3,3);this.p=add(this.p,worldDelta);this.Rwc=mm(this.Rwc,tr(R,3,3),3,3,3);
// triangulated points are in previous camera coordinates; transform to world.
for(const item of pose.points){const Pworld=add(this.p,mv(this.Rwc,scale(item.P,this.relativeScale),3,3));this.map.set(item.track.id,Pworld)}
this.lastAccepted={ok:true,stage:reproj===null?'essential+triangulation':'PnP corrected',ein:er.inliers.length,tri:pose.points.length,map:this.map.size,reproj,p:[...this.p],R:this.Rwc};return this.lastAccepted}
}