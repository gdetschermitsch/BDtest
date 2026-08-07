import{essentialRansac,recoverPose,optimizePose,project}from'./geometry.js';
import{I3,mm,mv,tr,add,sub,scale,norm}from'./linalg.js';

function cameraToWorldFromWorldToCamera(Rcw,tcw){const Rwc=tr(Rcw,3,3);return{Rwc,p:scale(mv(Rwc,tcw,3,3),-1)}}
function worldToCameraFromCameraToWorld(Rwc,p){const Rcw=tr(Rwc,3,3);return{Rcw,tcw:scale(mv(Rcw,p,3,3),-1)}}
function finitePose(p,R){return p.every(Number.isFinite)&&R.every(Number.isFinite)}
function reprojectionRMS(corr,Rcw,tcw,K){let e=0,n=0;for(const c of corr){const q=project(c.P,Rcw,tcw,K);if(!q)continue;const d=Math.hypot(q.x-c.u,q.y-c.v);if(d>25)continue;e+=d*d;n++}return n>=6?Math.sqrt(e/n):Infinity}

export class Slam{
 constructor(K){this.K=K;this.reset()}
 reset(){this.Rwc=I3();this.p=[0,0,0];this.map=new Map();this.relativeScale=0.12;this.frame=0;this.stationaryStreak=0;this.lastAccepted=null;this.stats={pnpAccepted:0,stationaryRejected:0,essentialAccepted:0}}
 mappedCorrespondences(tracks){const out=[];for(const q of tracks){const m=this.map.get(q.id);if(m)out.push({P:m.P,u:q.x,v:q.y,id:q.id})}return out}
 update(result,sensors={rate:[0,0,0]}){
  this.frame++;const tracks=result.tracks||[];const gyroMag=Math.hypot(...(sensors.rate||[0,0,0]));const motionPx=result.motionPx??Infinity;
  // A genuinely still camera is a degenerate essential-matrix problem. Hold the world instead of inventing translation.
  const stationaryEvidence=tracks.length>=20&&motionPx<0.65&&gyroMag<2.2;
  this.stationaryStreak=stationaryEvidence?this.stationaryStreak+1:0;
  if(stationaryEvidence){this.stats.stationaryRejected++;return{ok:false,held:true,stage:'stationary hold',motionPx,gyroMag,ein:0,tri:0,map:this.map.size,reproj:null,p:[...this.p]}}
  if(tracks.length<20)return{ok:false,held:true,stage:'need tracks',motionPx,gyroMag,ein:0,tri:0,map:this.map.size,p:[...this.p]};

  // First preference: solve the current camera directly against already-fixed world landmarks.
  const corr=this.mappedCorrespondences(tracks);
  if(corr.length>=12){
    const init=worldToCameraFromCameraToWorld(this.Rwc,this.p);
    const opt=optimizePose(corr,init.Rcw,init.tcw,this.K,10);
    if(opt&&opt.used>=10&&opt.rms<4.0){
      const abs=cameraToWorldFromWorldToCamera(opt.R,opt.t);
      const jump=norm(sub(abs.p,this.p));
      if(finitePose(abs.p,abs.Rwc)&&jump<0.65){
        this.p=abs.p;this.Rwc=abs.Rwc;this.stats.pnpAccepted++;
        this.lastAccepted={ok:true,stage:'landmark PnP correction',ein:0,tri:0,map:this.map.size,reproj:opt.rms,p:[...this.p],R:this.Rwc,motionPx,gyroMag,pnpUsed:opt.used,pnpIds:corr.slice(0,opt.used).map(c=>c.id)};return this.lastAccepted;
      }
    }
  }

  // Bootstrap / map-growth path: relative two-view geometry.
  const er=essentialRansac(tracks,this.K);if(!er)return{ok:false,held:true,stage:'essential rejected',motionPx,gyroMag,ein:0,tri:0,map:this.map.size,p:[...this.p]};
  const pose=recoverPose(er.E,er.inliers,this.K);if(!pose)return{ok:false,held:true,stage:'triangulation rejected',motionPx,gyroMag,ein:er.inliers.length,tri:0,map:this.map.size,p:[...this.p]};

  const prevRwc=this.Rwc.slice(),prevP=this.p.slice();
  const tRel=scale(pose.t,this.relativeScale), Cprev=scale(mv(tr(pose.R,3,3),tRel,3,3),-1);
  let predP=add(prevP,mv(prevRwc,Cprev,3,3));
  let predRwc=mm(prevRwc,tr(pose.R,3,3),3,3,3);
  if(!finitePose(predP,predRwc))return{ok:false,held:true,stage:'nonfinite pose rejected',motionPx,gyroMag,ein:er.inliers.length,tri:pose.points.length,map:this.map.size,p:[...this.p]};

  // Validate predicted absolute pose against old landmarks before committing it.
  let reproj=null,pnpUsed=0;
  if(corr.length>=10){
    const init=worldToCameraFromCameraToWorld(predRwc,predP), opt=optimizePose(corr,init.Rcw,init.tcw,this.K,10);
    if(opt&&opt.used>=8&&opt.rms<5.0){const abs=cameraToWorldFromWorldToCamera(opt.R,opt.t);if(norm(sub(abs.p,prevP))<0.8){predP=abs.p;predRwc=abs.Rwc;reproj=opt.rms;pnpUsed=opt.used}}
    if(reproj===null){const e=reprojectionRMS(corr,init.Rcw,init.tcw,this.K);if(!Number.isFinite(e)||e>8.0)return{ok:false,held:true,stage:'reprojection rejected',motionPx,gyroMag,ein:er.inliers.length,tri:pose.points.length,map:this.map.size,reproj:e,p:[...this.p]}}
  }

  this.p=predP;this.Rwc=predRwc;this.stats.essentialAccepted++;
  // Triangulated points live in the PREVIOUS camera frame. Transform with the previous world pose, not the updated pose.
  for(const item of pose.points){
    const Pw=add(prevP,mv(prevRwc,scale(item.P,this.relativeScale),3,3));
    if(!Pw.every(Number.isFinite))continue;
    const old=this.map.get(item.track.id);
    if(!old)this.map.set(item.track.id,{P:Pw,obs:1,last:this.frame});
    else{old.obs++;old.last=this.frame}
  }
  // Bound stale landmarks; current tracker IDs provide natural lifecycle.
  if(this.map.size>2500){for(const [id,m] of this.map){if(this.frame-m.last>120)this.map.delete(id);if(this.map.size<=2000)break}}
  this.lastAccepted={ok:true,stage:reproj===null?'essential+triangulation':'essential + PnP validated',
ein:er.inliers.length,tri:pose.points.length,map:this.map.size,reproj,p:[...this.p],R:this.Rwc,motionPx,gyroMag,pnpUsed,
essentialIds:er.inliers.map(q=>q.id),
relative:{R:pose.R,t:pose.t},
triEvidence:pose.points.slice(0,80).map(it=>({id:it.track.id,P:it.P.map(v=>+v.toFixed(6)),error:+it.error.toFixed(6),parallaxDeg:+(it.parallaxDeg||0).toFixed(4),depth1:+(it.depth1||0).toFixed(5),depth2:+(it.depth2||0).toFixed(5)}))
};return this.lastAccepted;
 }
}
