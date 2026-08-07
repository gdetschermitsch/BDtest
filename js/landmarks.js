import{project}from'./geometry.js';import{descDistance}from'./matcher.js';
export class LandmarkMap{
 constructor(){this.reset()}
 reset(){this.items=new Map();this.next=1;this.trackLink=new Map()}
 add(P,obs,quality={}){
   const id=this.next++;this.items.set(id,{id,P:P.slice(),desc:obs.desc?.slice()||null,obs:2,last:performance.now(),confirmations:2,quality:{...quality}});if(obs.id!=null)this.trackLink.set(obs.id,id);return id
 }
 observe(id,obs){const m=this.items.get(id);if(!m)return;m.obs++;m.last=performance.now();if(obs.id!=null)this.trackLink.set(obs.id,id);if(obs.desc&&m.desc){const w=Math.min(.15,1/m.obs);for(let i=0;i<m.desc.length;i++)m.desc[i]=(1-w)*m.desc[i]+w*obs.desc[i]}}
 correspondences(observations,Rcw,t,K){
   const corr=[],usedObs=new Set(),now=performance.now();
   // first exact active track links
   for(const o of observations){const lid=this.trackLink.get(o.id),m=lid&&this.items.get(lid);if(m){corr.push({P:m.P,u:o.x,v:o.y,lid,oid:o.id,desc:o.desc});usedObs.add(o.id);m.last=now}}
   // then projected descriptor re-identification
   for(const m of this.items.values()){
     if(corr.some(c=>c.lid===m.id))continue;
     const p=project(m.P,Rcw,t,K);if(!p)continue;let best=null,second=Infinity;
     for(const o of observations){if(usedObs.has(o.id)||Math.hypot(o.x-p.x,o.y-p.y)>24)continue;const d=descDistance(m.desc,o.desc);if(!best||d<best.d){second=best?best.d:second;best={o,d}}else if(d<second)second=d}
     if(best&&best.d<.68&&best.d/(second||9)<.84){corr.push({P:m.P,u:best.o.x,v:best.o.y,lid:m.id,oid:best.o.id,desc:best.o.desc});usedObs.add(best.o.id);this.trackLink.set(best.o.id,m.id)}
   }
   return corr
 }
 confirm(corr){for(const c of corr){const m=this.items.get(c.lid);if(m){m.confirmations++;m.obs++;m.last=performance.now()}}}
 qualifiedCount(){let n=0;for(const m of this.items.values())if(m.confirmations>=3&&m.quality.parallaxDeg>=.5)n++;return n}
 prune(){const now=performance.now();if(this.items.size<1800)return;for(const[id,m]of this.items){if(now-m.last>12000&&m.confirmations<4)this.items.delete(id);if(this.items.size<=1400)break}}
}