import{project}from'./geometry.js';import{mv,tr,scale,solve,norm,sub}from'./linalg.js';
export function optimizeTranslationKnownR(corr,Rcw,t0,K,iters=10){
 let t=t0.slice(),lambda=1e-3;
 function ev(tt){let e=0,rows=[];for(const c of corr){const p=project(c.P,Rcw,tt,K);if(!p)continue;const rx=c.u-p.x,ry=c.v-p.y,d=Math.hypot(rx,ry);if(d>20)continue;rows.push({c,p,rx,ry});e+=rx*rx+ry*ry}return{rows,e,rms:rows.length?Math.sqrt(e/(2*rows.length)):Infinity}}
 let cur=ev(t);if(cur.rows.length<4)return null;
 for(let it=0;it<iters;it++){const H=new Array(9).fill(0),g=new Array(3).fill(0),eps=1e-4;
   for(const row of cur.rows){const Jx=[],Jy=[];for(let k=0;k<3;k++){const tt=t.slice();tt[k]+=eps;const p=project(row.c.P,Rcw,tt,K);Jx[k]=p?(p.x-row.p.x)/eps:0;Jy[k]=p?(p.y-row.p.y)/eps:0}
     for(let a=0;a<3;a++){g[a]+=Jx[a]*row.rx+Jy[a]*row.ry;for(let b=0;b<3;b++)H[a*3+b]+=Jx[a]*Jx[b]+Jy[a]*Jy[b]}}
   for(let i=0;i<3;i++)H[i*3+i]+=lambda;const d=solve(H,g,3);if(!d)break;const cand=ev(t.map((v,i)=>v+d[i]));
   if(cand.rows.length>=4&&cand.e<cur.e){t=t.map((v,i)=>v+d[i]);cur=cand;lambda*=.4;if(norm(d)<1e-6)break}else lambda*=6
 }
 return{t,rms:cur.rms,used:cur.rows.length}
}
function error(c,R,t,K){const p=project(c.P,R,t,K);return p?Math.hypot(p.x-c.u,p.y-c.v):999}
export function pnpTranslationRansac(corr,Rcw,tSeed,K,iters=100,threshold=3.2){
 if(corr.length<6)return null;let best=null;
 for(let it=0;it<iters;it++){const s=[],used=new Set;while(s.length<4){const i=(Math.random()*corr.length)|0;if(!used.has(i)){used.add(i);s.push(corr[i])}}
   const o=optimizeTranslationKnownR(s,Rcw,tSeed,K,7);if(!o)continue;const inl=corr.filter(c=>error(c,Rcw,o.t,K)<threshold);
   if(!best||inl.length>best.inliers.length)best={t:o.t,inliers:inl}
 }
 if(!best||best.inliers.length<6)return null;const ref=optimizeTranslationKnownR(best.inliers,Rcw,best.t,K,12);if(!ref)return null;
 const Rwc=tr(Rcw,3,3),p=scale(mv(Rwc,ref.t,3,3),-1);return{p,t:ref.t,rms:ref.rms,used:ref.used,inliers:best.inliers}
}