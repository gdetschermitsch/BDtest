import{mv,tr,cross,dot,norm,normalize,symEig,add,sub,scale}from'./linalg.js';

export function ray(p,K){return normalize([(p.x-K.cx)/K.fx,(p.y-K.cy)/K.fy,1])}
export function projectRay(v,K){if(v[2]<=1e-8)return null;return{x:K.fx*v[0]/v[2]+K.cx,y:K.fy*v[1]/v[2]+K.cy}}
export function derotatedResiduals(matches,R21,K){
 const rows=[];for(const q of matches){const a=ray({x:q.x1,y:q.y1},K),pred=projectRay(mv(R21,a,3,3),K);if(!pred)continue;rows.push({...q,rx:q.x2-pred.x,ry:q.y2-pred.y,r:Math.hypot(q.x2-pred.x,q.y2-pred.y)})}return rows
}
function tFromRows(rows){
 let A=new Array(9).fill(0);
 for(const r of rows){const a=ray({x:r.x1,y:r.y1},r.K),b=ray({x:r.x2,y:r.y2},r.K),v=cross(b,mv(r.R,a,3,3));for(let i=0;i<3;i++)for(let j=0;j<3;j++)A[i*3+j]+=v[i]*v[j]}
 return normalize(symEig(A,3)[0].vector)
}
function epiResidual(q,R,t,K){const a=ray({x:q.x1,y:q.y1},K),b=ray({x:q.x2,y:q.y2},K),v=cross(b,mv(R,a,3,3));return Math.abs(dot(v,t))/(norm(v)+1e-12)}
export function knownRTranslationRansac(matches,R21,K,iters=180,threshold=0.004){
 if(matches.length<14)return null;let best=null;
 const rows=matches.map(q=>({...q,R:R21,K}));
 for(let it=0;it<iters;it++){
   const s=[],used=new Set;while(s.length<3){const i=(Math.random()*rows.length)|0;if(!used.has(i)){used.add(i);s.push(rows[i])}}
   const t=tFromRows(s),inliers=matches.filter(q=>epiResidual(q,R21,t,K)<threshold);
   if(!best||inliers.length>best.inliers.length)best={t,inliers}
 }
 if(!best||best.inliers.length<12)return null;
 const t=tFromRows(best.inliers.map(q=>({...q,R:R21,K})));
 return{t,inliers:best.inliers,residual:best.inliers.reduce((s,q)=>s+epiResidual(q,R21,t,K),0)/best.inliers.length}
}
export function triangulateRelative(q,R21,t21,K){
 const d1=ray({x:q.x1,y:q.y1},K),d2=ray({x:q.x2,y:q.y2},K),Rt=tr(R21,3,3),C2=scale(mv(Rt,t21,3,3),-1),v1=d1,v2=normalize(mv(Rt,d2,3,3));
 const w0=scale(C2,-1),a=dot(v1,v1),b=dot(v1,v2),c=dot(v2,v2),d=dot(v1,w0),e=dot(v2,w0),den=a*c-b*b;if(Math.abs(den)<1e-8)return null;
 const s=(b*e-c*d)/den,u=(a*e-b*d)/den,P1=scale(v1,s),P2=add(C2,scale(v2,u)),P=scale(add(P1,P2),.5),P2c=add(mv(R21,P,3,3),t21);
 if(P[2]<=0.03||P2c[2]<=0.03)return null;
 const ca=Math.max(-1,Math.min(1,dot(v1,v2))),parallax=Math.acos(ca)*180/Math.PI;
 return{P,error:norm(sub(P1,P2)),parallaxDeg:parallax,depth1:P[2],depth2:P2c[2]}
}
export function chooseTranslationSign(inliers,R21,t,K){
 function score(tt){const pts=[];for(const q of inliers){const p=triangulateRelative(q,R21,tt,K);if(p&&p.error<.12&&p.parallaxDeg>.25)pts.push({match:q,...p})}return pts}
 const a=score(t),b=score(scale(t,-1));return a.length>=b.length?{t,points:a}:{t:scale(t,-1),points:b}
}
