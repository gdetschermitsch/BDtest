import{dot,norm,sub,add,scale,cross,mm,mv,tr,symEig,svd3,det3,I3,rodrigues,solve}from'./linalg.js';
function normPt(p,K){return[(p.x-K.cx)/K.fx,(p.y-K.cy)/K.fy,1]}
function estimateE(sample,K){const A=[];for(const q of sample){const a=normPt({x:q.prevX,y:q.prevY},K),b=normPt(q,K);A.push(b[0]*a[0],b[0]*a[1],b[0],b[1]*a[0],b[1]*a[1],b[1],a[0],a[1],1)}let AtA=new Array(81).fill(0);for(let r=0;r<9;r++)for(let c=0;c<9;c++)for(let i=0;i<sample.length;i++)AtA[r*9+c]+=A[i*9+r]*A[i*9+c];let e=symEig(AtA,9)[0].vector;const {U,S,V}=svd3(e),s=(S[0]+S[1])/2,D=[s,0,0,0,s,0,0,0,0];return mm(mm(U,D,3,3,3),tr(V,3,3),3,3,3)}
function sampson(E,q,K){const x=normPt({x:q.prevX,y:q.prevY},K),xp=normPt(q,K),Ex=mv(E,x,3,3),Etx=mv(tr(E,3,3),xp,3,3),v=dot(xp,Ex);return v*v/(Ex[0]**2+Ex[1]**2+Etx[0]**2+Etx[1]**2+1e-12)}
export function essentialRansac(tracks,K,iters=180,threshold=2e-4){if(tracks.length<16)return null;let best=null;for(let it=0;it<iters;it++){const s=[],used=new Set;while(s.length<8){const i=(Math.random()*tracks.length)|0;if(!used.has(i)){used.add(i);s.push(tracks[i])}}const E=estimateE(s,K),inliers=tracks.filter(q=>sampson(E,q,K)<threshold);if(!best||inliers.length>best.inliers.length)best={E,inliers}}if(!best||best.inliers.length<14)return null;best.E=estimateE(best.inliers,K);return best}
function triangulateRay(q,R,t,K){const d1=normPt({x:q.prevX,y:q.prevY},K),d2=normPt(q,K),Rt=tr(R,3,3),C2=scale(mv(Rt,t,3,3),-1),v1=scale(d1,1/norm(d1)),v2=mv(Rt,d2,3,3);for(let i=0;i<3;i++)v2[i]/=norm(v2);const a=dot(v1,v1),b=dot(v1,v2),c=dot(v2,v2),w0=scale(C2,-1),d=dot(v1,w0),e=dot(v2,w0),den=a*c-b*b;if(Math.abs(den)<1e-7)return null;const s=(b*e-c*d)/den,u=(a*e-b*d)/den,P1=scale(v1,s),P2=add(C2,scale(v2,u)),P=scale(add(P1,P2),.5),Pc2=add(mv(R,P,3,3),t);if(P[2]<=.03||Pc2[2]<=.03)return null;
const ca=Math.max(-1,Math.min(1,dot(v1,v2)/(norm(v1)*norm(v2)+1e-12)));
const parallaxDeg=Math.acos(ca)*180/Math.PI;
return{P,error:norm(sub(P1,P2)),parallaxDeg,depth1:P[2],depth2:Pc2[2]}}
function decomposeE(E){const{U,V}=svd3(E),W=[0,-1,0,1,0,0,0,0,1],Wt=tr(W,3,3),Vt=tr(V,3,3),R1=mm(mm(U,W,3,3,3),Vt,3,3,3),R2=mm(mm(U,Wt,3,3,3),Vt,3,3,3);if(det3(R1)<0)for(let i=0;i<9;i++)R1[i]*=-1;if(det3(R2)<0)for(let i=0;i<9;i++)R2[i]*=-1;const t=[U[2],U[5],U[8]];return[[R1,t],[R1,scale(t,-1)],[R2,t],[R2,scale(t,-1)]]}
export function recoverPose(E,inliers,K){let best=null;for(const [R,t] of decomposeE(E)){const pts=[];for(const q of inliers.slice(0,100)){const p=triangulateRay(q,R,t,K);if(p&&p.error<.2)pts.push({track:q,...p})}if(!best||pts.length>best.points.length)best={R,t,points:pts}}return best&&best.points.length>=10?best:null}
export function project(P,R,t,K){const c=add(mv(R,P,3,3),t);if(c[2]<=1e-4)return null;return{x:K.fx*c[0]/c[2]+K.cx,y:K.fy*c[1]/c[2]+K.cy,z:c[2]}}
export function optimizePose(corr,R0,t0,K,iters=12){
  let w=[0,0,0], t=t0.slice(), lambda=1e-2;
  function evaluate(ww,tt){
    const R=mm(rodrigues(ww),R0,3,3,3), rows=[];
    let e=0,n=0;
    for(const c of corr){
      const p=project(c.P,R,tt,K); if(!p) continue;
      const rx=c.u-p.x, ry=c.v-p.y, d=Math.hypot(rx,ry); if(d>30) continue;
      rows.push({c,p,rx,ry}); e+=rx*rx+ry*ry; n+=2;
    }
    return {R,e,n,rows,rms:n?Math.sqrt(e/n):Infinity};
  }
  let cur=evaluate(w,t); if(cur.rows.length<8) return null;
  for(let it=0; it<iters; it++){
    const H=new Array(36).fill(0), g=new Array(6).fill(0), eps=2e-4;
    for(const row of cur.rows){
      const Jx=new Array(6).fill(0), Jy=new Array(6).fill(0);
      for(let k=0;k<6;k++){
        const ww=w.slice(), tt=t.slice();
        if(k<3) ww[k]+=eps; else tt[k-3]+=eps;
        const Rk=mm(rodrigues(ww),R0,3,3,3), pk=project(row.c.P,Rk,tt,K);
        if(pk){Jx[k]=(pk.x-row.p.x)/eps; Jy[k]=(pk.y-row.p.y)/eps;}
      }
      for(let a=0;a<6;a++){
        g[a]+=Jx[a]*row.rx+Jy[a]*row.ry;
        for(let b=0;b<6;b++) H[a*6+b]+=Jx[a]*Jx[b]+Jy[a]*Jy[b];
      }
    }
    for(let i=0;i<6;i++) H[i*6+i]+=lambda;
    const d=solve(H,g,6); if(!d) return null;
    const wn=w.map((v,i)=>v+d[i]), tn=t.map((v,i)=>v+d[i+3]), cand=evaluate(wn,tn);
    if(cand.rows.length>=8 && cand.e<cur.e){
      w=wn; t=tn; cur=cand; lambda=Math.max(1e-5,lambda*.35);
      if(norm(d)<1e-6) break;
    }else{
      lambda=Math.min(1e5,lambda*8);
    }
  }
  return cur.rows.length>=8 ? {R:cur.R,t,rms:cur.rms,used:cur.rows.length} : null;
}

