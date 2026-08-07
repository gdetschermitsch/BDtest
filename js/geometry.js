import{dot,norm,sub,add,scale,cross,mm,mv,tr,symEig,svd3,det3,I3,rodrigues,solve}from'./linalg.js';
function normPt(p,K){return[(p.x-K.cx)/K.fx,(p.y-K.cy)/K.fy,1]}
function estimateE(sample,K){const A=[];for(const q of sample){const a=normPt({x:q.prevX,y:q.prevY},K),b=normPt(q,K);A.push(b[0]*a[0],b[0]*a[1],b[0],b[1]*a[0],b[1]*a[1],b[1],a[0],a[1],1)}let AtA=new Array(81).fill(0);for(let r=0;r<9;r++)for(let c=0;c<9;c++)for(let i=0;i<sample.length;i++)AtA[r*9+c]+=A[i*9+r]*A[i*9+c];let e=symEig(AtA,9)[0].vector;const {U,S,V}=svd3(e),s=(S[0]+S[1])/2,D=[s,0,0,0,s,0,0,0,0];return mm(mm(U,D,3,3,3),tr(V,3,3),3,3,3)}
function sampson(E,q,K){const x=normPt({x:q.prevX,y:q.prevY},K),xp=normPt(q,K),Ex=mv(E,x,3,3),Etx=mv(tr(E,3,3),xp,3,3),v=dot(xp,Ex);return v*v/(Ex[0]**2+Ex[1]**2+Etx[0]**2+Etx[1]**2+1e-12)}
export function essentialRansac(tracks,K,iters=180,threshold=2e-4){if(tracks.length<16)return null;let best=null;for(let it=0;it<iters;it++){const s=[],used=new Set;while(s.length<8){const i=(Math.random()*tracks.length)|0;if(!used.has(i)){used.add(i);s.push(tracks[i])}}const E=estimateE(s,K),inliers=tracks.filter(q=>sampson(E,q,K)<threshold);if(!best||inliers.length>best.inliers.length)best={E,inliers}}if(!best||best.inliers.length<14)return null;best.E=estimateE(best.inliers,K);return best}
function triangulateRay(q,R,t,K){const d1=normPt({x:q.prevX,y:q.prevY},K),d2=normPt(q,K),Rt=tr(R,3,3),C2=scale(mv(Rt,t,3,3),-1),v1=scale(d1,1/norm(d1)),v2=mv(Rt,d2,3,3);for(let i=0;i<3;i++)v2[i]/=norm(v2);const a=dot(v1,v1),b=dot(v1,v2),c=dot(v2,v2),w0=scale(C2,-1),d=dot(v1,w0),e=dot(v2,w0),den=a*c-b*b;if(Math.abs(den)<1e-7)return null;const s=(b*e-c*d)/den,u=(a*e-b*d)/den,P1=scale(v1,s),P2=add(C2,scale(v2,u)),P=scale(add(P1,P2),.5),Pc2=add(mv(R,P,3,3),t);if(P[2]<=.03||Pc2[2]<=.03)return null;return{P,error:norm(sub(P1,P2))}}
function decomposeE(E){const{U,V}=svd3(E),W=[0,-1,0,1,0,0,0,0,1],Wt=tr(W,3,3),Vt=tr(V,3,3),R1=mm(mm(U,W,3,3,3),Vt,3,3,3),R2=mm(mm(U,Wt,3,3,3),Vt,3,3,3);if(det3(R1)<0)for(let i=0;i<9;i++)R1[i]*=-1;if(det3(R2)<0)for(let i=0;i<9;i++)R2[i]*=-1;const t=[U[2],U[5],U[8]];return[[R1,t],[R1,scale(t,-1)],[R2,t],[R2,scale(t,-1)]]}
export function recoverPose(E,inliers,K){let best=null;for(const [R,t] of decomposeE(E)){const pts=[];for(const q of inliers.slice(0,100)){const p=triangulateRay(q,R,t,K);if(p&&p.error<.2)pts.push({track:q,...p})}if(!best||pts.length>best.points.length)best={R,t,points:pts}}return best&&best.points.length>=10?best:null}
function project(P,R,t,K){const c=add(mv(R,P,3,3),t);if(c[2]<=1e-4)return null;return{x:K.fx*c[0]/c[2]+K.cx,y:K.fy*c[1]/c[2]+K.cy,z:c[2]}}
export function optimizePose(corr,R0,t0,K,iters=7){
  let w=[0,0,0], t=t0.slice();
  for(let it=0; it<iters; it++){
    const R=mm(rodrigues(w),R0,3,3,3), H=new Array(36).fill(0), g=new Array(6).fill(0);
    let used=0, err=0;
    for(const c of corr){
      const p=project(c.P,R,t,K); if(!p) continue;
      const rx=c.u-p.x, ry=c.v-p.y; if(Math.hypot(rx,ry)>12) continue;
      used++; err+=rx*rx+ry*ry;
      const J=[[],[]], eps=1e-4;
      for(let k=0;k<6;k++){
        const ww=w.slice(), tt=t.slice(); if(k<3)ww[k]+=eps;else tt[k-3]+=eps;
        const pp=project(c.P,mm(rodrigues(ww),R0,3,3,3),tt,K); if(!pp) continue;
        J[0][k]=(pp.x-p.x)/eps; J[1][k]=(pp.y-p.y)/eps;
      }
      for(let a=0;a<6;a++){
        g[a]+=J[0][a]*rx+J[1][a]*ry;
        for(let b=0;b<6;b++)H[a*6+b]+=J[0][a]*J[0][b]+J[1][a]*J[1][b];
      }
    }
    if(used<8)return null;
    for(let i=0;i<6;i++)H[i*6+i]+=1e-3;
    const d=solve(H,g,6); if(!d)return null;
    for(let i=0;i<3;i++){w[i]+=d[i];t[i]+=d[i+3]}
    if(norm(d)<1e-5)return{R:mm(rodrigues(w),R0,3,3,3),t,rms:Math.sqrt(err/(2*used)),used};
  }
  const R=mm(rodrigues(w),R0,3,3,3);let e=0,n=0;
  for(const c of corr){const p=project(c.P,R,t,K);if(p){e+=(c.u-p.x)**2+(c.v-p.y)**2;n++}}
  return{R,t,rms:Math.sqrt(e/(2*Math.max(1,n))),used:n};
}
