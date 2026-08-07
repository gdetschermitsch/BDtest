import{solve,norm,sub}from'./linalg.js';
export class ScaleEstimator{
 constructor(){this.reset()}
 reset(){this.intervals=[];this.history=[];this.scale=null;this.state='UNRESOLVED';this.residual=Infinity}
 addInterval(dpVis,preint){
   if(!preint||preint.dt<=0||norm(dpVis)<1e-4)return null;this.intervals.push({dp:dpVis.slice(),...preint});if(this.intervals.length>12)this.intervals.shift();return this.solve()
 }
 solve(){
   if(this.intervals.length<3){this.state='NEED EXCITATION';return null}
   // Unknowns [s,v0x,v0y,v0z]. v_i = v0 + cumulative beta.
   const rows=[],rhs=[];let B=[0,0,0];
   for(const I of this.intervals){for(let k=0;k<3;k++){const r=[I.dp[k],0,0,0];r[1+k]=-I.dt;rows.push(r);rhs.push(B[k]*I.dt+I.alpha[k])}for(let k=0;k<3;k++)B[k]+=I.beta[k]}
   const H=new Array(16).fill(0),g=new Array(4).fill(0);for(let i=0;i<rows.length;i++)for(let a=0;a<4;a++){g[a]+=rows[i][a]*rhs[i];for(let b=0;b<4;b++)H[a*4+b]+=rows[i][a]*rows[i][b]}
   for(let i=0;i<4;i++)H[i*4+i]+=1e-8;const x=solve(H,g,4);if(!x||!Number.isFinite(x[0])||x[0]<=.005||x[0]>20){this.state='UNSTABLE';return null}
   let e=0;for(let i=0;i<rows.length;i++){let y=0;for(let k=0;k<4;k++)y+=rows[i][k]*x[k];e+=(y-rhs[i])**2}const rms=Math.sqrt(e/rows.length);
   this.history.push(x[0]);if(this.history.length>8)this.history.shift();const mean=this.history.reduce((s,v)=>s+v,0)/this.history.length,sd=Math.sqrt(this.history.reduce((s,v)=>s+(v-mean)**2,0)/this.history.length),cv=sd/(mean||1);
   this.scale=x[0];this.residual=rms;this.state=this.history.length>=4&&cv<.18&&rms<.12?'METRIC LOCKED':'CONVERGING';return{scale:x[0],velocity0:x.slice(1),rms,cv,state:this.state}
 }
}