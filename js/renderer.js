import{tr,mv,sub}from'./linalg.js';
export class WorldRenderer{
 constructor(canvas){this.c=canvas;this.x=canvas.getContext('2d')}
 draw(state,K,W,H){
   this.c.width=innerWidth*devicePixelRatio;this.c.height=innerHeight*devicePixelRatio;this.x.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);this.x.clearRect(0,0,innerWidth,innerHeight);
   if(!state.metricPosition||state.scaleState!=='METRIC LOCKED')return;
   const Rcw=tr(state.Rwc,3,3),p=state.metricPosition,sx=innerWidth/W,sy=innerHeight/H;
   const P=q=>{const d=sub(q,p),c=mv(Rcw,d,3,3);if(c[2]<.05)return null;return{x:(K.fx*c[0]/c[2]+K.cx)*sx,y:(K.fy*c[1]/c[2]+K.cy)*sy}}
   this.x.lineWidth=1;
   for(let z=-5;z<=5;z+=.5)for(let axis=0;axis<2;axis++){const a=axis?[z,0,-5]:[-5,0,z],b=axis?[z,0,5]:[5,0,z],u=P(a),v=P(b);if(!u||!v)continue;this.x.strokeStyle=Math.abs(z)<.01?'rgba(255,255,255,.9)':'rgba(255,255,255,.28)';this.x.beginPath();this.x.moveTo(u.x,u.y);this.x.lineTo(v.x,v.y);this.x.stroke()}
 }
}