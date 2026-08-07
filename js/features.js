function grayFromRGBA(data,w,h,out){for(let i=0,j=0;i<w*h;i++,j+=4)out[i]=(77*data[j]+150*data[j+1]+29*data[j+2])>>8}
function median(a){if(!a.length)return 0;const b=[...a].sort((x,y)=>x-y),m=b.length>>1;return b.length&1?b[m]:(b[m-1]+b[m])/2}
export class FeatureTracker{
 constructor(w=320,h=180){this.configure(w,h)}
 configure(w,h){
   this.w=w;this.h=h;this.canvas=document.createElement('canvas');this.canvas.width=w;this.canvas.height=h;this.ctx=this.canvas.getContext('2d',{willReadFrequently:true});
   this.prev=new Uint8Array(w*h);this.curr=new Uint8Array(w*h);this.points=[];this.initial=false;this.id=1;this.frame=0;
 }
 grab(video){this.ctx.drawImage(video,0,0,this.w,this.h);grayFromRGBA(this.ctx.getImageData(0,0,this.w,this.h).data,this.w,this.h,this.curr)}
 signature(x,y){
   // 4x4 averages over an 8x8 neighborhood: compact appearance evidence, not an image.
   const out=[],w=this.w,h=this.h;
   for(let by=-4;by<4;by+=2)for(let bx=-4;bx<4;bx+=2){
     let s=0,n=0;
     for(let yy=0;yy<2;yy++)for(let xx=0;xx<2;xx++){
       const X=Math.max(0,Math.min(w-1,Math.round(x+bx+xx))),Y=Math.max(0,Math.min(h-1,Math.round(y+by+yy)));
       s+=this.curr[Y*w+X];n++;
     }
     out.push(Math.round(s/n));
   }
   return out;
 }
 detect(img,max=320){
   const w=this.w,h=this.h,c=[];
   for(let y=5;y<h-5;y+=2)for(let x=5;x<w-5;x+=2){
     let sxx=0,syy=0,sxy=0;
     for(let yy=-2;yy<=2;yy++)for(let xx=-2;xx<=2;xx++){const i=(y+yy)*w+x+xx,gx=img[i+1]-img[i-1],gy=img[i+w]-img[i-w];sxx+=gx*gx;syy+=gy*gy;sxy+=gx*gy}
     const tr=sxx+syy,det=sxx*syy-sxy*sxy,l=.5*(tr-Math.sqrt(Math.max(0,tr*tr-4*det)));
     if(l>9000)c.push({x,y,q:l})
   }
   c.sort((a,b)=>b.q-a.q);const o=[];
   for(const p of c){if(o.every(q=>(p.x-q.x)**2+(p.y-q.y)**2>64)){o.push({x:p.x,y:p.y,id:this.id++,age:1,q:p.q});if(o.length>=max)break}}
   return o
 }
 trackOne(p){
   const w=this.w,h=this.h,r=3,search=9;
   if(p.x<r+search||p.y<r+search||p.x>=w-r-search||p.y>=h-r-search)return null;
   let best=1e30,bx=0,by=0,second=1e30;
   for(let dy=-search;dy<=search;dy++)for(let dx=-search;dx<=search;dx++){
     let s=0;
     for(let yy=-r;yy<=r;yy++){let a=(p.y+yy)*w+p.x-r,b=(p.y+dy+yy)*w+p.x+dx-r;for(let xx=-r;xx<=r;xx++){const d=this.prev[a++]-this.curr[b++];s+=d*d}}
     if(s<best){second=best;best=s;bx=dx;by=dy}else if(s<second)second=s
   }
   if(best>9000||best/(second||1)>.92)return null;
   return{x:p.x+bx,y:p.y+by,id:p.id,age:p.age+1,prevX:p.x,prevY:p.y,error:best,ratio:best/(second||1)}
 }
 process(video){
   this.frame++;this.grab(video);
   if(!this.initial){this.points=this.detect(this.curr);this.prev.set(this.curr);this.initial=true;return{detected:this.points.length,tracks:[],motionPx:0,newFrame:true,seeded:this.points.length}}
   let tracks=[];for(const p of this.points){const q=this.trackOne(p);if(q)tracks.push(q)}
   const disp=tracks.map(q=>Math.hypot(q.x-q.prevX,q.y-q.prevY)),motionPx=median(disp);
   let next=tracks.map(q=>({x:q.x,y:q.y,id:q.id,age:q.age}));
   let seeded=0;
   if(next.length<180){const fresh=this.detect(this.curr,320);for(const p of fresh){if(next.every(q=>(p.x-q.x)**2+(p.y-q.y)**2>81)){next.push(p);seeded++;if(next.length>=320)break}}}
   this.points=next;this.prev.set(this.curr);
   return{detected:next.length,tracks,motionPx,newFrame:true,seeded}
 }

 descriptor(x,y){
   const vals=[],w=this.w,h=this.h;
   // 8x8 descriptor from a 16x16 patch; normalized removes brightness/contrast offsets.
   for(let gy=-7;gy<=7;gy+=2)for(let gx=-7;gx<=7;gx+=2){
     const X=Math.max(0,Math.min(w-1,Math.round(x+gx))),Y=Math.max(0,Math.min(h-1,Math.round(y+gy)));
     vals.push(this.curr[Y*w+X]/255);
   }
   const mean=vals.reduce((s,v)=>s+v,0)/vals.length;
   let n=0;for(let i=0;i<vals.length;i++){vals[i]-=mean;n+=vals[i]*vals[i]}
   n=Math.sqrt(n)||1;return vals.map(v=>v/n)
 }
 observations(max=260){
   return [...this.points].sort((a,b)=>(b.age||1)-(a.age||1)).slice(0,max).map(p=>({
     id:p.id,x:p.x,y:p.y,age:p.age||1,desc:this.descriptor(p.x,p.y)
   }))
 }

 telemetry(tracks,max=100){
   // Long-lived tracks first, capped to keep the export small.
   return [...tracks].sort((a,b)=>b.age-a.age).slice(0,max).map(q=>({
     id:q.id,age:q.age,x:+q.x.toFixed(2),y:+q.y.toFixed(2),px:+q.prevX.toFixed(2),py:+q.prevY.toFixed(2),
     err:+q.error.toFixed(1),ratio:+q.ratio.toFixed(4),sig:this.signature(q.x,q.y)
   }));
 }
 reset(){this.initial=false;this.points=[];this.id=1;this.frame=0}
}