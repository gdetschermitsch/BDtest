
function matPoints(points){
  const data=[];
  for(const p of points){ data.push(p.x,p.y); }
  return cv.matFromArray(points.length,1,cv.CV_32FC2,data);
}
function pointsFromMat(mat){
  const out=[];
  for(let i=0;i<mat.rows;i++) out.push({x:mat.data32F[i*2],y:mat.data32F[i*2+1]});
  return out;
}
export class VisionGeometry {
  constructor(width=320,height=240){
    this.w=width; this.h=height;
    this.canvas=document.createElement('canvas');
    this.canvas.width=width; this.canvas.height=height;
    this.ctx=this.canvas.getContext('2d',{willReadFrequently:true});
    this.prevGray=null;
    this.prevPoints=[];
    this.frameIndex=0;
    this.keyframe=null;
    this.keyframes=0;
    this.landmarks=[];
    this.pose={R:null,t:[0,0,0]};
    this.K=null;
  }
  initCameraModel(){
    const f=0.9*Math.max(this.w,this.h);
    this.K=cv.matFromArray(3,3,cv.CV_64F,[f,0,this.w/2,0,f,this.h/2,0,0,1]);
  }
  capture(video){
    this.ctx.drawImage(video,0,0,this.w,this.h);
    const rgba=cv.matFromImageData(this.ctx.getImageData(0,0,this.w,this.h));
    const gray=new cv.Mat();
    cv.cvtColor(rgba,gray,cv.COLOR_RGBA2GRAY);
    rgba.delete();
    return gray;
  }
  detect(gray){
    const corners=new cv.Mat(), mask=new cv.Mat();
    cv.goodFeaturesToTrack(gray,corners,300,0.008,7,mask,7,false,0.04);
    mask.delete();
    const pts=pointsFromMat(corners); corners.delete();
    return pts;
  }
  process(video){
    if(!this.K) this.initCameraModel();
    const gray=this.capture(video);
    this.frameIndex++;
    if(!this.prevGray){
      this.prevGray=gray;
      this.prevPoints=this.detect(gray);
      return {stage:1,features:this.prevPoints,tracked:[],inliers:[],pose:null,landmarks:this.landmarks};
    }
    if(this.prevPoints.length<60) this.prevPoints=this.detect(this.prevGray);
    if(this.prevPoints.length<8){
      this.prevGray.delete(); this.prevGray=gray;
      return {stage:1,features:this.prevPoints,tracked:[],inliers:[],pose:null,landmarks:this.landmarks,error:"insufficient features"};
    }
    const prevPts=matPoints(this.prevPoints), nextPts=new cv.Mat(), status=new cv.Mat(), err=new cv.Mat();
    cv.calcOpticalFlowPyrLK(this.prevGray,gray,prevPts,nextPts,status,err,new cv.Size(21,21),3,
      new cv.TermCriteria(cv.TermCriteria_COUNT|cv.TermCriteria_EPS,30,0.01));
    const nextArr=pointsFromMat(nextPts);
    const tracked=[];
    for(let i=0;i<this.prevPoints.length;i++){
      if(status.data[i] && err.data32F[i]<30){
        const p=this.prevPoints[i],q=nextArr[i];
        if(q.x>2&&q.y>2&&q.x<this.w-2&&q.y<this.h-2) tracked.push({p,q});
      }
    }
    prevPts.delete(); nextPts.delete(); status.delete(); err.delete();

    let geometry=null, inliers=[];
    if(tracked.length>=12 && typeof cv.findEssentialMat==="function" && typeof cv.recoverPose==="function"){
      const p1=matPoints(tracked.map(t=>t.p)), p2=matPoints(tracked.map(t=>t.q)), mask=new cv.Mat();
      try{
        const E=cv.findEssentialMat(p1,p2,this.K,cv.RANSAC,0.999,1.25,mask);
        const R=new cv.Mat(),tvec=new cv.Mat();
        const count=cv.recoverPose(E,p1,p2,this.K,R,tvec,mask);
        for(let i=0;i<tracked.length;i++) if(mask.data[i]) inliers.push(tracked[i]);
        const trans=[tvec.data64F[0],tvec.data64F[1],tvec.data64F[2]];
        const medianParallax=inliers.length?inliers.map(v=>Math.hypot(v.q.x-v.p.x,v.q.y-v.p.y)).sort((a,b)=>a-b)[Math.floor(inliers.length/2)]:0;
        geometry={E,R,tvec,trans,count,medianParallax};
      }catch(e){ geometry={error:e.message}; }
      p1.delete();p2.delete();mask.delete();
    }

    let poseUpdate=null;
    if(geometry && !geometry.error && inliers.length>=18 && geometry.medianParallax>1.2){
      // Essential-matrix translation is unit length: accumulated pose remains map-scale.
      const s=0.05;
      this.pose.t[0]+=geometry.trans[0]*s;
      this.pose.t[1]+=geometry.trans[1]*s;
      this.pose.t[2]+=geometry.trans[2]*s;
      this.pose.R=geometry.R.clone();
      this.keyframes++;
      poseUpdate={t:[...this.pose.t],R:this.pose.R,inliers:inliers.length};

      // Triangulate only on accepted geometric keyframes.
      if(typeof cv.triangulatePoints==="function"){
        try{
          const P1=cv.matFromArray(3,4,cv.CV_64F,[1,0,0,0,0,1,0,0,0,0,1,0]);
          const rv=geometry.R.data64F, tv=geometry.tvec.data64F;
          const P2=cv.matFromArray(3,4,cv.CV_64F,[rv[0],rv[1],rv[2],tv[0],rv[3],rv[4],rv[5],tv[1],rv[6],rv[7],rv[8],tv[2]]);
          const KP1=new cv.Mat(),KP2=new cv.Mat(); cv.gemm(this.K,P1,1,new cv.Mat(),0,KP1); cv.gemm(this.K,P2,1,new cv.Mat(),0,KP2);
          const a=matPoints(inliers.map(v=>v.p)),b=matPoints(inliers.map(v=>v.q));
          const a2=a.reshape(1,2),b2=b.reshape(1,2),hom=new cv.Mat();
          cv.triangulatePoints(KP1,KP2,a2,b2,hom);
          let added=0;
          for(let i=0;i<hom.cols;i++){
            const w=hom.data32F[i+3*hom.cols];
            if(Math.abs(w)<1e-6)continue;
            const X=hom.data32F[i]/w,Y=hom.data32F[i+hom.cols]/w,Z=hom.data32F[i+2*hom.cols]/w;
            if(Number.isFinite(X+Y+Z)&&Z>0&&Z<100){this.landmarks.push({x:X,y:Y,z:Z,age:1});added++}
          }
          if(this.landmarks.length>3000)this.landmarks.splice(0,this.landmarks.length-3000);
          P1.delete();P2.delete();KP1.delete();KP2.delete();a.delete();b.delete();a2.delete();b2.delete();hom.delete();
        }catch(e){ geometry.triangulationError=e.message; }
      }
    }

    const currentPoints=tracked.map(t=>t.q);
    this.prevGray.delete(); this.prevGray=gray;
    this.prevPoints=currentPoints.length>=60?currentPoints:this.detect(gray);
    return {stage:poseUpdate?3:2,features:this.prevPoints,tracked,inliers,pose:poseUpdate,geometry,landmarks:this.landmarks,keyframes:this.keyframes};
  }
  reset(){
    if(this.prevGray)this.prevGray.delete();
    if(this.K)this.K.delete();
    if(this.pose.R)this.pose.R.delete();
    this.prevGray=null;this.K=null;this.prevPoints=[];this.keyframes=0;this.landmarks=[];this.pose={R:null,t:[0,0,0]};
  }
}
