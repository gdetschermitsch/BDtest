export class EvidenceRecorder {
  constructor(){ this.reset(); }
  reset(){
    this.meta={};
    this.frames=[];
    this.motion=[];
    this.orientation=[];
    this.events=[];
    this.startedAt=null;
    this.endedAt=null;
    this.phase='idle';
  }
  start(meta){
    this.reset();
    this.startedAt=performance.now();
    this.meta={...meta, startedPerformanceMs:this.startedAt, startedISO:new Date().toISOString()};
    this.event('capture-start',{});
  }
  event(type,data={}){
    if(this.startedAt==null)return;
    this.events.push({t:+(performance.now()-this.startedAt).toFixed(3),type,...data});
  }
  setPhase(name){ this.phase=name; this.event('phase',{name}); }
  motionSample(s){
    if(this.startedAt==null)return;
    this.motion.push({t:+(performance.now()-this.startedAt).toFixed(3),
      gravity:s.gravity.map(v=>+v.toFixed(5)),
      accel:s.accel.map(v=>+v.toFixed(5)),
      rate:s.rate.map(v=>+v.toFixed(5))
    });
  }
  orientationSample(s){
    if(this.startedAt==null)return;
    this.orientation.push({t:+(performance.now()-this.startedAt).toFixed(3),
      heading:s.heading==null?null:+s.heading.toFixed(4),
      alpha:s.alpha==null?null:+s.alpha.toFixed(4),
      beta:s.beta==null?null:+s.beta.toFixed(4),
      gamma:s.gamma==null?null:+s.gamma.toFixed(4),
      absolute:!!s.absolute
    });
  }
  frame(d){
    if(this.startedAt==null)return;
    this.frames.push({phase:this.phase,...d});
  }
  stop(){
    if(this.startedAt==null)return;
    this.endedAt=performance.now();
    this.event('capture-stop',{});
    this.meta.durationMs=+(this.endedAt-this.startedAt).toFixed(3);
  }
  payload(extra={}){
    return {
      format:'CRUXTAIN_VIO_EVIDENCE',
      version:'3.1',
      meta:{...this.meta,...extra},
      events:this.events,
      motion:this.motion,
      orientation:this.orientation,
      frames:this.frames
    };
  }
  download(extra={}){
    const blob=new Blob([JSON.stringify(this.payload(extra))],{type:'application/json'});
    const a=document.createElement('a');
    a.href=URL.createObjectURL(blob);
    a.download=`CRUXTAIN_VIO_Evidence_${new Date().toISOString().replace(/[:.]/g,'-')}.json`;
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(()=>URL.revokeObjectURL(a.href),1000);
  }
}