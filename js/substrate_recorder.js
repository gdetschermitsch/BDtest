export class SubstrateRecorder{
constructor(){this.reset()}
reset(){this.meta={};this.motion=[];this.orientation=[];this.frames=[];this.derived=[];this.events=[];this.t0=null;this.phase='idle'}
start(meta){this.reset();this.t0=performance.now();this.meta={...meta,startedISO:new Date().toISOString()};this.event('start')}
time(){return +(performance.now()-this.t0).toFixed(3)}
event(type,data={}){if(this.t0!=null)this.events.push({t:this.time(),type,...data})}
setPhase(p){if(p!==this.phase){this.phase=p;this.event('phase',{phase:p})}}
motionSample(s){if(this.t0==null)return;this.motion.push({t:this.time(),gravity:s.gravity,accel:s.accel,rateDegPerSec:s.rate})}
orientationSample(s){if(this.t0==null)return;this.orientation.push({t:this.time(),heading:s.heading,alpha:s.alpha,beta:s.beta,gamma:s.gamma,absolute:s.absolute})}
frame(d){if(this.t0!=null)this.frames.push({t:this.time(),phase:this.phase,...d})}
derive(d){if(this.t0!=null)this.derived.push({t:this.time(),phase:this.phase,...d})}
stop(){this.event('stop');this.meta.durationMs=this.time()}
download(extra={}){const p={format:'CRUXTAIN_OBJECTIVE_SUBSTRATE',version:'1.0',definitions:{world:'+Y gravity-up; horizontal heading seeded from compass; origin = initialized device position; right-handed renderer conversion deferred',raw:'Browser-exposed measurements only',normalized:'unit/frame normalization only',derived:'mathematical hypotheses; never treated as raw measurement'},meta:{...this.meta,...extra},events:this.events,motion:this.motion,orientation:this.orientation,frames:this.frames,derived:this.derived};const b=new Blob([JSON.stringify(p)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(b);a.download=`CRUXTAIN_Substrate_${new Date().toISOString().replace(/[:.]/g,'-')}.json`;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
}