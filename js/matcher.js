export function descDistance(a,b){if(!a||!b||a.length!==b.length)return 9;let d=0;for(let i=0;i<a.length;i++){const x=a[i]-b[i];d+=x*x}return Math.sqrt(d)}
export function matchDescriptors(A,B,{maxDist=.72,ratio=.82,predict=null,radius=100}={}){
 const out=[],used=new Set;
 for(const a of A){
   let best=null,second=Infinity;
   const pr=predict?predict(a):null;
   for(const b of B){if(used.has(b.id))continue;if(pr&&Math.hypot(b.x-pr.x,b.y-pr.y)>radius)continue;const d=descDistance(a.desc,b.desc);if(!best||d<best.d){second=best?best.d:second;best={b,d}}else if(d<second)second=d}
   if(best&&best.d<maxDist&&best.d/(second||9)<ratio){used.add(best.b.id);out.push({a,b:best.b,d:best.d})}
 }
 return out
}