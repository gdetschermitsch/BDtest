export const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export function median(a){if(!a.length)return 0;const b=[...a].sort((x,y)=>x-y),m=b.length>>1;return b.length&1?b[m]:(b[m-1]+b[m])/2}
export function mad(a,m=median(a)){return median(a.map(v=>Math.abs(v-m)))||1e-6}
export function rotate2(x,y,a){const c=Math.cos(a),s=Math.sin(a);return [c*x-s*y,s*x+c*y]}
