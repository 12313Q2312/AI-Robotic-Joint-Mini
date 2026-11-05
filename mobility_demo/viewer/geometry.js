(function(root){
  'use strict';
  const add=(a,b)=>a.map((v,i)=>v+b[i]);
  const mul=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((v,x,k)=>v+x*b[k][j],0)));
  const apply=(r,v)=>r.map(row=>row.reduce((s,x,i)=>s+x*v[i],0));
  const rx=t=>[[1,0,0],[0,Math.cos(t),-Math.sin(t)],[0,Math.sin(t),Math.cos(t)]];
  const ry=t=>[[Math.cos(t),0,Math.sin(t)],[0,1,0],[-Math.sin(t),0,Math.cos(t)]];
  const rz=t=>[[Math.cos(t),-Math.sin(t),0],[Math.sin(t),Math.cos(t),0],[0,0,1]];
  function body(frame) {
    const pos=frame.slice(1,4),r=mul(mul(rz(frame[6]),ry(frame[5])),rx(frame[4]));
    const at=v=>add(pos,apply(r,v));
    const legs=[];
    for(let i=0;i<2;i++) {
      const q=frame.slice(7+i*5,12+i*5);
      const hip=at([0,i===0?-.115:.115,0]);
      const upper=mul(mul(r,rx(q[0])),ry(q[1]));
      const lower=mul(mul(r,rx(q[0])),ry(q[1]+q[2]));
      const knee=add(hip,apply(upper,[0,0,-.42]));
      const ankle=add(knee,apply(lower,[0,0,-.42]));
      const foot=mul(mul(mul(r,rx(q[0])),ry(q[1]+q[2]+q[3])),rx(q[4]));
      legs.push({hip,knee,ankle,foot});
    }
    return {pos,r,at,legs};
  }
  const api={add,mul,apply,rx,ry,rz,body};
  if(typeof module!=='undefined')module.exports=api;
  root.MobilityGeometry=api;
})(typeof globalThis!=='undefined'?globalThis:this);
