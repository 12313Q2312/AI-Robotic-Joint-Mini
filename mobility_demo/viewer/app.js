/* Playback is reconstructed from measured simulator states, not gait references. */
(()=>{'use strict';
const $=id=>document.getElementById(id),G=MobilityGeometry,data=window.MOBILITY_DEMO;
if(!data||!data.cases.length){$('case-title').textContent='尚无可回放数据。请先完成仿真并生成演示资料。';return;}
const canvas=$('scene'),ctx=canvas.getContext('2d');let current=data.cases[0],clock=0,playing=true,last=0,yaw=-.75,tilt=.26,drag=null;
data.cases.forEach((c,i)=>{const o=document.createElement('option');o.value=i;o.textContent=c.title;$('case').append(o);});
const fmt=(x,n=2)=>Number.isFinite(x)?x.toFixed(n):'—';
function metric(value,label){const d=document.createElement('div');d.className='metric';const b=document.createElement('b');b.textContent=value;const s=document.createElement('small');s.textContent=label;d.append(b,s);return d;}
function choose(i){current=data.cases[i];clock=0;playing=true;$('play').textContent='暂停';$('case-title').textContent=current.description||current.title;
 const s=current.summary,m=$('metrics');m.replaceChildren();
 if(current.type==='walking'){
  m.append(metric(fmt(s.forward_distance_m)+' m','实际前进距离'),metric(fmt(s.human_equivalent_positive_work_j,0)+' J','人体等效正功'),metric(fmt(s.max_trunk_tilt_deg,1)+'°','最大躯干倾角'),metric(String(s.stable_touchdown_count),'稳定落脚计数'));
  $('outcome').textContent=s.completed?'运行完成 · 另见分项验收':'失败记录';$('outcome').style.color=s.completed?'#7ed7c0':'#f2b478';
 }else{m.append(metric(fmt(s.peak_contact_force_n/1000)+' kN','峰值接触力'),metric(fmt(s.max_compression_m*100,1)+' cm','最大压缩'),metric(s.bottomed_out?'触底':'未触底','行程限制'),metric(fmt(s.max_energy_residual_fraction*100,4)+'%','能量平衡误差'));$('outcome').textContent='独立部件模型';$('outcome').style.color='#d5c790';}
 $('evidence').textContent=current.type==='walking'?'Webots 实际状态回放 · 合成人体':'独立一维缓冲模型 · 非全身跌倒验证';
 $('command').textContent=current.type==='walking'?`在项目目录运行：\npython mobility_demo/run_webots.py --scenario ${s.scenario} --mode ${s.mode} --run 你选择的新运行名\n\n历史证据：mobility_demo/runs/${current.id}.csv`:'在项目目录运行：\npython mobility_demo/cushion.py';$('command').hidden=true;
}
function resize(){const box=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;canvas.width=box.width*ratio;canvas.height=box.height*ratio;ctx.setTransform(ratio,0,0,ratio,0,0);}
const bounds=()=>canvas.getBoundingClientRect();
function project(p){const b=bounds(),focus=[.55,0,.85],v=p.map((x,i)=>x-focus[i]),u=v[0]*Math.cos(yaw)-v[1]*Math.sin(yaw),depth=v[0]*Math.sin(yaw)+v[1]*Math.cos(yaw),h=v[2]*Math.cos(tilt)-depth*Math.sin(tilt),scale=Math.min(b.width/2.9,b.height/2.18);return [b.width/2+u*scale,b.height*.51-h*scale,scale,depth];}
function line(a,b,color,width){const x=project(a),y=project(b);ctx.beginPath();ctx.moveTo(x[0],x[1]);ctx.lineTo(y[0],y[1]);ctx.strokeStyle=color;ctx.lineWidth=width*x[2];ctx.lineCap='round';ctx.stroke();}
function dot(p,r,color){const x=project(p);ctx.beginPath();ctx.arc(x[0],x[1],r*x[2],0,Math.PI*2);ctx.fillStyle=color;ctx.fill();}
function polygon(points,color){ctx.beginPath();points.map(project).forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=color;ctx.fill();}
function cuboid(center,size,r,color){const a=[];for(let i=0;i<8;i++){const v=size.map((s,k)=>(i&(1<<k)?1:-1)*s/2);a.push(G.add(center,G.apply(r,v)));}[[0,1,3,2],[0,2,6,4],[2,3,7,6],[1,5,7,3],[4,6,7,5]].forEach((face,i)=>polygon(face.map(j=>a[j]),color[i%color.length]));}
const identity=G.rx(0);
function floor(stairs){for(let x=-.8;x<=2.01;x+=.2)line([x,-.85,0],[x,.85,0],'#28434e',.003);for(let y=-.8;y<=.81;y+=.2)line([-.8,y,0],[2,y,0],'#28434e',.003);
 if(stairs)for(let i=2;i>=0;i--){const w=i<2?.4:1.1,h=.1*(i+1);cuboid([.2+.4*i+w/2,0,h/2],[w,1.5,h],identity,['#4f6874','#667f8a','#78909b']);}}
function bodyFrame(frame){const body=G.body(frame),at=body.at,depth=body.legs.map((l,i)=>({l,i,d:project(l.hip)[3]})).sort((a,b)=>b.d-a.d);
 for(const {l,i} of depth){line(l.hip,l.knee,'#8ea6b2',.105);line(l.knee,l.ankle,'#718d9d',.076);dot(l.knee,.055,'#a7bac3');const side=i===0?-1:1,offset=G.apply(body.r,[.025,side*.073,0]);if(current.summary.mode!=='no-device'){
  line(G.add(l.hip,offset),G.add(l.knee,offset),'#41cab4',.022);line(G.add(l.knee,offset),G.add(l.ankle,offset),'#41cab4',.020);dot(G.add(l.knee,offset),.043,'#0e6968');}
  const fc=G.add(l.ankle,G.apply(l.foot,[.045,0,-.035]));cuboid(fc,[.28,.14,.09],l.foot,['#263f4b','#3c5661','#4b6470']);const contact=frame[17+i]>0;dot([l.ankle[0]+.04,l.ankle[1],Math.max(0,l.ankle[2]-.08)],.025,contact?'#6de1ba':'#f1bd75');}
 line(at([0,0,.03]),at([0,0,.47]),'#d2dfe5',.29);line(at([0,-.11,0]),at([0,.11,0]),'#536c7a',.16);
 for(const side of [-1,1]){line(at([0,side*.22,.45]),at([0,side*.235,.12]),'#b5c8d0',.065);dot(at([0,side*.235,.10]),.04,'#c6a688');}
 dot(at([0,0,.64]),.105,'#d5b399');const head=project(at([0,0,.64]));ctx.strokeStyle='#b4b8b7';ctx.lineWidth=head[2]*.035;ctx.beginPath();ctx.arc(head[0],head[1]-.018*head[2],.086*head[2],Math.PI,2*Math.PI);ctx.stroke();dot(at([.098,0,.64]),.017,'#b9967f');dot(body.pos,.021,'#f0ce8e');
}
function walking(frame){floor(current.summary.scenario==='stairs');bodyFrame(frame);const r=frame[19]||0;$('risk').textContent=['正常','预测告警','失稳阈值触发','观测异常'][r]||'—';$('risk').style.color=r?'#bb6b3e':'#227d6b';$('prepare').textContent=frame[20]?'已发出 · 未接部署':'未发出';$('cloud').textContent=frame[21]?'模拟在线':'模拟离线';$('phase').textContent=(frame[17]>0&&frame[18]>0?'双脚接触':frame[17]>0?'右脚接触':frame[18]>0?'左脚接触':'双脚无接触')+' · 阶段 '+Math.max(0,Math.floor((clock-2)/(current.summary.scenario==='stairs'?6:4)));
}
function cushion(frame){floor(false);const y=frame[1],compression=frame[4],base=.40;const s=current.summary,p=s.parameters;const panelWidth=.48;
 cuboid([.55,0,base-.08],[panelWidth,.36,.16],identity,['#657985','#8196a1','#9cabb3']);
 const pad=Math.max(.004,p.stroke_m-compression);if(p.stroke_m>0)cuboid([.55,0,base+pad/2],[.40,.30,pad],identity,['#2d9c8e','#44bca8','#71d3bd']);
 cuboid([.55,0,base+p.stroke_m+y+.12],[.4,.3,.24],identity,['#b6c7d0','#d1dde3','#e4ecf0']);
 $('cloud').textContent='不参与该工况';$('risk').textContent='部件力学对照';$('prepare').textContent='预置缓冲，无部署模型';$('phase').textContent=`压缩 ${fmt(compression*100,1)} cm · 接触力 ${fmt(frame[3],0)} N`;
 const b=bounds();ctx.fillStyle='#8cabb6';ctx.font='12px Segoe UI';ctx.fillText('等效质量 '+p.mass_kg+' kg / 落差 '+p.drop_height_m+' m',24,b.height-24);
}
function frameAt(t){const f=current.frames;let lo=0,hi=f.length-1;while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(f[mid][0]<=t)lo=mid;else hi=mid-1;}return f[lo];}
function render(now){if(!last)last=now;const dt=Math.min(.1,(now-last)/1000);last=now;const end=current.frames.at(-1)[0];if(playing)clock=Math.min(end,clock+dt*Number($('speed').value));if(clock>=end){playing=false;$('play').textContent='重播';}
 const b=bounds();ctx.clearRect(0,0,b.width,b.height);const gradient=ctx.createLinearGradient(0,0,0,b.height);gradient.addColorStop(0,'#142d39');gradient.addColorStop(1,'#0c2029');ctx.fillStyle=gradient;ctx.fillRect(0,0,b.width,b.height);
 const f=frameAt(clock);if(current.type==='walking')walking(f);else cushion(f);$('time').textContent=fmt(clock)+' s';$('seek').value=end?String(clock/end*1000):'0';requestAnimationFrame(render);}
$('case').addEventListener('change',e=>choose(Number(e.target.value)));$('play').onclick=()=>{if(clock>=current.frames.at(-1)[0])clock=0;playing=!playing;$('play').textContent=playing?'暂停':'播放';};$('seek').oninput=e=>{clock=Number(e.target.value)/1000*current.frames.at(-1)[0];};$('camera').onclick=()=>{yaw=yaw<-.5?0:yaw<.5?Math.PI/2:-.75;};$('reproduce').onclick=()=>{$('command').hidden=!$('command').hidden;};
canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,yaw,tilt};canvas.setPointerCapture(e.pointerId);});canvas.addEventListener('pointermove',e=>{if(drag){yaw=drag.yaw+(e.clientX-drag.x)*.008;tilt=Math.max(.05,Math.min(.8,drag.tilt+(e.clientY-drag.y)*.004));}});canvas.addEventListener('pointerup',()=>drag=null);canvas.addEventListener('pointercancel',()=>drag=null);window.addEventListener('resize',resize);
choose(0);resize();requestAnimationFrame(render);
})();
