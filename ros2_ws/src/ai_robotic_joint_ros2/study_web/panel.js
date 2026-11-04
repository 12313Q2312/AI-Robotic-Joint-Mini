'use strict';
(() => {
  const token = document.querySelector('meta[name="panel-token"]').content;
  const el = id => document.getElementById(id);
  const buttons = selector => Array.from(document.querySelectorAll(selector));
  let session = '', state = null, connected = false, polling = false, heartbeatBusy=false;
  let focused=true, closed=false, actionEpoch=0, sending=false;
  let queue=[];
  let transitioning=false,claiming=false;
  let needsReclaim=false;
  let activeView='experience';
  let actionCounter=0;
  let resetAwaiting=false,lastResetSequence=null,lastSimTime=null,lastResetPending=false;
  const history=[];
  let lastPlotTime=null;
  const freshness={baseline:{time:null,changed:0},candidate:{time:null,changed:0}};
  const keys=new Set();
  const deg = rad => rad * 180 / Math.PI;
  const notice = text => {el('notice').textContent=text;};
  const show=(id,value,unit='',digits=3)=>{el(id).textContent=connected && Number.isFinite(value) ? value.toFixed(digits)+unit : '—';};
  function drawArm(side,sample) {
    const canvas=el(side+'-canvas'),ctx=canvas.getContext('2d');if(!ctx) return;
    ctx.clearRect(0,0,canvas.width,canvas.height);
    ctx.font='15px system-ui';ctx.fillStyle='#4c6475';
    if(!connected) {ctx.fillText('无新鲜双侧反馈 · 不展示运动',24,105);return;}
    ctx.fillStyle='#b8cad5';ctx.fillRect(55,143,50,25);
    ctx.strokeStyle='#d1dee6';ctx.lineWidth=2;ctx.setLineDash([5,5]);
    ctx.beginPath();ctx.moveTo(80,140);ctx.lineTo(260,140);ctx.stroke();ctx.setLineDash([]);
    ctx.strokeStyle=side==='baseline' ? '#1f689c' : '#a85017';ctx.lineWidth=12;ctx.lineCap='round';
    const x=80+160*Math.cos(sample.actual_angle_rad),y=140-160*Math.sin(sample.actual_angle_rad);
    ctx.beginPath();ctx.moveTo(80,140);ctx.lineTo(x,y);ctx.stroke();
    ctx.fillStyle='#273d4b';ctx.beginPath();ctx.arc(80,140,9,0,2*Math.PI);ctx.fill();
    ctx.fillStyle='#637e90';ctx.fillRect(x-10,y-10,20,20);
    ctx.fillStyle='#273d4b';ctx.fillText('实际 '+deg(sample.actual_angle_rad).toFixed(2)+'°',300,45);
    ctx.fillStyle='#4c6475';ctx.fillText('水平零位',295,145);
  }
  function drawHistory(t) {
    const canvas=el('history-plot'),ctx=canvas.getContext('2d');if(!ctx) return;
    if(!connected) {history.length=0;lastPlotTime=null;}
    else if(t.baseline.sim_time_s!==lastPlotTime) {
      history.push({time:t.baseline.sim_time_s,a:t.baseline.commanded_torque_nm,b:t.candidate.commanded_torque_nm});
      if(history.length>180) history.shift();lastPlotTime=t.baseline.sim_time_s;
    }
    ctx.clearRect(0,0,canvas.width,canvas.height);ctx.font='14px system-ui';ctx.fillStyle='#4c6475';
    if(!history.length) {ctx.fillText('等待新鲜双侧数据，历史暂不显示',24,105);return;}
    ctx.strokeStyle='#d1dee6';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(45,115);ctx.lineTo(canvas.width-20,115);ctx.stroke();
    ctx.fillText('+1 N·m',3,37);ctx.fillText('0',24,115);ctx.fillText('−1',17,197);
    const first=history[0].time,last=history[history.length-1].time,span=Math.max(0.001,last-first);
    for(const [field,color,label] of [['a','#1f689c','A · R0'],['b','#a85017','B · F-N']]) {
      ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();
      history.forEach((point,index)=>{const x=45+(canvas.width-70)*(point.time-first)/span,y=115-80*Math.max(-1,Math.min(1,point[field]));if(index)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();
      ctx.fillStyle=color;ctx.fillText(label,field==='a'?75:190,20);
    }
    ctx.fillStyle='#4c6475';ctx.fillText(first.toFixed(2)+' s',45,215);ctx.fillText(last.toFixed(2)+' s',canvas.width-90,215);
  }
  async function control(action,values={},keepalive=false,owner=session) {
    const abort=new AbortController(), timer=setTimeout(()=>abort.abort(),350);
    try {
      const response=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Token':token},
        body:JSON.stringify(Object.assign({action,session:owner},values)),keepalive,signal:abort.signal});
      const result=await response.json();
      if(!response.ok) throw new Error(result.error || '控制服务拒绝请求');
      return result;
    } finally {clearTimeout(timer);}
  }
  const authorized=()=>session && focused && !needsReclaim && !closed && !document.hidden;
  const canMove=()=>connected && authorized() && activeView==='experience' && !transitioning && !resetAwaiting && state && ['keyboard','angle'].includes(state.mode) && !state.reset_pending && !state.experiment_active;
  const canReset=()=>connected && authorized() && activeView==='experience' && !transitioning && !resetAwaiting && state && state.can_reset && !state.reset_pending && !state.experiment_active;
  const editing=target=>target && (/^(INPUT|TEXTAREA|SELECT)$/.test(target.tagName) || target.isContentEditable);
  function pump() {
    if(sending) return;
    const item=queue.shift();if(!item) return;
    const ready=item.action==='mode' ? authorized() : item.action==='reset' ? connected && authorized() && state && state.can_reset && !state.reset_pending && !state.experiment_active : canMove();
    if(item.motion && (item.epoch!==actionEpoch || !ready || item.owner!==session)) {item.resolve({cancelled:true});pump();return;}
    sending=true;
    control(item.action,item.values,item.keepalive,item.owner).then(item.resolve,item.reject)
      .finally(()=>{sending=false;pump();});
  }
  function command(action,values={},keepalive=false,priority=false) {
    // Allocate when intent enters the queue. Server rejects late older commands;
    // aborting a fetch alone cannot revoke an operation already in transit.
    const orderedValues=Object.assign({},values,{action_id:++actionCounter});
    return new Promise((resolve,reject)=>{
      const item={action,values:orderedValues,keepalive,owner:session,epoch:actionEpoch,motion:!['stop','release'].includes(action),resolve,reject};
      if(priority) queue.unshift(item);else queue.push(item);pump();
    }).catch(e=>{
      if(!['stop','release'].includes(action)) {state=null;invalidate();render();}
      notice('操作未确认：'+e.message+'；未发送的旧动作已取消，需要重新操作。');return null;
    });
  }
  function invalidate(action='stop',keepalive=false) {
    actionEpoch++;keys.clear();
    queue=queue.filter(item=>{if(item.motion) {item.resolve({cancelled:true});return false;}return true;});
    if(session) command(action,{},keepalive,true);
  }
  function render() {
    const wasConnected=connected;
    const t=state && state.telemetry;
    const finiteSample=s=>s && ['sim_time_s','reference_angle_rad','actual_angle_rad','velocity_rad_s','commanded_torque_nm'].every(k=>Number.isFinite(s[k]));
    connected=Boolean(state && state.app==='joint-mini-study' && state.protocol_version===1 && state.connected && t &&
      finiteSample(t.baseline) && finiteSample(t.candidate) && Math.abs(t.baseline.reference_angle_rad-t.candidate.reference_angle_rad)<1e-6 &&
      Math.abs(t.baseline.sim_time_s-t.candidate.sim_time_s)<0.05 &&
      ['baseline','candidate'].every(side=>Date.now()-freshness[side].changed<=500));
    if(wasConnected && !connected) invalidate();
    for(const b of [...buttons('[data-key]'),...buttons('[data-nudge]'),el('execute')]) b.disabled=!canMove();
    el('connect').disabled=Boolean(session) || claiming || closed || Boolean(state && state.experiment_active);
    el('stop').disabled=!session;
    el('reset').disabled=!canReset();
    el('payload').disabled=!canReset();
    el('reset-status').textContent=resetAwaiting || state && state.reset_pending ? '正在等待两侧复位确认；旧动作已取消。' : canReset() ? '可复位：两侧回到共同初始条件，不续跑旧动作。' : '服务尚未允许复位，请先停止并等待新鲜双侧反馈。';
    el('connection').textContent=needsReclaim && session ? '页面曾失焦 · 等待旧控制权释放后重新连接' : connected ? (session ? '两侧反馈在线 · 已取得控制权' : '两侧反馈在线 · 未取得控制权') : '无新鲜双侧反馈 · 禁止运动';
    el('model-notice').textContent=state && state.model_notice || '等待模型说明；当前不是最终 CAD 验收';
    el('reason').textContent=state && state.reason || '等待状态服务';
    const mode=state && state.mode || 'keyboard';
    for(const b of buttons('[data-mode]')) {b.classList.toggle('active',b.dataset.mode===mode);b.setAttribute('aria-pressed',b.dataset.mode===mode);}
    for(const name of ['keyboard','angle','ai']) el(name+'-panel').hidden=mode!==name;
    for(const side of ['baseline','candidate']) {
      el(side+'-actual').textContent=connected ? deg(t[side].actual_angle_rad).toFixed(2)+'°' : '—';
      el(side+'-error').textContent=connected ? deg(t[side].reference_angle_rad-t[side].actual_angle_rad).toFixed(2)+'°' : '—';
      const s=t && t[side],torque=s && s.commanded_torque_nm;
      show(side+'-torque',torque,' N·m');
      const extent=connected ? Math.min(1,Math.abs(torque))*50 : 0;
      el(side+'-bar').style.width=extent+'%';el(side+'-bar').style.left=connected && torque<0 ? (50-extent)+'%' : '50%';
      el(side+'-status').textContent=connected ? '实际速度 '+deg(s.velocity_rad_s).toFixed(2)+'°/s · '+(s.saturated?'力矩已限幅':'未限幅')+' · '+String(s.status||'状态未知') : '反馈无效或陈旧，历史值隐藏';
      drawArm(side,s);
    }
    show('requested',state && state.requested_angle_deg,'°',2);
    show('reference',t && t.baseline && deg(t.baseline.reference_angle_rad),'°',2);
    show('reference-speed',t && t.baseline && deg(t.baseline.reference_velocity_rad_s),'°/s',2);
    show('spring',t && t.spring_compression_m*1000,' mm',2);
    show('compensation',t && t.compensation_torque_nm,' N·m');show('loss',t && t.loss_torque_nm,' N·m');
    show('payload-mass',t && t.payload_mass_kg,' kg');
    el('candidate-label').textContent='B · '+(t && t.candidate_label || 'F-N')+' 未优化几何夹具，非最终 CAD';
    el('model-status').textContent='模型状态：'+(t && t.model_status || '无有效状态')+'；这里显示仿真命令，不是实测力矩。';
    drawHistory(t);
  }
  el('connect').addEventListener('click',async () => {
    if(session || claiming || !focused || closed || document.hidden || state && state.experiment_active) return;
    const epoch=actionEpoch;claiming=true;render();
    try {const result=await control('claim');
      if(typeof result.session!=='string' || !result.session) throw new Error('服务未返回有效控制权');
      if(epoch!==actionEpoch || !focused || closed || document.hidden) {await control('release',{action_id:1},true,result.session);notice('连接期间页面已离开，控制权已释放；请重新明确连接。');return;}
      session=result.session;actionCounter=0;needsReclaim=false;notice('已取得控制权，需新鲜双侧反馈和新的明确操作。');}
    catch(e) {notice(e.message);}
    finally {claiming=false;render();}
  });
  function down(key,repeat=false) {
    if(!canMove() || state.mode!=='keyboard' || repeat) return;
    if(key==='ArrowUp' || key==='ArrowDown') {if(!keys.has(key)) {keys.add(key);command('keys',{keys:Array.from(keys)});}}
    else if(key==='ArrowLeft' || key==='ArrowRight') command('nudge',{direction:key==='ArrowLeft' ? -1 : 1});
  }
  function up(key) {if(keys.delete(key) && session) command('keys',{keys:Array.from(keys)});}
  document.addEventListener('keydown',event => {
    if(editing(event.target) || !state || state.mode!=='keyboard' || !['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(event.key)) return;
    event.preventDefault();down(event.key,event.repeat);
  });
  document.addEventListener('keyup',event=>up(event.key));
  for(const button of buttons('[data-key]')) {
    button.addEventListener('pointerdown',event=>{event.preventDefault();button.setPointerCapture(event.pointerId);down(button.dataset.key);});
    for(const name of ['pointerup','pointercancel','lostpointercapture']) button.addEventListener(name,()=>up(button.dataset.key));
  }
  for(const button of buttons('[data-nudge]')) button.addEventListener('click',()=>{if(canMove() && state.mode==='keyboard') command('nudge',{direction:Number(button.dataset.nudge)});});
  document.addEventListener('focusin',event=>{if(editing(event.target) && keys.size) {invalidate();render();}});
  el('stop').addEventListener('click',()=>{invalidate();render();});
  for(const button of buttons('[data-mode]')) button.addEventListener('click',async ()=>{
    if(!authorized()) {notice('请先取得控制权。');return;}
    invalidate();transitioning=true;render();
    try {const result=await command('mode',{mode:button.dataset.mode});if(result && !result.cancelled && state) state.mode=button.dataset.mode;}
    finally {transitioning=false;render();}
  });
  el('angle-form').addEventListener('submit',event=>{
    event.preventDefault();if(!canMove() || state.mode!=='angle') return;
    const raw=el('angle').value.trim(),value=Number(raw);
    if(!raw || !Number.isFinite(value) || Math.abs(value)>45 || !el('angle').checkValidity()) {notice('请输入 −45° 到 +45° 的有限角度；非法输入不会执行。');return;}
    command('angle',{angle_deg:value});
  });
  el('reset').addEventListener('click',async ()=>{
    if(!canReset()) return;
    const raw=el('payload').value,index=Number(raw);
    if(!raw || !Number.isInteger(index) || ![0,1,2].includes(index)) {notice('请选择有效的负载。');return;}
    invalidate();transitioning=true;render();
    resetAwaiting=true;history.length=0;lastPlotTime=null;
    try {const result=await command('reset',{payload_index:index});if(result && !result.cancelled) notice('复位请求已提交；等待两侧仿真确认后，需要新的明确操作。');else resetAwaiting=false;}
    finally {transitioning=false;render();}
  });
  async function loadReport() {
    const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1500);
    try {const response=await fetch('/api/report',{cache:'no-store',signal:abort.signal});if(!response.ok) throw new Error('状态接口暂不可用');
      el('report').textContent=JSON.stringify(await response.json(),null,2)+'\n软件测试不等于真实 Webots 联调或公众试用。';}
    catch(e) {el('report').textContent='工程状态未能读取：'+e.message+'。请在项目目录查看 docs/GRAVITY_STUDY.md；不据此声称验收完成。';}
    finally {clearTimeout(timer);}
  }
  el('report-refresh').addEventListener('click',loadReport);
  for(const button of buttons('[data-view]')) button.addEventListener('click',()=>{
    invalidate();activeView=button.dataset.view;
    for(const name of ['experience','principle','engineering']) el(name+'-view').hidden=activeView!==name;
    for(const item of buttons('[data-view]')) {item.classList.toggle('active',item.dataset.view===activeView);item.setAttribute('aria-pressed',item.dataset.view===activeView);}
    if(activeView==='engineering') loadReport();render();
  });
  function retireLease() {
    // STOP is best effort. Never let succeeding heartbeats preserve an old
    // motion if this request was lost, even when focus returns quickly.
    needsReclaim=true;invalidate('stop',true);
    notice('页面已离开：停止请求已发送，不再续心跳；等待服务释放控制权后重新连接，不宣称已经停住。');render();
  }
  window.addEventListener('blur',()=>{focused=false;retireLease();});
  window.addEventListener('focus',()=>{focused=true;render();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden) retireLease();});
  window.addEventListener('pagehide',()=>{closed=true;invalidate('release',true);render();});
  async function tick() {
    render(); // A hung state request must not keep locally aged feedback enabled.
    if(session && focused && !needsReclaim && !heartbeatBusy && !document.hidden && !closed) {
      heartbeatBusy=true;
      control('heartbeat').catch(e=>{invalidate();session='';needsReclaim=false;resetAwaiting=false;notice('控制权失效：'+e.message);render();})
        .finally(()=>{heartbeatBusy=false;});
    }
    if(polling || document.hidden || closed) return;
    polling=true;
    try {
      const abort=new AbortController(), timer=setTimeout(()=>abort.abort(),350);
      try {const response=await fetch('/api/state',{cache:'no-store',signal:abort.signal});if(!response.ok) throw new Error('状态服务不可用');state=await response.json();}
      finally {clearTimeout(timer);}
      if(session && state.app==='joint-mini-study' && state.protocol_version===1 && state.has_owner===false) {
        invalidate();session='';needsReclaim=false;resetAwaiting=false;notice('服务器已释放控制权；旧动作已取消，请重新连接。');
      }
      const telemetry=state.telemetry,sequence=telemetry && telemetry.reset_sequence;
      const time=telemetry && telemetry.baseline && telemetry.baseline.sim_time_s;
      const resetChanged=(Number.isFinite(sequence) && lastResetSequence!==null && sequence!==lastResetSequence) ||
        (Number.isFinite(time) && lastSimTime!==null && time<lastSimTime);
      if(resetChanged) {
        invalidate();resetAwaiting=false;history.length=0;lastPlotTime=null;
        notice('仿真已复位或时间回退；旧动作已清除，需要重新操作。');
      }
      else if(lastResetPending && state.reset_pending===false) {
        invalidate();resetAwaiting=false;history.length=0;lastPlotTime=null;
        notice('复位等待已撤销或未确认完成；旧动作已取消，请重新操作，不将此视为复位成功。');
      }
      lastResetPending=state.reset_pending===true;
      if(Number.isFinite(sequence)) lastResetSequence=sequence;
      if(Number.isFinite(time)) lastSimTime=time;
      for(const side of ['baseline','candidate']) {
        const sample=state.telemetry && state.telemetry[side], stamp=sample && sample.sim_time_s;
        if(Number.isFinite(stamp) && stamp!==freshness[side].time) {freshness[side]={time:stamp,changed:Date.now()};}
      }
    }
    catch(e) {state=null;notice('连接中断：'+e.message);}
    finally {polling=false;render();}
  }
  render();setInterval(tick,100);tick();
})();
