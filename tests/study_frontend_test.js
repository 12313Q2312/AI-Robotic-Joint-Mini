'use strict';
// Node VM + synthetic DOM/HTTP software tests, not browser or Webots validation.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const web = path.join(__dirname, '../ros2_ws/src/ai_robotic_joint_ros2/study_web');

class Surface {
  constructor(tagName = 'BUTTON', dataset = {}) {
    this.tagName = tagName; this.dataset = dataset; this.listeners = {};
    this.value = ''; this.textContent = ''; this.disabled = false; this.hidden = false;
    this.style = {}; this.attributes = {}; this.width = 480; this.height = 220;
    this.classList = {toggle() {}}; this.drawing = [];
    this.context = new Proxy({}, {get: (o,k) => k in o ? o[k] : (...a) => this.drawing.push([k,...a]), set: (o,k,v) => {o[k]=v; return true;}});
  }
  addEventListener(name, callback) { (this.listeners[name] ||= []).push(callback); }
  fire(name, values = {}) {
    const event = Object.assign({target:this, preventDefault() {}, repeat:false, pointerId:1}, values);
    for (const callback of this.listeners[name] || []) callback(event);
  }
  setPointerCapture() {}
  setAttribute(k,v) {this.attributes[k]=String(v);}
  getContext() {return this.context;}
  checkValidity() {return this.value.trim() !== '' && Number.isFinite(Number(this.value)) && Math.abs(Number(this.value)) <= 45;}
}

async function fixture() {
  const document = new Surface('DOCUMENT'), window = new Surface('WINDOW');
  const elements = {};
  const byId = id => elements[id] ||= new Surface(id.includes('canvas') || id.includes('plot') ? 'CANVAS' : 'BUTTON');
  byId('angle').tagName = 'INPUT'; byId('payload').tagName = 'SELECT';
  const modes = ['keyboard','angle','ai'].map(mode => new Surface('BUTTON',{mode}));
  const arrows = ['ArrowUp','ArrowDown'].map(key => new Surface('BUTTON',{key}));
  const nudges = [-1,1].map(nudge => new Surface('BUTTON',{nudge:String(nudge)}));
  const views = ['experience','principle','engineering'].map(view => new Surface('BUTTON',{view}));
  document.hidden = false; document.activeElement = document; document.hasFocus = () => true;
  document.getElementById = byId;
  document.querySelector = () => ({content:'test-token'});
  document.querySelectorAll = selector => ({'[data-mode]':modes, '[data-key]':arrows, '[data-nudge]':nudges, '[data-view]':views})[selector] || [];
  const sample = angle => ({sim_time_s:1,requested_angle_rad:0,reference_angle_rad:0,reference_velocity_rad_s:0,
    actual_angle_rad:angle,velocity_rad_s:0,p_torque_nm:0,i_torque_nm:0,d_torque_nm:0,
    feedforward_torque_nm:0,commanded_torque_nm:0.3,saturated:false,anti_windup:false,mode:0,status:'running',command_sequence:1});
  const state = {app:'joint-mini-study',protocol_version:1,mode:'keyboard',connected:true,has_owner:true,
    requested_angle_deg:0,reason:'synthetic feedback',can_reset:true,reset_pending:false,experiment_active:false,
    model_notice:'F-N 未优化几何夹具，非最终 CAD',telemetry:{baseline:sample(0.1),candidate:sample(0.2),
      spring_compression_m:0.003,compensation_torque_nm:0.2,loss_torque_nm:0,payload_mass_kg:0.25,
      candidate_label:'F-N',model_status:'synthetic',can_reset:true,reset_sequence:0}};
  const requests = [], intervals = [], timers = [];
  let now = 1000, blockedAction = '', unblock, stateError = false, controlError='',claims=0;
  const fetch = async (url,options={}) => {
    if (url === '/api/control') {
      const p = JSON.parse(options.body); requests.push(p);
      assert.equal(options.headers['X-Panel-Token'],'test-token');
      if (p.action === blockedAction) await new Promise(r => {unblock=r;});
      if (p.action === controlError) throw new Error('synthetic control failure');
      if (p.action === 'mode') state.mode=p.mode;
      if (p.action === 'claim') {state.has_owner=true;claims++;}
      return {ok:true,json:async () => p.action === 'claim' ? {session:'owner-'+claims} : {ok:true}};
    }
    if (url === '/api/report') return {ok:true,json:async () => ({status:'pending',webots_motion:'pending'})};
    if (stateError) throw new Error('synthetic network failure');
    return {ok:true,json:async () => JSON.parse(JSON.stringify(state))};
  };
  const sourcePath = path.join(web,'panel.js');
  const source = fs.existsSync(sourcePath) ? fs.readFileSync(sourcePath,'utf8') : '';
  class TestDate extends Date {static now() {return now;}}
  vm.runInNewContext(source,{document,window,fetch,Date:TestDate,AbortController,
    setInterval:fn => {intervals.push(fn); return intervals.length;},
    setTimeout:fn => {timers.push(fn); return timers.length;}, clearTimeout() {},
    console,Math,Number,JSON});
  const settle = async () => {for(let i=0;i<12;i++) await new Promise(r=>setImmediate(r));};
  await settle(); byId('connect').fire('click'); await settle();
  const tick = async () => {for(const fn of intervals) fn(); await settle();};
  await tick(); requests.length=0;
  return {document,window,elements,byId,modes,arrows,nudges,views,state,requests,settle,tick,
    key:(key,repeat=false,target=document) => document.fire('keydown',{key,repeat,target}),
    up:key => document.fire('keyup',{key}), advance:ms=>{now+=ms;},
    block:action=>{blockedAction=action;}, unblock:()=>{blockedAction=''; if(unblock) unblock();},
    failState:()=>{stateError=true;},failControl:action=>{controlError=action;}};
}

const tests = [];
tests.push(['one missing feedback side disables every movement and clears displayed readings',async () => {
  const f=await fixture(); f.state.telemetry.candidate=null; await f.tick();
  assert.equal(f.byId('execute').disabled,true);
  assert.equal(f.arrows.every(b=>b.disabled),true);
  assert.equal(f.byId('baseline-actual').textContent,'—');
  f.key('ArrowUp'); await f.settle();
  assert.equal(f.requests.filter(r=>['keys','nudge','angle'].includes(r.action)).length,0);
}]);
tests.push(['press and release sends one held-key action and does not restart from OS repeats',async () => {
  const f=await fixture();f.key('ArrowUp');f.key('ArrowUp',true);await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
  assert.deepEqual(f.requests.find(r=>r.action==='keys').keys,['ArrowUp']);
  f.up('ArrowUp');await f.settle();assert.deepEqual(f.requests.at(-1).keys,[]);
}]);
tests.push(['focused heartbeat is independent of a delayed movement; blur cancels the unsent queue',async () => {
  const f=await fixture();f.block('keys');f.key('ArrowUp');await f.settle();
  await f.tick();assert.equal(f.requests.some(r=>r.action==='heartbeat'),true,'Focused heartbeat must not wait for delayed movement');
  const heartbeats=f.requests.filter(r=>r.action==='heartbeat').length;
  f.key('ArrowDown');f.window.fire('blur');await f.tick();
  assert.equal(f.requests.filter(r=>r.action==='keys').length,1,'Second movement must not leave the queue');
  assert.equal(f.requests.filter(r=>r.action==='heartbeat').length,heartbeats,'Blur must stop refreshing the lease');
  f.unblock();await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
  assert.equal(f.requests.some(r=>r.action==='stop'),true);
  f.window.fire('focus');f.key('ArrowUp',true);await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys').length,1,'Recovery requires a new explicit press');
}]);
tests.push(['unchanged simulation feedback becomes stale despite successful state HTTP responses',async () => {
  const f=await fixture();f.advance(501);await f.tick();
  assert.equal(f.byId('execute').disabled,true);
  assert.equal(f.byId('candidate-actual').textContent,'—');
  f.state.telemetry.baseline.sim_time_s=2;f.state.telemetry.candidate.sim_time_s=2;await f.tick();
  assert.equal(f.byId('execute').disabled,false);
  assert.equal(f.requests.some(r=>['keys','angle','nudge'].includes(r.action)),false,'Freshness recovery must not resume old movement');
}]);
tests.push(['editing an input never starts held-key movement',async () => {
  const f=await fixture();f.key('ArrowUp',false,f.byId('angle'));await f.settle();
  assert.equal(f.requests.some(r=>r.action==='keys'),false);
}]);
tests.push(['angle mode accepts a finite in-range target and rejects empty, invalid and out-of-range values',async () => {
  const f=await fixture();f.modes[1].fire('click');await f.settle();await f.tick();
  f.byId('angle').value='12.345';f.byId('angle-form').fire('submit');await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='angle').length,1);
  assert.equal(f.requests.find(r=>r.action==='angle').angle_deg,12.345);
  for(const value of ['','NaN','Infinity','46','-46']) {f.byId('angle').value=value;f.byId('angle-form').fire('submit');await f.settle();}
  assert.equal(f.requests.filter(r=>r.action==='angle').length,1);
}]);
tests.push(['keyboard nudge is single-step and pointer cancellation releases the held direction',async () => {
  const f=await fixture();f.key('ArrowRight');f.key('ArrowRight',true);await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='nudge').length,1);assert.equal(f.requests.find(r=>r.action==='nudge').direction,1);
  f.arrows[0].fire('pointerdown');await f.settle();f.arrows[0].fire('pointercancel');await f.settle();
  assert.deepEqual(f.requests.at(-1).keys,[]);
  f.key('ArrowUp');f.key('ArrowDown');await f.settle();assert.deepEqual(f.requests.at(-1).keys,['ArrowUp','ArrowDown']);
}]);
tests.push(['reset requires an explicit allowed payload selection and cancels the preceding action',async () => {
  const f=await fixture();f.byId('payload').value='1';f.byId('reset').fire('click');await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='reset').length,1);
  assert.equal(f.requests.find(r=>r.action==='reset').payload_index,1);
  assert.equal(f.requests.findIndex(r=>r.action==='stop') < f.requests.findIndex(r=>r.action==='reset'),true);
  f.state.can_reset=false;await f.tick();f.byId('reset').fire('click');await f.settle();
  assert.equal(f.byId('reset').disabled,true);
  assert.equal(f.requests.filter(r=>r.action==='reset').length,1);
}]);
tests.push(['arm diagrams use actual feedback, never the requested angle; stale feedback clears the diagram',async () => {
  const f=await fixture();const canvas=f.byId('candidate-canvas');
  const tip=canvas.drawing.filter(x=>x[0]==='lineTo').at(-1);
  assert.ok(tip,'The diagram must contain a feedback-driven rod');
  assert.ok(Math.abs(tip[1]-(80+160*Math.cos(0.2)))<1e-8);
  assert.ok(Math.abs(tip[2]-(140-160*Math.sin(0.2)))<1e-8);
  canvas.drawing.length=0;f.state.requested_angle_deg=42;await f.tick();
  assert.deepEqual(canvas.drawing.filter(x=>x[0]==='lineTo').at(-1),tip);
  canvas.drawing.length=0;f.state.telemetry.baseline=null;await f.tick();
  const lastClear=canvas.drawing.map(x=>x[0]).lastIndexOf('clearRect');
  assert.equal(canvas.drawing.slice(lastClear).some(x=>x[0]==='lineTo'),false);
  assert.equal(f.byId('candidate-torque').textContent,'—');
}]);
tests.push(['public entrances cancel movement and engineering status is read without invented downloads',async () => {
  const f=await fixture();f.key('ArrowUp');await f.settle();f.views[2].fire('click');await f.settle();
  assert.equal(f.byId('experience-view').hidden,true);
  assert.equal(f.byId('engineering-view').hidden,false);
  assert.equal(f.requests.some(r=>r.action==='stop'),true);
  assert.match(f.byId('report').textContent,/pending/);
  assert.equal(f.byId('report').textContent.includes('http'),false);
}]);
tests.push(['stop has a newer intent id than delayed movement and heartbeat has no action id',async () => {
  const f=await fixture();f.block('keys');f.key('ArrowUp');await f.tick();f.window.fire('blur');await f.tick();
  f.unblock();await f.settle();const move=f.requests.find(r=>r.action==='keys'),stop=f.requests.find(r=>r.action==='stop');
  assert.ok(Number.isSafeInteger(move.action_id));assert.ok(stop.action_id>move.action_id);
  assert.equal(Object.hasOwn(f.requests.find(r=>r.action==='heartbeat'),'action_id'),false);
}]);
tests.push(['reset acknowledgement cancels queued old intents and permits only a new explicit press',async () => {
  const f=await fixture();f.block('keys');f.key('ArrowUp');await f.settle();f.key('ArrowDown');
  f.state.telemetry.reset_sequence=1;await f.tick();f.unblock();await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
  f.key('ArrowUp',true);await f.settle();assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
  f.key('ArrowUp');await f.settle();assert.equal(f.requests.filter(r=>r.action==='keys').length,2);
}]);
tests.push(['AI is a non-motion placeholder and disables manual controls until a new manual mode is selected',async () => {
  const f=await fixture();f.modes[2].fire('click');await f.settle();await f.tick();
  assert.equal(f.byId('ai-panel').hidden,false);assert.equal(f.arrows.every(x=>x.disabled),true);
  f.key('ArrowUp');f.nudges[1].fire('click');f.byId('angle').value='20';f.byId('angle-form').fire('submit');await f.settle();
  assert.equal(f.requests.some(r=>['keys','nudge','angle'].includes(r.action)),false);
}]);
tests.push(['a failed control request cancels queued motion rather than replaying it later',async () => {
  const f=await fixture();f.block('keys');f.key('ArrowUp');await f.settle();f.key('ArrowDown');f.failControl('keys');f.unblock();await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
  assert.equal(f.requests.some(r=>r.action==='stop'),true);
  assert.equal(f.arrows.every(b=>b.disabled),true);
}]);
tests.push(['hidden pages and page close discard pending mode changes and release without replay',async () => {
  const f=await fixture();f.block('keys');f.key('ArrowUp');await f.settle();f.modes[1].fire('click');
  f.document.hidden=true;f.document.fire('visibilitychange');f.window.fire('pagehide');f.unblock();await f.settle();
  assert.equal(f.requests.some(r=>r.action==='mode'),false);
  assert.equal(f.requests.some(r=>r.action==='release'),true);
  f.document.hidden=false;f.document.fire('visibilitychange');f.key('ArrowUp');await f.tick();
  assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
}]);
tests.push(['protocol, synchronized reference, experiment and network gates disable both sides',async () => {
  const f=await fixture();f.state.protocol_version=2;await f.tick();assert.equal(f.arrows.every(x=>x.disabled),true);
  f.state.protocol_version=1;f.state.telemetry.candidate.reference_angle_rad=0.01;await f.tick();assert.equal(f.byId('execute').disabled,true);
  f.state.telemetry.candidate.reference_angle_rad=0;f.state.experiment_active=true;await f.tick();assert.equal(f.byId('reset').disabled,true);
  f.state.experiment_active=false;f.failState();await f.tick();assert.equal(f.byId('candidate-actual').textContent,'—');
}]);
tests.push(['history is bounded to 180 samples and reset waits for actual acknowledgement',async () => {
  const f=await fixture();for(let i=2;i<202;i++) {f.state.telemetry.baseline.sim_time_s=i;f.state.telemetry.candidate.sim_time_s=i;await f.tick();}
  const canvas=f.byId('history-plot'),lastClear=canvas.drawing.map(x=>x[0]).lastIndexOf('clearRect');
  assert.ok(canvas.drawing.slice(lastClear).filter(x=>x[0]==='lineTo').length<=360);
  f.byId('payload').value='2';f.byId('reset').fire('click');await f.settle();f.key('ArrowUp');await f.settle();
  assert.equal(f.requests.some(r=>r.action==='keys'),false,'Reset request is not an acknowledgement');
  f.state.telemetry.reset_sequence=1;f.state.telemetry.baseline.sim_time_s=0;f.state.telemetry.candidate.sim_time_s=0;await f.tick();
  f.key('ArrowUp');await f.settle();assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
}]);
tests.push(['HTML supplies local assets, token and the full public interface without CDN or fake downloads',async () => {
  const html=fs.readFileSync(path.join(web,'index.html'),'utf8'),css=fs.readFileSync(path.join(web,'panel.css'),'utf8');
  assert.match(html,/name="panel-token" content="__PANEL_TOKEN__"/);
  assert.equal(html.includes('$__PANEL_TOKEN__'),false,'Server replaces the bare placeholder, with no extra dollar sign');
  for(const id of ['connection','connect','stop','model-notice','reason','notice','requested','reference','reference-speed','baseline-canvas','candidate-canvas','baseline-actual','candidate-actual','baseline-error','candidate-error','baseline-torque','candidate-torque','baseline-bar','candidate-bar','baseline-status','candidate-status','candidate-label','keyboard-panel','angle-panel','ai-panel','angle-form','angle','execute','payload','reset','reset-status','spring','compensation','loss','payload-mass','model-status','history-plot','experience-view','principle-view','engineering-view','report-refresh','report']) assert.ok(html.includes('id="'+id+'"'),'Missing HTML element '+id);
  assert.equal(/https?:\/\/|<a[^>]+download|@import/.test(html+css),false);
}]);
tests.push(['a fresh server state without an owner revokes local control even if heartbeat succeeded',async () => {
  const f=await fixture();f.state.has_owner=false;await f.tick();f.key('ArrowUp');await f.settle();
  assert.equal(f.arrows.every(b=>b.disabled),true);assert.equal(f.byId('connect').disabled,false);
  assert.equal(f.requests.some(r=>r.action==='keys'),false);
}]);
tests.push(['a withdrawn reset clears local waiting without claiming success or resuming a previous movement',async () => {
  const f=await fixture();f.byId('payload').value='2';f.byId('reset').fire('click');await f.settle();
  f.state.reset_pending=true;f.state.connected=false;await f.tick();f.state.reset_pending=false;f.state.connected=true;
  f.state.telemetry.baseline.sim_time_s=2;f.state.telemetry.candidate.sim_time_s=2;await f.tick();
  assert.equal(f.arrows.every(b=>b.disabled),false);assert.equal(f.requests.some(r=>r.action==='keys'),false);
  assert.match(f.byId('notice').textContent,/取消|撤销|未确认/);
  f.key('ArrowUp');await f.settle();assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
}]);
for(const failed of [true,false]) tests.push([(failed?'failed':'delayed')+' stop on blur does not refresh a lease; focus recovery requires released ownership and a new claim',async () => {
  const f=await fixture();f.key('ArrowUp');await f.settle();
  if(failed)f.failControl('stop');else f.block('stop');
  f.window.fire('blur');await f.settle();const heartbeats=f.requests.filter(r=>r.action==='heartbeat').length;
  await f.tick();await f.tick();assert.equal(f.requests.filter(r=>r.action==='heartbeat').length,heartbeats);
  f.window.fire('focus');f.key('ArrowUp');await f.tick();
  assert.equal(f.requests.filter(r=>r.action==='heartbeat').length,heartbeats,'Returning focus must not revive the retired lease');
  assert.equal(f.requests.filter(r=>r.action==='keys').length,1,'Old control remains disabled before service release');
  if(!failed)f.unblock();else f.failControl('');await f.settle();
  f.state.has_owner=false;f.state.telemetry.baseline.sim_time_s=2;f.state.telemetry.candidate.sim_time_s=2;await f.tick();
  f.key('ArrowUp');await f.settle();assert.equal(f.requests.filter(r=>r.action==='keys').length,1);
  assert.equal(f.byId('connect').disabled,false);f.byId('connect').fire('click');await f.settle();await f.tick();f.key('ArrowUp');await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys').length,2);assert.equal(f.requests.filter(r=>r.action==='keys').at(-1).session,'owner-2');
}]);
(async () => {for(const [name,test] of tests) {await test(); console.log('PASS '+name);} console.log(`PASS ${tests.length} synthetic frontend behavior tests`);})()
  .catch(e=>{console.error(e);process.exitCode=1;});
