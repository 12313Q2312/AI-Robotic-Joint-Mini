'use strict';
// Browser-boundary regression only: this does not render a real browser.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const web = path.join(__dirname, '../ros2_ws/src/ai_robotic_joint_ros2/web');
const source = fs.readFileSync(path.join(web, 'panel.js'), 'utf8');
const html = fs.readFileSync(path.join(web, 'index.html'), 'utf8');

class Surface {
  constructor(tagName = 'BUTTON', dataset = {}) {
    this.tagName = tagName; this.dataset = dataset; this.listeners = {};
    this.classList = {toggle() {}}; this.value = ''; this.disabled = false;
  }
  addEventListener(name, callback) { (this.listeners[name] ||= []).push(callback); }
  fire(name, values = {}) {
    const event = Object.assign({target:this, preventDefault() {}, repeat:false}, values);
    for (const callback of this.listeners[name] || []) callback(event);
  }
  setPointerCapture() {}
  checkValidity() { return Number.isFinite(Number(this.value)) && Math.abs(Number(this.value)) <= 45; }
}

async function fixture() {
  const document = new Surface('DOCUMENT'), window = new Surface('WINDOW');
  const elements = Object.fromEntries(['notice','connect','stop','connection','reason','requested',
    'actual','reference','error','velocity','time','angle-form','angle','execute',
    'keyboard-panel','angle-panel','ai-panel'].map(id => [id, new Surface()]));
  elements.angle.tagName = 'INPUT';
  const modes = ['keyboard','angle','ai'].map(mode => new Surface('BUTTON', {mode}));
  const arrows = ['ArrowUp','ArrowDown'].map(key => new Surface('BUTTON', {key}));
  const nudges = [-1,1].map(nudge => new Surface('BUTTON', {nudge}));
  document.hidden = false;
  document.getElementById = id => elements[id];
  document.querySelector = () => ({content:'test-token'});
  document.querySelectorAll = selector => ({'[data-mode]':modes, '[data-key]':arrows,
    '[data-nudge]':nudges, '[data-key],[data-nudge],#execute':[...arrows,...nudges,elements.execute]})[selector];
  const requests = [];
  const state = {mode:'keyboard', connected:true, requested_angle_deg:0, reason:'synthetic test feedback', telemetry:null};
  let interval;
  const fetch = async (url, options) => {
    if (url === '/api/control') {
      const payload = JSON.parse(options.body); requests.push(payload);
      if (payload.action === 'mode') state.mode = payload.mode;
      return {ok:true, json:async () => payload.action === 'claim' ? {session:'test-owner'} : {ok:true}};
    }
    return {ok:true, json:async () => ({...state})};
  };
  vm.runInNewContext(source, {document, window, fetch, setInterval:callback => {interval = callback;}});
  const settle = async () => { for (let i=0; i<12; i++) await new Promise(resolve => setImmediate(resolve)); };
  await settle(); elements.connect.fire('click'); await settle(); await interval();
  requests.length = 0;
  return {document,window,elements,modes,arrows,nudges,requests,state,settle,
    tick:async () => {await interval(); await settle();},
    key:(key, repeat=false, target=document) => document.fire('keydown', {key,repeat,target}),
    up:key => document.fire('keyup', {key})};
}

(async () => {
  assert.match(html, /id="angle"[^>]*step="any"/, 'Angle input must accept any finite in-range angle, not only 0.25 multiples');
  let f = await fixture();
  f.key('ArrowUp'); f.key('ArrowUp', true); await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys').length, 1);
  f.up('ArrowUp'); await f.settle();
  assert.deepEqual(f.requests.at(-1).keys, []);

  f = await fixture(); f.key('ArrowRight'); f.key('ArrowRight', true); await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='nudge').length, 1);
  assert.equal(f.requests[0].direction, 1);

  f = await fixture(); f.key('ArrowUp', false, f.elements.angle); await f.settle();
  assert.equal(f.requests.length, 0, 'Editing ignores arrows');

  f = await fixture(); f.key('ArrowUp'); await f.settle();
  f.window.fire('blur'); await f.settle();
  const count = f.requests.length;
  f.key('ArrowUp', true); await f.settle();
  assert.equal(f.requests.length, count, 'OS repeat cannot restart after blur');
  assert.equal(f.requests.at(-1).action, 'stop');

  f = await fixture(); f.key('ArrowUp'); f.key('ArrowDown'); await f.settle();
  assert.deepEqual(f.requests.at(-1).keys, ['ArrowUp','ArrowDown']);
  f.document.hidden = true; f.document.fire('visibilitychange'); await f.settle();
  assert.equal(f.requests.at(-1).action, 'stop');
  f.window.fire('pagehide'); await f.settle();
  assert.equal(f.requests.at(-1).action, 'release');

  f = await fixture(); f.key('ArrowUp'); await f.settle();
  f.state.connected = false; await f.tick();
  f.state.connected = true; await f.tick();
  const moves = f.requests.filter(r=>r.action==='keys').length;
  f.key('ArrowUp', true); await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys').length, moves, 'No automatic restart after stale feedback');

  f = await fixture(); f.modes[2].fire('click'); await f.settle(); await f.tick();
  f.key('ArrowUp'); await f.settle();
  assert.equal(f.requests.filter(r=>r.action==='keys' || r.action==='angle' || r.action==='nudge').length, 0);
  console.log('PASS 8 frontend boundary groups: arbitrary angle, keys/repeat/release, nudge, editing, blur, hide/close, feedback loss, AI');
})().catch(error => {console.error(error); process.exitCode = 1;});
