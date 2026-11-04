'use strict';
const token = document.querySelector('meta[name="panel-token"]').content;
let session = '', mode = 'keyboard', connected = false, busy = false;
const keys = new Set();
const notice = text => { document.getElementById('notice').textContent = text; };
// Serialized HTTP commands preserve stop/mode/key ordering. Never retry a move.
let queue = Promise.resolve();
function send(action, values = {}, keepalive = false) {
  const owner = session;
  const payload = Object.assign({action, session: owner}, values);
  const operation = () => fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/json','X-Panel-Token':token}, body:JSON.stringify(payload), keepalive})
    .then(async response => { const result = await response.json(); if (!response.ok) throw new Error(result.error); return result; });
  const result = queue.then(operation);
  queue = result.catch(error => { notice(error.message); });
  return result;
}
function stop(keepalive = false) {
  keys.clear();
  if (session) send('stop', {}, keepalive).catch(() => {});
}
async function connect() {
  if (session) return;
  try { const result = await send('claim'); session = result.session; notice('已取得控制权；等待反馈后可操作。'); }
  catch (_) { /* Already shown. */ }
}
function isEditing(event) { return /INPUT|TEXTAREA|SELECT/.test(event.target.tagName) || event.target.isContentEditable; }
function down(key, repeat) {
  if (mode !== 'keyboard' || !connected || !session) return;
  if (key === 'ArrowUp' || key === 'ArrowDown') {
    if (repeat && !keys.has(key)) return; // No re-arming after blur/reset from OS repeats.
    if (!keys.has(key)) { keys.add(key); send('keys', {keys:Array.from(keys)}).catch(() => {}); }
  } else if (!repeat && (key === 'ArrowLeft' || key === 'ArrowRight')) {
    send('nudge', {direction:key === 'ArrowLeft' ? -1 : 1}).catch(() => {});
  }
}
function up(key) {
  if (keys.delete(key) && session) send('keys', {keys:Array.from(keys)}).catch(() => {});
}
document.addEventListener('keydown', event => {
  if (isEditing(event) || mode !== 'keyboard' || !event.key.startsWith('Arrow')) return;
  event.preventDefault(); down(event.key, event.repeat);
});
document.addEventListener('keyup', event => up(event.key));
document.querySelectorAll('[data-key]').forEach(button => {
  button.addEventListener('pointerdown', event => { event.preventDefault(); button.setPointerCapture(event.pointerId); down(button.dataset.key, false); });
  ['pointerup','pointercancel','lostpointercapture'].forEach(name => button.addEventListener(name, () => up(button.dataset.key)));
});
document.querySelectorAll('[data-nudge]').forEach(button => button.addEventListener('click', () => {
  if (connected && session) send('nudge', {direction:Number(button.dataset.nudge)}).catch(() => {});
}));
document.querySelectorAll('[data-mode]').forEach(button => button.addEventListener('click', async () => {
  if (!session) { notice('先连接控制权。'); return; }
  keys.clear();
  try { await send('mode', {mode:button.dataset.mode}); } catch (_) {}
}));
document.getElementById('angle-form').addEventListener('submit', event => {
  event.preventDefault();
  const field = document.getElementById('angle');
  if (field.value.trim() === '' || !field.checkValidity()) { notice('请输入 −45° 到 +45°之间的角度。'); return; }
  send('angle', {angle_deg:Number(field.value)}).catch(() => {});
});
document.getElementById('connect').addEventListener('click', connect);
document.getElementById('stop').addEventListener('click', () => stop());
window.addEventListener('blur', () => stop(true));
document.addEventListener('visibilitychange', () => { if (document.hidden) stop(true); });
window.addEventListener('pagehide', () => { keys.clear(); if (session) send('release', {}, true).catch(() => {}); });
const degrees = value => (value * 180 / Math.PI).toFixed(2);
async function tick() {
  if (busy || document.hidden) return;
  busy = true;
  try {
    if (session) {
      try { await send('heartbeat'); } catch (_) { session = ''; keys.clear(); }
    }
    const response = await fetch('/api/state', {cache:'no-store'});
    if (!response.ok) throw new Error('状态服务不可用');
    const state = await response.json();
    mode = state.mode; connected = state.connected;
    if (!connected) keys.clear();
    document.getElementById('connection').textContent = connected ? (session ? '已连接 · 人工控制' : '反馈在线 · 未取得控制权') : '无新鲜反馈 · 禁止启动';
    document.getElementById('connect').disabled = Boolean(session);
    document.getElementById('reason').textContent = state.reason;
    document.querySelectorAll('[data-mode]').forEach(button => button.classList.toggle('active', button.dataset.mode === mode));
    ['keyboard','angle','ai'].forEach(name => { document.getElementById(name + '-panel').hidden = mode !== name; });
    document.querySelectorAll('[data-key],[data-nudge],#execute').forEach(button => { button.disabled = !connected || !session; });
    document.getElementById('stop').disabled = !session;
    document.getElementById('requested').textContent = state.requested_angle_deg.toFixed(2) + '°';
    const t = state.telemetry;
    if (t) {
      document.getElementById('actual').textContent = degrees(t.actual_angle_rad);
      document.getElementById('reference').textContent = degrees(t.reference_angle_rad) + '°';
      document.getElementById('error').textContent = degrees(t.reference_angle_rad - t.actual_angle_rad) + '°';
      document.getElementById('velocity').textContent = degrees(t.velocity_rad_s) + '°/s';
      document.getElementById('time').textContent = t.sim_time_s.toFixed(3) + ' s';
    }
  } catch (error) { connected = false; keys.clear(); notice('连接中断：' + error.message); }
  finally { busy = false; }
}
setInterval(tick, 100);
tick();
