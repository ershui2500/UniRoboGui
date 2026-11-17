const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync('web/app.js', 'utf8');
const elements = new Map();
const makeElement = () => ({
  value: '', disabled: false, hidden: false, textContent: '', dataset: {},
  setAttribute(name, value) { this[name] = value; },
  listeners: {},
  addEventListener(name, callback) { this.listeners[name] = callback; },
  replaceChildren(...options) { this.options = options; this.value = ''; },
  append(...children) { this.children = children; },
  querySelectorAll(selector) {
    return (this.options || []).filter((item) => item.dataset?.pcInterface !== undefined &&
      (!selector.includes(':not(:disabled)') || !item.disabled));
  },
  contains(target) { return target === this || (this.options || []).includes(target); },
  focus() { context.document.activeElement = this; },
  click() { this.listeners.click?.({target: this}); },
});
const element = (id) => {
  if (!elements.has(id)) elements.set(id, makeElement());
  return elements.get(id);
};
element('pcInterfaceMenu').hidden = true;
let language = 'en';
let requests = 0;
let nextResponse;
const context = vm.createContext({
  $: element,
  document: {
    activeElement: null, createElement: makeElement, listeners: {},
    addEventListener(name, callback) { this.listeners[name] = callback; },
  },
  closeRobotMenu: () => {},
  appState: {
    pcSession: {state: 'disconnected', selected_product_id: 'g1', interface: '', mock: true, session_id: 'one'},
    pcBusy: false, pcInterfacesLoading: false, pcInterfacesError: false,
    pcInterfaces: [{name: 'eth-test', ipv4: ['192.0.2.2'], up: true},
                   {name: 'eth-other', ipv4: ['192.0.2.3'], up: true},
                   {name: 'eth-down', ipv4: [], up: false}],
    robotProducts: [{product_id: 'g1', pc_connect_supported: true},
                    {product_id: 'r1', pc_connect_supported: true}],
  },
  fetchWithTimeout: async () => { requests++; return nextResponse; },
  manifestText: (zh, en) => language === 'en' ? en : zh,
  setText: (id, text) => { element(id).textContent = text; },
  setConnection: (kind, label, detail) => {
    element('connection').textContent = label;
    element('connectionDetail').textContent = detail;
  },
});
vm.runInContext(source.slice(source.indexOf('function pcConnectionAction()'),
  source.indexOf('async function pcAction(')), context);
const run = (code) => vm.runInContext(code, context);
const render = () => run('renderPcSession()');
const state = context.appState;
class Socket {
  static OPEN = 1;
  constructor() { this.readyState = Socket.OPEN; this.listeners = {}; }
  addEventListener(name, callback) { this.listeners[name] = callback; }
}
const telemetryState = {
  pcSession: {session_id: 'telemetry-test', interface: 'eth-test', state: 'stale'},
  robotManifest: {capabilities: [{key: 'telemetry', parameters: {
    required_sources: 'low_state,bms,secondary_imu,mainboard,odometry',
  }}]},
  controlUnlocked: true,
};
const telemetryContext = vm.createContext({
  appState: telemetryState, WebSocket: Socket, location: {protocol: 'http:', host: 'localhost'},
  clearTimeout() {}, setConnection() {}, renderPcSession() {},
  render(data) { telemetryState.controlData = data.control; }, setText() {},
  capabilityAvailable: () => true, isLocomotionFsm: () => true,
  console, Date,
});
vm.runInContext(source.slice(source.indexOf('function connect()'),
  source.indexOf('function scheduleReconnect()')), telemetryContext);
vm.runInContext(source.slice(source.indexOf('function motionBaseReady()'),
  source.indexOf('function updateMotionControls()')), telemetryContext);
vm.runInContext('connect()', telemetryContext);
for (const product of ['g1', 'r1', 'go2', 'a2']) {
  telemetryState.pcSession.selected_product_id = product;
  const frame = {
    pc_session_id: 'telemetry-test', dds_initialized: true,
    sources: {low_state: {received: true, status: 'online'}, odometry: {received: true, status: 'offline'}},
    control: {initialized: true, enabled: true, sport_state_received: true, sport_state_age_ms: 60000},
  };
  const receive = () => telemetryState.socket.listeners.message({data: JSON.stringify(frame)});
  receive();
  assert.equal(telemetryState.pcSession.state, 'online', `${product}: auxiliary loss cannot disconnect LowState`);
  assert.equal(vm.runInContext('motionBaseReady()', telemetryContext), false, `${product}: stale motion state still blocks motion`);
  frame.control.sport_state_age_ms = 1;
  receive();
  assert.equal(vm.runInContext('motionBaseReady()', telemetryContext), true);
  for (const status of ['offline', 'delayed']) {
    frame.sources.low_state.status = status;
    receive();
    assert.equal(telemetryState.pcSession.state, 'stale');
  }
  frame.sources.low_state = {received: false, status: 'offline'};
  receive();
  assert.equal(telemetryState.pcSession.state, 'waiting_telemetry');
  frame.pc_session_id = 'old-session';
  frame.sources.low_state = {received: true, status: 'online'};
  receive();
  assert.equal(telemetryState.pcSession.state, 'waiting_telemetry', 'old data must not revive the session');
}
const controlElements = new Map();
const controlView = vm.createContext({
  appState: {robotManifest: {capabilities: [{key: 'locomotion', available: true}]}},
  $: (id) => {
    if (!controlElements.has(id)) controlElements.set(id, {classList: {add() {}, remove() {}}});
    return controlElements.get(id);
  },
  setText: (id, text) => controlElements.set(id, {textContent: text}),
  motionFsmFamily: () => '', fsmDisplayName: () => 'Fresh FSM', integer: String, number: String,
  manifestText: (zh, en) => en,
});
const controlStart = source.indexOf('function renderControl(');
vm.runInContext(source.slice(controlStart, source.indexOf('  const command = control.last_command', controlStart)) + '\n}', controlView);
for (const age of [null, -1, 1001, 60000]) {
  controlView.control = {initialized: true, enabled: true, sport_state_received: true,
    sport_state_age_ms: age, fsm_id: 1001, fsm_mode: 0};
  vm.runInContext('renderControl(control)', controlView);
  assert.equal(controlElements.get('topFsm').textContent, '--', 'cached FSM is not current motion state');
  assert.equal(controlElements.get('controlFsmName').textContent, 'Motion state unavailable');
}
controlView.control.sport_state_age_ms = 1;
vm.runInContext('renderControl(control)', controlView);
assert.equal(controlElements.get('topFsm').textContent, '1001');
render();
assert.equal(element('pcConnect').disabled, true);
assert.equal(element('pcConnect').textContent, 'Connect');
assert.equal(element('staleBanner').hidden, true);
assert.equal(run('shouldRefreshPcInterfaces()'), true);
element('pcInterface').value = 'eth-test';
render();
assert.equal(element('pcConnect').disabled, false);
state.pcSession.selected_product_id = 'r1';
render();
assert.equal(element('pcConnect').disabled, false);
state.pcSession.state = 'online';
render();
assert.match(element('connection').textContent, /R1 telemetry online/);
state.controlData = {enabled: true, sport_state_received: true, sport_state_age_ms: 60000};
render();
assert.match(element('connectionDetail').textContent, /Motion state unavailable/);
assert.equal(element('staleBanner').hidden, true, 'motion-state loss must not mark fresh base telemetry stale');
language = 'zh';
render();
assert.match(element('connectionDetail').textContent, /运控状态未更新/);
language = 'en';
state.controlData.sport_state_age_ms = 1;
render();
assert.doesNotMatch(element('connectionDetail').textContent, /unavailable/);
delete state.controlData;
state.pcSession.selected_product_id = 'g1';
state.pcSession.state = 'disconnected';
state.pcSession.interface = 'eth-test';
for (const connectionState of ['online', 'waiting_telemetry', 'stale', 'network_unavailable', 'error']) {
  state.pcSession.state = connectionState;
  render();
  assert.equal(element('pcConnect').textContent, 'Disconnect', connectionState);
  assert.equal(element('pcConnect').disabled, false, connectionState);
  assert.equal(run('shouldRefreshPcInterfaces()'), false, connectionState);
}
assert.match(element('connection').textContent, /Mock/);
assert.equal(element('staleBanner').hidden, false);
state.pcSession.arm_action_state = 'unknown';
render();
assert.equal(element('pcRecover').hidden, false);
assert.match(element('pcArmState').textContent, /unknown/);
Object.assign(state.pcSession, {recovery_required: true, worker_readonly: true, worker_alive: true});
render();
assert.equal(element('pcConnect').disabled, true);
assert.equal(element('robotMenuButton').disabled, true);
assert.equal(element('pcRecover').disabled, false, 'manual read-only rebuild remains available after data loss');
assert.equal(element('pcConfirmRecovery').hidden, false);
assert.equal(element('pcConfirmRecovery').disabled, false, 'missing telemetry must not trap a read-only session');
for (const connectionState of ['waiting_telemetry', 'stale', 'network_unavailable', 'error']) {
  state.pcSession.state = connectionState;
  render();
  assert.equal(element('pcConfirmRecovery').disabled, false, connectionState);
}
state.pcBusy = true;
render();
assert.equal(element('pcConfirmRecovery').disabled, true);
state.pcBusy = false;
state.pcSession.state = 'online';
render();
assert.equal(element('pcConfirmRecovery').disabled, false);
assert.match(element('pcRecoveryNotice').textContent, /telemetry only/);
language = 'zh';
render();
assert.equal(element('pcConfirmRecovery').textContent, '确认现场停止并断开');
assert.match(element('pcRecoveryNotice').textContent, /旧任务状态未知/);
language = 'en';
Object.assign(state.pcSession, {recovery_required: false, worker_readonly: false, arm_action_state: 'idle'});
element('pcInterface').value = 'eth-other';
render();
assert.equal(element('pcConnect').textContent, 'Connect');
assert.equal(element('pcConnect').disabled, false);
assert.match(element('pcNotice').textContent, /current connection will end/);
element('pcInterface').value = 'eth-down';
render();
assert.equal(element('pcConnect').disabled, true);
element('pcInterface').value = '';
render();
assert.equal(element('pcConnect').textContent, 'Disconnect');
state.pcBusy = true;
for (const [path, label] of [['/api/pc/connect', 'Connecting…'], ['/api/pc/disconnect', 'Disconnecting…'], ['/api/robot/switch', 'Switching…']]) {
  state.pcPendingAction = path;
  render();
  assert.equal(element('pcConnect').textContent, label);
  for (const id of ['pcConnect', 'pcInterface', 'robotMenuButton']) assert.equal(element(id).disabled, true);
}
assert.equal(run('shouldRefreshPcInterfaces()'), false);
state.pcBusy = false;
state.pcSession.interface = '';
state.pcSession.state = 'error';
render();
assert.equal(element('pcConnect').textContent, 'Disconnect');
state.pcSession.state = 'disconnected';
context.document.activeElement = element('pcInterfaceButton');
assert.equal(run('shouldRefreshPcInterfaces()'), true, 'closed menu does not pause detection on trigger focus');
context.document.activeElement = null;
state.pcInterfacesLoading = true;
assert.equal(run('shouldRefreshPcInterfaces()'), false);
state.pcInterfacesLoading = false;
language = 'zh';
render();
assert.equal(element('pcConnect').textContent, '连接');
assert.equal(element('pcDetailsLabel')['aria-label'], '连接信息');
assert.doesNotMatch(element('pcNotice').textContent, /首期|验收|物理设备/);
language = 'en';

(async () => {
  const interfaces = state.pcInterfaces;
  nextResponse = {ok: true, json: async () => ({interfaces})};
  element('pcInterface').value = 'eth-other';
  await run('loadPcInterfaces()');
  assert.equal(requests, 1);
  assert.equal(element('pcInterface').value, 'eth-other', 'refresh preserves choice');
  assert.equal(element('pcInterface').options[3].disabled, true);
  state.pcBusy = true;
  await run('loadPcInterfaces()');
  assert.equal(requests, 1, 'no list request during an action');
  state.pcBusy = false;
  nextResponse = {ok: false};
  await run('loadPcInterfaces()');
  assert.equal(state.pcInterfacesLoading, false);
  assert.equal(element('pcConnect').disabled, true, 'failed detection blocks connecting');
  assert.match(element('pcNotice').textContent, /retry automatically/);
  nextResponse = {ok: true, json: async () => ({interfaces})};
  await run('loadPcInterfaces()');
  assert.equal(element('pcConnect').disabled, false, 'detection recovers');
  nextResponse = {ok: true, json: async () => { state.pcBusy = true; return {interfaces: []}; }};
  await run('loadPcInterfaces()');
  assert.equal(state.pcInterfaces.length, 3, 'late list cannot overwrite an ongoing action');
  state.pcBusy = false;
  nextResponse = {ok: true, json: async () => ({interfaces: []})};
  await run('loadPcInterfaces()');
  assert.equal(element('pcInterface').value, '');
  assert.equal(element('pcConnect').disabled, true);
  // Exercise the actual timer and button wiring without a worker or real DDS.
  const timers = [];
  const actions = [];
  let listRequests = 0;
  context.window = {};
  context.location = {hash: '#robot'};
  context.document.body = {classList: {add() {}}};
  context.setInterval = (callback, ms) => timers.push({callback, ms});
  context.pcAction = async (path, payload) => actions.push({path, payload});
  context.fetchWithTimeout = async (path) => ({ok: true, json: async () => {
    if (path === '/api/pc/interfaces') { listRequests++; return {interfaces}; }
    return {runtime_mode: 'pc_sdk2', state: 'disconnected', selected_product_id: 'g1',
      interface: '', session_id: 'two', mock: true};
  }});
  vm.runInContext(source.slice(source.indexOf('function initializePcInterfaceMenu()'),
    source.indexOf('function setConnection(')), context);
  await run('initializePcSession()');
  assert.equal(listRequests, 1);
  const detectionTimer = timers.find((timer) => timer.ms === 5000);
  assert.ok(detectionTimer, 'auto-detection runs every five seconds');
  detectionTimer.callback();
  await new Promise(setImmediate);
  assert.equal(listRequests, 2, 'disconnected timer refreshes the list');
  state.pcSession.interface = 'eth-test';
  detectionTimer.callback();
  await new Promise(setImmediate);
  assert.equal(listRequests, 2, 'connected timer does not refresh the list');
  element('pcConnectionDetails').open = true;
  element('pcInterfaceButton').click();
  assert.equal(element('pcInterfaceMenu').hidden, false);
  assert.equal(element('pcConnectionDetails').open, false);
  assert.equal(run('shouldRefreshPcInterfaces()'), false, 'open menu pauses detection');
  const menu = element('pcInterfaceMenu');
  const down = menu.options.find((item) => item.dataset.pcInterface === 'eth-down');
  const other = menu.options.find((item) => item.dataset.pcInterface === 'eth-other');
  const menuClick = (option) => menu.listeners.click({target: {closest: () => option}});
  menuClick(down);
  assert.notEqual(element('pcInterface').value, 'eth-down', 'unready option cannot be selected');
  menuClick(other);
  assert.equal(element('pcInterface').value, 'eth-other');
  assert.equal(menu.hidden, true);
  assert.equal(context.document.activeElement, element('pcInterfaceButton'));
  element('pcInterfaceButton').click();
  element('pcInterfaceSwitcher').listeners.keydown({key: 'Home', preventDefault() {}});
  assert.equal(context.document.activeElement, menu.options[0]);
  element('pcInterfaceSwitcher').listeners.keydown({key: 'End', preventDefault() {}});
  assert.equal(context.document.activeElement, other);
  element('pcInterfaceSwitcher').listeners.keydown({key: 'Escape'});
  assert.equal(menu.hidden, true);
  element('pcConnectionDetails').open = true;
  context.document.listeners.click({target: element('pcConnectionDetails')});
  assert.equal(element('pcConnectionDetails').open, true, 'inside click preserves details');
  context.document.listeners.click({target: {}});
  assert.equal(element('pcConnectionDetails').open, false, 'outside click dismisses details');
  element('pcInterfaceButton').click();
  context.document.listeners.click({target: {}});
  assert.equal(menu.hidden, true, 'outside click dismisses the menu');
  state.activeRobotProductId = 'g1';
  element('pcInterface').value = 'eth-other';
  await element('pcConnect').listeners.click();
  assert.equal(actions[0].path, '/api/pc/connect');
  assert.equal(actions[0].payload.interface, 'eth-other');
  assert.equal(state.pcSession.interface, 'eth-test', 'selecting does not switch the active session');
  element('pcInterface').value = 'eth-test';
  await element('pcConnect').listeners.click();
  assert.equal(actions[1].path, '/api/pc/disconnect');
  await element('pcRecover').listeners.click();
  assert.equal(actions[2].path, '/api/pc/recover');
  assert.equal(actions[2].payload.session_id, 'two');
  let confirmation;
  context.window.confirm = (message) => { confirmation = message; return false; };
  await element('pcConfirmRecovery').listeners.click();
  assert.equal(actions.length, 3, 'cancelled field confirmation cannot clear isolation');
  assert.match(confirmation, /all previous motion/);
  context.window.confirm = () => true;
  await element('pcConfirmRecovery').listeners.click();
  assert.equal(actions[3].path, '/api/pc/confirm-recovery');
  assert.equal(actions[3].payload.session_id, 'two');
  assert.equal(actions[3].payload.confirmed, true);
  state.robotProducts.forEach((product) => { product.display_name = `Unitree ${product.product_id.toUpperCase()}`; });
  vm.runInContext(source.slice(source.indexOf('function renderRobotSwitcher()'),
    source.indexOf('async function loadRobotProducts()')), context);
  run('renderRobotSwitcher()');
  assert.equal(element('robotMenu').options[0].children[1].textContent, 'G1');
  assert.equal(element('robotMenu').options[1].children[1].textContent, 'R1');
  console.log('PC actions, auto-detection, styled menu selection/keyboard/dismissal and compact model labels passed');
})().catch((error) => { console.error(error); process.exitCode = 1; });
