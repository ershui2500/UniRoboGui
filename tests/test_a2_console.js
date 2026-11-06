const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const context = vm.createContext({window: {}});
vm.runInContext(fs.readFileSync('web/motion-policy.js', 'utf8'), context);
const policy = context.window.MotionPolicy;
const profile = fs.readFileSync('src/a2_profile.cpp', 'utf8');
const parameter = (name) => profile.match(new RegExp(`\\{"${name}", "([^"]+)"\\}`))[1];
const parameters = Object.fromEntries(['speed_presets', 'fsm_speed_presets', 'speed_labels'].map((name) => [name, parameter(name)]));
const expected = [[[.6,.4,1.5],[1.5,.5,2.5]],[[2,.25,1.2],[4,.3,1]],[[.5,.25,.8],[.6,.3,1]]];
for (let fsm = 3; fsm <= 5; fsm++) {
  for (let speed = 0; speed <= 1; speed++) {
    assert.deepEqual(Object.values(policy.preset(parameters, fsm, speed)), expected[fsm-3][speed]);
  }
}
assert.equal(policy.label(parameters, 1, 'en'), 'High');
assert.equal(policy.label(parameters, 1, 'zh-CN'), '高速');
assert.equal(policy.label({}, 1, 'en'), null);
assert.equal(policy.preset({}, 3, 0), null);
assert.equal(policy.preset({speed_presets: '0:NaN:1:1'}, 3, 0), null);
assert.equal(policy.preset({speed_presets: '0:0.2:0.1:0.3'}, 100, 0).forwardSpeed, .2);
assert.equal(policy.preset(parameters, 3, 3), null);
console.log('A2 FSM presets, two native speed levels, labels and legacy fallback passed.');

// A service RPC fault must not be presented as a missing launch flag.
const perceptionSource = fs.readFileSync('web/perception.js', 'utf8');
const availability = perceptionSource.slice(
  perceptionSource.indexOf('function updateNavigationAvailability()'),
  perceptionSource.indexOf('function updateWorkflowState('));
for (const language of ['zh-CN', 'en']) {
  const navContext = vm.createContext({
    latestStatus: {navigation_enabled: false, services: {error: 'slam_rpc_unverified:3203'}},
    sceneReady: true, multiRouteActive: false, navigationMode: 'single', goalSelected: false,
    waypoints: [], mapIsLoaded: () => false, currentGoalPose: () => null,
    navigationMaxDistance: () => 30, updateGoalSummary: () => {},
    navigateButton: {}, goalPickButton: {}, singleNavigationModeButton: {},
    multiNavigationModeButton: {}, clearWaypointsButton: {},
    cameraText: (zh, en) => language === 'en' ? en : zh,
  });
  vm.runInContext(availability + '\nupdateNavigationAvailability();', navContext);
  assert.equal(navContext.navigateButton.disabled, true);
  assert.match(navContext.navigateButton.title, /slam_operate/);
  assert.doesNotMatch(navContext.navigateButton.title, /--enable-navigation/);
  navContext.latestStatus.services.error = '';
  vm.runInContext('updateNavigationAvailability();', navContext);
  assert.match(navContext.navigateButton.title, /--enable-navigation/);
}
// A2 compatibility keeps the warning but allows operator navigation once ready.
const manualContext = vm.createContext({
  latestStatus: {navigation_enabled: true, services: {error: 'slam_rpc_unverified:3203'},
    pose_received: true, pose_age_ms: 20, mode: 'localizing', pose: {x: 0, y: 0}},
  sceneReady: true, multiRouteActive: false, navigationMode: 'single', goalSelected: true,
  waypoints: [], mapIsLoaded: () => true, currentGoalPose: () => ({x: 1, y: 0}),
  navigationMaxDistance: () => 30, updateGoalSummary: () => {},
  navigateButton: {}, goalPickButton: {}, singleNavigationModeButton: {},
  multiNavigationModeButton: {}, clearWaypointsButton: {},
  cameraText: (zh, en) => en,
});
vm.runInContext(availability + '\nupdateNavigationAvailability();', manualContext);
assert.equal(manualContext.navigateButton.disabled, false);
manualContext.latestStatus.pose_age_ms = 2000;
vm.runInContext('updateNavigationAvailability();', manualContext);
assert.equal(manualContext.navigateButton.disabled, true);
console.log('SLAM warning permits enabled operator tests while preserving freshness and deployment gates.');

// Leaving an idle keyboard controller must not stop SLAM navigation.
const appSource = fs.readFileSync('web/app.js', 'utf8');
const disableSource = appSource.slice(appSource.indexOf('function disableMotionControl('), appSource.indexOf('function currentMotionKeys('));
const stops = [];
const stopContext = vm.createContext({
  appState: {motionEnabled: true, motionInputOwned: false},
  releaseAllMotionInput: value => stops.push(value),
  $: () => ({}), updateMotionControls: () => {},
});
vm.runInContext(disableSource + '\ndisableMotionControl();', stopContext);
assert.equal(stops.pop(), false);
stopContext.appState.motionInputOwned = true;
vm.runInContext('disableMotionControl();', stopContext);
assert.equal(stops.pop(), true);
console.log('Idle keyboard panel changes preserve navigation; owned motion still stops.');

// Only A2's accepted, empty 3203 reply is whitelisted; all failures still reject.
const whitelistSource = perceptionSource.slice(
  perceptionSource.indexOf('function isWhitelistedSlamRpc('),
  perceptionSource.indexOf('function setBadge('));
const rpcContext = vm.createContext({
  window: {robotManifest: {capabilities: [{key:'slam', parameters:{empty_rpc_warning_whitelist:'3203'}}]}},
  cameraText: (zh, en) => en,
});
vm.runInContext(whitelistSource, rpcContext);
const known = {accepted: true, api_result: 3203, api_response: ''};
assert.equal(rpcContext.isWhitelistedSlamRpc(known), true);
for (const result of [
  {...known, api_result: 3204}, {...known, accepted: false},
  {...known, api_response: '{"succeed":false}'}, {...known, error: 'slam_topic_timeout'},
]) assert.equal(rpcContext.isWhitelistedSlamRpc(result), false);
const badgeSource = perceptionSource.slice(perceptionSource.indexOf('function updateStatus('),
  perceptionSource.indexOf('  slamMode.textContent =')) + '\n}';
const badges = [];
rpcContext.setBadge = (text, state) => badges.push({text, state});
vm.runInContext(badgeSource, rpcContext);
const status = {initialized: true, mock: false, mode: 'localizing',
  pose_received: true, pose_age_ms: 20, navigation_enabled: true,
  services: {error: 'slam_rpc_unverified:3203'}};
rpcContext.updateStatus(status);
assert.equal(badges.pop().state, 'online');
rpcContext.updateStatus({...status, error_code: 509});
assert.deepEqual(badges.pop(), {text: '错误 509', state: 'error'});
rpcContext.updateStatus({...status, services: {error: 'slam_rpc_unverified:3204'}});
assert.equal(badges.pop().state, 'warning');
rpcContext.window.robotManifest.capabilities[0].parameters = {};
assert.equal(rpcContext.isWhitelistedSlamRpc(known), false);
rpcContext.updateStatus(status);
assert.equal(badges.pop().state, 'warning');
(async () => {
  rpcContext.window.robotManifest.capabilities[0].parameters.empty_rpc_warning_whitelist = '3203';
  for (const result of [{...known, accepted: false, error: 'slam_api_failed'},
    {...known, accepted: false, api_result: 3204, error: 'slam_api_failed'},
    {...known, accepted: false, error: 'slam_topic_timeout'}]) {
    await assert.rejects(rpcContext.readCommandResponse({ok: false, status: 409,
      text: async () => JSON.stringify(result)}));
  }
  console.log('A2 3203 display whitelist hides known warnings; other products, codes and command failures remain visible.');
})().catch(error => { console.error(error); process.exitCode = 1; });

// Diagnostics must not treat cached LowState as a current motor fault or healthy reading.
const fields = new Map();
const element = () => ({children: [], hidden: false, className: '', textContent: '',
  replaceChildren() { this.children = []; }, append(...items) { this.children.push(...items); }});
const diagContext = vm.createContext({
  $: id => { if (!fields.has(id)) fields.set(id, element()); return fields.get(id); },
  setText: (id, value) => { if (!fields.has(id)) fields.set(id, element()); fields.get(id).textContent = value; },
  document: {createElement: element},
  sourceLabels: {low_state: 'LowState', bms: 'BMS', secondary_bms: 'Secondary BMS'},
  statusLabels: {online: 'Online', delayed: 'Delayed', offline: 'Offline'},
  capabilityParameters: () => ({required_sources: 'low_state', motor_fault_encoding: 'hg_motorstate'}),
  csvSet: value => new Set(value.split(',')), telemetrySourceDeclared: () => true,
  telemetrySourceFresh: (sources, key) => sources?.[key]?.status === 'online',
  integer: value => value == null ? '--' : String(value), number: value => String(value),
  manifestText: (zh, en) => en, renderDiagnosticBatteries: () => {},
});
vm.runInContext(appSource.slice(appSource.indexOf('function hex32('),
  appSource.indexOf('function decodeMotorState(')) + appSource.slice(
    appSource.indexOf('function maxMotorTemperature('), appSource.indexOf('function renderOdometry(')), diagContext);
const diagData = {dds_initialized: true, sources: {low_state: {status:'online'},
  bms: {status:'online'}, secondary_bms: {status:'offline'}}, joints: [], reserved_motor_slots: []};
diagContext.data = diagData;
vm.runInContext('renderDiagnostics(data)', diagContext);
assert.equal(fields.get('diagnosticOverallBadge').className, 'state-chip delayed');
diagData.sources.low_state.status = 'delayed';
diagData.joints = [{state_raw: 1, temperature_raw:[90], voltage_v: 50}];
vm.runInContext('renderDiagnostics(data)', diagContext);
assert.equal(fields.get('diagnosticMotorFaultCount').textContent, '--');
assert.equal(fields.get('diagnosticMotorShellTemp').textContent, '--');
assert.match(fields.get('diagnosticMotorFaultList').children[0].textContent, /unknown/);
diagData.sources.low_state.status = 'offline';
vm.runInContext('renderDiagnostics(data)', diagContext);
assert.equal(fields.get('diagnosticOverallBadge').className, 'state-chip offline');
diagData.sources.low_state.status = 'online';
diagData.joints = [];
diagData.sources.secondary_bms.status = 'online';
diagData.batteries = [{id: '1', state: {status: 'online'}, cell_voltage_raw: [0, 0]}];
vm.runInContext('renderDiagnostics(data)', diagContext);
assert.equal(fields.get('diagnosticOverallBadge').className, 'state-chip delayed');
assert.match(fields.get('diagnosticIssueList').children[0].textContent, /without valid cell voltages/);
diagData.batteries[0].cell_voltage_raw = [3500];
vm.runInContext('renderDiagnostics(data)', diagContext);
assert.equal(fields.get('diagnosticOverallBadge').className, 'state-chip online');
const packFields = new Map();
diagContext.card = {querySelector: key => {
  if (!packFields.has(key)) packFields.set(key, {}); return packFields.get(key);
}};
vm.runInContext('renderBatteryDiagnostics(card, {}, false, true)', diagContext);
assert.equal(packFields.get('[data-diagnostic-field="diagnosticBatteryHealth"]').textContent, '--');
vm.runInContext('renderBatteryDiagnostics(card, {soc_pct: 75, soh_pct: 98, cell_voltage_raw:[3500,3510]}, true, true)', diagContext);
assert.equal(packFields.get('[data-diagnostic-field="diagnosticBatteryHealth"]').textContent, '75% / 98%');
assert.equal(packFields.get('[data-diagnostic-field="diagnosticBatteryCellDelta"]').textContent, '10');
console.log('Diagnostics freshness, optional-source severity and battery rendering passed.');
