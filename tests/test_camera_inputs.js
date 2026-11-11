const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

const html = fs.readFileSync('web/index.html', 'utf8');
const app = fs.readFileSync('web/app.js', 'utf8');
const perception = fs.readFileSync('web/perception.js', 'utf8');
const panels = ['rgb', 'depth'].map((stream) => {
  const tag = html.match(new RegExp(`<section[^>]*id="${stream}CameraPanel"[^>]*>`))[0];
  return {
    dataset: {capability: `camera_${stream}`},
    hasAttribute: (name) => tag.includes(name),
    setAttribute() {}, removeAttribute() {},
  };
});
const manifestContext = vm.createContext({
  document: {querySelectorAll: () => panels},
  applyLocomotionPolicy() {}, applyArmActionPolicy() {}, applyControlActionPolicy() {},
  manifestCapabilityState: (descriptor) => descriptor?.state || 'unavailable',
  manifestCapabilityReason: () => 'camera_device_not_detected', setCapabilityNotice() {},
});
vm.runInContext(app.slice(app.indexOf('function applyManifestCapabilities('),
  app.indexOf('function renderRobotManifestPlaceholder(')), manifestContext);
for (const state of ['available', 'unavailable', 'unsupported']) {
  manifestContext.manifest = {capabilities: panels.map((panel) => ({
    key: panel.dataset.capability, implemented: true, state,
  }))};
  vm.runInContext('applyManifestCapabilities(manifest)', manifestContext);
  for (const panel of panels) assert.equal(panel.inert, state === 'unsupported');
}
vm.runInContext('applyManifestCapabilities(null)', manifestContext);
for (const panel of panels) assert.equal(panel.inert, true);
manifestContext.manifest = {capabilities: panels.map((panel) => ({
  key: panel.dataset.capability, implemented: true, state: 'unavailable',
}))};
vm.runInContext("applyManifestCapabilities(manifest, 'fetch_failed')", manifestContext);
for (const panel of panels) assert.equal(panel.inert, true);
for (const stream of ['rgb', 'depth']) {
  const input = html.match(new RegExp(`<input[^>]*id="${stream}CameraSource"[^>]*>`))[0];
  assert.doesNotMatch(input, /maxlength=/);
}

const stage = perception.slice(perception.indexOf('function stageCameraCommand('),
  perception.indexOf('function requestKey('));
const context = vm.createContext({
  latestCameraStatus: {fixed_policy: false}, window: {},
  rgbCameraSource: {value: ''}, depthCameraSource: {value: ''},
  cameraFeedback: {}, confirmTitle: {}, confirmWarning: {}, confirmSafetyTitle: {},
  confirmSafetyDetail: {}, dialogFeedback: {}, submitCommand: {},
  confirmDialog: {showModal() {}}, cameraText: (zh, en) => en,
  cameraCommandMeta: () => ['Start', 'Confirm'], COMMAND_META: {stop_camera: ['Stop', 'Restore services']}, pendingRequest: null,
});
vm.runInContext(stage, context);
for (const command of ['start_v4l2', 'start_realsense']) {
  for (const source of ['', '/dev/video0', '/dev/video1234', `/dev/video${'1'.repeat(100)}`, '/dev/video-dep', '/dev/v4l/by-id/usb-camera-video-index0', '/home/unitree/相机 RGB', '/tmp/camera-alias']) {
    context.rgbCameraSource.value = source;
    context.depthCameraSource.value = source;
    context.pendingRequest = null;
    vm.runInContext(`stageCameraCommand('${command}')`, context);
    assert.ok(context.pendingRequest, `${command}: ${source}`);
    assert.equal(context.pendingRequest.command,
      !source ? 'start_realsense' : 'start_v4l2');
    if (context.pendingRequest.command === 'start_v4l2') {
      assert.equal(context.pendingRequest.rgb_source, source);
      assert.equal(context.pendingRequest.depth_source, source);
    }
  }
  for (const source of ['relative-camera', 'udp://127.0.0.1:5000', '/', '/dev/camera\u0000tail', '/dev/camera\nname']) {
    context.rgbCameraSource.value = source;
    context.pendingRequest = null;
    vm.runInContext(`stageCameraCommand('${command}')`, context);
    assert.equal(context.pendingRequest, null);
  }
  context.rgbCameraSource.value = '';
  context.depthCameraSource.value = '/dev/video1234';
  vm.runInContext(`stageCameraCommand('${command}')`, context);
  assert.equal(context.pendingRequest.command, 'start_v4l2');
  assert.equal(context.pendingRequest.depth_source, '/dev/video1234');
  context.depthCameraSource.value = 'relative-camera';
  context.pendingRequest = null;
  vm.runInContext(`stageCameraCommand('${command}')`, context);
  assert.equal(context.pendingRequest, null);
}
context.latestCameraStatus = {fixed_policy: false, realsense_available: false};
context.rgbCameraSource.value = '';
context.depthCameraSource.value = '';
vm.runInContext("stageCameraCommand('start_v4l2')", context);
assert.equal(context.pendingRequest.command, 'start_v4l2');
context.latestCameraStatus = {fixed_policy: true};
context.rgbCameraSource.value = '/dev/video1234';
vm.runInContext("stageCameraCommand('start_realsense')", context);
assert.equal(context.pendingRequest.command, 'start_realsense');
assert.equal(context.pendingRequest.rgb_source, undefined);
// G1 status belongs to the confirmation dialog, outside the camera layout.
const moved = [];
const statusContext = vm.createContext({
  latestCameraStatus: null, pendingRequest: null,
  cameraFeedback: {}, confirmWarning: {after: (element) => moved.push(element)},
  rgbCameraSource: {closest: () => null}, depthCameraSource: {closest: () => null},
  startV4l2Camera: {}, startRealSenseCamera: {}, stopRgbCamera: {}, stopDepthCamera: {},
  cameraText: (zh, en) => en,
});
vm.runInContext(perception.slice(perception.indexOf('function updateCameraServiceStatus('),
  perception.indexOf('async function pollCameraStatus(')), statusContext);
const failedRgb = {
  running: true, first_person_service: {paused_by_web: true},
  rgb: {error: 'camera_read_failed', source: '/dev/video10'},
  depth: {online: true, source: '/dev/video4'},
};
statusContext.status = failedRgb;
vm.runInContext('updateCameraServiceStatus(status)', statusContext);
assert.equal(moved[0], statusContext.cameraFeedback);
assert.equal(statusContext.cameraFeedback.hidden, true);
statusContext.pendingRequest = {feedback: 'camera'};
vm.runInContext('updateCameraServiceStatus(status)', statusContext);
assert.equal(statusContext.cameraFeedback.hidden, false);
assert.match(statusContext.cameraFeedback.textContent, /480|--/);
statusContext.pendingRequest = {feedback: 'perception'};
vm.runInContext('updateCameraServiceStatus(status)', statusContext);
assert.equal(statusContext.cameraFeedback.hidden, true);
for (const command of ['stop_rgb', 'stop_depth']) {
  context.latestCameraStatus = {fixed_policy: false};
  vm.runInContext(`stageCameraCommand('${command}')`, context);
  assert.equal(context.pendingRequest.command, 'stop');
}

assert.doesNotMatch(fs.readFileSync('web/styles.css', 'utf8').match(/#cameraFeedback\s*\{[^}]+\}/)?.[0] || '', /grid-area:\s*camera-feedback/);
console.log('Camera recovery controls, device validation, shared startup, stop routing and dialog-only G1 feedback passed.');
