import * as THREE from "/assets/vendor/three/three.module.js";
import { OrbitControls } from "/assets/vendor/three/examples/jsm/controls/OrbitControls.js";
import { applyRobotJointValues, loadRobotUrdfInstance, resolveModel } from "/robot-viewer.js";

const NAV_TASK_STORAGE_KEY = "g1-sdk2-navigation-tasks-v1";
const MAP_NAME_PATTERN = /^test(?:[1-9]|10)\.pcd$/;
const MAP_NAME_WARNING = "地图名称无效。\n\n为防止多个 PCD 地图文件占用过多磁盘空间，按 Unitree 官方建议，本页面只允许 test1.pcd ~ test10.pcd，并采用覆盖式保存。\n\n请修改地图名称后再继续。";

function isSupportedMapName(value) {
  return MAP_NAME_PATTERN.test(String(value || "").trim());
}

function normalizeStoredMapName(value) {
  const name = String(value || "").trim();
  if (isSupportedMapName(name)) return name;
  return /^test(?:[1-9]|10)$/.test(name) ? `${name}.pcd` : "";
}

function cleanStoredNavigationPose(pose) {
  const x = Number(pose?.x);
  const y = Number(pose?.y);
  const yaw = Number(pose?.yaw);
  if (![x, y, yaw].every(Number.isFinite)) return null;
  return { x, y, z: 0, yaw };
}

function cleanStoredNavigationTask(task) {
  const name = String(task?.name || "").trim().slice(0, 40);
  const savedMapName = normalizeStoredMapName(String(task?.map_name || "").trim().slice(0, 64));
  const mode = task?.mode === "multi" ? "multi" : "single";
  const poses = Array.isArray(task?.poses)
    ? task.poses.map(cleanStoredNavigationPose).filter(Boolean).slice(0, 100)
    : [];
  if (!name || !savedMapName || !poses.length || (mode === "single" && poses.length !== 1)) return null;
  return {
    id: String(task?.id || "").trim() || `nav-${Date.now()}-${Math.random().toString(16).slice(2)}`,
    name,
    map_name: savedMapName,
    mode,
    poses,
    saved_at: Number(task?.saved_at) || Date.now(),
  };
}

function loadNavigationTasks() {
  try {
    const parsed = JSON.parse(localStorage.getItem(NAV_TASK_STORAGE_KEY) || "[]");
    if (!Array.isArray(parsed)) return [];
    return parsed.map(cleanStoredNavigationTask).filter(Boolean).sort((a, b) => b.saved_at - a.saved_at);
  } catch (_) {
    return [];
  }
}

function persistNavigationTask(task) {
  const normalized = cleanStoredNavigationTask({ ...task, saved_at: Date.now() });
  if (!normalized) throw new Error("任务名称、地图或导航点无效");
  const tasks = loadNavigationTasks().filter((item) => item.id !== normalized.id);
  if (tasks.some((item) => item.name === normalized.name)) {
    throw new Error("同名任务已存在，请改名或先选中该任务再覆盖");
  }
  tasks.unshift(normalized);
  localStorage.setItem(NAV_TASK_STORAGE_KEY, JSON.stringify(tasks));
  return normalized;
}

function renameStoredNavigationTask(id, name) {
  const taskId = String(id || "");
  const nextName = String(name || "").trim().slice(0, 40);
  if (!taskId || !nextName) throw new Error("请先选择任务并输入新名称");
  const tasks = loadNavigationTasks();
  const task = tasks.find((item) => item.id === taskId);
  if (!task) throw new Error("本地任务不存在");
  if (tasks.some((item) => item.id !== taskId && item.name === nextName)) throw new Error("同名任务已存在");
  task.name = nextName;
  task.saved_at = Date.now();
  localStorage.setItem(NAV_TASK_STORAGE_KEY, JSON.stringify(tasks));
  return task;
}

function removeStoredNavigationTask(id) {
  const taskId = String(id || "");
  const tasks = loadNavigationTasks();
  const next = tasks.filter((item) => item.id !== taskId);
  if (next.length === tasks.length) return false;
  localStorage.setItem(NAV_TASK_STORAGE_KEY, JSON.stringify(next));
  return true;
}

const workspace = document.getElementById("sensor-visualization");
const perceptionGrid = workspace?.querySelector(".perception-grid");
const mapView = document.getElementById("map-view");
const mapExpandButton = document.getElementById("toggleMapWorkspace");
const mapLoading = document.getElementById("mapLoading");
const mapHelp = document.getElementById("mapHelp");
const slamBadge = document.getElementById("slamStatusBadge");
const slamMode = document.getElementById("slamMode");
const slamPose = document.getElementById("slamPose");
const slamPointCount = document.getElementById("slamPointCount");
const slamTaskState = document.getElementById("slamTaskState");
const initialPoseSummary = document.getElementById("initialPoseSummary");
const goalPoseSummary = document.getElementById("goalPoseSummary");
const wifiStreamBadge = document.getElementById("wifiStreamBadge");
const rawStatus = document.getElementById("perceptionRaw");
const feedback = document.getElementById("perceptionFeedback");
const progressBar = document.getElementById("navigationProgressBar");
const mapName = document.getElementById("slamMapName");
const initialX = document.getElementById("initialX");
const initialY = document.getElementById("initialY");
const initialYaw = document.getElementById("initialYaw");
const goalX = document.getElementById("goalX");
const goalY = document.getElementById("goalY");
const goalYaw = document.getElementById("goalYaw");
const navigateButton = document.getElementById("sendNavigationGoal");
const startMappingButton = document.getElementById("startMapping");
const finishMappingButton = document.getElementById("finishMapping");
const loadMapButton = document.getElementById("loadMap");
const downloadMapButton = document.getElementById("downloadMap");
const exitMapButton = document.getElementById("exitMap");
const initialPoseButton = document.getElementById("initialPoseMode");
const pauseNavigationButton = document.getElementById("pauseNavigation");
const resumeNavigationButton = document.getElementById("resumeNavigation");
const cancelNavigationButton = document.getElementById("cancelNavigation");
const stopSlamButton = document.getElementById("stopSlam");
const goalPickButton = document.getElementById("goalPickMode");
const singleNavigationModeButton = document.getElementById("singleNavigationMode");
const multiNavigationModeButton = document.getElementById("multiNavigationMode");
const clearWaypointsButton = document.getElementById("clearWaypoints");
const navigationTaskName = document.getElementById("navigationTaskName");
const navigationTaskList = document.getElementById("navigationTaskList");
const saveNavigationTaskButton = document.getElementById("saveNavigationTask");
const renameNavigationTaskButton = document.getElementById("renameNavigationTask");
const runNavigationTaskButton = document.getElementById("runNavigationTask");
const deleteNavigationTaskButton = document.getElementById("deleteNavigationTask");
const navigationObstacleAlert = document.getElementById("navigationObstacleAlert");

function validateMapName(showPopup = false) {
  const name = mapName.value.trim();
  const valid = isSupportedMapName(name);
  mapName.setCustomValidity(
    valid ? "" : (window.UiI18n?.t("仅允许 test1.pcd ~ test10.pcd") || "仅允许 test1.pcd ~ test10.pcd"),
  );
  if (!valid && showPopup) {
    window.alert(MAP_NAME_WARNING);
    mapName.focus();
    mapName.reportValidity();
  }
  return valid;
}

window.addEventListener("ui-language-change", () => validateMapName(false));

const confirmDialog = document.getElementById("perceptionConfirmDialog");
const confirmForm = document.getElementById("perceptionConfirmForm");
const confirmTitle = document.getElementById("perceptionConfirmTitle");
const confirmWarning = document.getElementById("perceptionConfirmWarning");
const confirmSafetyTitle = document.getElementById("perceptionConfirmSafetyTitle");
const confirmSafetyDetail = document.getElementById("perceptionConfirmSafetyDetail");
const dialogFeedback = document.getElementById("perceptionDialogFeedback");
const submitCommand = document.getElementById("submitPerceptionCommand");
const cameraServiceBadge = document.getElementById("cameraServiceBadge");
const depthCameraBadge = document.getElementById("depthCameraBadge");
const cameraFeedback = document.getElementById("cameraFeedback");
const rgbCameraSource = document.getElementById("rgbCameraSource");
const depthCameraSource = document.getElementById("depthCameraSource");
const startRealSenseCamera = document.getElementById("startRealSenseCamera");

let renderer;
let scene;
let camera;
let controls;
let liveLayer;
let globalLayer;
let obstacleBoundaryLayer;
let trajectoryLayer;
let robotMarker;
let initialMarker;
let targetMarker;
let mapRobot;
let mapRobotMount;
let mapRobotFile = "";
let mapRobotLoadGeneration = 0;
let latestRobotTelemetry = window.g1LatestTelemetry || null;
let sceneReady = false;
let lastFrameSequence = -1;
let lastGlobalSequence = -1;
let latestStatus = null;
let latestCameraStatus = null;
let latestFrame = null;
let initialPoseSelected = false;
let goalSelected = false;
let pickingGoal = false;
let pendingRequest = null;
let pointerStart = null;
let rgbObjectUrl = null;
let depthObjectUrl = null;
let lastSlamMode = "offline";
let poseTool = "browse";
let viewTool = "pan";
let mapHeading = 0;
let mapHeadingHoldFrame = 0;
let mapHeadingHoldDirection = 0;
let mapHeadingHoldLastTime = 0;
let navigationMode = "single";
let waypoints = [];
let waypointMarkers = [];
let multiRouteActive = false;
let activeWaypointIndex = -1;
let routeRequestInFlight = false;
let loadedMapName = "";
let mapNameDirty = false;
let selectedNavigationTaskId = "";
const nativeVisibility = { global: true, live: true };

const MODE_LABELS = {
  offline: "离线",
  ready: "待命",
  mapping: "建图中",
  localizing: "定位中",
  navigating: "导航中",
  paused: "已暂停",
  arrived: "已到达",
  stopped: "已关闭",
};

const COMMAND_META = {
  start_mapping: ["开始建图", "将先自动开启 lidar_driver 和 unitree_slam，再调用官方 API 1801；失败时会回滚本次开启的服务。"],
  finish_mapping: ["结束并保存地图", "将调用官方 API 1802，并覆盖同名 PCD 文件。"],
  load_map: ["加载地图", "Unitree 官方只提供 API 1804 加载地图并初始化位姿；本次先用当前可用位姿作为初始猜测，没有有效位姿时使用 X:0 / Y:0 / W:0。加载成功后才开放手动重定位。"],
  initialize_pose: ["执行重定位", "当前地图已加载；将使用刚才在地图中选择的 X / Y / W 再次调用官方 API 1804 完成重定位。"],
  navigate: ["执行位姿导航", "机器人将自主移动到所选目标。官方接口要求目标距离不超过 10 m。"],
  navigate_route: ["执行多点导航", "将按顺序逐点调用官方 API 1102；每一段距离均不超过 10 m，到达前一点后才会发送下一点。"],
  pause_navigation: ["暂停导航", "将调用官方 API 1201，暂停当前导航任务。"],
  resume_navigation: ["恢复导航", "机器人可能立即继续运动，请重新检查周围环境。"],
  cancel_navigation: ["取消导航", "官方没有独立取消接口：若正在行走，将先调用 API 1201 立即暂停，再清除当前单点目标或多点执行队列；已加载地图和定位保持可用。"],
  exit_map: ["退出地图", "将调用官方 API 1901 关闭 SLAM 定位并清空当前已加载地图状态；退出后导航会被锁定，必须重新加载地图并定位后才能再次导航。"],
  stop_slam: ["关闭 SLAM", "将调用官方 API 1901 并关闭 unitree_slam；lidar_driver 保持待命，页面会回退显示 Mid-360 实时点云。此操作不会调用 StopMove。"],
  start_v4l2: ["自动检测并启动摄像头", "将临时释放占用 D435i 的摄像头服务，再按 V4L2 像素格式自动识别当前 RGB 与 Z16 深度设备；不依赖固定 /dev/videoN。手填设备仅作为覆盖。"],
  start_realsense: ["启动 D435i RGB + 深度", "优先通过 librealsense2 与机器人现有相机服务并发共享，不中断 master_service；仅设备无法并发访问时才使用兼容的暂停/自动识别兜底链路。"],
  stop_camera: ["停止 Web 摄像头", "释放 Web 自己的取流资源；共享 teleimager 模式不会停止机器人现有第一人称服务。"],
};

function workspaceVisible() {
  return workspace && !workspace.hidden && document.visibilityState === "visible";
}

function resizeMapViewport() {
  if (!renderer || !camera || !mapView) return;
  const width = Math.max(1, mapView.clientWidth);
  const height = Math.max(1, mapView.clientHeight);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderMap();
}

function setMapWorkspaceExpanded(expanded) {
  const active = Boolean(expanded && perceptionGrid && mapExpandButton);
  perceptionGrid?.classList.toggle("map-expanded", active);
  workspace?.classList.toggle("map-workspace-expanded", active);
  mapExpandButton?.setAttribute("aria-pressed", String(active));
  if (mapExpandButton) {
    mapExpandButton.textContent = active ? "↙ 恢复完整工作台" : "⛶ 放大地图区";
    mapExpandButton.setAttribute("aria-label", active ? "恢复完整工作台" : "放大地图工作区");
    mapExpandButton.title = active ? "恢复摄像头、三维姿态与地图的完整工作台布局" : "放大地图工作区，临时隐藏摄像头和三维姿态区域";
  }
  requestAnimationFrame(() => requestAnimationFrame(resizeMapViewport));
}

function setFeedback(text, state = "") {
  feedback.textContent = text;
  feedback.className = `perception-feedback ${state}`.trim();
}

async function readCommandResponse(response) {
  const raw = await response.text();
  let result = {};
  if (raw) {
    try {
      result = JSON.parse(raw);
    } catch {
      if (!response.ok) throw new Error(`HTTP ${response.status}；原始响应：${raw}`);
      throw new Error(`响应不是有效 JSON；原始响应：${raw}`);
    }
  }
  if (!response.ok || !result.accepted) {
    const details = [result.error || `HTTP ${response.status}`];
    if (result.api_result !== undefined) details.push(`SDK api_result=${result.api_result}`);
    if (result.api_response) details.push(`官方返回：${result.api_response}`);
    throw new Error(details.join("；"));
  }
  return result;
}

function setBadge(text, state = "pending") {
  slamBadge.textContent = text;
  slamBadge.className = `perception-badge ${state}`;
}

function rosPoint(point) {
  return [Number(point?.[0]) || 0, Number(point?.[2]) || 0, -(Number(point?.[1]) || 0)];
}

function yawFromQuaternion(pose) {
  const z = Number(pose?.q_z) || 0;
  const w = Number(pose?.q_w) || 1;
  const x = Number(pose?.q_x) || 0;
  const y = Number(pose?.q_y) || 0;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function initializeMap() {
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" });
  } catch (error) {
    setBadge("WebGL 不可用", "error");
    mapLoading.querySelector("strong").textContent = "点云渲染不可用";
    mapLoading.querySelector("span").textContent = error?.message || "请启用浏览器硬件加速";
    return false;
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x050a11, 1);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  mapView.prepend(renderer.domElement);

  scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x050a11, 0.018);
  camera = new THREE.PerspectiveCamera(44, 1, 0.02, 180);
  camera.position.set(0, 18, 0.01);
  camera.up.set(0, 0, -1);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0);
  controls.enableDamping = false;
  controls.enablePan = true;
  controls.screenSpacePanning = true;
  controls.mouseButtons.LEFT = THREE.MOUSE.PAN;
  controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
  controls.mouseButtons.RIGHT = null;
  controls.minDistance = 0.8;
  controls.maxDistance = 80;
  controls.addEventListener("change", renderMap);

  scene.add(new THREE.HemisphereLight(0xcdeeff, 0x17202c, 1.8));
  const grid = new THREE.GridHelper(90, 90, 0x28506a, 0x17283a);
  grid.material.transparent = true;
  grid.material.opacity = 0.55;
  scene.add(grid);
  const axes = new THREE.AxesHelper(1.2);
  axes.rotation.x = Math.PI / 2;
  scene.add(axes);

  robotMarker = createPoseMarker(0x54ddff, 0.3);
  initialMarker = createPoseMarker(0x68f5b5, 0.34);
  targetMarker = createPoseMarker(0xffc861, 0.36);
  initialMarker.visible = false;
  targetMarker.visible = false;
  scene.add(robotMarker, initialMarker, targetMarker);

  new ResizeObserver(resizeMapViewport).observe(mapView);
  renderer.domElement.addEventListener("pointerdown", beginPoseDrag);
  renderer.domElement.addEventListener("pointermove", updatePoseDrag);
  renderer.domElement.addEventListener("pointerup", selectGoalFromPointer);
  renderer.domElement.addEventListener("pointercancel", cancelPoseDrag);
  renderer.domElement.addEventListener("contextmenu", (event) => event.preventDefault());
  renderer.domElement.addEventListener("webglcontextlost", (event) => {
    event.preventDefault();
    sceneReady = false;
    setBadge("WebGL 中断", "error");
    mapLoading.hidden = false;
  });
  sceneReady = true;
  setPoseTool("browse");
  resizeMapViewport();
  return true;
}

function createPoseMarker(color, size, label = "") {
  const group = new THREE.Group();
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute([
    size, 0.055, 0,
    -size * 0.62, 0.055, -size * 0.46,
    -size * 0.62, 0.055, size * 0.46,
  ], 3));
  geometry.setIndex([0, 1, 2]);
  geometry.computeVertexNormals();
  group.add(new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide })));
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(size * 0.72, size * 0.82, 40),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.55, side: THREE.DoubleSide }),
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.025;
  group.add(ring);
  if (label) {
    const canvas = document.createElement("canvas");
    canvas.width = 64;
    canvas.height = 64;
    const context = canvas.getContext("2d");
    context.clearRect(0, 0, 64, 64);
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.font = "900 34px system-ui, sans-serif";
    context.lineWidth = 5;
    context.strokeStyle = "rgba(255,255,255,.92)";
    context.fillStyle = "#071019";
    context.strokeText(label, 32, 34);
    context.fillText(label, 32, 34);
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
    }));
    sprite.position.set(-size * 0.08, 0.09, 0);
    sprite.scale.set(size * 0.72, size * 0.72, 1);
    sprite.renderOrder = 12;
    group.add(sprite);
  }
  return group;
}

function disposeMapRobot() {
  if (mapRobotMount) scene?.remove(mapRobotMount);
  mapRobot?.traverse?.((object) => {
    object.geometry?.dispose?.();
    if (Array.isArray(object.material)) object.material.forEach((material) => material.dispose?.());
    else object.material?.dispose?.();
  });
  mapRobot = null;
  mapRobotMount = null;
}

function updateMapRobotPose(pose) {
  if (!mapRobotMount || !pose) return;
  mapRobotMount.position.set(Number(pose.x) || 0, 0, -(Number(pose.y) || 0));
  mapRobotMount.rotation.y = yawFromQuaternion(pose);
  mapRobotMount.visible = true;
}

function syncMapRobotTelemetry(telemetry) {
  latestRobotTelemetry = telemetry || latestRobotTelemetry;
  if (!sceneReady || !latestRobotTelemetry?.robot || latestRobotTelemetry?.joints?.length !== 29) return;

  let selection;
  try {
    selection = resolveModel(latestRobotTelemetry.robot);
  } catch (error) {
    ++mapRobotLoadGeneration;
    mapRobotFile = "";
    disposeMapRobot();
    console.warn("地图 URDF 机型校验失败", error);
    renderMap();
    return;
  }
  if (!selection) {
    ++mapRobotLoadGeneration;
    mapRobotFile = "";
    disposeMapRobot();
    renderMap();
    return;
  }

  if (mapRobot && mapRobotFile === selection.file) {
    applyRobotJointValues(mapRobot, latestRobotTelemetry.joints);
    updateMapRobotPose(latestFrame?.pose || latestStatus?.pose);
    renderMap();
    return;
  }
  if (mapRobotFile === selection.file) return;

  const generation = ++mapRobotLoadGeneration;
  mapRobotFile = selection.file;
  disposeMapRobot();
  loadRobotUrdfInstance(
    latestRobotTelemetry.robot,
    latestRobotTelemetry.joints,
    (candidate, loadedSelection) => {
      if (generation !== mapRobotLoadGeneration || !sceneReady) {
        candidate.traverse?.((object) => {
          object.geometry?.dispose?.();
          if (Array.isArray(object.material)) object.material.forEach((material) => material.dispose?.());
          else object.material?.dispose?.();
        });
        return;
      }
      mapRobot = candidate;
      applyRobotJointValues(mapRobot, latestRobotTelemetry?.joints);
      const bounds = new THREE.Box3().setFromObject(mapRobot);
      if (!bounds.isEmpty()) mapRobot.position.y -= bounds.min.y;

      mapRobotMount = new THREE.Group();
      mapRobotMount.name = "slam_robot_pose";
      mapRobotMount.visible = false;
      mapRobotMount.add(mapRobot);
      scene.add(mapRobotMount);
      mapRobotFile = loadedSelection.file;
      updateMapRobotPose(latestFrame?.pose || latestStatus?.pose);
      renderMap();
    },
    (error) => {
      if (generation !== mapRobotLoadGeneration) return;
      console.warn("地图 URDF 加载失败", error);
    },
  );
}

function renderMap() {
  if (renderer && scene && camera) renderer.render(scene, camera);
}

function disposeLayer(layer) {
  if (!layer) return;
  scene.remove(layer);
  layer.geometry?.dispose();
  layer.material?.dispose();
}

function decodePointPayload(root, name) {
  const encoding = root?.[`${name}_encoding`];
  const encoded = root?.[`${name}_data`];
  const requestedCount = Number(root?.[`${name}_count`]) || 0;
  if (!encoded || requestedCount <= 0) {
    return Array.isArray(root?.[name]) ? root[name] : [];
  }

  const binary = atob(encoded);
  const bytes = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index);
  }
  if (encoding === "base64_u16le_xyz") {
    const packed = new Uint16Array(bytes.buffer);
    const origin = root?.[`${name}_origin`];
    const scale = root?.[`${name}_scale`];
    if (Array.isArray(origin) && origin.length === 3 &&
        Array.isArray(scale) && scale.length === 3) {
      return {
        packed,
        count: Math.min(requestedCount, Math.floor(packed.length / 3)),
        origin: origin.map((value) => Number(value) || 0),
        scale: scale.map((value) => Number(value) || 0),
        quantized: true,
      };
    }
  }
  if (encoding === "base64_f32le_xyz") {
    const packed = new Float32Array(bytes.buffer);
    return {
      packed,
      count: Math.min(requestedCount, Math.floor(packed.length / 3)),
    };
  }
  return [];
}

function pointCount(points) {
  return points?.packed ? points.count : Array.isArray(points) ? points.length : 0;
}

function pointXyz(points, index) {
  if (points?.packed) {
    const offset = index * 3;
    if (points.quantized) {
      return [
        points.origin[0] + points.packed[offset] * points.scale[0],
        points.origin[1] + points.packed[offset + 1] * points.scale[1],
        points.origin[2] + points.packed[offset + 2] * points.scale[2],
      ];
    }
    return [points.packed[offset], points.packed[offset + 1], points.packed[offset + 2]];
  }
  return points[index] || [0, 0, 0];
}

function liveHeightColor(color, height, minHeight, maxHeight) {
  const span = Math.max(0.12, maxHeight - minHeight);
  const normalized = Math.max(0, Math.min(1, (height - minHeight) / span));
  // RViz-style Z-axis colouring: low points are blue, then cyan/green/yellow,
  // and the highest returns are red. HSL keeps the gradient readable on the
  // dark map without requiring a shader or an external colour-map asset.
  color.setHSL((1 - normalized) * 0.66, 0.94, 0.56);
}

function updateObstacleBoundary(points) {
  disposeLayer(obstacleBoundaryLayer);
  obstacleBoundaryLayer = null;
  const count = pointCount(points);
  if (!count) return;

  const resolution = 0.10;
  const cells = new Map();
  for (let index = 0; index < count; index += 1) {
    const point = pointXyz(points, index);
    const height = Number(point?.[2]);
    const x = Number(point?.[0]);
    const z = -Number(point?.[1]);
    if (!Number.isFinite(x) || !Number.isFinite(z) || !Number.isFinite(height)) continue;
    if (height < 0.05 || height > 2.6) continue;
    const cellX = Math.floor(x / resolution);
    const cellZ = Math.floor(z / resolution);
    const key = `${cellX},${cellZ}`;
    const cell = cells.get(key) || { x: cellX, z: cellZ, count: 0, maxHeight: height };
    cell.count += 1;
    cell.maxHeight = Math.max(cell.maxHeight, height);
    cells.set(key, cell);
  }
  if (!cells.size) return;

  const occupied = [];
  for (const cell of cells.values()) {
    let neighbours = 0;
    for (let dx = -1; dx <= 1; dx += 1) {
      for (let dz = -1; dz <= 1; dz += 1) {
        if ((dx || dz) && cells.has(`${cell.x + dx},${cell.z + dz}`)) neighbours += 1;
      }
    }
    // Drop only isolated single returns. Adjacent tiny obstacles remain visible
    // instead of disappearing behind the old contour-length threshold.
    if (cell.count >= 2 || neighbours >= 1) occupied.push(cell);
  }
  if (!occupied.length) return;

  const geometry = new THREE.BoxGeometry(1, 1, 1);
  const material = new THREE.MeshStandardMaterial({
    color: 0x54ddff,
    transparent: true,
    opacity: 0.30,
    roughness: 0.82,
    metalness: 0,
    depthWrite: false,
  });
  obstacleBoundaryLayer = new THREE.InstancedMesh(geometry, material, occupied.length);
  const matrix = new THREE.Matrix4();
  const position = new THREE.Vector3();
  const scale = new THREE.Vector3();
  const rotation = new THREE.Quaternion();
  const width = resolution * 0.84;
  occupied.forEach((cell, index) => {
    const visualHeight = Math.max(0.08, Math.min(2.6, cell.maxHeight));
    position.set(
      (cell.x + 0.5) * resolution,
      visualHeight * 0.5,
      (cell.z + 0.5) * resolution,
    );
    scale.set(width, visualHeight, width);
    matrix.compose(position, rotation, scale);
    obstacleBoundaryLayer.setMatrixAt(index, matrix);
  });
  obstacleBoundaryLayer.instanceMatrix.needsUpdate = true;
  obstacleBoundaryLayer.visible = nativeVisibility.global !== false;
  obstacleBoundaryLayer.renderOrder = 4;
  scene.add(obstacleBoundaryLayer);
}

function updatePointLayer(current, points, kind) {
  disposeLayer(current);
  const count = pointCount(points);
  if (!count) return null;
  const positions = new Float32Array(count * 3);
  const colors = new Float32Array(count * 3);
  const color = new THREE.Color();
  let liveMinHeight = -0.25;
  let liveMaxHeight = 2.2;
  if (kind === "live") {
    const heights = [];
    for (let index = 0; index < count; index += 1) {
      const height = Number(pointXyz(points, index)?.[2]);
      if (Number.isFinite(height) && height >= -0.6 && height <= 3.2) heights.push(height);
    }
    if (heights.length) {
      heights.sort((a, b) => a - b);
      liveMinHeight = heights[Math.floor((heights.length - 1) * 0.05)];
      liveMaxHeight = heights[Math.ceil((heights.length - 1) * 0.95)];
      if (liveMaxHeight - liveMinHeight < 0.12) {
        const centre = (liveMaxHeight + liveMinHeight) / 2;
        liveMinHeight = centre - 0.06;
        liveMaxHeight = centre + 0.06;
      }
    }
  }
  for (let index = 0; index < count; index += 1) {
    const point = pointXyz(points, index);
    const [x, y, z] = rosPoint(point);
    positions[index * 3] = x;
    positions[index * 3 + 1] = y;
    positions[index * 3 + 2] = z;
    const rawHeight = Number(point?.[2]);
    const height = Math.max(-0.25, Math.min(2.8, Number.isFinite(rawHeight) ? rawHeight : 0));
    if (kind === "global") {
      // Persistent map: quiet steel floor, blue/cyan vertical structure.
      if (height < 0.05) color.setRGB(0.22, 0.31, 0.40);
      else color.setHSL(0.58 - Math.min(0.09, height * 0.025), 0.78, 0.47 + Math.min(0.25, height * 0.08));
    } else {
      liveHeightColor(color, Number.isFinite(rawHeight) ? rawHeight : 0, liveMinHeight, liveMaxHeight);
    }
    colors[index * 3] = color.r;
    colors[index * 3 + 1] = color.g;
    colors[index * 3 + 2] = color.b;
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geometry.computeBoundingSphere();
  const material = new THREE.PointsMaterial({
    size: kind === "global" ? 0.050 : 0.075,
    vertexColors: true,
    transparent: true,
    opacity: kind === "global" ? 0.86 : 0.98,
    sizeAttenuation: true,
  });
  const layer = new THREE.Points(geometry, material);
  layer.visible = nativeVisibility[kind] !== false;
  layer.renderOrder = kind === "global" ? 1 : 6;
  scene.add(layer);
  return layer;
}

function updateTrajectory(points) {
  disposeLayer(trajectoryLayer);
  trajectoryLayer = null;
  const count = pointCount(points);
  if (count < 2) return;
  const positions = [];
  for (let index = 0; index < count; index += 1) {
    positions.push(...rosPoint(pointXyz(points, index)));
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  trajectoryLayer = new THREE.Line(
    geometry,
    new THREE.LineBasicMaterial({ color: 0x6df1c0, transparent: true, opacity: 0.9 }),
  );
  trajectoryLayer.position.y += 0.045;
  scene.add(trajectoryLayer);
}

function updatePoseMarker(marker, pose) {
  if (!marker || !pose) return;
  marker.position.set(Number(pose.x) || 0, Number(pose.z) || 0, -(Number(pose.y) || 0));
  marker.rotation.y = yawFromQuaternion(pose);
}

function setTopView(distance = 18) {
  document.getElementById("mapTopView").setAttribute("aria-pressed", "true");
  document.getElementById("mapThreeView").setAttribute("aria-pressed", "false");
  const target = controls.target.clone();
  const axis = new THREE.Vector3(0, 1, 0);
  camera.position.copy(target).add(new THREE.Vector3(0, distance, 0.01).applyAxisAngle(axis, mapHeading));
  camera.up.set(0, 0, -1).applyAxisAngle(axis, mapHeading);
  camera.lookAt(target);
  controls.update();
}

function setThreeView() {
  document.getElementById("mapTopView").setAttribute("aria-pressed", "false");
  document.getElementById("mapThreeView").setAttribute("aria-pressed", "true");
  const target = controls.target.clone();
  const axis = new THREE.Vector3(0, 1, 0);
  camera.position.copy(target).add(new THREE.Vector3(7.5, 6.5, 7.5).applyAxisAngle(axis, mapHeading));
  camera.up.set(0, 1, 0);
  camera.lookAt(target);
  controls.update();
}

function fitMap() {
  if (!sceneReady) return;
  const box = new THREE.Box3();
  [globalLayer, liveLayer].forEach((layer) => {
    if (layer) box.expandByObject(layer);
  });
  if (box.isEmpty()) {
    controls.target.set(0, 0, 0);
    setTopView();
    return;
  }
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const radius = Math.max(3, size.x, size.z) * 0.72;
  controls.target.copy(center);
  setTopView(Math.min(65, radius * 1.65));
}

function locateRobot() {
  if (!latestStatus?.pose_received) return;
  const pose = latestStatus.pose;
  controls.target.set(Number(pose.x) || 0, Number(pose.z) || 0, -(Number(pose.y) || 0));
  setTopView();
}

function adjustMapHeading(delta) {
  if (!sceneReady || !camera || !controls) return;
  mapHeading = (mapHeading + delta) % (Math.PI * 2);
  const axis = new THREE.Vector3(0, 1, 0);
  camera.position.sub(controls.target).applyAxisAngle(axis, delta).add(controls.target);
  camera.up.applyAxisAngle(axis, delta).normalize();
  camera.lookAt(controls.target);
  controls.update();
}

function stopMapHeadingHold() {
  if (mapHeadingHoldFrame) cancelAnimationFrame(mapHeadingHoldFrame);
  mapHeadingHoldFrame = 0;
  mapHeadingHoldDirection = 0;
  mapHeadingHoldLastTime = 0;
}

function continueMapHeadingHold(time) {
  if (!mapHeadingHoldDirection) return;
  if (mapHeadingHoldLastTime) {
    const seconds = Math.min(0.05, (time - mapHeadingHoldLastTime) / 1000);
    adjustMapHeading(mapHeadingHoldDirection * seconds * 1.35);
  }
  mapHeadingHoldLastTime = time;
  mapHeadingHoldFrame = requestAnimationFrame(continueMapHeadingHold);
}

function startMapHeadingHold(direction, event) {
  if (event.button !== 0 || !sceneReady) return;
  event.preventDefault();
  if (poseTool !== "browse") setPoseTool("browse");
  stopMapHeadingHold();
  mapHeadingHoldDirection = direction;
  adjustMapHeading(direction * THREE.MathUtils.degToRad(2.5));
  event.currentTarget.setPointerCapture?.(event.pointerId);
  mapHeadingHoldFrame = requestAnimationFrame(continueMapHeadingHold);
}

function groundPoint(event) {
  if (!sceneReady) return null;
  const rect = renderer.domElement.getBoundingClientRect();
  const pointer = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1,
  );
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(pointer, camera);
  return raycaster.ray.intersectPlane(
    new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), new THREE.Vector3(),
  );
}

function activePoseMarker() {
  return poseTool === "initial" ? initialMarker : targetMarker;
}

function poseSummaryText(x, y, yaw) {
  return `X: ${Number(x).toFixed(2)} · Y: ${Number(y).toFixed(2)} · W: ${Number(yaw).toFixed(0)}°`;
}

function disposePoseMarker(marker) {
  if (!marker || !scene) return;
  scene.remove(marker);
  marker.traverse?.((child) => {
    child.geometry?.dispose?.();
    child.material?.map?.dispose?.();
    child.material?.dispose?.();
  });
}

function clearWaypointMarkers() {
  waypointMarkers.forEach(disposePoseMarker);
  waypointMarkers = [];
}

function refreshWaypointMarkers() {
  clearWaypointMarkers();
  if (!sceneReady || navigationMode !== "multi") return;
  waypointMarkers = waypoints.map((pose, index) => {
    const marker = createPoseMarker(0xffc861, 0.28, String(index + 1));
    marker.position.set(pose.x, 0, -pose.y);
    marker.rotation.y = pose.yaw;
    scene.add(marker);
    return marker;
  });
}

function updateGoalSummary() {
  if (navigationMode === "multi") {
    const last = waypoints[waypoints.length - 1];
    goalPoseSummary.textContent = last
      ? `${waypoints.length} 点 · ${poseSummaryText(last.x, last.y, last.yaw * 180 / Math.PI)}`
      : "0 点 · X: -- · Y: -- · W: --";
    goalPoseSummary.classList.toggle("selected", waypoints.length > 0);
    clearWaypointsButton.hidden = false;
    clearWaypointsButton.disabled = multiRouteActive || waypoints.length === 0;
    navigateButton.textContent = multiRouteActive ? "多点导航中" : "开始多点";
  } else {
    clearWaypointsButton.hidden = true;
    navigateButton.textContent = "开始导航";
  }
}

function setNavigationMode(mode) {
  if (!["single", "multi"].includes(mode) || mode === navigationMode) return;
  if (multiRouteActive || ["navigating", "paused"].includes(latestStatus?.mode)) return;
  navigationMode = mode;
  goalSelected = false;
  if (targetMarker) targetMarker.visible = false;
  waypoints = [];
  clearWaypointMarkers();
  goalPoseSummary.textContent = mode === "multi"
    ? "0 点 · X: -- · Y: -- · W: --"
    : "X: -- · Y: -- · W: --";
  goalPoseSummary.classList.remove("selected");
  singleNavigationModeButton.classList.toggle("active", mode === "single");
  multiNavigationModeButton.classList.toggle("active", mode === "multi");
  singleNavigationModeButton.setAttribute("aria-pressed", String(mode === "single"));
  multiNavigationModeButton.setAttribute("aria-pressed", String(mode === "multi"));
  updateGoalSummary();
  updateNavigationAvailability();
  renderMap();
}


function currentNavigationTaskPoses() {
  if (navigationMode === "multi") return waypoints.map((pose) => ({ ...pose }));
  const pose = goalSelected ? currentGoalPose() : null;
  return pose ? [pose] : [];
}

function renderNavigationTaskList() {
  const tasks = loadNavigationTasks();
  navigationTaskList.replaceChildren();
  if (!tasks.length) {
    const option = document.createElement("option");
    option.textContent = "暂无本地导航任务";
    option.disabled = true;
    navigationTaskList.append(option);
  } else {
    tasks.forEach((task) => {
      const option = document.createElement("option");
      option.value = task.id;
      option.textContent = `${task.name} · ${task.mode === "multi" ? "多点" : "单点"} · ${task.poses.length}点 · ${task.map_name}`;
      navigationTaskList.append(option);
    });
  }
  const selectedExists = tasks.some((task) => task.id === selectedNavigationTaskId);
  if (selectedExists) navigationTaskList.value = selectedNavigationTaskId;
  else {
    selectedNavigationTaskId = "";
    navigationTaskList.selectedIndex = -1;
  }
  const disabled = !selectedNavigationTaskId;
  renameNavigationTaskButton.disabled = disabled;
  runNavigationTaskButton.disabled = disabled;
  deleteNavigationTaskButton.disabled = disabled;
}

function selectedStoredNavigationTask() {
  if (!selectedNavigationTaskId) return null;
  return loadNavigationTasks().find((task) => task.id === selectedNavigationTaskId) || null;
}

function saveCurrentNavigationTask() {
  const name = navigationTaskName.value.trim();
  const currentMapName = mapName.value.trim();
  const poses = currentNavigationTaskPoses();
  if (!name) {
    setFeedback("请输入导航任务名称后再保存。", "error");
    navigationTaskName.focus();
    return;
  }
  if (!isSupportedMapName(currentMapName)) {
    setFeedback("当前地图名称无效；仅允许 test1.pcd ~ test10.pcd。", "error");
    validateMapName(true);
    return;
  }
  if (!poses.length) {
    setFeedback(navigationMode === "multi" ? "请先添加至少一个多点导航目标。" : "请先选择一个单点导航目标。", "error");
    return;
  }
  try {
    const selected = selectedStoredNavigationTask();
    const updateExisting = selected?.name === name;
    const saved = persistNavigationTask({
      id: updateExisting ? selected.id : "",
      name,
      map_name: currentMapName,
      mode: navigationMode,
      poses,
    });
    selectedNavigationTaskId = saved.id;
    navigationTaskName.value = saved.name;
    renderNavigationTaskList();
    setFeedback(`${updateExisting ? "已更新" : "已创建"}本地导航任务“${saved.name}”（${saved.mode === "multi" ? `${saved.poses.length} 个点` : "单点"}）。`, "success");
  } catch (error) {
    setFeedback(`本地任务保存失败：${error.message}`, "error");
  }
}

function applyNavigationTask(task, startNavigation = false) {
  if (!task) {
    setFeedback("本地导航任务不存在，请刷新任务列表。", "error");
    return;
  }
  if (multiRouteActive || ["navigating", "paused"].includes(latestStatus?.mode)) {
    setFeedback("当前导航任务尚未结束，不能切换本地任务。", "error");
    return;
  }

  if (task.mode !== navigationMode) setNavigationMode(task.mode);
  waypoints = [];
  clearWaypointMarkers();
  goalSelected = false;
  if (targetMarker) targetMarker.visible = false;

  mapName.value = task.map_name;
  mapNameDirty = task.map_name !== loadedMapName;
  selectedNavigationTaskId = task.id;
  navigationTaskName.value = task.name;

  if (task.mode === "multi") {
    waypoints = task.poses.map((pose) => ({ ...pose }));
    goalSelected = waypoints.length > 0;
    refreshWaypointMarkers();
    const last = waypoints[waypoints.length - 1];
    if (last) {
      goalX.value = last.x.toFixed(2);
      goalY.value = last.y.toFixed(2);
      goalYaw.value = (last.yaw * 180 / Math.PI).toFixed(1);
    }
  } else {
    const pose = task.poses[0];
    goalX.value = pose.x.toFixed(2);
    goalY.value = pose.y.toFixed(2);
    goalYaw.value = (pose.yaw * 180 / Math.PI).toFixed(1);
    goalSelected = true;
    if (targetMarker) {
      targetMarker.visible = true;
      targetMarker.position.set(pose.x, 0, -pose.y);
      targetMarker.rotation.y = pose.yaw;
    }
    goalPoseSummary.textContent = poseSummaryText(pose.x, pose.y, pose.yaw * 180 / Math.PI);
    goalPoseSummary.classList.add("selected");
  }

  setPoseTool("browse");
  updateGoalSummary();
  updateWorkflowState(latestStatus || {});
  updateNavigationAvailability();
  renderNavigationTaskList();
  renderMap();

  if (!startNavigation) {
    setFeedback(`已加载本地任务“${task.name}”，无需重新在地图选点。`, "success");
    return;
  }
  if (navigateButton.disabled) {
    const mapHint = mapIsLoaded() ? "" : `请先加载地图“${task.map_name}”并完成重定位；`;
    setFeedback(`已加载任务“${task.name}”；${mapHint}当前状态尚不能开始导航。`, "error");
    return;
  }
  stageCommand(task.mode === "multi" ? "navigate_route" : "navigate");
}

function renameSelectedNavigationTask() {
  if (!selectedNavigationTaskId) return;
  try {
    const renamed = renameStoredNavigationTask(selectedNavigationTaskId, navigationTaskName.value);
    navigationTaskName.value = renamed.name;
    renderNavigationTaskList();
    setFeedback(`本地导航任务已重命名为“${renamed.name}”。`, "success");
  } catch (error) {
    setFeedback(`重命名失败：${error.message}`, "error");
  }
}

function deleteSelectedNavigationTask() {
  const task = selectedStoredNavigationTask();
  if (!task) return;
  if (!window.confirm(`确定删除本地导航任务“${task.name}”吗？`)) return;
  try {
    removeStoredNavigationTask(task.id);
    selectedNavigationTaskId = "";
    navigationTaskName.value = "";
    renderNavigationTaskList();
    setFeedback(`已删除本地导航任务“${task.name}”。`, "success");
  } catch (error) {
    setFeedback(`删除失败：${error.message}`, "error");
  }
}

function routeDistancesAllowed() {
  if (!waypoints.length || !latestStatus?.pose) return false;
  let previous = latestStatus.pose;
  return waypoints.every((pose) => {
    const allowed = Math.hypot(pose.x - Number(previous.x || 0), pose.y - Number(previous.y || 0)) <= 10;
    previous = pose;
    return allowed;
  });
}

function cancelMultiRoute(clear = false) {
  multiRouteActive = false;
  activeWaypointIndex = -1;
  routeRequestInFlight = false;
  if (clear) {
    waypoints = [];
    clearWaypointMarkers();
    goalSelected = false;
  }
  updateGoalSummary();
}

async function submitNavigationWaypoint(index) {
  const pose = waypoints[index];
  if (!pose || routeRequestInFlight) return;
  routeRequestInFlight = true;
  try {
    const response = await fetch("/api/perception/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        command: "navigate",
        map_name: mapName.value.trim(),
        pose,
        confirmed: true,
        request_key: requestKey(),
      }),
    });
    const result = await readCommandResponse(response);
    activeWaypointIndex = index;
    setFeedback(`多点导航：正在前往第 ${index + 1} / ${waypoints.length} 个目标。`, "running");
  } catch (error) {
    cancelMultiRoute();
    setFeedback(`多点导航提交失败：${error.message}`, "error");
  } finally {
    routeRequestInFlight = false;
    updateNavigationAvailability();
  }
}

function advanceMultiRoute() {
  if (!multiRouteActive || routeRequestInFlight) return;
  const nextIndex = activeWaypointIndex + 1;
  if (nextIndex >= waypoints.length) {
    multiRouteActive = false;
    activeWaypointIndex = -1;
    setFeedback(`多点导航完成，共到达 ${waypoints.length} 个目标。`, "success");
    updateGoalSummary();
    updateNavigationAvailability();
    return;
  }
  submitNavigationWaypoint(nextIndex);
}

function beginPoseDrag(event) {
  if (event.button !== 0) return;
  if (!pickingGoal || !sceneReady) {
    pointerStart = { x: event.clientX, y: event.clientY };
    return;
  }
  const hit = groundPoint(event);
  if (!hit) return;
  const marker = activePoseMarker();
  pointerStart = {
    x: event.clientX,
    y: event.clientY,
    hit,
    current: hit.clone(),
    previous: { visible: marker.visible, position: marker.position.clone(), rotationY: marker.rotation.y },
  };
  controls.enabled = false;
  renderer.domElement.setPointerCapture?.(event.pointerId);
  marker.visible = true;
  marker.position.set(hit.x, 0, hit.z);
  marker.rotation.y = 0;
  renderMap();
}

function updatePoseDrag(event) {
  if (!pickingGoal || !pointerStart?.hit) return;
  const current = groundPoint(event);
  if (!current) return;
  pointerStart.current.copy(current);
  const dx = current.x - pointerStart.hit.x;
  const dy = -(current.z - pointerStart.hit.z);
  if (Math.hypot(dx, dy) > 0.03) activePoseMarker().rotation.y = Math.atan2(dy, dx);
  renderMap();
}

function restorePosePreview() {
  if (!pointerStart?.previous) return;
  const marker = activePoseMarker();
  marker.visible = pointerStart.previous.visible;
  marker.position.copy(pointerStart.previous.position);
  marker.rotation.y = pointerStart.previous.rotationY;
}

function cancelPoseDrag(event) {
  if (controls) controls.enabled = true;
  if (renderer?.domElement?.hasPointerCapture?.(event.pointerId)) {
    renderer.domElement.releasePointerCapture(event.pointerId);
  }
  restorePosePreview();
  pointerStart = null;
  renderMap();
}

function selectGoalFromPointer(event) {
  if (!pickingGoal || !pointerStart || !sceneReady) return;
  const hit = pointerStart.hit || groundPoint(event);
  const end = pointerStart.current || hit;
  controls.enabled = true;
  if (renderer.domElement.hasPointerCapture?.(event.pointerId)) renderer.domElement.releasePointerCapture(event.pointerId);
  if (!hit) { pointerStart = null; return; }
  const x = hit.x;
  const y = -hit.z;
  const robotPose = latestStatus?.pose || { x: 0, y: 0 };
  const dragX = end.x - hit.x;
  const dragY = -(end.z - hit.z);
  const dragDistance = Math.hypot(dragX, dragY);
  if (dragDistance <= 0.06) {
    restorePosePreview();
    pointerStart = null;
    setFeedback("请按住地图位置后拖动鼠标选择朝向，再松手保存。", "running");
    renderMap();
    return;
  }
  const yaw = Math.atan2(dragY, dragX);
  pointerStart = null;

  if (poseTool === "initial") {
    const yawDegrees = yaw * 180 / Math.PI;
    initialX.value = x.toFixed(2);
    initialY.value = y.toFixed(2);
    initialYaw.value = yawDegrees.toFixed(1);
    initialPoseSelected = true;
    initialMarker.visible = true;
    initialMarker.position.set(x, 0, -y);
    initialMarker.rotation.y = yaw;
    initialPoseSummary.textContent = poseSummaryText(x, y, yawDegrees);
    initialPoseSummary.classList.add("selected");
    setPoseTool("browse");
    setFeedback("初始位姿已选择，正在进入地图加载与重定位确认。", "running");
    updateWorkflowState(latestStatus || {});
    renderMap();
    stageCommand("initialize_pose");
    return;
  }

  const yawDegrees = yaw * 180 / Math.PI;
  const pose = { x, y, z: 0, yaw };
  goalX.value = x.toFixed(2);
  goalY.value = y.toFixed(2);
  goalYaw.value = yawDegrees.toFixed(1);

  if (navigationMode === "multi") {
    const previous = waypoints.length ? waypoints[waypoints.length - 1] : robotPose;
    const distance = Math.hypot(x - Number(previous.x || 0), y - Number(previous.y || 0));
    targetMarker.visible = false;
    if (distance > 10) {
      setFeedback(`该段距离 ${distance.toFixed(2)} m，超过官方 10 m 限制，请选择更近的下一个点。`, "error");
      renderMap();
      return;
    }
    waypoints.push(pose);
    goalSelected = true;
    refreshWaypointMarkers();
    updateGoalSummary();
    updateNavigationAvailability();
    setFeedback(`已加入第 ${waypoints.length} 个导航点（本段 ${distance.toFixed(2)} m）；可继续点选，或点击“开始多点”。`, "success");
    renderMap();
    return;
  }

  goalSelected = true;
  targetMarker.visible = true;
  targetMarker.position.set(x, 0, -y);
  targetMarker.rotation.y = yaw;
  goalPoseSummary.textContent = poseSummaryText(x, y, yawDegrees);
  goalPoseSummary.classList.add("selected");
  setPoseTool("browse");
  updateNavigationAvailability();
  const distance = Math.hypot(x - Number(robotPose.x || 0), y - Number(robotPose.y || 0));
  setFeedback(
    distance > 10
      ? `目标距离 ${distance.toFixed(2)} m，超过 10 m 限制，请重新选择。`
      : `导航目标已保存，距离机器人 ${distance.toFixed(2)} m；确认安全后点击“开始导航”。`,
    distance > 10 ? "error" : "success",
  );
  renderMap();
}

function updateMapHelp() {
  if (!mapHelp) return;
  if (poseTool === "initial") {
    mapHelp.textContent = "重定位：左键按住地图确定位置 → 拖动选择朝向 → 松开保存；完成或取消后恢复默认平移。";
    return;
  }
  if (poseTool === "goal") {
    mapHelp.textContent = `${navigationMode === "multi" ? "多点导航" : "导航目标"}：左键按住落点 → 拖动选择朝向 → 松开保存；完成或取消后恢复默认平移。`;
    return;
  }
  mapHelp.textContent = viewTool === "rotate"
    ? "旋转：左键按住拖动旋转视角 · 滚轮缩放；方向：长按 ↶ / ↷ 调整地图朝向；右键无地图操作。G1 URDF + 蓝色箭头=机器人，绿色箭头=初始位姿，黄色箭头=导航目标。"
    : "平移：左键按住拖动平移地图 · 滚轮缩放；方向：长按 ↶ / ↷ 调整地图朝向；右键无地图操作。G1 URDF + 蓝色箭头=机器人，绿色箭头=初始位姿，黄色箭头=导航目标。";
}

function setViewTool(tool) {
  viewTool = tool === "rotate" ? "rotate" : "pan";
  document.querySelectorAll("[data-view-tool]").forEach((button) => {
    const active = button.dataset.viewTool === viewTool;
    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  if (controls) {
    controls.mouseButtons.LEFT = viewTool === "rotate" ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN;
    controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
    controls.mouseButtons.RIGHT = null;
  }
  updateMapHelp();
}

function setPoseTool(tool) {
  const previousTool = poseTool;
  poseTool = tool === "initial" || tool === "goal" ? tool : "browse";
  pickingGoal = poseTool !== "browse";
  document.querySelectorAll("[data-pose-tool]").forEach((button) => {
    const active = button.dataset.poseTool === poseTool;
    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  mapView.classList.toggle("goal-picking", pickingGoal);
  if (poseTool === "browse" && previousTool !== "browse") setViewTool("pan");
  else updateMapHelp();
}

function updateNavigationAvailability() {
  const poseFresh = latestStatus?.pose_received && Number(latestStatus.pose_age_ms) >= 0 && Number(latestStatus.pose_age_ms) <= 1500;
  const localizedMode = mapIsLoaded() && ["localizing", "arrived"].includes(latestStatus?.mode);
  const navigationBusy = multiRouteActive || ["navigating", "paused"].includes(latestStatus?.mode);
  const goalPose = currentGoalPose();
  const robotPose = latestStatus?.pose;
  const goalDistance = navigationMode === "single" && goalSelected && goalPose && robotPose
    ? Math.hypot(goalPose.x - Number(robotPose.x || 0), goalPose.y - Number(robotPose.y || 0))
    : 0;
  const targetReady = navigationMode === "multi" ? waypoints.length > 0 : goalSelected;
  const distanceAllowed = navigationMode === "multi" ? routeDistancesAllowed() : !goalSelected || goalDistance <= 10;

  navigateButton.disabled = !(sceneReady && targetReady && distanceAllowed && latestStatus?.navigation_enabled && poseFresh && localizedMode && !navigationBusy);
  goalPickButton.disabled = !(sceneReady && poseFresh && localizedMode && !navigationBusy);
  singleNavigationModeButton.disabled = navigationBusy;
  multiNavigationModeButton.disabled = navigationBusy;
  clearWaypointsButton.disabled = navigationMode !== "multi" || multiRouteActive || waypoints.length === 0;
  updateGoalSummary();

  if (!sceneReady) {
    navigateButton.title = "点云地图不可用，导航已禁用";
  } else if (latestStatus && !latestStatus.navigation_enabled) {
    navigateButton.title = "真机导航未启用：服务启动时需要 --enable-navigation";
  } else if (!poseFresh || !localizedMode) {
    navigateButton.title = "请先加载地图完成重定位，并等待 SLAM 位姿稳定";
  } else if (!targetReady) {
    navigateButton.title = navigationMode === "multi" ? "请先添加至少一个多点导航目标" : "请先在地图选择导航目标";
  } else if (!distanceAllowed) {
    navigateButton.title = navigationMode === "multi" ? "多点路线存在超过 10 m 的分段" : `目标距离 ${goalDistance.toFixed(2)} m，超过 10 m 限制`;
  } else if (navigationBusy) {
    navigateButton.title = "当前导航任务尚未结束";
  } else {
    navigateButton.title = navigationMode === "multi" ? "确认后按顺序逐点发送官方位姿导航命令" : "打开安全确认后发送导航目标";
  }
}

function updateWorkflowState(status, poseFresh = false) {
  const ready = Boolean(status.initialized);
  const mode = status.mode || "offline";
  const taskAllowsLocalization = ["ready", "stopped", "localizing", "arrived"].includes(mode);
  const canSetPose = sceneReady && mapIsLoaded() && taskAllowsLocalization;

  startMappingButton.disabled = !ready || !["ready", "stopped"].includes(mode);
  finishMappingButton.disabled = !ready || mode !== "mapping";
  initialPoseButton.disabled = !canSetPose;
  loadMapButton.disabled = !ready || !taskAllowsLocalization;
  downloadMapButton.disabled = !isSupportedMapName(mapName.value.trim());
  exitMapButton.disabled = !ready || !status.map_loaded;
  pauseNavigationButton.disabled = !ready || mode !== "navigating";
  resumeNavigationButton.disabled = !ready || mode !== "paused";
  cancelNavigationButton.disabled = !ready || !["navigating", "paused"].includes(mode);
  stopSlamButton.disabled = !ready || mode === "stopped";

  startMappingButton.title = startMappingButton.disabled
    ? "仅待命或已关闭状态可开始建图"
    : "开始新的 SLAM 建图任务";
  finishMappingButton.title = finishMappingButton.disabled
    ? "保存地图仅在“建图中”状态可用"
    : "调用官方 API 1802 保存当前地图";
  initialPoseButton.title = !mapIsLoaded()
    ? "请先加载当前地图；加载成功后才能选择重定位位姿"
    : canSetPose
      ? "在地图按住位置并拖动选择初始位姿与朝向；松开后自动执行重定位"
      : "建图或导航进行中不能修改重定位位姿";
  loadMapButton.title = loadMapButton.disabled
    ? "当前任务状态不允许加载地图"
    : mapIsLoaded() ? "重新加载当前地图" : "先加载地图，成功后解锁重定位";
  downloadMapButton.title = downloadMapButton.disabled
    ? "仅允许 test1.pcd ~ test10.pcd"
    : `下载机器人 PC1 上 /home/unitree/${mapName.value.trim()}`;
  exitMapButton.title = exitMapButton.disabled
    ? "当前没有已加载地图"
    : "退出当前地图并关闭 SLAM 定位；退出后导航锁定";
  pauseNavigationButton.title = pauseNavigationButton.disabled ? "当前没有正在执行的导航任务" : "暂停当前导航任务";
  resumeNavigationButton.title = resumeNavigationButton.disabled ? "只有导航已暂停时才能继续" : "继续已暂停的导航任务";
  cancelNavigationButton.title = cancelNavigationButton.disabled
    ? "当前没有可取消的导航任务"
    : "取消当前单点或多点导航；停止运动但保留已加载地图";

}

function updateStatus(status) {
  latestStatus = status;
  const ready = Boolean(status.initialized);
  const poseFresh = status.pose_received && status.pose_age_ms >= 0 && status.pose_age_ms <= 1500;
  if (!ready) setBadge("SLAM 离线", "error");
  else if (status.error_code) setBadge(`错误 ${status.error_code}`, "error");
  else if (!status.mock && status.lidar_inputs && !status.lidar_inputs.ready) setBadge("雷达/IMU等待", "warning");
  else if (!poseFresh && status.mode === "ready") setBadge("等待启动建图/定位", "warning");
  else if (!poseFresh) setBadge("位姿等待中", "warning");
  else setBadge(status.mock ? "MOCK 在线" : "SLAM 在线", "online");
  slamMode.textContent = MODE_LABELS[status.mode] || status.mode || "--";
  slamPose.textContent = status.pose_received
    ? poseSummaryText(status.pose.x, status.pose.y, yawFromQuaternion(status.pose) * 180 / Math.PI)
    : "未收到";
  if (status.map_name && !mapNameDirty && document.activeElement !== mapName) {
    mapName.value = status.map_name;
  }
  if (!mapNameDirty && status.map_loaded && status.map_name) {
    loadedMapName = status.map_name;
  } else if (!status.map_loaded) {
    loadedMapName = "";
  }
  slamPointCount.textContent = `${status.live_points || 0} / ${status.global_points || 0}`;
  if (wifiStreamBadge) {
    wifiStreamBadge.textContent = status.point_filter?.transport_encoding === "base64_u16le_xyz"
      ? "Wi-Fi 压缩流"
      : "Wi-Fi 紧凑流";
  }
  const taskLabel = status.mode === "mapping" ? "建图中"
    : status.mode === "localizing" ? "定位中"
      : status.mode === "navigating" ? `导航 ${(Number(status.progress || 0) * 100).toFixed(0)}%`
        : status.mode === "paused" ? "导航已暂停"
          : status.mode === "arrived" ? "已到达"
            : "待命";
  slamTaskState.textContent = `${status.obstacle ? "检测到障碍" : "环境正常"} / ${taskLabel}`;
  progressBar.style.width = `${Math.max(0, Math.min(100, Number(status.progress || 0) * 100))}%`;
  rawStatus.textContent = JSON.stringify(status, null, 2);
  updatePoseMarker(robotMarker, status.pose);
  updateMapRobotPose(status.pose);
  const planningActive = status.mode === "navigating" || status.mode === "paused";
  const navigationBlocked = Boolean(status.obstacle && planningActive);
  if (navigationObstacleAlert) navigationObstacleAlert.hidden = !navigationBlocked;
  slamTaskState.classList.toggle("blocked", navigationBlocked);
  const trailActive = planningActive || status.mode === "mapping";
  if (!trailActive && trailActive !== (lastSlamMode === "navigating" || lastSlamMode === "paused" || lastSlamMode === "mapping")) {
    disposeLayer(trajectoryLayer);
    trajectoryLayer = null;
  }
  if (status.mode === "arrived" && lastSlamMode !== "arrived") {
    if (multiRouteActive) advanceMultiRoute();
    else if (navigationMode === "single") {
      goalSelected = false;
      targetMarker.visible = false;
      goalPoseSummary.textContent = "X: -- · Y: -- · W: --";
      goalPoseSummary.classList.remove("selected");
    }
  }
  if (status.mode === "mapping" && lastSlamMode !== "mapping") {
    loadedMapName = "";
    cancelMultiRoute(true);
    initialPoseSelected = false;
    goalSelected = false;
    initialMarker.visible = false;
    targetMarker.visible = false;
    initialPoseSummary.textContent = "X: -- · Y: -- · W: --";
    goalPoseSummary.textContent = navigationMode === "multi" ? "0 点 · X: -- · Y: -- · W: --" : "X: -- · Y: -- · W: --";
    initialPoseSummary.classList.remove("selected");
    goalPoseSummary.classList.remove("selected");
  }
  if (status.mode === "stopped" && lastSlamMode !== "stopped") {
    loadedMapName = "";
    cancelMultiRoute(true);
    targetMarker.visible = false;
    initialMarker.visible = false;
    initialPoseSelected = false;
    goalSelected = false;
    initialPoseSummary.textContent = "X: -- · Y: -- · W: --";
    goalPoseSummary.textContent = navigationMode === "multi" ? "0 点 · X: -- · Y: -- · W: --" : "X: -- · Y: -- · W: --";
    initialPoseSummary.classList.remove("selected");
    goalPoseSummary.classList.remove("selected");
    liveLayer = updatePointLayer(liveLayer, [], "live");
    setPoseTool("browse");
  }
  if (navigationMode === "single" && ["navigating", "paused"].includes(status.mode) && status.target_set && !goalSelected) {
    updatePoseMarker(targetMarker, status.target);
    targetMarker.visible = true;
    const targetYaw = yawFromQuaternion(status.target) * 180 / Math.PI;
    goalPoseSummary.textContent = poseSummaryText(status.target.x, status.target.y, targetYaw);
    goalPoseSummary.classList.add("selected");
  }
  updateWorkflowState(status, poseFresh);
  updateNavigationAvailability();
  lastSlamMode = status.mode;
  if (sceneReady && Number(status.global_sequence) !== lastGlobalSequence) loadGlobalMap(status.global_sequence);
  renderMap();
}

async function loadGlobalMap(sequence = null, quiet = false) {
  try {
    const response = await fetch("/api/perception/global-map", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    const actualSequence = Number(data.global_sequence);
    if (sequence !== null && actualSequence !== Number(sequence)) return false;
    const points = decodePointPayload(data, "points");
    globalLayer = updatePointLayer(globalLayer, points, "global");
    updateObstacleBoundary(points);
    lastGlobalSequence = actualSequence;
    mapLoading.hidden = Boolean(globalLayer || liveLayer);
    renderMap();
    return pointCount(points) > 0;
  } catch (error) {
    if (!quiet) setFeedback(`全局地图加载失败：${error.message}`, "error");
    return false;
  }
}

async function waitForLoadedGlobalMap(previousSequence) {
  for (let attempt = 0; attempt < 16; attempt += 1) {
    const loaded = await loadGlobalMap(null, true);
    if (loaded && lastGlobalSequence !== Number(previousSequence)) {
      fitMap();
      return true;
    }
    await new Promise((resolve) => setTimeout(resolve, 250));
  }
  return false;
}

async function downloadMapFile() {
  const name = mapName.value.trim();
  if (!validateMapName(true)) {
    setFeedback("地图名称无效；仅允许 test1.pcd ~ test10.pcd。", "error");
    return;
  }
  try {
    const response = await fetch(`/api/perception/map-file?map_name=${encodeURIComponent(name)}`, { cache: "no-store" });
    if (!response.ok) {
      const detail = await response.json().catch(() => ({}));
      throw new Error(detail.error || `HTTP ${response.status}`);
    }
    const blob = await response.blob();
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = name;
    document.body.append(anchor);
    anchor.click();
    anchor.remove();
    URL.revokeObjectURL(url);
    setFeedback(`地图 ${name} 已开始下载。`, "success");
  } catch (error) {
    setFeedback(`地图下载失败：${error.message}`, "error");
  }
}

async function pollStatus() {
  if (workspaceVisible()) {
    try {
      const response = await fetch("/api/perception/status", { cache: "no-store" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      updateStatus(await response.json());
    } catch (error) {
      setBadge("SLAM 接口断开", "error");
      setFeedback(`状态读取失败：${error.message}`, "error");
    }
  }
  setTimeout(pollStatus, 1200);
}

async function pollFrame() {
  if (workspaceVisible()) {
    try {
      const response = await fetch("/api/perception/frame", { cache: "no-store" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const frame = await response.json();
      latestFrame = frame;
      if (Number(frame.sequence) !== lastFrameSequence) {
        liveLayer = updatePointLayer(liveLayer, decodePointPayload(frame, "live_points"), "live");
        if (["mapping", "navigating", "paused"].includes(latestStatus?.mode)) {
          updateTrajectory(decodePointPayload(frame, "trajectory"));
        }
        updatePoseMarker(robotMarker, frame.pose);
        updateMapRobotPose(frame.pose);
        lastFrameSequence = Number(frame.sequence);
        mapLoading.hidden = Boolean(liveLayer || globalLayer);
        renderMap();
      }
    } catch (error) {
      mapLoading.hidden = false;
      mapLoading.querySelector("strong").textContent = "点云数据中断";
      mapLoading.querySelector("span").textContent = error.message;
    }
  }
  setTimeout(pollFrame, 650);
}

function cameraElements(stream) {
  const prefix = stream === "rgb" ? "rgb" : "depth";
  return {
    image: document.getElementById(`${prefix}CameraImage`),
    viewport: document.getElementById(`${prefix}CameraViewport`),
    empty: document.getElementById(`${prefix}CameraEmpty`),
    status: document.getElementById(`${prefix}CameraStatus`),
    badge: stream === "rgb" ? cameraServiceBadge : depthCameraBadge,
  };
}

function syncCameraAspect(stream) {
  const elements = cameraElements(stream);
  const width = elements.image.naturalWidth;
  const height = elements.image.naturalHeight;
  if (!(width > 0 && height > 0)) return;
  elements.viewport.style.setProperty("--camera-aspect", `${width} / ${height}`);
  elements.viewport.dataset.frameSize = `${width}×${height}`;
}

function initializeCameraViewports() {
  ["rgb", "depth"].forEach((stream) => {
    const elements = cameraElements(stream);
    elements.image.addEventListener("load", () => syncCameraAspect(stream));
  });
}

function updateCameraStatus(stream, state) {
  const elements = cameraElements(stream);
  const label = stream === "rgb" ? "RGB" : "深度";
  elements.badge.textContent = state.online ? `${label} 已启动` : state.error ? `${label} 启动失败` : `${label} 未启动`;
  elements.badge.className = `perception-badge ${state.online ? "online" : state.error ? "error" : "warning"}`;
  elements.status.className = `camera-status ${state.online ? "online" : state.error ? "error" : ""}`.trim();
  elements.status.querySelector("strong").textContent = state.online ? "画面在线" : state.configured ? "连接失败" : "未配置";
  const size = Number(state.jpeg_bytes) > 0
    ? ` · ${(Number(state.jpeg_bytes) / 1024).toFixed(1)} KiB/帧`
    : "";
  elements.status.querySelector("small").textContent = state.error ||
    `${state.source || "等待自动检测"}${size}`;
  const sourceInput = stream === "rgb" ? rgbCameraSource : depthCameraSource;
  if (!sourceInput.value) {
    sourceInput.placeholder = state.source?.startsWith("/dev/video")
      ? `${state.source}（自动）`
      : "留空自动检测";
  }
  elements.empty.hidden = Boolean(state.online);
}

function updateCameraServiceStatus(status) {
  latestCameraStatus = status;
  const captureError = status.rgb?.error || status.depth?.error;
  if (status.first_person_service?.error) {
    cameraFeedback.textContent = `摄像头占用服务切换失败：${status.first_person_service.error}`;
    cameraFeedback.className = "perception-feedback error";
  } else if (status.first_person_service?.paused_by_web) {
    const detected = [status.rgb?.source && `RGB ${status.rgb.source}`, status.depth?.source && `深度 ${status.depth.source}`].filter(Boolean).join(" · ");
    cameraFeedback.textContent = `摄像头占用服务已由 Web 临时暂停${detected ? `；已识别 ${detected}` : ""}；Wi-Fi 低带宽输出 ${status.output_width || "--"}×${status.output_height || "--"}。`;
    cameraFeedback.className = "perception-feedback success";
  } else if (!status.running && captureError) {
    cameraFeedback.textContent = `摄像头启动失败，已自动恢复原摄像头服务：${captureError === "teleimager_camera_owner_active" ? "teleimager 仍占用 D435i，请更新摄像头辅助脚本" : captureError}`;
    cameraFeedback.className = "perception-feedback error";
  } else {
    cameraFeedback.textContent = "启动时会先临时释放当前摄像头占用服务，再扫描 /dev/video* 的像素格式自动识别 RGB 与 Z16 深度流；设备框仅用于手动覆盖。";
    cameraFeedback.className = "perception-feedback";
  }
  startRealSenseCamera.disabled = !status.mock && status.backend === "unavailable";
  startRealSenseCamera.title = "自动检测当前 D435i RGB 与深度设备";
}

async function pollCameraStatus() {
  if (workspaceVisible()) {
    try {
      const response = await fetch("/api/camera/status", { cache: "no-store" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const status = await response.json();
      updateCameraServiceStatus(status);
      updateCameraStatus("rgb", status.rgb || {});
      updateCameraStatus("depth", status.depth || {});
    } catch (error) {
      ["rgb", "depth"].forEach((stream) => updateCameraStatus(stream, { configured: true, error: error.message }));
    }
  }
  setTimeout(pollCameraStatus, 1800);
}

async function loadCameraFrame(stream) {
  if (!workspaceVisible()) return;
  if (!latestCameraStatus?.[stream]?.online) return;
  const elements = cameraElements(stream);
  try {
    const response = await fetch(`/api/camera/${stream}/frame.jpg`, { cache: "no-store" });
    if (!response.ok) return;
    const url = URL.createObjectURL(await response.blob());
    if (stream === "rgb") {
      if (rgbObjectUrl) URL.revokeObjectURL(rgbObjectUrl);
      rgbObjectUrl = url;
    } else {
      if (depthObjectUrl) URL.revokeObjectURL(depthObjectUrl);
      depthObjectUrl = url;
    }
    elements.image.src = url;
    elements.empty.hidden = true;
  } catch (_) {
    // Status polling carries the actionable error and avoids flickering video.
  }
}

function pollCameraFrames() {
  Promise.all([loadCameraFrame("rgb"), loadCameraFrame("depth")]).finally(() => {
    setTimeout(pollCameraFrames, 420);
  });
}

function readPose(xInput, yInput, yawInput) {
  const x = Number(xInput.value);
  const y = Number(yInput.value);
  const yawDegrees = Number(yawInput.value);
  if (![x, y, yawDegrees].every(Number.isFinite) || Math.abs(x) > 45 || Math.abs(y) > 45 || Math.abs(yawDegrees) > 180) return null;
  return { x, y, z: 0, yaw: yawDegrees * Math.PI / 180 };
}

function currentInitialPose() {
  return readPose(initialX, initialY, initialYaw);
}

function currentGoalPose() {
  return readPose(goalX, goalY, goalYaw);
}

function mapIsLoaded() {
  const name = mapName.value.trim();
  return Boolean(name && loadedMapName === name);
}

function currentMapLoadPose() {
  const poseFresh = latestStatus?.pose_received &&
    Number(latestStatus.pose_age_ms) >= 0 &&
    Number(latestStatus.pose_age_ms) <= 1500;
  if (!poseFresh) return { x: 0, y: 0, z: 0, yaw: 0 };
  return {
    x: Number(latestStatus.pose.x) || 0,
    y: Number(latestStatus.pose.y) || 0,
    z: Number(latestStatus.pose.z) || 0,
    yaw: yawFromQuaternion(latestStatus.pose),
  };
}

function buildRequest(command) {
  const request = { command, map_name: mapName.value.trim(), confirmed: false };
  if (command === "navigate_route") {
    if (navigationMode !== "multi" || !waypoints.length) {
      setFeedback("请先切换到“多点”并在地图添加至少一个导航点。", "error");
      return null;
    }
    if (!routeDistancesAllowed()) {
      setFeedback("多点路线存在超过 10 m 的分段，请重新选择。", "error");
      return null;
    }
    request.waypoints = waypoints.map((pose) => ({ ...pose }));
    return request;
  }
  if (["finish_mapping", "initialize_pose"].includes(command) && !isSupportedMapName(request.map_name)) {
    setFeedback("地图名称无效；仅允许 test1.pcd ~ test10.pcd。", "error");
    validateMapName(true);
    return null;
  }
  if (command === "initialize_pose") {
    if (!mapIsLoaded()) {
      setFeedback("请先点击“加载地图”，地图加载成功后才能重定位。", "error");
      return null;
    }
    request.pose = currentInitialPose();
    if (!request.pose) {
      setFeedback("第 2 步的定位起始位姿超出范围或格式错误。", "error");
      return null;
    }
  }
  if (command === "navigate") {
    request.pose = currentGoalPose();
    if (!request.pose) {
      setFeedback("第 3 步的导航目标位姿超出范围或格式错误。", "error");
      return null;
    }
  }
  if (command === "navigate") {
    if (!goalSelected) {
      setFeedback("请先点击“在地图选择目标”，或手动修改第 3 步的目标坐标。", "error");
      return null;
    }
    const pose = latestStatus?.pose;
    if (pose && Math.hypot(request.pose.x - pose.x, request.pose.y - pose.y) > 10) {
      setFeedback("目标距离超过官方 10 m 限制，请选择更近的点。", "error");
      return null;
    }
  }
  return request;
}

function stageMapLoad() {
  const name = mapName.value.trim();
  if (!validateMapName(true)) {
    setFeedback("地图名称无效；仅允许 test1.pcd ~ test10.pcd。", "error");
    return;
  }
  pendingRequest = {
    endpoint: "/api/perception/command",
    feedback: "perception",
    ui_action: "load_map",
    command: "load_map",
    map_name: name,
    pose: currentMapLoadPose(),
    confirmed: false,
  };
  const [title, warning] = COMMAND_META.load_map;
  confirmTitle.textContent = title;
  confirmWarning.textContent = warning;
  confirmSafetyTitle.textContent = "确认要加载此地图";
  confirmSafetyDetail.textContent = "加载成功后才会开放“地图选初始位姿”；后续选点会自动再次执行重定位。";
  dialogFeedback.textContent = "";
  submitCommand.disabled = false;
  confirmDialog.showModal();
}

function stageCommand(command, uiAction = "") {
  const request = buildRequest(command);
  if (!request) return;
  pendingRequest = { endpoint: "/api/perception/command", feedback: "perception", ui_action: uiAction, ...request };
  const [title, warning] = COMMAND_META[uiAction || command];
  confirmTitle.textContent = title;
  confirmWarning.textContent = warning;
  confirmSafetyTitle.textContent = ["navigate", "navigate_route", "resume_navigation"].includes(command)
    ? "该指令可能让机器人自主移动"
    : command === "cancel_navigation"
      ? "确认取消当前导航任务"
      : uiAction === "exit_map"
        ? "退出后当前地图将立即失效"
        : "确认 SLAM 环境与任务状态正确";
  confirmSafetyDetail.textContent = "确认雷达与定位正常，机器人可靠支撑，周围人员和障碍物已清空，且 App 没有同时使用导航。";
  dialogFeedback.textContent = "";
  submitCommand.disabled = false;
  confirmDialog.showModal();
}

function stageCameraCommand(command) {
  const request = { command: command === "stop_camera" ? "stop" : command };
  if (command === "start_v4l2") {
    request.rgb_source = rgbCameraSource.value.trim();
    request.depth_source = depthCameraSource.value.trim();
    const valid = (value) => value === "" || /^\/dev\/video\d{1,3}$/.test(value);
    if (!valid(request.rgb_source) || !valid(request.depth_source)) {
      cameraFeedback.textContent = "手动覆盖设备必须是 /dev/videoN；两项都留空时会自动检测。";
      cameraFeedback.className = "perception-feedback error";
      return;
    }
  }
  pendingRequest = { endpoint: "/api/camera/command", feedback: "camera", ...request };
  const [title, warning] = COMMAND_META[command];
  confirmTitle.textContent = title;
  confirmWarning.textContent = warning;
  confirmSafetyTitle.textContent = "Web 取流会临时独占当前 D435i";
  confirmSafetyDetail.textContent = "启动时会暂停占用 D435i 的 master_service / teleimager.service，再自动识别当前 RGB 与 Z16 深度设备；停止后仅恢复本页实际暂停的服务。";
  dialogFeedback.textContent = "";
  submitCommand.disabled = false;
  confirmDialog.showModal();
}

function requestKey() {
  if (globalThis.crypto?.randomUUID) return `slam-${globalThis.crypto.randomUUID()}`;
  return `slam-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

async function executePending() {
  if (!pendingRequest) return;
  submitCommand.disabled = true;
  const { endpoint, feedback: feedbackTarget, ui_action: uiAction, ...request } = pendingRequest;
  if (request.command === "navigate_route") {
    confirmDialog.close();
    pendingRequest = null;
    multiRouteActive = true;
    activeWaypointIndex = -1;
    updateGoalSummary();
    updateNavigationAvailability();
    setFeedback(`多点导航已开始，共 ${request.waypoints.length} 个目标。`, "running");
    await submitNavigationWaypoint(0);
    return;
  }
  dialogFeedback.textContent = uiAction === "load_map"
    ? "正在启动雷达和 SLAM 并加载地图；首次冷启动通常需要 10–20 秒，请勿重复点击…"
    : request.command === "initialize_pose"
      ? "正在应用新的重定位位姿…"
    : request.command === "start_mapping"
      ? "正在启动雷达和 SLAM，确认在线后开始建图…"
      : "正在提交…";
  const body = { ...request, confirmed: true, request_key: requestKey() };
  const globalSequenceBeforeCommand = lastGlobalSequence;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 25000);
  try {
    const response = await fetch(endpoint, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
      signal: controller.signal,
    });
    clearTimeout(timeout);
    const result = await readCommandResponse(response);
    confirmDialog.close();
    const metaKey = uiAction || (body.command === "stop" ? "stop_camera" : body.command);
    let mapLoadNote = "";
    if (uiAction === "load_map") {
      loadedMapName = body.map_name;
      mapNameDirty = false;
      initialPoseSelected = false;
      initialMarker.visible = false;
      initialPoseSummary.textContent = "X: -- · Y: -- · W: --";
      initialPoseSummary.classList.remove("selected");
      setPoseTool("browse");
      updateWorkflowState(latestStatus || {});
      const globalMapReady = await waitForLoadedGlobalMap(globalSequenceBeforeCommand);
      mapLoadNote = globalMapReady ? "；全局点云已同步显示" : "；仍在等待定位全局点云";
    } else if (body.command === "initialize_pose") {
      initialPoseSelected = false;
      initialMarker.visible = false;
      initialPoseSummary.textContent = "X: -- · Y: -- · W: --";
      initialPoseSummary.classList.remove("selected");
    } else if (body.command === "pause_navigation" && latestStatus) {
      latestStatus = { ...latestStatus, mode: "paused", paused: true };
      updateWorkflowState(latestStatus);
      updateNavigationAvailability();
    } else if (body.command === "resume_navigation" && latestStatus) {
      latestStatus = { ...latestStatus, mode: "navigating", paused: false };
      updateWorkflowState(latestStatus);
      updateNavigationAvailability();
    } else if (body.command === "cancel_navigation") {
      cancelMultiRoute(false);
      if (latestStatus) {
        latestStatus = {
          ...latestStatus,
          mode: "localizing",
          paused: false,
          target_set: false,
          progress: 0,
        };
        updateWorkflowState(latestStatus);
      }
      setPoseTool("browse");
      updateNavigationAvailability();
      renderMap();
    } else if (body.command === "stop_slam") {
      loadedMapName = "";
      cancelMultiRoute(true);
      initialPoseSelected = false;
      goalSelected = false;
      if (initialMarker) initialMarker.visible = false;
      if (targetMarker) targetMarker.visible = false;
      setPoseTool("browse");
      updateNavigationAvailability();
      renderMap();
    }
    const autoStarted = [result.lidar_started && "雷达", result.slam_started && "SLAM"].filter(Boolean);
    const serviceNote = autoStarted.length ? `；已自动启动${autoStarted.join("和")}` : "；依赖服务已在线";
    const message = `${COMMAND_META[metaKey][0]}已由服务接受（请求 ${result.request_id}${feedbackTarget === "perception" ? serviceNote : ""}）${mapLoadNote}`;
    if (feedbackTarget === "camera") {
      cameraFeedback.textContent = message;
      cameraFeedback.className = "perception-feedback success";
    } else {
      setFeedback(message, "success");
    }
    pendingRequest = null;
  } catch (error) {
    clearTimeout(timeout);
    dialogFeedback.textContent = error.name === "AbortError"
      ? "提交超时：服务可能仍在启动，请查看下方服务状态后重试。"
      : `提交失败：${error.message}`;
    dialogFeedback.className = "control-dialog-feedback error";
    submitCommand.disabled = false;
  }
}

document.getElementById("mapZoomIn").addEventListener("click", () => {
  if (!sceneReady) return;
  camera.position.lerp(controls.target, 0.18); controls.update();
});
document.getElementById("mapZoomOut").addEventListener("click", () => {
  if (!sceneReady) return;
  camera.position.sub(controls.target).multiplyScalar(1.22).add(controls.target); controls.update();
});
document.getElementById("mapFit").addEventListener("click", fitMap);
document.getElementById("mapLocate").addEventListener("click", locateRobot);
mapExpandButton?.addEventListener("click", () => {
  setMapWorkspaceExpanded(!perceptionGrid?.classList.contains("map-expanded"));
});
document.getElementById("mapTopView").addEventListener("click", () => sceneReady && setTopView());
document.getElementById("mapThreeView").addEventListener("click", () => sceneReady && setThreeView());
document.querySelectorAll("[data-view-tool]").forEach((button) => {
  button.addEventListener("click", () => {
    if (!sceneReady) return;
    if (poseTool !== "browse") setPoseTool("browse");
    setViewTool(button.dataset.viewTool);
  });
});
[
  [document.getElementById("mapHeadingLeft"), -1],
  [document.getElementById("mapHeadingRight"), 1],
].forEach(([button, direction]) => {
  button.addEventListener("pointerdown", (event) => startMapHeadingHold(direction, event));
  button.addEventListener("pointerup", stopMapHeadingHold);
  button.addEventListener("pointercancel", stopMapHeadingHold);
  button.addEventListener("lostpointercapture", stopMapHeadingHold);
  button.addEventListener("click", (event) => {
    if (event.detail === 0) adjustMapHeading(direction * THREE.MathUtils.degToRad(5));
  });
});
window.addEventListener("blur", stopMapHeadingHold);
document.addEventListener("visibilitychange", () => document.hidden && stopMapHeadingHold());
document.querySelectorAll("[data-pose-tool]").forEach((button) => {
  button.addEventListener("click", () => {
    if (!sceneReady) return;
    const tool = button.dataset.poseTool;
    setPoseTool(tool !== "browse" && poseTool === tool ? "browse" : tool);
    if (poseTool === "initial") {
      setFeedback("重定位选点已开启：在地图按住一个位置，拖动选择朝向，松开保存。", "running");
    } else if (poseTool === "goal") {
      setFeedback("导航目标选点已开启：在地图按住目标位置，拖动选择朝向，松开保存。", "running");
    }
  });
});
document.getElementById("startMapping").addEventListener("click", () => stageCommand("start_mapping"));
document.getElementById("finishMapping").addEventListener("click", () => stageCommand("finish_mapping"));
loadMapButton.addEventListener("click", stageMapLoad);
downloadMapButton.addEventListener("click", downloadMapFile);
exitMapButton.addEventListener("click", () => stageCommand("stop_slam", "exit_map"));
mapName.addEventListener("input", () => {
  validateMapName(false);
  mapNameDirty = true;
  if (mapName.value.trim() !== loadedMapName) {
    loadedMapName = "";
    initialPoseSelected = false;
    initialMarker.visible = false;
    initialPoseSummary.textContent = "X: -- · Y: -- · W: --";
    initialPoseSummary.classList.remove("selected");
    if (poseTool === "initial") setPoseTool("browse");
  }
  updateWorkflowState(latestStatus || {});
  updateNavigationAvailability();
  renderMap();
});
mapName.addEventListener("change", () => {
  if (mapName.value.trim() && !validateMapName(true)) {
    setFeedback("地图名称无效；请使用 test1.pcd ~ test10.pcd。", "error");
  }
});
singleNavigationModeButton.addEventListener("click", () => setNavigationMode("single"));
multiNavigationModeButton.addEventListener("click", () => setNavigationMode("multi"));
saveNavigationTaskButton.addEventListener("click", saveCurrentNavigationTask);
renameNavigationTaskButton.addEventListener("click", renameSelectedNavigationTask);
runNavigationTaskButton.addEventListener("click", () => applyNavigationTask(selectedStoredNavigationTask(), true));
deleteNavigationTaskButton.addEventListener("click", deleteSelectedNavigationTask);
navigationTaskList.addEventListener("change", () => {
  selectedNavigationTaskId = navigationTaskList.value;
  const task = selectedStoredNavigationTask();
  if (task) applyNavigationTask(task, false);
});
navigationTaskList.addEventListener("dblclick", () => applyNavigationTask(selectedStoredNavigationTask(), true));
clearWaypointsButton.addEventListener("click", () => {
  if (multiRouteActive) return;
  waypoints = [];
  goalSelected = false;
  clearWaypointMarkers();
  updateGoalSummary();
  updateNavigationAvailability();
  renderMap();
});
navigateButton.addEventListener("click", () => stageCommand(navigationMode === "multi" ? "navigate_route" : "navigate"));
document.getElementById("pauseNavigation").addEventListener("click", () => stageCommand("pause_navigation"));
document.getElementById("resumeNavigation").addEventListener("click", () => stageCommand("resume_navigation"));
cancelNavigationButton.addEventListener("click", () => stageCommand("cancel_navigation"));
document.getElementById("stopSlam").addEventListener("click", () => stageCommand("stop_slam"));
document.getElementById("startV4l2Camera").addEventListener("click", () => stageCameraCommand("start_v4l2"));
startRealSenseCamera.addEventListener("click", () => stageCameraCommand("start_realsense"));
document.getElementById("stopCamera").addEventListener("click", () => stageCameraCommand("stop_camera"));
document.getElementById("stopDepthCamera").addEventListener("click", () => stageCameraCommand("stop_camera"));
document.getElementById("cancelPerceptionCommand").addEventListener("click", () => {
  pendingRequest = null;
  confirmDialog.close();
});
confirmForm.addEventListener("submit", (event) => {
  event.preventDefault();
  executePending();
});
[initialX, initialY, initialYaw].forEach((input) => input.addEventListener("input", () => {
  const pose = currentInitialPose();
  if (!pose || !initialMarker) return;
  initialMarker.visible = true;
  initialMarker.position.set(pose.x, 0, -pose.y);
  initialMarker.rotation.y = pose.yaw;
  renderMap();
}));
[goalX, goalY, goalYaw].forEach((input) => input.addEventListener("input", () => {
  const pose = currentGoalPose();
  if (!pose) return;
  goalSelected = true;
  if (targetMarker) {
    targetMarker.visible = true;
    targetMarker.position.set(pose.x, 0, -pose.y);
    targetMarker.rotation.y = pose.yaw;
  }
  updateNavigationAvailability();
  renderMap();
}));
document.querySelectorAll("[data-camera-fullscreen]").forEach((button) => {
  button.addEventListener("click", () => {
    document.getElementById(button.dataset.cameraFullscreen)?.requestFullscreen?.();
  });
});
document.querySelectorAll("[data-native-layer]").forEach((button) => {
  button.addEventListener("click", () => {
    const name = button.dataset.nativeLayer;
    nativeVisibility[name] = !nativeVisibility[name];
    button.classList.toggle("active", nativeVisibility[name]);
    if (name === "global" && globalLayer) globalLayer.visible = nativeVisibility[name];
    if (name === "global" && obstacleBoundaryLayer) obstacleBoundaryLayer.visible = nativeVisibility[name];
    if (name === "live" && liveLayer) liveLayer.visible = nativeVisibility[name];
    renderMap();
  });
});
window.addEventListener("g1:telemetry", (event) => syncMapRobotTelemetry(event.detail));
window.addEventListener("g1:workspace-change", (event) => {
  if (event.detail?.workspace === "console") {
    requestAnimationFrame(() => {
      renderer?.setSize(Math.max(1, mapView.clientWidth), Math.max(1, mapView.clientHeight), false);
      camera && (camera.aspect = Math.max(1, mapView.clientWidth) / Math.max(1, mapView.clientHeight));
      camera?.updateProjectionMatrix();
      renderMap();
    });
  }
});
initializeCameraViewports();
renderNavigationTaskList();
if (initializeMap()) {
  setTopView();
  syncMapRobotTelemetry(latestRobotTelemetry);
  pollFrame();
}
pollStatus();
pollCameraStatus();
pollCameraFrames();
