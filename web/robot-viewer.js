import * as THREE from "/assets/vendor/three/three.module.js";
import { OrbitControls } from "/assets/vendor/three/examples/jsm/controls/OrbitControls.js";
import URDFLoader from "/assets/vendor/urdf-loader/URDFLoader.js";

const ROS_TO_THREE = new THREE.Quaternion().setFromEuler(
  new THREE.Euler(-Math.PI / 2, 0, 0, "XYZ"),
);
const THREE_TO_ROS = ROS_TO_THREE.clone().invert();

const viewer = document.getElementById("robotViewer");
const loading = document.getElementById("viewerLoading");
const overlay = document.getElementById("viewerOverlay");
const modelState = document.getElementById("modelState");
const modelFile = document.getElementById("modelFile");
const resetButton = document.getElementById("resetRobotView");
const tableBody = document.getElementById("jointTableBody");

let renderer;
let scene;
let camera;
let controls;
let robot;
let modelMount;
let currentModel;
let latestData = window.g1LatestTelemetry || null;
let selectedJointName = null;
let selectedMeshes = [];
let loadGeneration = 0;
let sceneReady = false;
let debugJointValues = null;
const modelBounds = new THREE.Box3();
const modelFocus = new THREE.Vector3();
const focusShift = new THREE.Vector3();

function setModelStatus(text, state = "pending") {
  modelState.textContent = text;
  modelState.className = `model-state ${state}`;
}

function setLoading(title, detail) {
  loading.hidden = false;
  loading.querySelector("strong").textContent = title;
  loading.querySelector("small").textContent = detail;
}

function manifestRobotName() {
  const name = window.robotManifest?.identity?.display_name;
  if (typeof name === "string" && name.trim()) return name.trim();
  return window.UiI18n?.language === "en" ? "Robot" : "机器人";
}

function viewerText(zh, en) {
  return window.UiI18n?.language === "en" ? en : zh;
}

function manifestJointDescriptor(name) {
  return window.robotManifest?.joints?.find((joint) => joint?.name === name) || null;
}

function manifestUrdfJointName(name) {
  const urdfName = manifestJointDescriptor(name)?.urdf_joint_name;
  return typeof urdfName === "string" && urdfName ? urdfName : null;
}

function semanticJointName(urdfName) {
  return window.robotManifest?.joints?.find((joint) => joint?.urdf_joint_name === urdfName)?.name || null;
}

function initializeScene() {
  let rendererError;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: "high-performance" });
  } catch (error) {
    rendererError = error;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: false, alpha: true, powerPreference: "default" });
    } catch (fallbackError) {
      rendererError = fallbackError || rendererError;
    }
  }
  if (!renderer) {
    setModelStatus("WebGL 不可用", "error");
    setLoading("三维视图不可用", rendererError?.message || "请启用浏览器硬件加速；关节列表仍可使用");
    resetButton.disabled = true;
    return false;
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x000000, 0);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  viewer.prepend(renderer.domElement);

  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(35, 1, 0.01, 100);
  camera.position.set(2.5, 1.5, 2.8);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = false;
  controls.enablePan = false;
  controls.minDistance = 1;
  controls.maxDistance = 8;
  controls.target.set(0, 0.85, 0);
  controls.addEventListener("change", renderScene);

  scene.add(new THREE.HemisphereLight(0xbfeaff, 0x18202d, 2.2));
  const key = new THREE.DirectionalLight(0xffffff, 2.4);
  key.position.set(3, 5, 4);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x54ddff, 1.4);
  rim.position.set(-4, 2, -3);
  scene.add(rim);
  const grid = new THREE.GridHelper(5, 20, 0x28506a, 0x18283a);
  grid.material.transparent = true;
  grid.material.opacity = 0.45;
  scene.add(grid);

  const resize = () => {
    const width = Math.max(1, viewer.clientWidth);
    const height = Math.max(1, viewer.clientHeight);
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderScene();
  };
  new ResizeObserver(resize).observe(viewer);
  resize();
  renderer.domElement.addEventListener("pointerup", selectFromPointer);
  renderer.domElement.addEventListener("webglcontextlost", (event) => {
    event.preventDefault();
    sceneReady = false;
    setModelStatus("WebGL 上下文中断", "error");
    setLoading("三维渲染已中断", "请刷新页面重新建立 WebGL 上下文；关节列表仍可使用");
    resetButton.disabled = true;
  });
  sceneReady = true;
  resetButton.disabled = false;
  return true;
}

function renderScene() {
  if (renderer && scene && camera) renderer.render(scene, camera);
}

function disposeRobot() {
  if (!robot) return;
  scene.remove(modelMount);
  robot.traverse((object) => {
    object.geometry?.dispose?.();
    if (Array.isArray(object.material)) object.material.forEach((material) => material.dispose?.());
    else object.material?.dispose?.();
  });
  robot = null;
  modelMount = null;
  selectedMeshes = [];
}

function prepareMaterials(root) {
  root.traverse((object) => {
    if (!object.isMesh) return;
    if (Array.isArray(object.material)) object.material = object.material.map((material) => material.clone());
    else if (object.material) object.material = object.material.clone();
  });
}

function currentModelFocus() {
  if (!modelMount) return null;
  modelBounds.setFromObject(modelMount);
  if (modelBounds.isEmpty()) return null;
  return modelBounds.getCenter(modelFocus);
}

function anchorViewToRobot() {
  const center = currentModelFocus();
  if (!center || !controls || !camera) return;
  focusShift.copy(center).sub(controls.target);
  if (focusShift.lengthSq() < 1e-10) return;
  controls.target.copy(center);
  camera.position.add(focusShift);
  controls.update();
}

function fitView() {
  if (!modelMount) return;
  modelBounds.setFromObject(modelMount);
  if (modelBounds.isEmpty()) return;
  const size = modelBounds.getSize(new THREE.Vector3());
  const center = modelBounds.getCenter(new THREE.Vector3());
  const radius = Math.max(size.x, size.y, size.z) * 0.72;
  controls.target.copy(center);
  camera.position.set(center.x + radius * 1.7, center.y + radius * .45, center.z + radius * 1.9);
  camera.near = Math.max(.01, radius / 100);
  camera.far = Math.max(20, radius * 20);
  camera.updateProjectionMatrix();
  controls.update();
  renderScene();
}

function validateJoints(candidate) {
  const expectedJoints = window.robotManifest?.joints;
  if (!Array.isArray(expectedJoints) || !expectedJoints.length) {
    throw new Error(viewerText("Robot Manifest 缺少关节 Schema", "Robot Manifest is missing the joint schema"));
  }
  const expectedNames = expectedJoints.map((joint) => joint?.urdf_joint_name);
  if (expectedNames.some((name) => typeof name !== "string" || !name)) {
    throw new Error(viewerText("Robot Manifest 关节元数据无效", "Robot Manifest joint metadata is invalid"));
  }
  const missing = expectedNames.filter((name) => !candidate.joints[name]);
  if (missing.length) {
    throw new Error(viewerText(
      `URDF 关节映射不完整：${missing.join(", ")}`,
      `Incomplete URDF joint mapping: ${missing.join(", ")}`,
    ));
  }
}

function resolveModel(robotInfo) {
  const manifestModel = window.robotManifest?.model;
  if (!manifestModel) return null;

  const machineId = Number(robotInfo?.mode_machine_raw);
  if (!Number.isFinite(machineId)) return null;

  const manifestMachineId = Number(window.robotManifest?.diagnostics?.mode_machine_raw);
  const manifestMatchesTelemetry = Number.isFinite(manifestMachineId) && manifestMachineId === machineId;
  if (manifestMatchesTelemetry && manifestModel.supported === false) return null;
  if (!manifestMatchesTelemetry && robotInfo?.model_supported === false) return null;

  const file = manifestMatchesTelemetry ? manifestModel.urdf_file : robotInfo?.urdf_file;
  const assetRoot = manifestModel.asset_root;
  const packageName = manifestModel.package;
  if (
    typeof file !== "string" || !file || file.includes("/") || file.includes("..") ||
    typeof assetRoot !== "string" || !assetRoot.startsWith("/assets/") || assetRoot.includes("..") ||
    typeof packageName !== "string" || !packageName || packageName.includes("/") || packageName.includes("..")
  ) {
    throw new Error(viewerText(
      `拒绝无效模型元数据：${file || "--"}`,
      `Rejected invalid model metadata: ${file || "--"}`,
    ));
  }

  const name = manifestMatchesTelemetry ? manifestModel.model_name : robotInfo?.model_name;
  return { machineId, file, assetRoot, packageName, name: name || `${manifestRobotName()} mode ${machineId}` };
}

function createUrdfLoader(manager, selection) {
  const loader = new URDFLoader(manager);
  loader.packages = { [selection.packageName]: selection.assetRoot };
  loader.fetchOptions = { cache: "reload" };
  return loader;
}

function loadRobotUrdfInstance(robotInfo, joints, onLoad, onError) {
  let selection;
  try {
    selection = resolveModel(robotInfo);
    if (!selection) throw new Error(`不支持 mode_machine ${Number(robotInfo?.mode_machine_raw)}`);
  } catch (error) {
    onError?.(error);
    return null;
  }

  const manager = new THREE.LoadingManager();
  manager.onError = (url) => onError?.(new Error(`模型资源加载失败：${url.split("/").pop() || url}`));
  const loader = createUrdfLoader(manager, selection);
  loader.load(
    `${selection.assetRoot}/${selection.file}`,
    (candidate) => {
      try {
        validateJoints(candidate, joints);
        candidate.quaternion.copy(ROS_TO_THREE);
        prepareMaterials(candidate);
        onLoad?.(candidate, selection);
      } catch (error) {
        onError?.(error);
      }
    },
    undefined,
    (error) => onError?.(error instanceof Error ? error : new Error(String(error))),
  );
  return selection.file;
}

function applyRobotJointValues(targetRobot, joints) {
  if (!targetRobot || !joints) return;
  joints.forEach((joint) => {
    const urdfName = manifestUrdfJointName(joint.name);
    const urdfJoint = urdfName ? targetRobot.joints?.[urdfName] : null;
    const value = Number(joint.q_rad);
    if (urdfJoint && Number.isFinite(value)) urdfJoint.setJointValue(value);
  });
}

function loadModel(robotInfo) {
  if (!sceneReady || !scene) return;
  const machineId = Number(robotInfo?.mode_machine_raw);
  let selection;
  try {
    selection = resolveModel(robotInfo);
  } catch (error) {
    currentModel = null;
    disposeRobot();
    setModelStatus("机型校验失败", "error");
    setLoading("拒绝加载模型", error.message);
    modelFile.textContent = `URDF -- · mode_machine ${machineId}`;
    return;
  }
  const file = selection?.file;
  if (!file) {
    currentModel = null;
    disposeRobot();
    setModelStatus(`不支持机型 ${machineId}`, "error");
    setLoading("无法选择官方模型", `mode_machine ${machineId} 未映射，关节表仍可使用`);
    modelFile.textContent = `URDF -- · mode_machine ${machineId}`;
    return;
  }
  if (currentModel === file && robot) return;
  currentModel = file;
  const generation = ++loadGeneration;
  disposeRobot();
  setModelStatus("正在加载", "pending");
  const displayName = manifestRobotName();
  setLoading(
    window.UiI18n?.language === "en"
      ? `Loading official ${displayName} model`
      : `正在加载官方 ${displayName} 模型`,
    file,
  );
  modelFile.textContent = `${selection.name} · 本地 URDF ${file}`;

  const manager = new THREE.LoadingManager();
  manager.onError = (url) => {
    if (generation !== loadGeneration) return;
    setModelStatus("资源加载失败", "error");
    setLoading("模型资源加载失败", url.split("/").pop() || url);
  };
  const loader = createUrdfLoader(manager, selection);
  loader.load(
    `${selection.assetRoot}/${file}`,
    (candidate) => {
      if (generation !== loadGeneration) return;
      if (!sceneReady || !scene) return;
      try {
        validateJoints(candidate);
      } catch (error) {
        setModelStatus("模型校验失败", "error");
        setLoading("URDF 关节校验失败", error.message);
        return;
      }
      try {
        robot = candidate;
        robot.quaternion.copy(ROS_TO_THREE);
        modelMount = new THREE.Group();
        modelMount.name = "hip_imu_pose";
        modelMount.add(robot);
        prepareMaterials(robot);
        scene.add(modelMount);
        applyJointValues();
        loading.hidden = true;
        setModelStatus(`M${machineId} 已识别`, "ready");
        fitView();
        window.dispatchEvent(new CustomEvent("g1:robot-model-ready", {
          detail: getRobotJointMetadata(),
        }));
      } catch (error) {
        disposeRobot();
        setModelStatus("模型挂载失败", "error");
        setLoading("三维模型挂载失败", error?.message || file);
      }
    },
    undefined,
    (error) => {
      if (generation !== loadGeneration) return;
      setModelStatus("模型加载失败", "error");
      setLoading("无法读取 URDF", error?.message || file);
    },
  );
}

function applyHipOrientation() {
  if (!modelMount) return;
  const values = latestData?.imu?.hip?.quaternion_wxyz;
  if (!Array.isArray(values) || values.length !== 4) return;
  const [w, x, y, z] = values.map(Number);
  if (![w, x, y, z].every(Number.isFinite)) return;
  const norm = Math.hypot(w, x, y, z);
  if (norm < 0.5) return;
  const rosOrientation = new THREE.Quaternion(x, y, z, w).normalize();
  modelMount.quaternion.copy(ROS_TO_THREE)
    .multiply(rosOrientation)
    .multiply(THREE_TO_ROS);
}

function applyJointValues() {
  if (!robot || !latestData?.joints) return;
  if (debugJointValues === null) {
    applyRobotJointValues(robot, latestData.joints);
  } else {
    latestData.joints.forEach((joint) => {
      const urdfName = manifestUrdfJointName(joint.name);
      const urdfJoint = urdfName ? robot.joints[urdfName] : null;
      const value = debugJointValues.get(joint.name);
      if (urdfJoint && Number.isFinite(value)) urdfJoint.setJointValue(value);
    });
  }
  if (debugJointValues === null) applyHipOrientation();
  else modelMount?.quaternion.identity();
  anchorViewToRobot();
  renderScene();
}

function jointFromObject(object) {
  let current = object;
  while (current && current !== robot) {
    if (current.isURDFJoint) return current;
    current = current.parent;
  }
  return null;
}

function selectFromPointer(event) {
  if (!robot || event.button !== 0) return;
  const rect = renderer.domElement.getBoundingClientRect();
  const pointer = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1,
  );
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.intersectObject(robot, true).find((entry) => entry.object.isMesh);
  const urdfJoint = hit && jointFromObject(hit.object);
  if (!urdfJoint?.urdfName) return;
  const name = semanticJointName(urdfJoint.urdfName);
  if (!name || !latestData?.joints?.some((joint) => joint.name === name)) return;
  selectJoint(name, true);
}

function materialList(mesh) {
  return Array.isArray(mesh.material) ? mesh.material : [mesh.material];
}

function clearHighlight() {
  selectedMeshes.forEach((mesh) => {
    materialList(mesh).forEach((material) => {
      if (material?.emissive && material.userData.g1OriginalEmissive !== undefined) {
        material.emissive.setHex(material.userData.g1OriginalEmissive);
        delete material.userData.g1OriginalEmissive;
      }
    });
  });
  selectedMeshes = [];
}

function highlightJoint(name) {
  clearHighlight();
  const urdfName = manifestUrdfJointName(name);
  const joint = urdfName ? robot?.joints?.[urdfName] : null;
  if (!joint) return;
  joint.traverse((object) => {
    if (!object.isMesh) return;
    selectedMeshes.push(object);
    materialList(object).forEach((material) => {
      if (!material?.emissive) return;
      material.userData.g1OriginalEmissive = material.emissive.getHex();
      material.emissive.setHex(0x16778f);
    });
  });
  renderScene();
}

function format(value, digits = 3) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed.toFixed(digits) : "--";
}

function selectJoint(name, scrollTable = false) {
  selectedJointName = name;
  document.querySelectorAll("#jointTableBody tr").forEach((row) => {
    row.classList.toggle("joint-selected", row.dataset.jointName === name);
  });
  highlightJoint(name);
  if (scrollTable) {
    document.querySelector(`#jointTableBody tr[data-joint-name="${CSS.escape(name)}"]`)?.scrollIntoView({ block: "nearest" });
  }
}

tableBody.addEventListener("click", (event) => {
  const row = event.target.closest("tr[data-joint-name]");
  if (row) selectJoint(row.dataset.jointName);
});

window.addEventListener("g1:telemetry", (event) => {
  latestData = event.detail;
  const machineId = Number(latestData?.robot?.mode_machine_raw);
  if (window.robotManifest && Number.isFinite(machineId)) loadModel(latestData.robot);
  applyJointValues();
  if (!selectedJointName && latestData?.joints?.length) selectJoint(latestData.joints[0].name);
});
window.addEventListener("unirobo:manifest", (event) => {
  if (!event.detail) {
    currentModel = null;
    disposeRobot();
    setModelStatus(viewerText("Manifest 不可用", "Manifest unavailable"), "error");
    setLoading(
      viewerText("无法选择机器人模型", "Unable to select robot model"),
      viewerText("Robot Manifest 获取失败，关节列表仍可使用", "Robot Manifest failed to load; the joint list remains available"),
    );
    return;
  }
  const machineId = Number(latestData?.robot?.mode_machine_raw);
  if (Number.isFinite(machineId)) loadModel(latestData.robot);
  applyJointValues();
});
window.addEventListener("g1:connection", (event) => {
  overlay.hidden = event.detail?.online !== false;
});
window.addEventListener("g1:workspace-change", (event) => {
  if (event.detail?.workspace === "robot") requestAnimationFrame(renderScene);
});
window.addEventListener("g1:robot-viewer-dock-change", () => {
  requestAnimationFrame(() => {
    if (modelMount) fitView();
    else renderScene();
  });
});
resetButton.addEventListener("click", fitView);

function setDebugJointValues(values) {
  debugJointValues = values === null ? null : new Map(values);
  applyJointValues();
}

function getRobotJointMetadata() {
  if (!robot || !latestData?.joints) return [];
  return latestData.joints.map((joint) => {
    const urdfName = manifestUrdfJointName(joint.name);
    const urdfJoint = urdfName ? robot.joints[urdfName] : null;
    return {
      index: joint.index,
      name: joint.name,
      nameZh: joint.name_zh,
      movable: urdfJoint?.jointType === "revolute",
      lower: Number(urdfJoint?.limit?.lower),
      upper: Number(urdfJoint?.limit?.upper),
    };
  });
}

if (initializeScene() && latestData && window.robotManifest) {
  const machineId = Number(latestData?.robot?.mode_machine_raw);
  if (Number.isFinite(machineId)) loadModel(latestData.robot);
}
if (window.g1ConnectionOnline === false) overlay.hidden = false;

export {
  resolveModel,
  loadRobotUrdfInstance,
  applyRobotJointValues,
  setDebugJointValues,
  getRobotJointMetadata,
};
