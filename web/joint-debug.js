import { getRobotJointMetadata, setDebugJointValues } from "/robot-viewer.js";

const $ = (id) => document.getElementById(id);
const RAD_TO_DEG = 180 / Math.PI;
const groups = [
  ["左腿", 0, 6],
  ["右腿", 6, 12],
  ["腰部", 12, 15],
  ["左臂", 15, 22],
  ["右臂", 22, 29],
];
const errorLabels = {
  debug_mode_required: "请先让机器人进入阻尼/零力矩状态并使用遥控器 L2 + R2 进入调试模式。",
  joint_out_of_range: "关节目标超出 URDF limit，后端已拒绝。",
  lowstate_unavailable: "LowState 不可用或已超时。",
  control_busy: "其他控制任务正在执行。",
  upper_body_joint_not_allowed: "上半身模式禁止下发腿部关节。",
  pr_mode_required: "当前不是与官方 URDF 一致的 PR 模式。",
  control_lease_expired: "页面控制心跳已中断，后端已自动释放控制。",
  dds_publish_failed: "DDS 发布失败，后端已自动释放控制。",
  service_state_unavailable: "无法读取机器人服务状态，已禁止下发。",
  sport_state_stale: "运控状态不可用或已超时，已禁止上半身下发。",
  upper_body_fsm_not_allowed: "上半身仅允许在常规运控（FSM 500/501）或走跑运控（FSM 801/802）下发；零力矩和阻尼模式已禁止。",
  invalid_teach_action_name: "请输入有效的示教动作名称。",
  joint_teach_not_recording: "当前没有可保存的示教录制。",
  joint_teach_too_short: "录制时间太短，请至少录制 0.1 秒。",
  joint_teach_action_not_found: "本地示教动作不存在。",
  joint_teach_model_mismatch: "该动作由另一种 G1 机型录制，已禁止播放。",
  joint_teach_store_failed: "本地示教文件保存失败。",
  joint_teach_duration_limit: "录制已达到 120 秒上限，请停止并保存。",
  joint_teach_remote_binding_not_allowed: "该按键组合不允许用于自定义动作；只能选择 F1 / F3 组合键。",
  joint_teach_remote_binding_in_use: "该遥控器组合键已经绑定到其他示教动作。",
};

let status = null;
let metadata = [];
let draft = new Map();
let currentWorkspace = location.hash.slice(1) || "console";
let pendingAction = null;
let statusTimer = null;
let heartbeatTimer = null;

function showActionFeedback(message, error = false) {
  const feedback = $("jointDebugActionFeedback");
  feedback.hidden = !message;
  feedback.textContent = message;
  feedback.className = `joint-debug-action-feedback${error ? " error" : ""}`;
}

function selectedMode() {
  return document.querySelector('[name="jointDebugMode"]:checked')?.value || "upper_body";
}

function selectedUnit() {
  return document.querySelector('[name="jointDebugUnit"]:checked')?.value || "rad";
}

function unitValue(rad) {
  return selectedUnit() === "deg" ? rad * RAD_TO_DEG : rad;
}

function radValue(value) {
  return selectedUnit() === "deg" ? value / RAD_TO_DEG : value;
}

function groupControls() {
  const mode = selectedMode();
  const host = $("jointDebugGroups");
  host.dataset.mode = mode;
  host.replaceChildren();
  groups.forEach(([label, begin, end]) => {
    if (mode === "upper_body" && end <= 12) return;
    const section = document.createElement("section");
    section.className = "joint-debug-group";
    const heading = document.createElement("h3");
    heading.textContent = label;
    section.append(heading);
    metadata.filter((joint) => joint.index >= begin && joint.index < end)
      .forEach((joint) => {
        const row = document.createElement("label");
        row.className = "joint-debug-row";
        const disabled = !joint.movable;
        const value = draft.get(joint.name) ?? 0;
        const lower = unitValue(joint.lower);
        const upper = unitValue(joint.upper);
        const current = Number(window.g1LatestTelemetry?.joints?.[joint.index]?.q_rad);
        const unitName = selectedUnit();
        const currentText = Number.isFinite(current) ? unitValue(current).toFixed(unitName === "deg" ? 1 : 3) : "--";
        row.title = `#${joint.index} ${joint.name}`;
        row.innerHTML = `<span class="joint-debug-name"><strong>${joint.nameZh || joint.name}</strong><small>#${joint.index} · 当前 ${currentText} ${unitName}</small></span>`;
        const slider = document.createElement("input");
        slider.type = "range";
        slider.min = String(lower);
        slider.max = String(upper);
        slider.step = "any";
        slider.value = String(unitValue(value));
        slider.disabled = disabled;
        const number = document.createElement("input");
        number.type = "number";
        number.min = String(lower);
        number.max = String(upper);
        number.step = unitName === "deg" ? "0.1" : "0.001";
        number.value = unitValue(value).toFixed(unitName === "deg" ? 1 : 3);
        number.disabled = disabled;
        const unit = document.createElement("span");
        unit.className = "joint-debug-unit";
        unit.textContent = unitName;
        const update = (raw) => {
          const parsed = Number(raw);
          if (!Number.isFinite(parsed) || parsed < lower || parsed > upper) return;
          slider.value = String(parsed);
          number.value = parsed.toFixed(unitName === "deg" ? 1 : 3);
          draft.set(joint.name, radValue(parsed));
          setDebugJointValues(draft);
        };
        slider.addEventListener("input", () => update(slider.value));
        number.addEventListener("change", () => update(number.value));
        row.append(slider, number, unit);
        section.append(row);
      });
    host.append(section);
  });
  updatePermission();
}

function resetPose(useCurrent) {
  draft = new Map(metadata.map((joint) => {
    const current = Number(window.g1LatestTelemetry?.joints?.[joint.index]?.q_rad);
    return [joint.name, useCurrent && Number.isFinite(current) ? current : 0];
  }));
  setDebugJointValues(draft);
  groupControls();
}

function allowed() {
  if (!status || metadata.length !== 29) return false;
  return selectedMode() === "full_body"
    ? status.full_body_allowed === true
    : status.upper_body_allowed === true;
}

function updatePermission() {
  const permission = $("jointDebugPermission");
  const canApply = allowed();
  permission.textContent = canApply ? "允许下发" : "禁止下发";
  permission.className = `model-state ${canApply ? "ready" : "error"}`;
  $("jointDebugApply").disabled = !canApply;
  $("jointDebugAllowed").textContent = canApply ? "是（仍需二次确认）" : "否";
  const warning = $("jointDebugWarning");
  if (selectedMode() === "upper_body" && status?.sport_state_fresh !== true) {
    warning.textContent = errorLabels.sport_state_stale;
    warning.className = "joint-debug-warning error";
  } else if (selectedMode() === "upper_body" && status?.upper_body_fsm_allowed !== true) {
    warning.textContent = errorLabels.upper_body_fsm_not_allowed;
    warning.className = "joint-debug-warning error";
  } else if (selectedMode() === "full_body" && status?.ai_sport_active !== false) {
    warning.textContent = errorLabels.debug_mode_required;
    warning.className = "joint-debug-warning error";
  } else if (!status?.lowstate_fresh) {
    warning.textContent = errorLabels.lowstate_unavailable;
    warning.className = "joint-debug-warning error";
  } else {
    warning.textContent = "Slider 只更新数字孪生，不会实时下发。";
    warning.className = "joint-debug-warning";
  }
}

function renderTeach(next) {
  const panel = $("jointDebugTeach");
  panel.hidden = selectedMode() !== "upper_body";
  if (panel.hidden) return;
  const select = $("jointTeachSelect");
  const selected = select.value;
  select.replaceChildren();
  const actions = Array.isArray(next?.teach_actions) ? next.teach_actions : [];
  const bindingLabels = new Map((Array.isArray(next?.remote_binding_options) ? next.remote_binding_options : [])
    .map((binding) => [binding.id, binding.label || binding.id]));
  if (!actions.length) select.add(new Option("暂无本地动作", ""));
  actions.forEach((action) => {
    const mismatch = action.mode_machine !== next.mode_machine;
    const hold = action.hold_after_playback === true ? " · 播放后保持" : "";
    const remote = action.remote_binding ? ` · 遥控 ${bindingLabels.get(action.remote_binding) || action.remote_binding}` : "";
    const label = `${action.name} · ${Number(action.duration_s).toFixed(1)}s${hold}${remote}${mismatch ? " · 机型不匹配" : ""}`;
    select.add(new Option(label, action.name));
  });
  if ([...select.options].some((option) => option.value === selected)) select.value = selected;
  const state = next?.teach_state || "idle";
  const hasRecording = Boolean(next?.teach_record_name);
  const controlIdle = next?.dds_state === "idle";
  $("jointTeachName").disabled = state !== "idle" || hasRecording;
  $("jointTeachFinishMode").disabled = state === "holding" || state === "playing";
  $("jointTeachRecord").disabled = !allowed() || !controlIdle || hasRecording;
  $("jointTeachSave").disabled = !hasRecording || state === "releasing" ||
    Number(next?.teach_recorded_frames || 0) < 2;
  $("jointTeachPlay").disabled = !allowed() || !controlIdle || !select.value;
  $("jointTeachDelete").disabled = !controlIdle || !select.value;
  $("jointTeachRelease").disabled = state !== "holding";

  const selectedAction = actions.find((action) => action.name === select.value);
  const bindingSelect = $("jointTeachRemoteBinding");
  const bindingValue = bindingSelect.dataset.action === select.value
    ? bindingSelect.value
    : selectedAction?.remote_binding || "";
  const usedBindings = new Set(actions
    .filter((action) => action.name !== select.value && action.remote_binding)
    .map((action) => action.remote_binding));
  bindingSelect.replaceChildren(new Option("不绑定", ""));
  (Array.isArray(next?.remote_binding_options) ? next.remote_binding_options : []).forEach((binding) => {
    const option = new Option(binding.label || binding.id, binding.id);
    option.disabled = usedBindings.has(binding.id);
    bindingSelect.add(option);
  });
  bindingSelect.value = bindingValue;
  bindingSelect.dataset.action = select.value;
  bindingSelect.disabled = !controlIdle || !selectedAction;
  $("jointTeachBindRemote").disabled = !controlIdle || !selectedAction;
  const remoteHint = $("jointTeachRemoteHint");
  remoteHint.textContent = next?.remote_control_ready
    ? "监听 rt/wirelesscontroller；遥控器物理键使用 F1 / F3，按下组合键沿触发一次；动作保持时可单击 START 或再次按该动作绑定键恢复控制权。"
    : `遥控器监听当前不可用${next?.remote_control_error ? `：${next.remote_control_error}` : ""}；绑定仍可保存，监听恢复后生效。`;

  const hint = $("jointTeachHint");
  if (state === "releasing") {
    hint.textContent = `正在平滑把 arm_sdk weight 升到 1，接管机器人上半身控制权；接管完成后进入手掰示教并开始录制“${next.teach_record_name}”。`;
    hint.className = "active";
  } else if (state === "recording") {
    hint.textContent = `正在录制“${next.teach_record_name}” · ${next.teach_recorded_frames || 0} 帧；腰部偏航 kp=0/kd=10 可手掰，腰部横滚/俯仰使用正常 kp=300/kd=3 固定，手臂 kd=1.5、手腕 kd=0.5。`;
    hint.className = "active";
  } else if (state === "holding") {
    hint.textContent = "当前动作已结束；arm_sdk weight=1，正在按正常上肢增益保持最后一帧。保持状态不会因页面心跳超时自动释放；点击“恢复控制权”、单击遥控器 START，或再次按当前动作的遥控绑定键都可主动归还，LowState/FSM 安全互锁失效时仍会自动释放。";
    hint.className = "active";
  } else if (state === "playing") {
    hint.textContent = "已直接接管 arm_sdk，正从当前真实姿态限速过渡到动作首帧并按原始 20 Hz 轨迹播放；不会先回默认姿态。";
    hint.className = "active";
  } else {
    hint.textContent = "点击开始录制后会在首个 DDS 周期直接进入手掰状态，不再等待 weight 缓慢接管；腰部偏航 kp=0/kd=10，腰部横滚/俯仰用正常 kp=300/kd=3 固定，手臂 kp=0/kd=1.5，手腕 kp=0/kd=0.5。";
    hint.className = "";
  }
}

function renderStatus(next) {
  status = next;
  const mode = selectedMode();
  $("jointDebugCurrentMode").textContent = mode === "upper_body" ? "上半身 · arm_sdk" : "全身 · lowcmd";
  const fsmNames = { 0: "零力矩", 1: "阻尼", 500: "常规运控", 501: "常规运控", 801: "走跑运控", 802: "走跑运控" };
  $("jointDebugFsmState").textContent = next.sport_state_fresh ? `${fsmNames[next.fsm_id] || "其他模式"} · FSM ${next.fsm_id}` : "状态不可用";
  $("jointDebugSportService").textContent = next.ai_sport_found ? (next.ai_sport_active ? "未进入 / ai_sport 运行中" : "已检测 / ai_sport 已停止") : "无法检测";
  $("jointDebugDdsState").textContent = next.dds_state || "--";
  $("jointDebugLowState").textContent = next.lowstate_fresh ? "新鲜" : "不可用";
  if (next.last_error) showActionFeedback(errorLabels[next.last_error] || next.last_error, true);
  updatePermission();
  renderTeach(next);
  if (next.dds_state === "active" && !heartbeatTimer) {
    heartbeatTimer = setInterval(sendHeartbeat, 500);
  } else if (next.dds_state !== "active" && heartbeatTimer) {
    clearInterval(heartbeatTimer);
    heartbeatTimer = null;
  }
}

async function teachRequest(operation) {
  const name = operation === "save" ? "" :
    (operation === "record" ? $("jointTeachName").value.trim() : $("jointTeachSelect").value);
  if (operation !== "save" && !name) {
    showActionFeedback(errorLabels.invalid_teach_action_name, true);
    return;
  }
  const releaseControl = $("jointTeachFinishMode").value !== "hold";
  const prompts = {
    record: "确认机器人已平稳站立、周围无人员和障碍物，并由操作员手持遥控器急停。点击确认后会直接接管 arm_sdk 并进入手掰录制，不再先经过默认姿态；腰部横滚/俯仰仍按正常运动增益固定。",
    save: releaseControl
      ? "停止录制并保存，然后平滑恢复机器人上肢控制权？同名动作会被覆盖。"
      : "停止录制并保存，但继续保持 arm_sdk 控制权和最后一帧动作？之后需要点击“恢复控制权”才会主动归还。",
    play: `确认播放本地示教动作“${name}”？腰部和双臂会实际运动，请预留安全空间并手持急停。${status?.teach_actions?.find((action) => action.name === name)?.hold_after_playback ? "该动作播放结束后会继续保持最后姿态，需点击“恢复控制权”才会归还。" : ""}`,
    delete: `确认删除本地示教动作“${name}”？`,
  };
  if (!window.confirm(prompts[operation])) return;
  try {
    const body = operation === "save"
      ? { confirmed: true, release_control: releaseControl }
      : { name, confirmed: true };
    const response = await fetch(`/api/control/joint-debug/teach/${operation}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) throw new Error(result.error || `HTTP ${response.status}`);
    const messages = {
      record: "已直接接管上半身控制，可以立即开始手掰录制。",
      save: releaseControl ? "示教动作已保存并恢复控制权。" : "示教动作已保存，正在保持最后一帧动作。",
      play: "示教动作已开始播放。",
      delete: "本地示教动作已删除。",
    };
    showActionFeedback(messages[operation]);
    await refreshStatus();
  } catch (error) {
    showActionFeedback(errorLabels[error.message] || error.message, true);
  }
}

async function bindTeachRemote() {
  const name = $("jointTeachSelect").value;
  if (!name) {
    showActionFeedback(errorLabels.joint_teach_action_not_found, true);
    return;
  }
  const binding = $("jointTeachRemoteBinding").value;
  try {
    const response = await fetch("/api/control/joint-debug/teach/bind", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, binding }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) throw new Error(result.error || `HTTP ${response.status}`);
    showActionFeedback(binding
      ? `已将“${name}”绑定到遥控器 ${binding}。`
      : `已取消“${name}”的遥控器绑定。`);
    await refreshStatus();
  } catch (error) {
    showActionFeedback(errorLabels[error.message] || error.message, true);
  }
}

async function refreshStatus() {
  if (currentWorkspace !== "joint-debug") return;
  try {
    const response = await fetch("/api/control/joint-debug/status", { cache: "no-store" });
    renderStatus(await response.json());
  } catch (error) {
    $("jointDebugWarning").textContent = `调试状态读取失败：${error.message}`;
  }
}

async function sendHeartbeat() {
  if (currentWorkspace !== "joint-debug" || document.hidden || window.g1ConnectionOnline === false) return;
  try { await fetch("/api/control/joint-debug/heartbeat", { method: "POST" }); } catch (_) { /* lease expires */ }
}

function stage(action) {
  if (action === "stop" && status?.dds_state === "idle") {
    showActionFeedback("当前没有活动的调试控制，无需释放。");
    return;
  }
  pendingAction = action;
  const full = selectedMode() === "full_body";
  const restoringHold = action === "stop" && status?.teach_state === "holding";
  $("jointDebugConfirmTitle").textContent = action === "stop"
    ? (restoringHold ? "确认恢复上肢控制权" : "确认停止 / 释放调试控制")
    : `确认应用${full ? "全身" : "上半身"}关节目标`;
  $("jointDebugConfirmWarning").textContent = action === "stop"
    ? (restoringHold
        ? "将平滑把 arm_sdk weight 从 1 降到 0，停止保持最后动作并归还机器人上肢控制权。"
        : "上半身将平滑降低 weight 后停止 rt/arm_sdk；全身将发送禁用/零增益帧后停止。")
    : full ? errorLabels.debug_mode_required : "将从最新 LowState 平滑插值并直接发布 rt/arm_sdk。";
  $("jointDebugFeedback").textContent = "";
  $("jointDebugConfirmDialog").showModal();
}

async function executePending(event) {
  event.preventDefault();
  const action = pendingAction;
  if (!action) return;
  const body = action === "stop" ? { confirmed: true } : {
    mode: selectedMode(),
    confirmed: true,
    joints: metadata
      .filter((joint) => joint.movable && (selectedMode() === "full_body" || joint.index >= 12))
      .map((joint) => ({ index: joint.index, q: draft.get(joint.name) })),
  };
  $("jointDebugConfirm").disabled = true;
  $("jointDebugFeedback").textContent = action === "stop" ? "正在安全释放控制…" : "正在提交并等待后端接管…";
  try {
    const response = await fetch(`/api/control/joint-debug/${action}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) throw new Error(result.error || `HTTP ${response.status}`);
    $("jointDebugConfirmDialog").close();
    showActionFeedback(action === "stop" ? "调试控制已安全释放。" : "目标已接受，后端 DDS 控制循环已启动。");
    await refreshStatus();
  } catch (error) {
    const message = errorLabels[error.message] || error.message;
    $("jointDebugFeedback").textContent = message;
    showActionFeedback(message, true);
  } finally {
    $("jointDebugConfirm").disabled = false;
  }
}

window.addEventListener("g1:robot-model-ready", (event) => {
  metadata = event.detail;
  if (currentWorkspace === "joint-debug") resetPose(false);
});
window.addEventListener("g1:workspace-change", (event) => {
  currentWorkspace = event.detail?.workspace;
  if (currentWorkspace === "joint-debug") {
    metadata = getRobotJointMetadata();
    resetPose(false);
    refreshStatus();
    clearInterval(statusTimer);
    statusTimer = setInterval(refreshStatus, 1000);
  } else {
    setDebugJointValues(null);
    clearInterval(statusTimer);
    statusTimer = null;
  }
});
document.querySelectorAll('[name="jointDebugMode"], [name="jointDebugUnit"]').forEach((input) => {
  input.addEventListener("change", () => {
    groupControls();
    if (status) renderStatus(status);
  });
});
$("jointDebugReset").addEventListener("click", () => resetPose(false));
$("jointDebugLoadCurrent").addEventListener("click", () => resetPose(true));
$("jointDebugApply").addEventListener("click", () => stage("apply"));
$("jointDebugStop").addEventListener("click", () => stage("stop"));
$("jointTeachRecord").addEventListener("click", () => teachRequest("record"));
$("jointTeachSave").addEventListener("click", () => teachRequest("save"));
$("jointTeachPlay").addEventListener("click", () => teachRequest("play"));
$("jointTeachDelete").addEventListener("click", () => teachRequest("delete"));
$("jointTeachRelease").addEventListener("click", () => stage("stop"));
$("jointTeachBindRemote").addEventListener("click", bindTeachRemote);
$("jointTeachSelect").addEventListener("change", () => renderTeach(status));
$("jointTeachFinishMode").addEventListener("change", () => renderTeach(status));
$("jointDebugCancel").addEventListener("click", () => $("jointDebugConfirmDialog").close());
$("jointDebugConfirmForm").addEventListener("submit", executePending);

if (currentWorkspace === "joint-debug") {
  metadata = getRobotJointMetadata();
  if (metadata.length) resetPose(false);
  refreshStatus();
  statusTimer = setInterval(refreshStatus, 1000);
}
