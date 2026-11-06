import { getRobotJointMetadata, setDebugJointValues } from "/robot-viewer.js";

const $ = (id) => document.getElementById(id);
const RAD_TO_DEG = 180 / Math.PI;
const groupLabels = {
  left_leg: ["左腿", "Left Leg"],
  right_leg: ["右腿", "Right Leg"],
  waist: ["腰部", "Waist"],
  left_arm: ["左臂", "Left Arm"],
  right_arm: ["右臂", "Right Arm"],
  head: ["头部", "Head"],
  front_right_leg: ["右前腿", "Front Right Leg"],
  front_left_leg: ["左前腿", "Front Left Leg"],
  rear_right_leg: ["右后腿", "Rear Right Leg"],
  rear_left_leg: ["左后腿", "Rear Left Leg"],
};
const errorLabels = {
  debug_mode_required: ["全身 rt/lowcmd 需要先停止当前产品的高层运动服务，并进入低层调试状态。", "Full-body rt/lowcmd requires the product's high-level motion service to be stopped first."],
  joint_out_of_range: ["关节目标超出 URDF limit，后端已拒绝。", "The backend rejected a joint target outside the URDF limits."],
  joint_state_invalid: ["LowState 关节角度无效，已停止下发。", "Invalid LowState joint position; command output stopped."],
  lowstate_unavailable: ["LowState 不可用或已超时。", "LowState is unavailable or stale."],
  control_busy: ["其他控制任务正在执行。", "Another control task is running."],
  upper_body_joint_not_allowed: "上半身模式禁止下发腿部关节。",
  pr_mode_required: ["当前不是与官方 URDF 一致的 PR 模式。", "The robot is not in the PR mode matching the official URDF."],
  control_lease_expired: ["页面控制心跳已中断，后端已自动释放控制。", "The page heartbeat stopped, so the backend released control."],
  dds_publish_failed: ["DDS 发布失败，后端已自动释放控制。", "DDS publishing failed, so the backend released control."],
  service_state_unavailable: ["无法读取机器人服务状态，已禁止下发。", "The motion-service state is unavailable; command submission is disabled."],
  sport_state_stale: "运控状态不可用或已超时，已禁止上半身下发。",
  upper_body_fsm_not_allowed: "当前 FSM 不允许通过 arm_sdk 控制上肢或头部，请先切换到该产品声明的运控 FSM。",
  invalid_teach_action_name: "请输入有效的示教动作名称。",
  joint_teach_not_recording: "当前没有可保存的示教录制。",
  joint_teach_too_short: "录制时间太短，请至少录制 0.1 秒。",
  joint_teach_action_not_found: "本地示教动作不存在。",
  joint_teach_product_mismatch: "该示教动作属于其他机器人产品，已禁止播放。",
  joint_teach_variant_mismatch: "该示教动作属于其他产品变体，已禁止播放。",
  joint_teach_model_mismatch: "该动作由另一种 G1 机型录制，已禁止播放。",
  joint_teach_store_failed: "本地示教文件保存失败。",
  joint_teach_duration_limit: "录制已达到 120 秒上限，请停止并保存。",
  joint_teach_remote_binding_not_allowed: uiText(
    "该按键组合不允许用于自定义动作；只能选择页面列出的组合键。",
    "This key combination cannot trigger a custom action; choose one listed on the page.",
  ),
  joint_teach_remote_binding_in_use: "该遥控器组合键已经绑定到其他示教动作。",
  joint_teach_disabled: ["示教控制尚未启用，请先完成关节增益配置。", "Teaching control is disabled; configure joint gains first."],
  joint_teach_remote_control_unsupported: "当前产品未开放示教动作的遥控器绑定。",
  joint_teach_remote_protocol_unconfirmed: ["A2 遥控器原始数据协议尚未确认。", "A2 raw remote protocol has not been confirmed."],
  motion_service_busy: "高层运控正在切换，请稍后重试。",
  motion_mode_not_supported: "当前固件不支持该高层运控模式。",
  motion_service_switch_failed: "高层运控切换失败，后端已停止后续动作。",
  motion_service_confirmation_timeout: "高层运控切换后未能及时确认状态，后端已停止后续动作。",
  motion_mode_switch_unsupported: ["当前产品未开放网页高低层模式切换。", "Web-based high/low-level mode switching is unavailable for this product."],
  fsm_confirmation_timeout: "机器人未在预期时间内到达目标姿态，后端已停止后续动作。",
  recovery_stand_confirmation_timeout: uiText(
    "机器人未在预期时间内稳定站起，已停止切换到灵动模式。",
    "The robot did not reach a stable standing pose in time; switching to Free Walk was stopped.",
  ),
};

let status = null;
let metadata = [];
let draft = new Map();
let currentWorkspace = location.hash.slice(1) || "console";
let pendingAction = null;
let statusTimer = null;
let heartbeatTimer = null;
let modeSwitchPending = false;

function uiText(zh, en = zh) {
  return window.UiI18n?.language === "en" ? en : zh;
}

function errorText(code) {
  const label = errorLabels[code];
  return Array.isArray(label) ? uiText(label[0], label[1]) : label;
}

function showActionFeedback(message, error = false) {
  const feedback = $("jointDebugActionFeedback");
  feedback.hidden = !message;
  feedback.textContent = message;
  feedback.className = `joint-debug-action-feedback${error ? " error" : ""}`;
}

function selectedMode() {
  return document.querySelector('[name="jointDebugMode"]:checked')?.value || "full_body";
}

function selectedUnit() {
  return document.querySelector('[name="jointDebugUnit"]:checked')?.value || "rad";
}

function renderControlTopics(next) {
  const inputs = [...document.querySelectorAll('[name="jointDebugMode"]')];
  const supported = [];
  inputs.forEach((input) => {
    const topic = next?.control_topics?.[input.value];
    const label = input.closest("label");
    label.hidden = !topic;
    if (topic) supported.push(input);
    const topicLabel = label?.querySelector("small");
    if (topicLabel) topicLabel.textContent = topic;
  });
  if (!supported.some((input) => input.checked) && supported.length) {
    supported[0].checked = true;
  }
  const modeSwitch = document.querySelector(".joint-debug-mode-switch");
  if (modeSwitch) modeSwitch.hidden = supported.length <= 1;
}

function unitValue(rad) {
  return selectedUnit() === "deg" ? rad * RAD_TO_DEG : rad;
}

function radValue(value) {
  return selectedUnit() === "deg" ? value / RAD_TO_DEG : value;
}

function jointInMode(joint, mode) {
  return Array.isArray(joint.controlModes) && joint.controlModes.includes(mode);
}

function groupControls() {
  const mode = selectedMode();
  const host = $("jointDebugGroups");
  host.dataset.mode = mode;
  host.dataset.hasHead = String(metadata.some((joint) =>
    joint.displayGroup === "head" && jointInMode(joint, mode)
  ));
  host.replaceChildren();
  const orderedGroups = [...new Set(metadata.map((joint) => joint.displayGroup).filter(Boolean))];
  const visibleGroups = orderedGroups.filter((group) => metadata.some((joint) =>
    joint.displayGroup === group && jointInMode(joint, mode)
  ));
  const quadrupedGroups = [
    "front_left_leg", "front_right_leg", "rear_left_leg", "rear_right_leg",
  ];
  host.dataset.layout = quadrupedGroups.every((group) => visibleGroups.includes(group))
    ? "quadruped"
    : "default";
  orderedGroups.forEach((group) => {
    const groupJoints = metadata.filter((joint) => joint.displayGroup === group);
    if (!groupJoints.some((joint) => jointInMode(joint, mode))) return;
    const section = document.createElement("section");
    section.className = "joint-debug-group";
    section.dataset.group = group;
    const heading = document.createElement("h3");
    const label = groupLabels[group];
    heading.textContent = label ? uiText(label[0], label[1]) : group.replaceAll("_", " ");
    section.append(heading);
    groupJoints.filter((joint) => jointInMode(joint, mode))
      .forEach((joint) => {
        const row = document.createElement("label");
        row.className = "joint-debug-row";
        const disabled = !joint.movable;
        const value = draft.get(joint.name) ?? 0;
        const lower = unitValue(joint.lower);
        const upper = unitValue(joint.upper);
        const unitName = selectedUnit();
        const precision = unitName === "deg" ? 1 : 3;
        row.dataset.index = joint.index;
        row.title = `#${joint.index} ${joint.name} · URDF ${lower.toFixed(precision)}–${upper.toFixed(precision)} ${unitName}`;
        const jointLabel = window.UiI18n?.language === "en"
          ? joint.name
          : joint.nameZh || joint.name;
        row.innerHTML = `<span class="joint-debug-name"><strong>${jointLabel}</strong><small class="joint-debug-current"></small><small class="joint-debug-limit">URDF ${uiText("限位", "limits")} ${lower.toFixed(precision)}–${upper.toFixed(precision)} ${unitName}</small></span>`;
        const slider = document.createElement("input");
        slider.type = "range";
        slider.min = String(lower);
        slider.max = String(upper);
        slider.step = "any";
        slider.value = String(unitValue(value));
        slider.disabled = disabled;
        slider.setAttribute("aria-label", uiText(`${jointLabel}目标滑块`, `${jointLabel} target slider`));
        const number = document.createElement("input");
        number.type = "number";
        number.min = String(lower);
        number.max = String(upper);
        number.step = unitName === "deg" ? "0.1" : "0.001";
        number.value = unitValue(value).toFixed(unitName === "deg" ? 1 : 3);
        number.disabled = disabled;
        number.setAttribute("aria-label", uiText(`${jointLabel}目标数值`, `${jointLabel} target value`));
        const unit = document.createElement("span");
        unit.className = "joint-debug-unit";
        unit.textContent = unitName;
        const update = (raw, finish = false) => {
          let parsed = raw.trim() === "" ? NaN : Number(raw);
          if (finish) {
            parsed = Number.isFinite(parsed)
              ? Math.min(upper, Math.max(lower, parsed))
              : unitValue(draft.get(joint.name) ?? value);
          }
          if (!Number.isFinite(parsed) || parsed < lower || parsed > upper) return;
          slider.value = String(parsed);
          if (finish) number.value = parsed.toFixed(precision);
          draft.set(joint.name, radValue(parsed));
          setDebugJointValues(draft);
        };
        slider.addEventListener("input", () => update(slider.value, true));
        number.addEventListener("input", () => update(number.value));
        number.addEventListener("change", () => update(number.value, true));
        number.addEventListener("blur", () => update(number.value, true));
        row.append(slider, number, unit);
        section.append(row);
      });
    host.append(section);
  });
  updateCurrentPose();
  updatePermission();
}

function updateCurrentPose() {
  document.querySelectorAll("#jointDebugGroups .joint-debug-row").forEach((row) => {
    const index = Number(row.dataset.index);
    const current = status?.lowstate_fresh === true ? status.joints?.[index]?.current : null;
    const value = typeof current === "number" && Number.isFinite(current)
      ? unitValue(current).toFixed(selectedUnit() === "deg" ? 1 : 3) : "--";
    row.querySelector(".joint-debug-current").textContent =
      `#${index} · ${uiText("当前", "Current")} ${value} ${selectedUnit()}`;
  });
}

function resetPose(useCurrent) {
  draft = new Map(metadata.map((joint) => {
    const current = status?.lowstate_fresh === true
      ? status.joints?.[joint.index]?.current : null;
    const target = useCurrent && Number.isFinite(current) ? current : 0;
    return [joint.name, Math.min(joint.upper, Math.max(joint.lower, target))];
  }));
  setDebugJointValues(draft);
  groupControls();
}

function allowed() {
  if (!status || !Array.isArray(status.joints) ||
      metadata.length !== status.joints.length) return false;
  return status[`${selectedMode()}_allowed`] === true;
}

function updatePermission() {
  const permission = $("jointDebugPermission");
  const canApply = allowed();
  permission.textContent = canApply
    ? uiText("允许下发", "Command allowed")
    : uiText("禁止下发", "Command blocked");
  permission.className = `model-state ${canApply ? "ready" : "error"}`;
  $("jointDebugApply").disabled = !canApply;
  $("jointDebugLoadCurrent").disabled = status?.lowstate_fresh !== true;
  $("jointDebugStop").disabled = !status?.dds_state || status.dds_state === "idle";
  const switchSupported = status?.motion_mode_switch_supported === true;
  const serviceActive = status?.motion_service_active ?? status?.ai_sport_active;
  const controlIdle = status?.dds_state === "idle";
  $("jointDebugEnterLowLevel").hidden = !switchSupported;
  $("jointDebugExitLowLevel").hidden = !switchSupported;
  $("jointDebugEnterLowLevel").disabled = modeSwitchPending || !controlIdle || serviceActive !== true;
  $("jointDebugExitLowLevel").disabled = modeSwitchPending || serviceActive !== false;
  const gainHint = $("jointDebugGainHint");
  const kp = Number(status?.position_kp);
  const kd = Number(status?.position_kd);
  gainHint.hidden = !switchSupported || !Number.isFinite(kp) || !Number.isFinite(kd);
  if (!gainHint.hidden) gainHint.textContent = uiText(
    `官方站立增益：KP ${kp} / KD ${kd}`,
    `Official stand gains: KP ${kp} / KD ${kd}`
  );
  $("jointDebugAllowed").textContent = canApply
    ? uiText("是（仍需二次确认）", "Yes (confirmation still required)")
    : uiText("否", "No");
  const warning = $("jointDebugWarning");
  const topic = status?.control_topics?.[selectedMode()];
  if (status?.initialized !== true) {
    const capability = window.robotManifest?.capabilities?.find((item) => item.key === "joint_debug");
    warning.textContent = uiText(
      capability?.parameters?.reason_zh || "关节控制当前不可用；可使用滑块预览目标姿态。",
      capability?.parameters?.reason_en || "Joint control is unavailable; use the sliders to preview target poses.",
    );
    warning.className = "joint-debug-warning error";
  } else if (topic === "rt/arm_sdk" && status?.sport_state_fresh !== true) {
    warning.textContent = errorText("sport_state_stale");
    warning.className = "joint-debug-warning error";
  } else if (topic === "rt/arm_sdk" && status?.upper_body_fsm_allowed !== true) {
    warning.textContent = errorText("upper_body_fsm_not_allowed");
    warning.className = "joint-debug-warning error";
  } else if (topic === "rt/lowcmd" &&
             (status?.motion_service_active ?? status?.ai_sport_active) !== false) {
    warning.textContent = errorText("debug_mode_required");
    warning.className = "joint-debug-warning error";
  } else if (!status?.lowstate_fresh) {
    warning.textContent = errorText("lowstate_unavailable");
    warning.className = "joint-debug-warning error";
  } else {
    warning.textContent = uiText(
      "Slider 只更新数字孪生，不会实时下发。",
      "Sliders update only the digital twin; no command is sent until Apply.",
    );
    warning.className = "joint-debug-warning";
  }
}

function renderTeach(next) {
  const panel = $("jointDebugTeach");
  const capability = window.robotManifest?.capabilities?.find((item) => item.key === "joint_teach");
  const teachMode = next?.teach_mode || capability?.parameters?.control_mode || "upper_body";
  const teachAvailable = panel.dataset.capabilityState === "available";
  const teachTopic = next?.teach_control_topic || "rt/arm_sdk";
  panel.hidden = capability?.implemented !== true || selectedMode() !== teachMode;
  if (panel.hidden) return;
  const select = $("jointTeachSelect");
  const selected = select.value;
  select.replaceChildren();
  const actions = Array.isArray(next?.teach_actions) ? next.teach_actions : [];
  const bindingLabels = new Map((Array.isArray(next?.remote_binding_options) ? next.remote_binding_options : [])
    .map((binding) => [binding.id, binding.label || binding.id]));
  if (!actions.length) select.add(new Option("暂无本地动作", ""));
  actions.forEach((action) => {
    const mismatch = action.product_id !== next.product_id ||
      action.variant !== next.variant || action.mode_machine !== next.mode_machine;
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
  if (!teachAvailable) {
    ["jointTeachRecord", "jointTeachSave", "jointTeachPlay", "jointTeachDelete", "jointTeachRelease"]
      .forEach((id) => { $(id).disabled = true; });
  }

  const selectedAction = actions.find((action) => action.name === select.value);
  const remoteSupported = next?.remote_binding_supported === true;
  const remoteControls = $("jointTeachRemoteControls");
  if (remoteControls) remoteControls.hidden = capability?.parameters?.remote_binding !== "true" && !remoteSupported;
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
  bindingSelect.disabled = !teachAvailable || !remoteSupported || !controlIdle || !selectedAction;
  $("jointTeachBindRemote").disabled = !teachAvailable || !remoteSupported || !controlIdle || !selectedAction;
  const remoteHint = $("jointTeachRemoteHint");
  remoteHint.textContent = !remoteSupported
    ? (errorText(next?.remote_control_error) || uiText("遥控器示教绑定暂不可用。", "Remote taught-action binding is currently unavailable."))
    : next?.remote_control_ready
      ? uiText(
        `遥控器订阅已建立；需先停止高层运控，再按绑定组合键触发一次。按 START 可停止当前调试并释放控制；动作保持时也可再次按绑定键释放。`,
        `Remote subscribed. Stop high-level motion control before pressing a binding; press START to stop debug control and release it, or press the binding again to release a held action.`,
      )
      : uiText(
        `遥控器监听当前不可用${next?.remote_control_error ? `：${next.remote_control_error}` : ""}；绑定仍可保存，监听恢复后生效。`,
        `Remote listening is unavailable${next?.remote_control_error ? `: ${next.remote_control_error}` : ""}; bindings remain saved and take effect after recovery.`,
      );


  if (remoteSupported && next?.remote_control_ready) {
    remoteHint.textContent += uiText(
      ` 已收 ${next.remote_message_count || 0} 条，键值 ${next.remote_keys || 0}。`,
      ` Received ${next.remote_message_count || 0} messages; keys ${next.remote_keys || 0}.`,
    );
    if (next.remote_last_binding) remoteHint.textContent += uiText(
      ` 最近触发：${next.remote_last_binding} → ${next.remote_last_action}。`,
      ` Last trigger: ${next.remote_last_binding} → ${next.remote_last_action}.`,
    );
    if (next.last_error) remoteHint.textContent += ` ${errorText(next.last_error)}`;
  }

  const hint = $("jointTeachHint");
  if (!teachAvailable) {
    hint.textContent = uiText("示教当前不可用，请查看上述原因。", "Teaching is currently unavailable; see the reason above.");
    hint.className = "";
  } else if (state === "releasing") {
    hint.textContent = uiText(
      `正在通过 ${teachTopic} 平滑接管示教关节；完成后开始录制“${next.teach_record_name}”。`,
      `Taking over taught joints through ${teachTopic}; recording “${next.teach_record_name}” starts when ready.`,
    );
    hint.className = "active";
  } else if (state === "recording") {
    hint.textContent = `正在录制“${next.teach_record_name}” · ${next.teach_recorded_frames || 0} 帧；后端按当前产品的示教增益保持阻尼并持续采样真实 LowState。`;
    hint.className = "active";
  } else if (state === "holding") {
    hint.textContent = uiText(
      `动作已结束，正通过 ${teachTopic} 保持最后一帧。点击“恢复控制权”、单击遥控器 START，或再次按绑定键可归还控制；安全互锁失效时仍会自动释放。`,
      `The final frame is held through ${teachTopic}. Click Release Control, press START, or press the binding again to release; safety interlock failures still release automatically.`,
    );
    hint.className = "active";
  } else if (state === "playing") {
    hint.textContent = uiText(
      `已通过 ${teachTopic} 接管，正从当前真实姿态限速过渡到动作首帧，并按 20 Hz 轨迹播放。`,
      `Control is active through ${teachTopic}; transitioning from the live pose to the first frame and replaying at 20 Hz.`,
    );
    hint.className = "active";
  } else {
    hint.textContent = uiText(
      `开始录制后会按当前产品声明的 ${teachTopic} 关节集合进入手掰状态；录制、播放和释放均使用安全互锁与限速插值。`,
      `Recording uses the product-declared ${teachTopic} joint set; recording, playback, and release retain safety interlocks and velocity limiting.`,
    );
    hint.className = "";
  }
}

function renderStatus(next) {
  status = next;
  let schemaChanged = false;
  const signature = (items) => items.map((joint) =>
    `${joint.index}:${joint.name}:${joint.displayGroup || ""}:${(joint.controlModes || []).join(",")}:${joint.movable === true}:${joint.lower}:${joint.upper}`
  ).join("|");
  if (Array.isArray(next?.joints) && next.joints.length) {
    const nextMetadata = next.joints.map((joint) => ({
      index: Number(joint.index),
      name: joint.name,
      nameZh: joint.name_zh,
      displayGroup: joint.display_group,
      upperBody: joint.upper_body === true,
      controlModes: Array.isArray(joint.control_modes) ? joint.control_modes : [],
      movable: joint.movable === true,
      lower: Number(joint.lower),
      upper: Number(joint.upper),
    }));
    schemaChanged = signature(metadata) !== signature(nextMetadata);
    metadata = nextMetadata;
  }
  if (!next?.joints?.length) {
    const fallbackMetadata = getRobotJointMetadata();
    schemaChanged = signature(metadata) !== signature(fallbackMetadata);
    metadata = fallbackMetadata;
    const modes = [...new Set(metadata.flatMap((joint) => joint.controlModes || []))];
    renderControlTopics({ control_topics: Object.fromEntries(modes.map((mode) => [mode, "—"])) });
  } else renderControlTopics(next);
  const mode = selectedMode();
  const modeLabels = {
    upper_body: uiText("上半身", "Upper body"),
    full_body: uiText("全身", "Full body"),
  };
  $("jointDebugCurrentMode").textContent = `${modeLabels[mode] || mode} · ${next?.control_topics?.[mode] || "--"}`;
  const fsmNames = {
    0: uiText("零力矩", "Zero torque"),
    1: uiText("阻尼", "Damping"),
    4: uiText("锁定站立", "Locked stand"),
    500: uiText("常规运控", "Standard motion"),
    501: uiText("常规运控", "Standard motion"),
    701: uiText("躺起", "Lie-to-stand"),
    702: uiText("姿态切换", "Posture transition"),
    801: uiText("走跑运控", "Walk/run control"),
    802: uiText("走跑运控", "Walk/run control"),
    811: uiText("主运控", "Main motion control"),
  };
  const stateLabels = window.robotManifest?.capabilities?.find(
    (capability) => capability.key === "locomotion")?.parameters?.state_labels || "";
  const stateEntry = stateLabels.split(";").find((entry) => entry.startsWith(`${next.fsm_id}:`));
  const profileNames = stateEntry?.slice(stateEntry.indexOf(":") + 1).split("|");
  const fsmName = profileNames
    ? uiText(profileNames[0], profileNames[1] || profileNames[0])
    : fsmNames[next.fsm_id] || uiText("其他模式", "Other mode");
  $("jointDebugFsmState").textContent = next.sport_state_fresh
    ? `${fsmName} · FSM ${next.fsm_id}`
    : uiText("状态不可用", "Unavailable");
  const serviceFound = next.motion_service_found ?? next.ai_sport_found;
  const serviceActive = next.motion_service_active ?? next.ai_sport_active;
  const serviceNames = Array.isArray(next.motion_service_names)
    ? next.motion_service_names.join(" / ")
    : "ai_sport";
  const activeService = next.active_motion_service || serviceNames;
  $("jointDebugSportService").textContent = serviceFound
    ? serviceActive
      ? uiText(`未进入 / ${activeService} 运行中`, `Blocked / ${activeService} is active`)
      : uiText("已检测 / 高层运动服务已停止", "Ready / high-level motion service stopped")
    : uiText("无法检测", "Unavailable");
  const ddsStates = {
    idle: uiText("空闲", "Idle"),
    active: uiText("控制中", "Active"),
    stopping: uiText("停止中", "Stopping"),
    releasing: uiText("释放中", "Releasing"),
    released: uiText("已释放", "Released"),
  };
  $("jointDebugDdsState").textContent = ddsStates[next.dds_state] || next.dds_state || "--";
  $("jointDebugLowState").textContent = next.lowstate_fresh
    ? uiText("新鲜", "Fresh")
    : uiText("不可用", "Unavailable");
  if (next.initialized !== true) {
    const unavailable = uiText("关节控制未就绪", "Joint control not ready");
    $("jointDebugSportService").textContent = unavailable;
    $("jointDebugDdsState").textContent = unavailable;
    $("jointDebugLowState").textContent = uiText("未提供调试状态", "Debug status not provided");
    $("jointDebugFsmState").textContent = uiText("请查看综合工作台", "See the workstation");
  }
  if (next.last_error) showActionFeedback(errorText(next.last_error) || next.last_error, true);
  if (schemaChanged) groupControls();
  else updatePermission();
  updateCurrentPose();
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
  const teachMode = status?.teach_mode || "upper_body";
  const teachTopic = status?.teach_control_topic || "rt/arm_sdk";
  const fullBodyWarning = teachMode === "full_body"
    ? uiText("机器人必须可靠悬挂或支撑，", "The robot must be securely suspended or supported, ")
    : "";
  const prompts = {
    record: uiText(
      `确认${fullBodyWarning}周围无人员和障碍物，并由操作员手持遥控器急停。确认后会通过 ${teachTopic} 进入手掰录制，不先回默认姿态。`,
      `${fullBodyWarning}clear people and obstacles, and keep the remote emergency stop ready. Confirm to enter hand-guided recording through ${teachTopic} without first returning to a default pose.`,
    ),
    save: releaseControl
      ? uiText("停止录制并保存，然后释放当前示教控制？同名动作会被覆盖。", "Stop and save, then release taught-action control? An action with the same name will be replaced.")
      : uiText(`停止录制并保存，但继续通过 ${teachTopic} 保持最后一帧？之后需点击“恢复控制权”才会归还。`, `Stop and save, but keep the final frame through ${teachTopic}? Use “Release Control” afterward.`),
    play: `确认播放本地示教动作“${name}”？示教轨迹中的关节会实际运动，请预留安全空间并手持急停。${status?.teach_actions?.find((action) => action.name === name)?.hold_after_playback ? "该动作播放结束后会继续保持最后姿态，需点击“恢复控制权”才会归还。" : ""}`,
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
      record: uiText(`已通过 ${teachTopic} 接管示教关节，可开始手掰录制。`, `Taught joints are controlled through ${teachTopic}; hand-guided recording can begin.`),
      save: releaseControl ? "示教动作已保存并恢复控制权。" : "示教动作已保存，正在保持最后一帧动作。",
      play: "示教动作已开始播放。",
      delete: "本地示教动作已删除。",
    };
    showActionFeedback(messages[operation]);
    await refreshStatus();
  } catch (error) {
    showActionFeedback(errorText(error.message) || error.message, true);
  }
}

async function bindTeachRemote() {
  const name = $("jointTeachSelect").value;
  if (!name) {
    showActionFeedback(errorLabels.joint_teach_action_not_found, true);
    return;
  }
  const binding = $("jointTeachRemoteBinding").value;
  const bindingLabel = $("jointTeachRemoteBinding").selectedOptions[0]?.textContent || binding;
  try {
    const response = await fetch("/api/control/joint-debug/teach/bind", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, binding }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) throw new Error(result.error || `HTTP ${response.status}`);
    showActionFeedback(binding
      ? `已将“${name}”绑定到遥控器 ${bindingLabel}。`
      : `已取消“${name}”的遥控器绑定。`);
    await refreshStatus();
  } catch (error) {
    showActionFeedback(errorText(error.message) || error.message, true);
  }
}

async function refreshStatus() {
  if (currentWorkspace !== "joint-debug") return;
  try {
    const response = await fetch("/api/control/joint-debug/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    renderStatus(await response.json());
  } catch (error) {
    status = null;
    if (heartbeatTimer) clearInterval(heartbeatTimer);
    heartbeatTimer = null;
    updatePermission();
    updateCurrentPose();
    $("jointDebugWarning").textContent = uiText(
      `调试状态读取失败：${error.message}`,
      `Failed to read debug status: ${error.message}`,
    );
    $("jointDebugWarning").className = "joint-debug-warning error";
  }
}

async function sendHeartbeat() {
  if (currentWorkspace !== "joint-debug" || document.hidden || window.g1ConnectionOnline === false) return;
  try { await fetch("/api/control/joint-debug/heartbeat", { method: "POST" }); } catch (_) { /* lease expires */ }
}

function stage(action) {
  if (action === "stop" && status?.dds_state === "idle") {
    showActionFeedback(uiText(
      "当前没有活动的调试控制，无需释放。",
      "No active debug control needs to be released.",
    ));
    return;
  }
  pendingAction = action;
  const enteringLowLevel = action === "enter-low-level";
  const exitingLowLevel = action === "exit-low-level";
  const modeLabels = {
    upper_body: uiText("上半身", "upper-body"),
    full_body: uiText("全身", "full-body"),
  };
  const selectedLabel = modeLabels[selectedMode()] || selectedMode();
  const restoringHold = action === "stop" && status?.teach_state === "holding";
  const activeMode = status?.mode || selectedMode();
  const activeTopic = status?.control_topics?.[activeMode];
  const selectedTopic = status?.control_topics?.[selectedMode()];
  $("jointDebugConfirmTitle").textContent = enteringLowLevel
    ? uiText("确认进入底层模式", "Confirm low-level mode entry")
    : exitingLowLevel
      ? uiText("确认退出底层模式", "Confirm low-level mode exit")
      : action === "stop"
    ? (restoringHold
        ? uiText("确认恢复机器人控制权", "Confirm control release")
        : uiText("确认停止 / 释放调试控制", "Confirm stop / release debug control"))
    : uiText(`确认应用${selectedLabel}关节目标`, `Confirm ${selectedLabel} joint targets`);
  $("jointDebugConfirmWarning").textContent = enteringLowLevel
    ? uiText(
      "机器人将先自动趴下并确认安全姿态，再关闭 MCF 高层运控。切换完成前请勿触碰机器人或发送其他控制指令。",
      "The robot will crouch and confirm its pose before disabling MCF. Do not touch it or send other commands during the transition.",
    )
    : exitingLowLevel
      ? uiText(
        "后端将先停止 LowCmd，再启动 MCF，通过恢复站立动作起身，最后进入灵动模式。请清空周围人员和障碍物。",
        "The backend will stop LowCmd, restart MCF, recover standing, then enter Free Walk. Clear the area first.",
      )
      : action === "stop"
    ? (restoringHold
        ? activeTopic === "rt/lowcmd"
          ? uiText("将发送禁用/零增益帧，停止保持最后动作并释放 LowCmd 控制。", "A disabled, zero-gain frame will stop the final-pose hold and release LowCmd control.")
          : "将平滑降低 arm_sdk weight，停止保持最后动作并归还控制权。"
        : activeTopic === "rt/lowcmd"
          ? uiText("将发送禁用/零增益帧并停止 rt/lowcmd，随后释放调试控制资源。", "A disabled, zero-gain frame will stop rt/lowcmd before debug-control resources are released.")
          : activeTopic === "rt/arm_sdk"
            ? "将平滑降低 weight 后停止 rt/arm_sdk，随后释放调试控制资源。"
            : "将停止当前 DDS 调试输出并释放控制资源。")
    : selectedTopic === "rt/lowcmd"
      ? uiText(
        "将从最新 LowState 平滑插值并通过 rt/lowcmd 应用关节目标。",
        "Joint targets will be interpolated from the latest LowState and applied through rt/lowcmd.",
      )
      : selectedTopic === "rt/arm_sdk"
        ? "将从最新 LowState 平滑插值并直接发布 rt/arm_sdk。"
        : "后端未声明该模式的控制通道，已禁止下发。";
  $("jointDebugFeedback").textContent = "";
  $("jointDebugConfirmDialog").showModal();
}

async function executePending(event) {
  event.preventDefault();
  const action = pendingAction;
  if (!action) return;
  const modeSwitch = action === "enter-low-level" || action === "exit-low-level";
  const body = modeSwitch ? {
    mode: action === "enter-low-level" ? "low_level" : "high_level",
    confirmed: true,
  } : action === "stop" ? { confirmed: true } : {
    mode: selectedMode(),
    confirmed: true,
    joints: metadata
      .filter((joint) => joint.movable && jointInMode(joint, selectedMode()))
      .map((joint) => ({ index: joint.index, q: draft.get(joint.name) })),
  };
  $("jointDebugConfirm").disabled = true;
  modeSwitchPending = modeSwitch;
  updatePermission();
  $("jointDebugFeedback").textContent = modeSwitch
    ? (action === "enter-low-level"
        ? uiText("正在自动趴下并关闭 MCF…", "Crouching and disabling MCF…")
        : uiText("正在启动 MCF、恢复站立并进入灵动模式…", "Starting MCF and recovering standing…"))
    : action === "stop"
      ? uiText("正在安全释放控制…", "Releasing control safely…")
      : uiText("正在提交并等待后端接管…", "Submitting and waiting for backend control…");
  try {
    const endpoint = modeSwitch ? "mode" : action;
    const response = await fetch(`/api/control/joint-debug/${endpoint}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) throw new Error(result.error || `HTTP ${response.status}`);
    $("jointDebugConfirmDialog").close();
    showActionFeedback(modeSwitch
      ? (action === "enter-low-level"
          ? uiText("机器人已趴下，MCF 已关闭，可以开始底层调试。", "The robot is crouched and MCF is off; low-level debugging is ready.")
          : uiText("MCF 已恢复，机器人已站起并进入灵动模式。", "MCF is restored and the robot is standing in Free Walk mode."))
      : action === "stop"
        ? uiText("调试控制已安全释放。", "Debug control was released safely.")
        : uiText("目标已接受，后端 DDS 控制循环已启动。", "Targets accepted; the backend DDS control loop is running."));
    await refreshStatus();
  } catch (error) {
    const message = errorText(error.message) || error.message;
    $("jointDebugFeedback").textContent = message;
    showActionFeedback(message, true);
  } finally {
    modeSwitchPending = false;
    $("jointDebugConfirm").disabled = false;
    updatePermission();
  }
}

window.addEventListener("g1:robot-model-ready", (event) => {
  if (status?.joints?.length) return;
  if (status) {
    renderStatus(status);
  } else {
    metadata = event.detail;
    if (currentWorkspace === "joint-debug") groupControls();
  }
});
window.addEventListener("ui-language-change", () => {
  if (currentWorkspace !== "joint-debug") return;
  groupControls();
  if (status) renderStatus(status);
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
$("jointDebugEnterLowLevel").addEventListener("click", () => stage("enter-low-level"));
$("jointDebugExitLowLevel").addEventListener("click", () => stage("exit-low-level"));
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
