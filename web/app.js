"use strict";

const $ = (id) => document.getElementById(id);
const SAVED_API_KEY_MASK = "••••••••••••";

const sourceLabels = {
  low_state: "低层状态",
  bms: "电池 BMS",
  secondary_imu: "躯干 IMU",
  mainboard: "主板状态",
  odometry: "里程计",
};

const statusLabels = {
  online: "在线",
  delayed: "延迟",
  offline: "离线",
};

const motorStateFaults = [
  [0x00000001, "过流"],
  [0x00000002, "瞬态过压"],
  [0x00000004, "持续过压"],
  [0x00000008, "瞬态欠压"],
  [0x00000010, "芯片过热"],
  [0x00000020, "MOS 过热/过冷"],
  [0x00000040, "MOS 温度异常"],
  [0x00000080, "壳体过热/过冷"],
  [0x00000100, "壳体温度异常"],
  [0x00000200, "绕组过热"],
  [0x00000400, "转子编码器 1 错误"],
  [0x00000800, "转子编码器 2 错误"],
  [0x00001000, "输出编码器错误"],
  [0x00002000, "标定/BOOT 数据错误"],
  [0x00004000, "异常复位"],
  [0x00008000, "电机锁定，主控认证错误"],
  [0x00010000, "芯片验证错误"],
  [0x00020000, "标定模式警告"],
  [0x00040000, "通信校验错误"],
  [0x00080000, "驱动版本过低"],
  [0x40000000, "电机端判断，PC 连接超时"],
  [0x80000000, "PC 端判读，电机断联超时"],
];

const appState = {
  socket: null,
  reconnectDelay: 1000,
  reconnectTimer: null,
  lastMessageAt: 0,
  lastData: null,
  jointRows: new Map(),
  asrHistory: [],
  lastAsrKey: "",
  voiceReady: false,
  ttsBusy: false,
  asrRequestInFlight: false,
  volumeRequestInFlight: false,
  volumeDraft: false,
  volumePointerId: null,
  volumeHoldUntil: 0,
  llmModeRequestInFlight: false,
  llmChatRequestInFlight: false,
  llmModeDraft: null,
  llmConfigDirty: false,
  customerVoiceConfigLoaded: false,
  customerVoiceConfigDirty: false,
  customerVoiceConfigRequestInFlight: false,
  customerConfigPersistenceAvailable: false,
  customerQaReplaceAvailable: false,
  lastCustomerConfigSyncAt: 0,
  llmActiveMode: null,
  llmModeAwaitingTelemetry: null,
  llmInputMode: "text",
  voiceTab: "llm",
  controlTab: "mode",
  lastLlmResponse: "",
  teachActionsKey: "",
  controlUnlocked: false,
  controlData: null,
  pendingControl: null,
  motionEnabled: false,
  keyboardMotionKeys: new Set(),
  pointerMotionKeys: new Map(),
  motionRequestInFlight: false,
  pendingMotionRequest: null,
  motionFsmFamily: null,
  motionPreset: {
    label: "低速",
    speedMode: 0,
    forwardSpeed: 0.5,
    lateralSpeed: 0.2,
    yawSpeed: 0.5,
  },
};

function number(value, digits = 3) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) {
    return "--";
  }
  return Number(value).toFixed(digits);
}

function integer(value) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) {
    return "--";
  }
  return String(Math.trunc(Number(value)));
}

function arrayText(values, digits = 3) {
  if (!Array.isArray(values)) return "--";
  return values.map((value) => number(value, digits)).join(" / ");
}

function setText(id, value) {
  const element = $(id);
  if (element) element.textContent = value;
}

function setConnection(kind, label, detail) {
  const pill = $("connectionPill");
  pill.classList.remove("online", "offline");
  if (kind) pill.classList.add(kind);
  setText("connectionLabel", label);
  setText("connectionDetail", detail);
  setText("footerStatus", `${label} · ${detail}`);
  if (kind === "online" || kind === "offline") {
    window.g1ConnectionOnline = kind === "online";
    window.dispatchEvent(
      new CustomEvent("g1:connection", {
        detail: { online: kind === "online", label, detail },
      }),
    );
  }
}

function connect() {
  clearTimeout(appState.reconnectTimer);
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const url = `${protocol}//${location.host}/ws/telemetry`;
  setConnection("", "正在连接", "建立 WebSocket");

  let socket;
  try {
    socket = new WebSocket(url);
  } catch (error) {
    scheduleReconnect();
    return;
  }
  appState.socket = socket;

  socket.addEventListener("open", () => {
    appState.reconnectDelay = 1000;
    setConnection("online", "遥测已连接", "只读 WebSocket");
    updateControlButtons();
  });

  socket.addEventListener("message", (event) => {
    try {
      const data = JSON.parse(event.data);
      appState.lastMessageAt = Date.now();
      appState.lastData = data;
      $("staleBanner").hidden = true;
      render(data);
    } catch (error) {
      console.error("无法解析遥测数据", error);
    }
  });

  socket.addEventListener("close", () => {
    if (appState.socket === socket) appState.socket = null;
    if (appState.motionEnabled) {
      disableMotionControl("遥测连接断开，已自动停止运动");
    }
    setConnection("offline", "连接已断开", "正在自动重连");
    $("staleBanner").hidden = false;
    updateControlButtons();
    scheduleReconnect();
  });

  socket.addEventListener("error", () => socket.close());
}

function scheduleReconnect() {
  clearTimeout(appState.reconnectTimer);
  appState.reconnectTimer = setTimeout(connect, appState.reconnectDelay);
  appState.reconnectDelay = Math.min(appState.reconnectDelay * 2, 10000);
}

function render(data) {
  window.g1LatestTelemetry = data;
  const robot = data.robot || {};
  const sources = data.sources || {};

  setText("sequence", integer(data.sequence));
  setText("modeMachine", integer(robot.mode_machine_raw));
  setText("modePr", `mode_pr ${integer(robot.mode_pr_raw)}`);
  setText(
    "robotVersion",
    Array.isArray(robot.version) ? robot.version.join(".") : "--",
  );
  setText("robotTick", `tick ${integer(robot.tick)}`);

  const timestamp = Number(data.server_time_ms);
  setText(
    "lastUpdate",
    Number.isFinite(timestamp)
      ? new Date(timestamp).toLocaleTimeString("zh-CN", { hour12: false })
      : "--:--:--",
  );

  const onlineCount = Object.values(sources).filter(
    (source) => source.status === "online",
  ).length;
  if (data.dds_initialized) {
    setText("ddsState", onlineCount === 5 ? "DDS 全部在线" : "DDS 部分在线");
    setText("ddsDetail", `只读模式 · ${onlineCount}/5 数据源在线`);
  } else {
    setText("ddsState", "DDS 初始化失败");
    setText("ddsDetail", data.dds_error || "请检查网卡与 SDK2 环境");
  }

  renderSources(sources);
  renderBattery(data.battery || {});
  renderOdometry(data.odometry || {});
  renderVoice(data.voice || {});
  renderControl(data.control || {});
  renderJoints(data.joints || []);
  renderDiagnostics(data);

  setText(
    "reservedSlots",
    JSON.stringify(data.reserved_motor_slots || [], null, 2),
  );
  setText("bmsRaw", JSON.stringify(data.battery || {}, null, 2));
  setText("mainboardRaw", JSON.stringify(data.mainboard || {}, null, 2));
  window.dispatchEvent(new CustomEvent("g1:telemetry", { detail: data }));
}

function setVoiceBadge(id, label, state) {
  const badge = $(id);
  if (!badge) return;
  badge.textContent = label;
  badge.classList.remove("ok", "error", "pending");
  badge.classList.add(state);
}

function setVoiceTab(tab, focus = false) {
  const nextTab = ["llm", "asr", "tts"].includes(tab) ? tab : "llm";
  appState.voiceTab = nextTab;
  document.querySelectorAll("[data-voice-tab]").forEach((button) => {
    const active = button.dataset.voiceTab === nextTab;
    button.classList.toggle("active", active);
    button.setAttribute("aria-selected", String(active));
    button.tabIndex = active ? 0 : -1;
    if (active && focus) button.focus();
  });
  document.querySelectorAll("[data-voice-panel]").forEach((panel) => {
    const active = panel.dataset.voicePanel === nextTab;
    panel.hidden = !active;
    panel.classList.toggle("active", active);
  });
}

function setControlTab(tab, focus = false) {
  const tabs = ["mode", "motion", "arm"];
  const nextTab = tabs.includes(tab) ? tab : "mode";
  const leavingMotion = appState.controlTab === "motion" && nextTab !== "motion";
  appState.controlTab = nextTab;

  document.querySelectorAll("[data-control-tab]").forEach((button) => {
    const active = button.dataset.controlTab === nextTab;
    button.classList.toggle("active", active);
    button.setAttribute("aria-selected", String(active));
    button.tabIndex = active ? 0 : -1;
    if (active && focus) button.focus();
  });
  document.querySelectorAll("[data-control-panel]").forEach((panel) => {
    const active = panel.dataset.controlPanel === nextTab;
    panel.hidden = !active;
    panel.classList.toggle("active", active);
  });

  if (leavingMotion && appState.motionEnabled) {
    disableMotionControl("已切换到其他控制功能，运动已自动停止");
  }
}

function latestAsrMessage() {
  return String(appState.lastData?.voice?.asr?.text || "").trim();
}

function normalizeCustomerApiUrl(value) {
  let normalized = String(value || "").trim().replace(/\/+$/, "");
  if (!normalized) return "";
  if (!normalized.endsWith("/chat/completions")) {
    normalized += "/chat/completions";
  }
  return normalized;
}

function activeLlmMessage() {
  return appState.llmInputMode === "asr"
    ? latestAsrMessage()
    : $("llmMessage").value.trim();
}

function setLlmInputMode(mode, focus = false) {
  const nextMode = mode === "asr" ? "asr" : "text";
  appState.llmInputMode = nextMode;
  document.querySelectorAll("[data-llm-source]").forEach((button) => {
    const active = button.dataset.llmSource === nextMode;
    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  document.querySelectorAll("[data-llm-input-panel]").forEach((panel) => {
    panel.hidden = panel.dataset.llmInputPanel !== nextMode;
  });
  updateLlmComposerState();
  if (focus && nextMode === "text") $("llmMessage").focus();
}

function updateLlmComposerState(voice = appState.lastData?.voice || {}) {
  const llm = voice.llm || {};
  const asr = voice.asr || {};
  const asrText = String(asr.text || "").trim();
  const textDraft = $("llmMessage").value;
  setText("llmMessageCharCount", `${textDraft.length} / 4096`);
  const llmAsrPreview = $("llmAsrPreview");
  if (asrText) llmAsrPreview.dataset.i18nSkip = "true";
  else delete llmAsrPreview.dataset.i18nSkip;
  setText("llmAsrPreview", asrText || "尚未收到可发送的 ASR 识别结果");

  if (asrText) {
    const receivedAt = Number(asr.received_time_ms);
    const timeLabel = Number.isFinite(receivedAt) && receivedAt > 0
      ? new Date(receivedAt).toLocaleTimeString("zh-CN", { hour12: false })
      : "时间未知";
    const confidence = Number.isFinite(Number(asr.confidence))
      ? `${(Number(asr.confidence) * 100).toFixed(1)}%`
      : "--";
    setText(
      "llmAsrSourceMeta",
      `最近识别 ${timeLabel} · ${asr.language || "语言未知"} · 置信度 ${confidence}`,
    );
  } else {
    setText(
      "llmAsrSourceMeta",
      "打开 ASR 接收并对机器人说话后，可在这里直接发送识别文字。",
    );
  }

  const telemetryMode = llm.mode === "customer" ? "customer" : "builtin";
  const activeMode = appState.llmActiveMode || telemetryMode;
  const modeTransitionPending =
    appState.llmModeRequestInFlight ||
    appState.llmModeDraft !== null ||
    appState.llmModeAwaitingTelemetry !== null;
  const builtinReady =
    activeMode === "builtin" &&
    voice.chat_go_closed === false &&
    llm.builtin_api_available === true &&
    llm.builtin_response_subscribed === true;
  const customerReady =
    activeMode === "customer" &&
    voice.chat_go_closed === true &&
    llm.customer_api_available !== false &&
    llm.customer_api_configured === true;
  const interactionReady =
    !modeTransitionPending && (builtinReady || customerReady);
  const llmBusy = appState.llmChatRequestInFlight || llm.request_state === "running";
  const message = activeLlmMessage();
  $("sendLlmMessage").disabled =
    !interactionReady || llmBusy || !message;
  $("llmMessage").disabled = appState.llmChatRequestInFlight;
  $("clearLlmMessage").disabled =
    appState.llmChatRequestInFlight || textDraft.length === 0;
  $("useLatestAsr").disabled =
    !asrText || appState.llmChatRequestInFlight || modeTransitionPending;
  $("sendLatestAsrToLlm").disabled = !asrText || modeTransitionPending;
  document.querySelectorAll("[data-llm-source]").forEach((button) => {
    button.disabled = appState.llmChatRequestInFlight || modeTransitionPending;
  });
  $("speakLlmResponse").disabled = llm.mode !== "customer";
  $("speakLlmResponse").title = llm.mode === "customer"
    ? "客户模型回复后使用机器人 TTS 播报"
    : "内置笨笨同学会通过原生语音链路播报，无需重复 TTS";

  const hasResponse = Boolean(appState.lastLlmResponse.trim());
  $("copyLlmResponse").disabled = !hasResponse;
  $("speakLlmResponseNow").disabled =
    !hasResponse || !appState.voiceReady || appState.ttsBusy || llmBusy;
}

function renderVoice(voice) {
  const llm = voice.llm || {};
  const llmMode = llm.mode === "customer" ? "customer" : "builtin";
  const chatGoOk =
    llmMode === "builtin"
      ? voice.chat_go_closed === false
      : voice.chat_go_closed === true;
  setVoiceBadge(
    "chatGoBadge",
    llmMode === "builtin"
      ? chatGoOk
        ? `${voice.chat_go_service_name || "chat_go"} 笨笨同学已启用`
        : "chat_go 未启用"
      : chatGoOk
        ? `${voice.chat_go_service_name || "chat_go"} 已为客户模型关闭`
        : "客户模式下 chat_go 仍在运行",
    chatGoOk ? "ok" : voice.initialized ? "error" : "pending",
  );

  const audioAvailable =
    (voice.vui_service_found && Number(voice.vui_service_status_raw) === 0) ||
    (voice.initialized && Number(voice.volume_api_result) === 0);
  setVoiceBadge(
    "vuiBadge",
    audioAvailable ? "vui_service 可用" : "vui_service 异常",
    audioAvailable ? "ok" : voice.initialized ? "error" : "pending",
  );
  setVoiceBadge(
    "asrBadge",
    voice.asr_subscribed ? "ASR 接收已打开" : "ASR 接收已关闭",
    voice.asr_subscribed ? "ok" : voice.initialized ? "error" : "pending",
  );
  $("enableAsr").disabled =
    !voice.initialized || voice.asr_subscribed || appState.asrRequestInFlight;
  $("disableAsr").disabled =
    !voice.initialized || !voice.asr_subscribed || appState.asrRequestInFlight;
  const asrFeedback = $("asrControlFeedback");
  if (voice.asr_control_error) {
    asrFeedback.textContent = `ASR 开关失败：${voice.asr_control_error}`;
    asrFeedback.className = "asr-control-feedback error";
  } else if (voice.asr_subscribed) {
    asrFeedback.textContent = voice.asr?.received
      ? "已收到 ASR 结果；关闭按钮会停止本 Web 服务接收识别消息。"
      : "ASR 接收已打开。若仍无识别结果，请在 App【设备→数据→音频→语音助手】或遥控器 L1+L2 开启唤醒模式。";
    asrFeedback.className = "asr-control-feedback success";
  } else {
    asrFeedback.textContent = "ASR 接收已关闭；TTS 与扬声器音量仍可使用。";
    asrFeedback.className = "asr-control-feedback";
  }

  setText(
    "volumeStatus",
    Number(voice.volume_pct) >= 0 ? `音量 ${integer(voice.volume_pct)}%` : "音量 --",
  );
  if (Number(voice.volume_pct) >= 0 &&
      document.activeElement !== $("volumeSlider") &&
      !appState.volumeDraft &&
      Date.now() >= appState.volumeHoldUntil &&
      !appState.volumeRequestInFlight) {
    $("volumeSlider").value = String(Math.max(0, Math.min(100, Number(voice.volume_pct))));
    setText("volumeValue", `${integer(voice.volume_pct)}%`);
  }
  const playLabels = {
    "-1": "播放状态 --",
    0: "当前未播放",
    1: "正在播放",
  };
  setText(
    "playState",
    playLabels[String(voice.play_state_raw)] ||
      `播放状态 raw ${integer(voice.play_state_raw)}`,
  );

  const asr = voice.asr || {};
  if (asr.received) {
    $("asrText").dataset.i18nSkip = "true";
    setText("asrText", asr.text || "识别结果为空");
    setText("asrLanguage", asr.language || "--");
    setText(
      "asrConfidence",
      Number.isFinite(Number(asr.confidence))
        ? `${(Number(asr.confidence) * 100).toFixed(1)}%`
        : "--",
    );
    setText("asrAngle", integer(asr.angle_raw));
    setText("asrSpeaker", integer(asr.speaker_id_raw));

    const key = `${asr.index}|${asr.timestamp_raw}|${asr.text}`;
    if (key !== appState.lastAsrKey) {
      appState.lastAsrKey = key;
      appState.asrHistory.unshift({
        text: asr.text || "识别结果为空",
        language: asr.language || "--",
        time: Number(asr.received_time_ms) || Date.now(),
      });
      appState.asrHistory = appState.asrHistory.slice(0, 8);
      renderAsrHistory();
    }
  }

  const tts = voice.tts || {};
  appState.voiceReady = voice.initialized === true && audioAvailable;
  $("applyVolume").disabled =
    !appState.voiceReady || appState.volumeRequestInFlight;
  appState.ttsBusy =
    tts.state === "queued" || tts.state === "running" ||
    Number(voice.play_state_raw) === 1;
  const feedback = $("ttsFeedback");
  feedback.className = "tts-feedback";
  if (tts.state === "queued") {
    feedback.textContent = `请求 #${integer(tts.request_id)} 已进入播报队列`;
    feedback.classList.add("running");
  } else if (tts.state === "running") {
    feedback.textContent = `请求 #${integer(tts.request_id)} 正在合成并播放`;
    feedback.classList.add("running");
  } else if (tts.state === "succeeded") {
    feedback.textContent = `请求 #${integer(tts.request_id)} 播报完成`;
    feedback.classList.add("success");
  } else if (tts.state === "failed") {
    feedback.textContent = `TTS 调用失败，API 返回 ${integer(tts.api_result)}`;
    feedback.classList.add("error");
  } else if (voice.initialization_error) {
    feedback.textContent = voice.initialization_error;
    feedback.classList.add("error");
  } else if (appState.voiceReady) {
    feedback.textContent = "语音服务就绪，播报会从机器人扬声器输出";
  } else {
    feedback.textContent = "等待语音服务初始化";
  }

  renderLlm(voice, chatGoOk);
  maybeSyncCustomerVoiceConfigFromFile();
  setText("voiceRaw", JSON.stringify(voice, null, 2));
  updateTtsButton();
}

function displayLlmResponse(llm) {
  const response = String(llm.last_response || "").trim();
  if (llm.mode !== "customer" && response.toLowerCase() === "ack") {
    return "笨笨同学已接收请求（DDS ack）。当前固件不会通过 rt/api/gpt/response 返回回答文字，完整回答由机器人原生扬声器输出。";
  }
  return response;
}

function renderLlm(voice, chatGoOk) {
  const llm = voice.llm || {};
  const telemetryMode = llm.mode === "customer" ? "customer" : "builtin";
  if (
    appState.llmModeAwaitingTelemetry === null ||
    telemetryMode === appState.llmModeAwaitingTelemetry
  ) {
    appState.llmActiveMode = telemetryMode;
    if (telemetryMode === appState.llmModeAwaitingTelemetry) {
      appState.llmModeAwaitingTelemetry = null;
    }
  }
  const activeMode = appState.llmActiveMode || telemetryMode;
  const select = $("llmModeSelect");
  const customerOption = select.querySelector('option[value="customer"]');
  if (customerOption) {
    customerOption.disabled = llm.customer_api_available === false;
  }
  select.disabled = voice.initialized !== true || appState.llmModeRequestInFlight;
  if (!appState.llmModeDraft && document.activeElement !== select) {
    select.value = activeMode;
  }
  const draftMode = appState.llmModeDraft || activeMode;
  $("customerLlmConfig").hidden = draftMode !== "customer";
  $("builtinLlmHint").hidden = draftMode !== "builtin";

  if (!appState.llmModeDraft && !appState.llmConfigDirty) {
    if (document.activeElement !== $("customerLlmApiUrl")) {
      $("customerLlmApiUrl").value = llm.customer_api_url || "";
    }
    if (document.activeElement !== $("customerLlmModel")) {
      $("customerLlmModel").value = llm.customer_model || "";
    }
  }
  const keyInput = $("customerLlmApiKey");
  const keyConfigured = llm.customer_api_key_configured === true;
  keyInput.placeholder = keyConfigured
    ? "已保存到机器人配置文件；输入新 Key 可替换"
    : "无需鉴权的内网模型可留空";
  if (!appState.llmConfigDirty && document.activeElement !== keyInput) {
    keyInput.value = keyConfigured ? SAVED_API_KEY_MASK : "";
    keyInput.dataset.savedKeyMask = keyConfigured ? "true" : "false";
  }
  setText(
    "customerVoiceConfigSummary",
    `API ${llm.customer_api_configured ? "已保存" : "未配置"} · ${integer(llm.customer_qa_count || 0)} 条固定问答 · ${llm.customer_wake_enabled ? `唤醒“${llm.customer_wake_word || "未设置"}”` : "唤醒触发关闭"} · ${llm.customer_tts_backend === "kokoro" ? "Kokoro" : "Unitree TTS"}`,
  );

  const customerReady =
    activeMode === "customer" &&
    voice.chat_go_closed === true &&
    llm.customer_api_available !== false &&
    llm.customer_api_configured === true;
  const builtinReady =
    activeMode === "builtin" &&
    chatGoOk &&
    llm.builtin_api_available === true &&
    llm.builtin_response_subscribed === true;
  setText(
    "llmModeBadge",
    activeMode === "builtin"
      ? builtinReady
        ? "内置 · DDS 已连接"
        : "笨笨同学通道异常"
      : customerReady
        ? `客户模型 · ${llm.customer_model || "已接入"}`
        : "客户模型未就绪",
  );

  const modeFeedback = $("llmModeFeedback");
  modeFeedback.className = "llm-feedback";
  if (appState.llmModeRequestInFlight) {
    modeFeedback.textContent = "正在切换互动引擎并同步 chat_go 服务状态…";
    modeFeedback.classList.add("running");
  } else if (appState.llmModeAwaitingTelemetry !== null) {
    modeFeedback.textContent = "切换已受理，正在等待机器人遥测确认新模式…";
    modeFeedback.classList.add("running");
  } else if (appState.llmModeDraft && appState.llmModeDraft !== activeMode) {
    modeFeedback.textContent = appState.llmModeDraft === "customer"
      ? "客户模式尚未生效；请先保存增强配置与API，保存成功后会自动切换。"
      : "内置模式尚未生效，正在等待自动切换。";
    modeFeedback.classList.add("error");
  } else if ([
    "chat_go_enable_failed",
    "chat_go_disable_failed",
    "chat_go_switch_exception",
    "unknown_builtin_gpt_channel_error",
  ].includes(llm.last_error)) {
    modeFeedback.textContent = `互动引擎操作失败：${voiceError(llm.last_error)}`;
    modeFeedback.classList.add("error");
  } else if (builtinReady) {
    modeFeedback.textContent =
      `笨笨同学已就绪：发送 ${llm.builtin_request_topic || "rt/api/gpt/request"}，接收 ${llm.builtin_response_topic || "rt/api/gpt/response"}。`;
    modeFeedback.classList.add("success");
  } else if (customerReady) {
    modeFeedback.textContent = `chat_go 已关闭，客户 API 已接管：${llm.customer_model || "未命名模型"}`;
    modeFeedback.classList.add("success");
  } else if (activeMode === "customer" && llm.customer_api_available === false) {
    modeFeedback.textContent = "当前构建未包含 libcurl，无法使用客户 API 模式。";
    modeFeedback.classList.add("error");
  } else if (activeMode === "builtin" && chatGoOk) {
    modeFeedback.textContent = "chat_go 已启用，但内置 GPT DDS 发布或响应订阅通道未就绪。";
    modeFeedback.classList.add("error");
  } else {
    modeFeedback.textContent = "互动引擎状态尚未就绪。";
  }

  updateLlmComposerState(voice);

  const chatFeedback = $("llmChatFeedback");
  chatFeedback.className = "llm-feedback";
  if (appState.llmChatRequestInFlight || llm.request_state === "running") {
    chatFeedback.textContent = activeMode === "builtin"
      ? `消息已发布到 ${llm.builtin_request_topic || "rt/api/gpt/request"}，等待笨笨同学 DDS 响应…`
      : "客户大模型正在生成回复…";
    chatFeedback.classList.add("running");
  } else if (llm.request_state === "failed" && llm.last_error) {
    chatFeedback.textContent = `${activeMode === "builtin" ? "笨笨同学" : "客户大模型"}互动失败：${voiceError(llm.last_error)}`;
    chatFeedback.classList.add("error");
  } else if (llm.request_state === "succeeded" && llm.last_response) {
    const builtinAck = activeMode === "builtin" &&
      String(llm.last_response).trim().toLowerCase() === "ack";
    chatFeedback.textContent = activeMode === "builtin"
      ? builtinAck
        ? `笨笨同学已接收请求 · DDS ack · status ${integer(llm.response_status_code)}`
        : `已收到笨笨同学响应 · status ${integer(llm.response_status_code)}`
      : llm.last_response_source === "qa"
        ? "已命中本地固定问答库，未调用客户大模型 API。"
        : "客户大模型回复成功。";
    chatFeedback.classList.add("success");
  } else if (builtinReady) {
    chatFeedback.textContent = appState.llmInputMode === "asr"
      ? "笨笨同学已就绪，将通过 DDS 发送最近一次 ASR 文字。"
      : "笨笨同学已就绪，可直接输入文字；Enter 发送，Shift + Enter 换行。";
    chatFeedback.classList.add("success");
  } else if (customerReady) {
    chatFeedback.textContent = appState.llmInputMode === "asr"
      ? "客户大模型已就绪，将发送最近一次 ASR 识别结果。"
      : "客户大模型已就绪，可直接输入文字；Enter 发送，Shift + Enter 换行。";
    chatFeedback.classList.add("success");
  } else {
    chatFeedback.textContent = "输入草稿会保留；互动引擎就绪后即可发送。";
  }

  if (llm.last_response) {
    appState.lastLlmResponse = displayLlmResponse(llm);
    $("llmResponse").textContent = appState.lastLlmResponse;
  }
  updateLlmComposerState(voice);
}

async function handleLlmModeChange() {
  const mode = $("llmModeSelect").value;
  appState.llmModeDraft = mode;
  $("customerLlmConfig").hidden = mode !== "customer";
  $("builtinLlmHint").hidden = mode !== "builtin";
  updateLlmComposerState();
  if (mode === appState.llmActiveMode && appState.llmModeAwaitingTelemetry === null) {
    appState.llmModeDraft = null;
    return;
  }
  await applyLlmMode(mode);
}

function markLlmConfigDirty() {
  appState.llmConfigDirty = true;
  appState.customerVoiceConfigDirty = true;
  const feedback = $("customerVoiceConfigFeedback");
  feedback.className = "llm-feedback";
  feedback.textContent = "API 或增强配置有未保存修改。";
  updateLlmComposerState();
}

function dedupeCustomerQaEntries(entries) {
  if (!Array.isArray(entries)) return [];
  const unique = [];
  const indexByQuestion = new Map();
  entries.forEach((entry) => {
    const question = String(entry?.question || "").replace(/[\r\n]+/g, " ").trim();
    const answer = String(entry?.answer || "").trim();
    if (!question || !answer) return;
    if (indexByQuestion.has(question)) {
      unique[indexByQuestion.get(question)] = { question, answer };
      return;
    }
    indexByQuestion.set(question, unique.length);
    unique.push({ question, answer });
  });
  return unique;
}

function formatCustomerQaEntries(entries) {
  return dedupeCustomerQaEntries(entries)
    .map((entry) => `${entry.question} => ${entry.answer.replace(/\r?\n/g, "\\n")}`)
    .join("\n");
}

function parseCustomerQaEntries(text) {
  const entries = [];
  String(text || "").split(/\r?\n/).forEach((rawLine, index) => {
    const line = rawLine.trim();
    if (!line) return;
    const separator = line.indexOf("=>");
    if (separator <= 0 || separator >= line.length - 2) {
      throw new Error(`固定问答第 ${index + 1} 行格式应为“问题 => 回答”`);
    }
    const question = line.slice(0, separator).trim();
    const answer = line.slice(separator + 2).trim().replace(/\\n/g, "\n");
    if (!question || !answer) {
      throw new Error(`固定问答第 ${index + 1} 行不能为空`);
    }
    entries.push({ question, answer });
  });
  return dedupeCustomerQaEntries(entries);
}

function markCustomerVoiceConfigDirty() {
  appState.customerVoiceConfigDirty = true;
  const feedback = $("customerVoiceConfigFeedback");
  feedback.className = "llm-feedback";
  feedback.textContent = "增强配置与 API 有未保存修改。";
}

function maybeSyncCustomerVoiceConfigFromFile() {
  const dialog = $("customerConfigFileDialog");
  const now = Date.now();
  if (
    appState.customerVoiceConfigRequestInFlight ||
    appState.customerVoiceConfigDirty ||
    appState.llmConfigDirty ||
    (dialog && dialog.open) ||
    now - appState.lastCustomerConfigSyncAt < 2000
  ) {
    return;
  }
  const customerVisible =
    appState.llmActiveMode === "customer" || $("llmModeSelect").value === "customer";
  if (!customerVisible) return;
  appState.lastCustomerConfigSyncAt = now;
  loadCustomerVoiceConfig(true);
}

async function loadCustomerVoiceConfig(force = false) {
  if (appState.customerVoiceConfigRequestInFlight) return;
  if (appState.customerVoiceConfigLoaded && !force) return;
  appState.customerVoiceConfigRequestInFlight = true;
  const feedback = $("customerVoiceConfigFeedback");
  try {
    const response = await fetch("/api/voice/llm/customer-config", { cache: "no-store" });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    appState.customerConfigPersistenceAvailable =
      Object.prototype.hasOwnProperty.call(result, "config_path");
    appState.customerQaReplaceAvailable = result.qa_delete_semantics === true;
    $("saveCustomerVoiceConfig").disabled = !appState.customerQaReplaceAvailable;
    $("openCustomerConfigFile").disabled = !appState.customerQaReplaceAvailable;
    $("saveCustomerConfigFile").disabled = !appState.customerQaReplaceAvailable;
    const apiFieldsAvailable = Object.prototype.hasOwnProperty.call(result, "api_url") ||
      Object.prototype.hasOwnProperty.call(result, "model") ||
      Object.prototype.hasOwnProperty.call(result, "api_key_configured");
    if (apiFieldsAvailable && (!appState.llmConfigDirty || force)) {
      $("customerLlmApiUrl").value = result.api_url || "";
      $("customerLlmModel").value = result.model || "";
      const keyInput = $("customerLlmApiKey");
      keyInput.value = result.api_key_configured === true ? SAVED_API_KEY_MASK : "";
      keyInput.dataset.savedKeyMask = result.api_key_configured === true ? "true" : "false";
      appState.llmConfigDirty = false;
    }
    if (!appState.customerVoiceConfigDirty || force) {
      $("customerRolePrompt").value = result.role_prompt || "";
      $("customerWakeWord").value = result.wake_word || "";
      $("customerWakeEnabled").checked = result.wake_enabled === true;
      $("customerTtsBackend").value = result.tts_backend === "kokoro" ? "kokoro" : "unitree";
      $("customerQaPairs").value = formatCustomerQaEntries(result.qa_entries);
      appState.customerVoiceConfigDirty = false;
    }
    appState.customerVoiceConfigLoaded = true;
    appState.lastCustomerConfigSyncAt = Date.now();
    feedback.className = "llm-feedback success";
    feedback.textContent = !appState.customerQaReplaceAvailable
      ? `固定问答已读取，但当前后端尚未加载删除修复；为防止删除项被旧配置重新合并，保存按钮已禁用，请重启 g1-web-control 后再保存。`
      : apiFieldsAvailable
        ? `配置文件已实时同步 · API ${result.api_url && result.model ? "已保存" : "未填写"} · Key ${result.api_key_configured ? "已保存" : "未保存"} · 固定问答 ${dedupeCustomerQaEntries(result.qa_entries).length} 条`
        : `增强配置已加载 · 固定问答 ${dedupeCustomerQaEntries(result.qa_entries).length} 条`;
  } catch (error) {
    feedback.className = "llm-feedback error";
    feedback.textContent = `增强配置读取失败：${voiceError(error.message)}`;
  } finally {
    appState.customerVoiceConfigRequestInFlight = false;
  }
}

async function saveCustomerVoiceConfig() {
  if (appState.customerVoiceConfigRequestInFlight) return;
  if (!appState.customerQaReplaceAvailable) {
    const feedback = $("customerVoiceConfigFeedback");
    feedback.className = "llm-feedback error";
    feedback.textContent = "当前后端尚未加载固定回答删除修复，已阻止保存；请先重启 g1-web-control。";
    return;
  }
  const feedback = $("customerVoiceConfigFeedback");
  let qaEntries = [];
  try {
    qaEntries = parseCustomerQaEntries($("customerQaPairs").value);
  } catch (error) {
    feedback.className = "llm-feedback error";
    feedback.textContent = error.message;
    return;
  }

  const apiUrl = $("customerLlmApiUrl").value.trim();
  const model = $("customerLlmModel").value.trim();
  const keyInput = $("customerLlmApiKey");
  const visibleApiKey = keyInput.value;
  const savedKeyMask =
    visibleApiKey === SAVED_API_KEY_MASK &&
    keyInput.dataset.savedKeyMask === "true";
  const apiKey = savedKeyMask ? "" : visibleApiKey;
  const currentLlm = appState.lastData?.voice?.llm || {};
  const preserveApiKey =
    savedKeyMask ||
    (apiKey.length === 0 &&
     currentLlm.customer_api_key_configured === true &&
     normalizeCustomerApiUrl(apiUrl) ===
       normalizeCustomerApiUrl(currentLlm.customer_api_url || ""));

  appState.customerVoiceConfigRequestInFlight = true;
  $("saveCustomerVoiceConfig").disabled = true;
  feedback.className = "llm-feedback running";
  feedback.textContent = "正在保存 API、角色、问答库、唤醒短语与 TTS 配置…";
  try {
    const response = await fetch("/api/voice/llm/customer-config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        api_url: apiUrl,
        api_key: apiKey,
        preserve_api_key: preserveApiKey,
        model,
        role_prompt: $("customerRolePrompt").value.trim(),
        wake_word: $("customerWakeWord").value.trim(),
        wake_enabled: $("customerWakeEnabled").checked,
        tts_backend: $("customerTtsBackend").value,
        qa_entries: qaEntries,
      }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    const savedQaEntries = dedupeCustomerQaEntries(result.qa_entries);
    $("customerQaPairs").value = formatCustomerQaEntries(savedQaEntries);
    $("customerLlmApiUrl").value = result.api_url || "";
    $("customerLlmModel").value = result.model || "";
    keyInput.value = result.api_key_configured === true ? SAVED_API_KEY_MASK : "";
    keyInput.dataset.savedKeyMask = result.api_key_configured === true ? "true" : "false";
    appState.customerVoiceConfigLoaded = true;
    appState.customerVoiceConfigDirty = false;
    appState.llmConfigDirty = false;
    appState.lastCustomerConfigSyncAt = Date.now();
    feedback.className = "llm-feedback success";
    feedback.textContent = `增强配置与 API 已保存 · 固定问答 ${savedQaEntries.length} 条 · TTS ${result.tts_backend === "kokoro" ? "Kokoro" : "Unitree"}`;
    if ($("llmModeSelect").value === "customer" && appState.llmActiveMode !== "customer") {
      await applyLlmMode("customer");
    }
  } catch (error) {
    feedback.className = "llm-feedback error";
    feedback.textContent = `增强配置与 API 保存失败：${voiceError(error.message)}`;
  } finally {
    appState.customerVoiceConfigRequestInFlight = false;
    $("saveCustomerVoiceConfig").disabled = false;
  }
}

function customerConfigDocument(result) {
  return JSON.stringify({
    api_url: result.api_url || "",
    model: result.model || "",
    api_key: result.api_key_configured === true ? SAVED_API_KEY_MASK : "",
    role_prompt: result.role_prompt || "",
    wake_word: result.wake_word || "",
    wake_enabled: result.wake_enabled === true,
    tts_backend: result.tts_backend === "unitree" ? "unitree" : "kokoro",
    qa_entries: dedupeCustomerQaEntries(result.qa_entries),
  }, null, 2);
}

async function reloadCustomerConfigFile(openDialog = false) {
  const dialog = $("customerConfigFileDialog");
  const feedback = $("customerConfigFileFeedback");
  feedback.className = "control-dialog-feedback";
  feedback.textContent = "正在读取机器人配置文件…";
  try {
    const response = await fetch("/api/voice/llm/customer-config", { cache: "no-store" });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    if (!Object.prototype.hasOwnProperty.call(result, "config_path")) {
      throw new Error("customer_config_file_api_unavailable");
    }
    $("customerConfigFilePath").value = result.config_path || "config/customer_voice.json";
    $("customerConfigFileEditor").value = customerConfigDocument(result);
    feedback.className = "control-dialog-feedback success";
    feedback.textContent = `已加载配置文件 · API Key ${result.api_key_configured ? "已保存（掩码显示）" : "未保存"}`;
    if (openDialog && !dialog.open) dialog.showModal();
  } catch (error) {
    feedback.className = "control-dialog-feedback error";
    feedback.textContent = `配置文件读取失败：${voiceError(error.message)}`;
    if (openDialog && !dialog.open) dialog.showModal();
  }
}

async function openCustomerConfigFile() {
  if (!appState.customerQaReplaceAvailable) {
    const feedback = $("customerVoiceConfigFeedback");
    feedback.className = "llm-feedback error";
    feedback.textContent = "当前后端尚未加载配置文件实时同步/替换版本，请先重启 g1-web-control。";
    return;
  }
  await reloadCustomerConfigFile(true);
}

async function saveCustomerConfigFile() {
  const feedback = $("customerConfigFileFeedback");
  if (!appState.customerQaReplaceAvailable) {
    feedback.className = "control-dialog-feedback error";
    feedback.textContent = "当前后端尚未加载固定回答删除修复，已阻止写文件；请先重启 g1-web-control。";
    return;
  }
  let documentConfig;
  try {
    documentConfig = JSON.parse($("customerConfigFileEditor").value);
    if (!documentConfig || typeof documentConfig !== "object" || Array.isArray(documentConfig)) {
      throw new Error("配置文件必须是 JSON 对象");
    }
    ["api_url", "model", "api_key", "role_prompt", "wake_word", "tts_backend"].forEach((field) => {
      if (typeof documentConfig[field] !== "string") {
        throw new Error(`${field} 必须是字符串`);
      }
    });
    if (typeof documentConfig.wake_enabled !== "boolean") {
      throw new Error("wake_enabled 必须是 true 或 false");
    }
    if (!Array.isArray(documentConfig.qa_entries)) {
      throw new Error("qa_entries 必须是数组");
    }
    documentConfig.qa_entries.forEach((entry, index) => {
      if (!entry || typeof entry !== "object" ||
          typeof entry.question !== "string" || typeof entry.answer !== "string") {
        throw new Error(`qa_entries[${index}] 必须包含字符串 question 和 answer`);
      }
    });
    if (!["kokoro", "unitree"].includes(documentConfig.tts_backend)) {
      throw new Error("tts_backend 只能是 kokoro 或 unitree");
    }
  } catch (error) {
    feedback.className = "control-dialog-feedback error";
    feedback.textContent = `JSON 格式错误：${error.message}`;
    return;
  }

  const visibleKey = typeof documentConfig.api_key === "string" ? documentConfig.api_key : "";
  const preserveApiKey = visibleKey === SAVED_API_KEY_MASK;
  $("saveCustomerConfigFile").disabled = true;
  feedback.className = "control-dialog-feedback running";
  feedback.textContent = "正在原子保存配置文件…";
  try {
    const response = await fetch("/api/voice/llm/customer-config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        api_url: String(documentConfig.api_url || "").trim(),
        api_key: preserveApiKey ? "" : visibleKey,
        preserve_api_key: preserveApiKey,
        model: String(documentConfig.model || "").trim(),
        role_prompt: String(documentConfig.role_prompt || ""),
        wake_word: String(documentConfig.wake_word || "").trim(),
        wake_enabled: documentConfig.wake_enabled === true,
        tts_backend: documentConfig.tts_backend,
        qa_entries: dedupeCustomerQaEntries(documentConfig.qa_entries),
      }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    $("customerConfigFileEditor").value = customerConfigDocument(result);
    $("customerConfigFilePath").value = result.config_path || "config/customer_voice.json";
    feedback.className = "control-dialog-feedback success";
    feedback.textContent = "配置文件已保存；新 API/Key 已覆盖旧值并立即更新运行时配置。";
    appState.customerVoiceConfigLoaded = false;
    appState.customerVoiceConfigDirty = false;
    appState.llmConfigDirty = false;
    await loadCustomerVoiceConfig(true);
  } catch (error) {
    feedback.className = "control-dialog-feedback error";
    feedback.textContent = `配置文件保存失败：${voiceError(error.message)}`;
  } finally {
    $("saveCustomerConfigFile").disabled = false;
  }
}

async function applyLlmMode(mode = $("llmModeSelect").value) {
  if (appState.llmModeRequestInFlight) return false;
  appState.llmModeRequestInFlight = true;
  $("llmModeSelect").disabled = true;
  const feedback = $("llmModeFeedback");
  feedback.className = "llm-feedback running";
  feedback.textContent = mode === "customer"
    ? "正在关闭 chat_go，并使用已保存的客户 API 自动切换…"
    : "正在启用 chat_go 笨笨同学…";
  try {
    const response = await fetch("/api/voice/llm/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    appState.llmModeDraft = null;
    appState.llmActiveMode = mode;
    appState.llmModeAwaitingTelemetry = mode;
    feedback.className = "llm-feedback success";
    feedback.textContent = mode === "customer"
      ? "客户大模型已自动接管；当前使用最后一次保存的增强配置与 API。"
      : "chat_go 笨笨同学已自动启用，文本与 ASR 可通过 DDS 直接互动。";
    return true;
  } catch (error) {
    appState.llmModeDraft = mode;
    feedback.className = "llm-feedback error";
    feedback.textContent = mode === "customer" && error.message === "customer_api_not_configured"
      ? "客户 API 尚未保存；请填写后点击“保存增强配置与API”，保存成功后会自动切换到客户大模型。"
      : `互动引擎自动切换失败：${voiceError(error.message)}`;
    return false;
  } finally {
    appState.llmModeRequestInFlight = false;
    $("llmModeSelect").disabled = false;
    updateLlmComposerState();
  }
}

async function speakLlmReply(text, force = false) {
  if ((!force && !$("speakLlmResponse").checked) || !text) return "";
  try {
    const response = await fetch("/api/voice/tts", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ text, speaker_id: -1 }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    return ` · 已提交机器人播报 #${result.request_id}`;
  } catch (error) {
    return ` · 播报失败：${voiceError(error.message)}`;
  }
}

async function submitLlmChat(event) {
  event.preventDefault();
  if (
    appState.llmChatRequestInFlight ||
    appState.llmModeRequestInFlight ||
    appState.llmModeDraft !== null ||
    appState.llmModeAwaitingTelemetry !== null
  ) return;
  const message = activeLlmMessage();
  if (!message) return;
  const sourceLabel = appState.llmInputMode === "asr" ? "最近 ASR" : "键盘文本";
  appState.llmChatRequestInFlight = true;
  $("sendLlmMessage").disabled = true;
  const feedback = $("llmChatFeedback");
  feedback.className = "llm-feedback running";
  const currentMode = appState.llmActiveMode ||
    (appState.lastData?.voice?.llm?.mode === "customer"
      ? "customer"
      : "builtin");
  feedback.textContent = currentMode === "builtin"
    ? `正在通过 rt/api/gpt/request 发送${sourceLabel}…`
    : `正在发送${sourceLabel}并调用客户大模型…`;
  try {
    const response = await fetch("/api/voice/llm/chat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        mode: currentMode,
        message,
        auto_tts: currentMode === "customer" && $("speakLlmResponse").checked,
      }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    const resultMode = result.mode === "customer" ? "customer" : "builtin";
    if (result.pending) {
      feedback.className = "llm-feedback running";
      feedback.textContent =
        `请求 #${result.request_id} 已发布，等待 rt/api/gpt/response 返回笨笨同学反馈…`;
      return;
    }
    appState.lastLlmResponse = result.response ||
      (resultMode === "builtin"
        ? "已收到笨笨同学响应，但响应中没有可显示的文本。"
        : "客户大模型返回了空回复");
    $("llmResponse").textContent = appState.lastLlmResponse;
    updateLlmComposerState();
    const speakResult = resultMode === "customer"
      ? result.tts_request_id
        ? ` · 已自动提交机器人播报 #${result.tts_request_id}`
        : result.tts_error
          ? ` · 自动播报失败：${voiceError(result.tts_error)}`
          : ""
      : "";
    feedback.className = "llm-feedback success";
    feedback.textContent = resultMode === "builtin"
      ? "已收到笨笨同学反馈；原生 chat_go 负责语音播报。"
      : result.response_source === "qa"
        ? `命中本地固定问答，未调用模型 API${speakResult}`
        : `客户大模型回复成功${speakResult}`;
  } catch (error) {
    feedback.className = "llm-feedback error";
    feedback.textContent = `大模型互动失败：${voiceError(error.message)}`;
  } finally {
    appState.llmChatRequestInFlight = false;
    updateLlmComposerState();
  }
}

async function copyLlmResponse() {
  if (!appState.lastLlmResponse) return;
  const feedback = $("llmChatFeedback");
  try {
    await navigator.clipboard.writeText(appState.lastLlmResponse);
    feedback.className = "llm-feedback success";
    feedback.textContent = "大模型回复已复制到剪贴板。";
  } catch (error) {
    feedback.className = "llm-feedback error";
    feedback.textContent = "复制失败，请手动选择回复文字。";
  }
}

async function speakCurrentLlmResponse() {
  if (!appState.lastLlmResponse) return;
  const feedback = $("llmChatFeedback");
  feedback.className = "llm-feedback running";
  feedback.textContent = "正在提交大模型回复到机器人扬声器…";
  const result = await speakLlmReply(appState.lastLlmResponse, true);
  feedback.className = result.includes("失败")
    ? "llm-feedback error"
    : "llm-feedback success";
  feedback.textContent = result ? result.replace(/^ · /, "") : "播报请求未提交。";
}

function renderAsrHistory() {
  const history = $("asrHistory");
  history.replaceChildren();
  if (appState.asrHistory.length === 0) {
    const empty = document.createElement("li");
    empty.className = "empty-history";
    empty.textContent = "尚未收到 ASR 结果";
    history.append(empty);
    return;
  }
  appState.asrHistory.forEach((item) => {
    const row = document.createElement("li");
    const time = document.createElement("time");
    time.textContent = new Date(item.time).toLocaleTimeString(
      window.UiI18n?.language === "en" ? "en-US" : "zh-CN",
      { hour12: false },
    );
    const text = document.createElement("span");
    text.dataset.i18nSkip = "true";
    text.textContent = item.text;
    const language = document.createElement("small");
    language.dataset.i18nSkip = "true";
    language.textContent = item.language;
    row.append(time, text, language);
    history.append(row);
  });
}

function updateTtsBackendState() {
  const kokoro = $("ttsBackend").value === "kokoro";
  $("ttsSpeaker").disabled = kokoro;
  if (kokoro) $("ttsSpeaker").value = "-1";
}

function updateTtsButton() {
  const text = $("ttsText").value.trim();
  const button = $("ttsSubmit");
  button.disabled = !appState.voiceReady || appState.ttsBusy || !text;
  if (appState.ttsBusy) {
    setText("ttsSubmitLabel", "TTS 正在处理");
  } else if (!appState.voiceReady) {
    setText("ttsSubmitLabel", "语音服务未就绪");
  } else {
    setText("ttsSubmitLabel", "发送到机器人扬声器");
  }
}

async function submitTts(event) {
  event.preventDefault();
  const text = $("ttsText").value.trim();
  if (!text || !appState.voiceReady || appState.ttsBusy) return;

  const ttsBackend = $("ttsBackend").value;
  const speakerId = ttsBackend === "kokoro" ? -1 : Number($("ttsSpeaker").value);
  appState.ttsBusy = true;
  updateTtsButton();
  const feedback = $("ttsFeedback");
  feedback.className = "tts-feedback running";
  feedback.textContent = "正在提交 TTS 请求…";

  try {
    const response = await fetch("/api/voice/tts", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ text, speaker_id: speakerId, tts_backend: ttsBackend }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    const backendLabel = result.tts_backend === "kokoro" ? "Kokoro 本地低延迟" : "Unitree 原生 TTS";
    const speakerLabel = Number(result.speaker_id) === -1
      ? "中英文自动"
      : `speaker ${result.speaker_id}`;
    feedback.textContent = `请求 #${result.request_id} 已进入播报队列 · ${backendLabel} · ${speakerLabel}`;
  } catch (error) {
    appState.ttsBusy = false;
    feedback.className = "tts-feedback error";
    feedback.textContent = `TTS 提交失败：${voiceError(error.message)}`;
    updateTtsButton();
  }
}

const voiceErrorLabels = {
  voice_not_ready: "语音服务未就绪",
  invalid_volume: "音量必须在 0 到 100 之间",
  volume_api_error: "机器人音量接口调用失败",
  invalid_speaker_id: "发音人参数无效",
  invalid_tts_backend: "TTS 引擎无效，请选择 Kokoro 或 Unitree",
  mixed_language_requires_auto: "中英文混合请使用“自动识别并分段”发音模式",
  speaker_language_mismatch: "所选发音人与文本语言不匹配",
  duplicate_tts_request: "相同文本刚刚已提交，已阻止重复播报",
  tts_busy: "上一条 TTS 正在提交",
  audio_busy: "机器人扬声器正在播放，请等待结束",
  invalid_utf8: "文本编码无效",
  invalid_llm_mode: "大模型互动模式无效",
  llm_mode_mismatch: "互动模式刚刚发生变化，请确认当前模式后重试",
  invalid_customer_api_url: "客户接口地址必须是有效的 HTTP 或 HTTPS 地址",
  invalid_customer_model: "客户模型名称不能为空或过长",
  customer_api_key_too_long: "客户接口密钥过长",
  customer_api_key_endpoint_changed: "API 地址已改变，不能复用旧 Key；请输入新 Key 或明确留空",
  customer_role_too_long: "角色提示词过长",
  customer_wake_word_too_long: "唤醒短语过长",
  customer_wake_word_empty: "启用唤醒触发前请先填写唤醒短语",
  invalid_customer_tts_backend: "客户 TTS 后端无效",
  customer_qa_too_many_entries: "固定问答最多 100 条",
  customer_qa_empty_entry: "固定问答的问题和回答都不能为空",
  customer_qa_entry_too_long: "固定问答单条内容过长",
  customer_config_invalid_json: "客户配置文件格式无效",
  customer_api_config_invalid: "配置文件中的客户 API 地址、模型或 Key 无效",
  customer_config_permissions_failed: "无法将客户配置文件权限收紧为仅当前用户可读写",
  customer_config_file_api_unavailable: "后端尚未加载新版客户配置接口；重启 g1-web-control 后即可打开配置文件编辑器",
  customer_config_open_failed: "无法创建客户配置文件",
  customer_config_write_failed: "写入客户增强配置文件失败",
  customer_config_save_failed: "保存客户增强配置失败",
  local_tts_unavailable: "Kokoro 本地 TTS 服务不可用",
  customer_api_unavailable: "当前构建未提供客户接口调用能力",
  customer_api_not_configured: "客户接口尚未配置",
  customer_llm_not_active: "当前未启用客户大模型",
  builtin_llm_not_active: "内置笨笨同学当前未启用",
  builtin_llm_busy: "上一条笨笨同学请求仍在等待响应",
  builtin_gpt_channel_unavailable: "内置 GPT DDS 发布或响应订阅通道不可用",
  builtin_gpt_publish_failed: "向 rt/api/gpt/request 发布消息失败",
  builtin_gpt_response_timeout: "等待 rt/api/gpt/response 超过 45 秒",
  unknown_builtin_gpt_channel_error: "初始化内置 GPT DDS 通道时发生未知错误",
  customer_api_transport_init_failed: "客户接口网络组件初始化失败",
  customer_api_http_error: "客户接口返回错误状态",
  customer_api_invalid_response: "客户接口回复格式不兼容",
  chat_go_enable_failed: "无法启用 chat_go 笨笨同学",
  chat_go_disable_failed: "无法关闭 chat_go，未切换到客户模型",
  chat_go_switch_exception: "切换 chat_go 服务时发生异常",
  llm_message_empty: "发送内容不能为空",
  llm_message_too_long: "发送内容超过 4096 字节",
};

function voiceError(error) {
  if (String(error || "").startsWith("builtin_gpt_response_error_")) {
    return `笨笨同学 DDS 响应失败，状态码 ${String(error).split("_").pop()}`;
  }
  return voiceErrorLabels[error] || error || "未知语音错误";
}

async function setAsrEnabled(enabled) {
  if (appState.asrRequestInFlight) return;
  appState.asrRequestInFlight = true;
  const feedback = $("asrControlFeedback");
  feedback.className = "asr-control-feedback running";
  feedback.textContent = enabled ? "正在打开 ASR 接收…" : "正在关闭 ASR 接收…";
  try {
    const response = await fetch("/api/voice/asr", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    feedback.className = "asr-control-feedback success";
    feedback.textContent = enabled
      ? "ASR 接收已打开；请说话测试。"
      : "ASR 接收已关闭。";
  } catch (error) {
    feedback.className = "asr-control-feedback error";
    feedback.textContent = `ASR 开关失败：${voiceError(error.message)}`;
  } finally {
    appState.asrRequestInFlight = false;
  }
}

async function applyVolume() {
  if (appState.volumeRequestInFlight) return;
  if (!appState.voiceReady) {
    appState.volumeDraft = false;
    return;
  }
  const volume = Number($("volumeSlider").value);
  appState.volumeRequestInFlight = true;
  $("applyVolume").disabled = true;
  const feedback = $("volumeFeedback");
  feedback.className = "volume-feedback running";
  feedback.textContent = `正在设置音量 ${volume}%…`;
  try {
    const response = await fetch("/api/voice/volume", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ volume }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    feedback.className = "volume-feedback success";
    feedback.textContent = `机器人扬声器音量已设置为 ${result.volume}%`;
    $("volumeSlider").value = String(result.volume);
    setText("volumeValue", `${result.volume}%`);
    appState.volumeHoldUntil = Date.now() + 2000;
  } catch (error) {
    feedback.className = "volume-feedback error";
    feedback.textContent = `音量设置失败：${voiceError(error.message)}`;
  } finally {
    appState.volumeRequestInFlight = false;
    appState.volumeDraft = false;
    $("applyVolume").disabled = !appState.voiceReady;
  }
}

const fsmNames = {
  0: "零力矩",
  1: "阻尼",
  2: "位控下蹲",
  3: "位控落座",
  4: "锁定站立",
  500: "常规运控",
  501: "常规运控 · 3DoF 腰",
  706: "平衡下蹲",
  702: "躺起",
  801: "走跑运控",
  802: "走跑运控",
};

const controlErrorLabels = {
  control_not_ready: "机器人控制服务未就绪",
  invalid_request_key: "请求标识无效",
  unknown_or_disallowed_command: "该控制指令未开放",
  confirmation_required: "尚未确认执行",
  control_busy: "已有控制指令正在执行",
  sport_state_stale: "运动状态数据已过期，拒绝执行",
  robot_not_static: "机器人处于动态状态，当前禁止切换",
  motion_active: "机器人仍在响应移动指令，请先停止",
  invalid_velocity: "速度参数无效",
  velocity_out_of_range: "速度超过当前档位或服务端上限",
  motion_fsm_not_allowed: "当前 FSM 不支持平移运动",
  invalid_speed_mode: "速度档位无效",
  speed_mode_requires_walkrun: "高速仅允许在走跑 FSM 801/802 使用",
  motion_precondition_changed: "机器人运动状态已经变化，指令已停止",
  arm_action_fsm_not_allowed: "当前 FSM 不允许上肢动作",
  arm_action_robot_not_static: "机器人正在运动，禁止上肢动作",
  arm_action_not_available_on_firmware: "当前固件未提供该上肢动作",
  invalid_teach_action_name: "示教动作名称无效",
  teach_action_not_available_on_firmware: "当前固件动作列表中没有该示教动作",
  arm_action_not_supported_by_model: "当前机器人自由度配置不支持该动作",
  sdk_api_error: "SDK 控制接口返回错误",
  fsm_confirmation_timeout: "SDK 已响应，但未确认目标 FSM",
};

function controlError(error) {
  return controlErrorLabels[error] || error || "未知控制错误";
}

function renderTeachActions(control) {
  const select = $("teachActionSelect");
  const hint = $("teachActionHint");
  let actions = [];
  if (Number(control.action_list_api_result) === 0 && control.action_list_raw) {
    try {
      const root = JSON.parse(control.action_list_raw);
      if (Array.isArray(root) && Array.isArray(root[1])) {
        actions = root[1]
          .filter((action) => action && typeof action.name === "string" && action.name)
          .map((action) => ({
            name: action.name,
            time: Number.isFinite(Number(action.time)) ? Number(action.time) : null,
          }));
      }
    } catch (error) {
      actions = [];
    }
  }

  const key = JSON.stringify(actions);
  if (key !== appState.teachActionsKey) {
    const previous = select.value;
    select.replaceChildren();
    if (actions.length === 0) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "当前固件未返回示教动作";
      select.append(option);
    } else {
      actions.forEach((action) => {
        const option = document.createElement("option");
        option.value = action.name;
        option.textContent =
          action.time === null
            ? action.name
            : `${action.name} · ${number(action.time, 1)} s`;
        select.append(option);
      });
      if (actions.some((action) => action.name === previous)) {
        select.value = previous;
      }
    }
    appState.teachActionsKey = key;
  }

  const hasActions = actions.length > 0;
  select.disabled = !hasActions;
  hint.className = `teach-action-hint${hasActions ? " success" : ""}`;
  hint.textContent = hasActions
    ? `已从固件读取 ${actions.length} 个示教动作；名称区分大小写，执行前仍需二次确认。`
    : Number(control.action_list_api_result) === 0
      ? "当前固件动作列表中没有示教动作，请先在 Unitree App 中录制并保存。"
      : `读取固件动作列表失败，API ${integer(control.action_list_api_result)}。`;
}

function renderControl(control) {
  appState.controlData = control;
  const nextMotionFsmFamily = motionFsmFamily(control);
  if (
    nextMotionFsmFamily &&
    nextMotionFsmFamily !== appState.motionFsmFamily
  ) {
    renderMotionPresetOptions(nextMotionFsmFamily);
    const preset = motionPresetValues(0, nextMotionFsmFamily);
    appState.motionPreset = {
      label: "低速",
      speedMode: 0,
      ...preset,
    };
    setText(
      "motionSpeedValue",
      `低速 · ${number(preset.forwardSpeed, 1)} m/s`,
    );
  }
  appState.motionFsmFamily = nextMotionFsmFamily;
  const ready = control.initialized === true && control.enabled === true;
  const badge = $("controlBadge");
  badge.classList.remove("locked", "unlocked", "error");
  if (!ready) {
    badge.classList.add("error");
    badge.textContent = "控制服务异常";
  } else if (appState.controlUnlocked) {
    badge.classList.add("unlocked");
    badge.textContent = "本页控制已解锁";
  } else {
    badge.classList.add("locked");
    badge.textContent = "控制已锁定";
  }
  setText(
    "topControlLock",
    ready ? (appState.controlUnlocked ? "已解锁" : "已锁定") : "服务异常",
  );

  setText("controlFsmId", control.sport_state_received ? integer(control.fsm_id) : "--");
  setText("topFsm", control.sport_state_received ? integer(control.fsm_id) : "--");
  setText(
    "controlFsmName",
    control.sport_state_received
      ? fsmNames[Number(control.fsm_id)] || "未知或固件新增模式"
      : "等待 rt/sportmodestate",
  );
  setText(
    "controlFsmMode",
    control.sport_state_received ? integer(control.fsm_mode) : "--",
  );
  setText(
    "controlFsmModeHint",
    Number(control.fsm_mode) === 0
      ? "静态 · 允许评估切换"
      : "动态 · 大部分切换禁止",
  );
  setText("controlTaskId", control.sport_state_received ? integer(control.task_id) : "--");
  setText(
    "controlTaskTime",
    control.sport_state_received
      ? `${number(control.task_time_s, 2)} s`
      : "--",
  );

  const command = control.last_command || {};
  const stateLabels = {
    idle: "空闲",
    queued: "等待执行",
    running: "正在执行",
    succeeded: "执行成功",
    failed: "执行失败",
  };
  setText("controlLastState", stateLabels[command.state] || command.state || "空闲");
  const detail = command.request_id
    ? `#${integer(command.request_id)} · ${command.command || "--"}${
        command.action_name ? ` · ${command.action_name}` : ""
      }${
        command.state === "failed"
          ? ` · ${controlError(command.error)} · API ${integer(command.api_result)}`
          : ""
      }`
    : "尚未提交控制指令";
  setText("controlLastDetail", detail);
  setText("controlRaw", JSON.stringify(control, null, 2));
  renderTeachActions(control);
  renderMotionState(control.motion || {});
  document.querySelectorAll("[data-fsm-id]").forEach((button) => {
    button.classList.toggle(
      "active-fsm",
      control.sport_state_received === true &&
        Number(button.dataset.fsmId) === Number(control.fsm_id),
    );
  });
  updateControlButtons();
}

function updateControlButtons() {
  const control = appState.controlData || {};
  const commandState = control.last_command?.state;
  const busy = commandState === "queued" || commandState === "running";
  const motionActive =
    control.motion?.active === true || currentMotionVector().active;
  const sportFresh =
    control.sport_state_received === true &&
    Number(control.sport_state_age_ms) <= 1000;
  const staticState = sportFresh && Number(control.fsm_mode) === 0;
  const armStatic =
    sportFresh &&
    (Number(control.fsm_mode) === 0 || Number(control.fsm_mode) === 3);
  const armFsm = [500, 501, 801, 802].includes(Number(control.fsm_id));
  const baseReady =
    appState.controlUnlocked &&
    control.initialized === true &&
    control.enabled === true &&
    !busy &&
    appState.socket?.readyState === WebSocket.OPEN;

  document.querySelectorAll(".control-command").forEach((button) => {
    const category = button.dataset.category;
    const command = button.dataset.command;
    let allowed = baseReady;
    if (command !== "damp" && command !== "stop_move") {
      allowed = allowed && staticState && !motionActive;
    }
    if (category === "arm_action") {
      allowed = baseReady && armStatic && armFsm;
    }
    if (command === "execute_custom") {
      allowed = allowed && Boolean($("teachActionSelect").value);
    }
    button.disabled = !allowed;
  });
  updateMotionControls();
}

function isLocomotionFsm(control = appState.controlData || {}) {
  return [500, 501, 801, 802].includes(Number(control.fsm_id));
}

function isWalkRunFsm(control = appState.controlData || {}) {
  return [801, 802].includes(Number(control.fsm_id));
}

function motionFsmFamily(control = appState.controlData || {}) {
  if ([500, 501].includes(Number(control.fsm_id))) return "regular";
  if (isWalkRunFsm(control)) return "walkrun";
  return null;
}

function motionPresetValues(speedMode, family = appState.motionFsmFamily) {
  if (speedMode === 3) {
    return { forwardSpeed: 3.0, lateralSpeed: 1.0, yawSpeed: 1.5 };
  }
  if (family === "walkrun") {
    return speedMode === 1
      ? { forwardSpeed: 1.0, lateralSpeed: 0.6, yawSpeed: 1.3 }
      : { forwardSpeed: 0.5, lateralSpeed: 0.4, yawSpeed: 1.1 };
  }
  return speedMode === 1
    ? { forwardSpeed: 1.0, lateralSpeed: 0.35, yawSpeed: 0.8 }
    : { forwardSpeed: 0.5, lateralSpeed: 0.2, yawSpeed: 0.5 };
}

function renderMotionPresetOptions(family) {
  document.querySelectorAll(".motion-speed-button").forEach((button) => {
    const speedMode = Number(button.dataset.speedMode);
    const preset = motionPresetValues(speedMode, family);
    button.dataset.forwardSpeed = String(preset.forwardSpeed);
    button.dataset.lateralSpeed = String(preset.lateralSpeed);
    button.dataset.yawSpeed = String(preset.yawSpeed);
    button.querySelector("span").textContent =
      `前进 ${preset.forwardSpeed.toFixed(1)} m/s · 横移 ${preset.lateralSpeed.toFixed(2)} m/s · 转向 ${preset.yawSpeed.toFixed(2)} rad/s`;
  });
}

function motionPresetAllowed() {
  const speedMode = appState.motionPreset.speedMode;
  return speedMode === 0 || speedMode === 1 ||
    (speedMode === 3 && isWalkRunFsm());
}

function motionBaseReady() {
  const control = appState.controlData || {};
  return (
    appState.controlUnlocked &&
    control.initialized === true &&
    control.enabled === true &&
    control.sport_state_received === true &&
    Number(control.sport_state_age_ms) <= 1000 &&
    isLocomotionFsm(control) &&
    appState.socket?.readyState === WebSocket.OPEN
  );
}

function updateMotionControls() {
  const baseReady = motionBaseReady();
  const presetReady = motionPresetAllowed();
  const ready = baseReady && presetReady && appState.motionEnabled;
  document.querySelectorAll(".motion-pad-button").forEach((button) => {
    button.disabled = !ready;
  });
  $("enableMotionControl").disabled =
    !baseReady || !presetReady || appState.motionEnabled;
  $("disableMotionControl").disabled = !appState.motionEnabled;
  document.querySelectorAll(".motion-speed-button").forEach((button) => {
    const requiresWalkRun = Number(button.dataset.speedMode) === 3;
    button.disabled =
      !baseReady || (requiresWalkRun && !isWalkRunFsm());
    button.classList.toggle(
      "selected",
      Number(button.dataset.speedMode) ===
        appState.motionPreset.speedMode,
    );
  });

  const badge = $("motionControlBadge");
  badge.classList.remove("locked", "ready", "active", "error");
  if (!isLocomotionFsm()) {
    badge.classList.add("locked");
    badge.textContent = "请先进入运控 FSM";
  } else if (!presetReady) {
    badge.classList.add("locked");
    badge.textContent = "当前档位需要走跑 FSM";
  } else if (!appState.motionEnabled) {
    badge.classList.add("locked");
    badge.textContent = "运动控制未启用";
  } else {
    const vector = currentMotionVector();
    badge.classList.add(vector.active ? "active" : "ready");
    badge.textContent = vector.active ? "正在响应按键" : "键盘控制已就绪";
  }
}

function renderMotionState(motion) {
  const stateLabels = {
    queued: "等待发送",
    running: "正在发送",
    active: "运动指令生效",
    stopped: "已停止",
    failed: "发送失败",
  };
  setText(
    "motionCommandState",
    stateLabels[motion.state] || motion.state || "已停止",
  );
  if (motion.state === "failed") {
    const feedback = $("motionFeedback");
    feedback.className = "error";
    feedback.textContent = `运动接口失败：${controlError(motion.error)}，API ${integer(
      motion.api_result,
    )}`;
    appState.motionEnabled = false;
    releaseAllMotionInput(false);
  }
  if (appState.motionEnabled && !isLocomotionFsm()) {
    disableMotionControl("FSM 已离开运控模式，运动控制已自动停止");
  } else if (appState.motionEnabled && !motionPresetAllowed()) {
    disableMotionControl("当前速度档位不适用于该 FSM，运动已自动停止");
  }
}

function openMotionEnableConfirmation() {
  if (
    !motionBaseReady() ||
    !motionPresetAllowed() ||
    appState.motionEnabled
  ) {
    return;
  }
  appState.pendingControl = {
    localAction: "enable_motion",
    label: "启用键盘连续运动",
  };
  setText("controlConfirmTitle", "确认启用：键盘运动控制");
  setText(
    "controlConfirmWarning",
    `当前选择${appState.motionPreset.label}（前进最高 ${number(
      appState.motionPreset.forwardSpeed,
      1,
    )} m/s）。W/S 控制前后，A/D 控制转向，Q/E 控制横向平移；松开、窗口失焦或切换标签页会停止。`,
  );
  $("submitControlCommand").disabled = false;
  setText("controlDialogFeedback", "");
  $("controlConfirmDialog").showModal();
  $("submitControlCommand").focus();
}

function enableMotionControl() {
  appState.motionEnabled = true;
  const feedback = $("motionFeedback");
  feedback.className = "ok";
  feedback.textContent = "键盘运动已启用：按住移动，松开停止";
  updateMotionControls();
}

function disableMotionControl(reason = "键盘运动控制已停用") {
  appState.motionEnabled = false;
  releaseAllMotionInput(true);
  const feedback = $("motionFeedback");
  feedback.className = "";
  feedback.textContent = reason;
  updateMotionControls();
}

function currentMotionKeys() {
  const keys = new Set(appState.keyboardMotionKeys);
  for (const key of appState.pointerMotionKeys.values()) {
    keys.add(key);
  }
  return keys;
}

function currentMotionVector() {
  const keys = currentMotionKeys();
  let forward = Number(keys.has("w")) - Number(keys.has("s"));
  let lateral = Number(keys.has("q")) - Number(keys.has("e"));
  const turn = Number(keys.has("a")) - Number(keys.has("d"));
  const active = forward !== 0 || lateral !== 0 || turn !== 0;
  if (forward !== 0 && lateral !== 0) {
    forward /= Math.SQRT2;
    lateral /= Math.SQRT2;
  }
  const preset = appState.motionPreset;
  return {
    active,
    vx: active ? forward * preset.forwardSpeed : 0,
    vy: active ? lateral * preset.lateralSpeed : 0,
    vyaw: active ? turn * preset.yawSpeed : 0,
    speedMode: preset.speedMode,
    keys,
  };
}

function motionDirectionLabel(vector) {
  const labels = [];
  if (vector.vx > 0) labels.push("向前");
  if (vector.vx < 0) labels.push("向后");
  if (vector.vy > 0) labels.push("向左");
  if (vector.vy < 0) labels.push("向右");
  if (vector.vyaw > 0) labels.push("左转");
  if (vector.vyaw < 0) labels.push("右转");
  return labels.join(" + ") || "--";
}

function renderMotionVector(vector) {
  setText("motionDirection", motionDirectionLabel(vector));
  setText(
    "motionVelocity",
    `前后 ${number(vector.vx, 2)} m/s · 横移 ${number(
      vector.vy,
      2,
    )} m/s · 转向 ${number(vector.vyaw, 2)} rad/s`,
  );
  document.querySelectorAll(".motion-pad-button").forEach((button) => {
    button.classList.toggle(
      "pressed",
      vector.keys.has(button.dataset.motionKey),
    );
  });
  updateMotionControls();
}

async function flushMotionRequests() {
  if (appState.motionRequestInFlight) return;
  appState.motionRequestInFlight = true;
  while (appState.pendingMotionRequest) {
    const request = appState.pendingMotionRequest;
    appState.pendingMotionRequest = null;
    try {
      const response = await fetch("/api/control/velocity", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(request),
        keepalive: request.active === false,
      });
      const result = await response.json();
      if (!response.ok || !result.accepted) {
        throw new Error(result.error || `HTTP ${response.status}`);
      }
    } catch (error) {
      const feedback = $("motionFeedback");
      feedback.className = "error";
      feedback.textContent = `运动请求失败：${controlError(error.message)}`;
      appState.motionEnabled = false;
      appState.keyboardMotionKeys.clear();
      appState.pointerMotionKeys.clear();
      renderMotionVector(currentMotionVector());
      break;
    }
  }
  appState.motionRequestInFlight = false;
}

function queueMotionRequest(vector) {
  appState.pendingMotionRequest = {
    vx: vector.active ? vector.vx : 0,
    vy: vector.active ? vector.vy : 0,
    vyaw: vector.active ? vector.vyaw : 0,
    speed_mode: vector.active ? vector.speedMode : 0,
    active: vector.active,
  };
  flushMotionRequests();
}

function applyMotionInput() {
  const vector = currentMotionVector();
  renderMotionVector(vector);
  if (!appState.motionEnabled || !motionBaseReady()) {
    if (vector.active) releaseAllMotionInput(true);
    return;
  }
  queueMotionRequest(vector);
}

function releaseAllMotionInput(sendStop = true) {
  appState.keyboardMotionKeys.clear();
  appState.pointerMotionKeys.clear();
  const stopped = currentMotionVector();
  renderMotionVector(stopped);
  if (sendStop) queueMotionRequest(stopped);
}

function emergencyStopMotion(reason = "已发送立即停止") {
  releaseAllMotionInput(true);
  const feedback = $("motionFeedback");
  feedback.className = "ok";
  feedback.textContent = reason;
}

function selectMotionPreset(button) {
  if (button.disabled) return;
  if (currentMotionVector().active) {
    emergencyStopMotion("切换速度档位，已先停止当前运动");
  }
  appState.motionPreset = {
    label: button.querySelector("strong").textContent,
    speedMode: Number(button.dataset.speedMode),
    forwardSpeed: Number(button.dataset.forwardSpeed),
    lateralSpeed: Number(button.dataset.lateralSpeed),
    yawSpeed: Number(button.dataset.yawSpeed),
  };
  setText(
    "motionSpeedValue",
    `${appState.motionPreset.label} · ${number(
      appState.motionPreset.forwardSpeed,
      1,
    )} m/s`,
  );
  const feedback = $("motionFeedback");
  feedback.className =
    appState.motionPreset.speedMode === 3 ? "error" : "ok";
  feedback.textContent =
    appState.motionPreset.speedMode === 3
      ? "高速档为官方最高 3.0 m/s，仅可在空旷场地使用"
      : `已选择${appState.motionPreset.label}`;
  renderMotionVector(currentMotionVector());
}

function unlockControl() {
  const hint = $("controlLockHint");
  appState.controlUnlocked = true;
  hint.className = "ok";
  hint.textContent = "本页控制已解锁；每条指令仍需点击一次弹窗确认";
  renderControl(appState.controlData || {});
}

function lockControl() {
  if (appState.motionEnabled) {
    disableMotionControl("本页控制已锁定，运动已停止");
  }
  appState.controlUnlocked = false;
  appState.pendingControl = null;
  const hint = $("controlLockHint");
  hint.className = "";
  hint.textContent = "本页控制已锁定；这只是防误触，不是访问认证";
  if ($("controlConfirmDialog").open) $("controlConfirmDialog").close();
  renderControl(appState.controlData || {});
}

function openControlConfirmation(button) {
  if (button.disabled || !appState.controlUnlocked) return;
  const command = button.dataset.command;
  const actionName =
    command === "execute_custom" ? $("teachActionSelect").value : "";
  if (command === "execute_custom" && !actionName) return;
  const label =
    command === "execute_custom"
      ? `示教动作 ${actionName}`
      : button.dataset.label;
  appState.pendingControl = {
    category: button.dataset.category,
    command,
    argument: Number(button.dataset.argument || 0),
    actionName,
    label,
  };
  setText("controlConfirmTitle", `确认执行：${appState.pendingControl.label}`);
  const targetFsm = Number(button.dataset.fsmId);
  const noBalanceFsm = [0, 1, 2, 3, 4].includes(targetFsm);
  setText(
    "controlConfirmWarning",
    command === "execute_custom"
      ? `即将播放 App 录制的示教动作“${actionName}”。该动作可能包含较大幅度或不可预期的上肢运动，请确认机器人可靠支撑且周围无人。`
      : noBalanceFsm
        ? "该模式没有平衡控制，机器人可能立即失去支撑。确认机器人已经可靠固定。"
        : "该动作可能造成机器人移动、倒地或与周围物体碰撞。确认安全空间充足。",
  );
  $("submitControlCommand").disabled = false;
  setText("controlDialogFeedback", "");
  $("controlConfirmDialog").showModal();
  $("submitControlCommand").focus();
}

function createRequestKey() {
  if (globalThis.crypto?.randomUUID) {
    return crypto.randomUUID();
  }
  return `request-${Date.now()}-${Math.random().toString(36).slice(2, 12)}`;
}

async function submitControl(event) {
  event.preventDefault();
  const pending = appState.pendingControl;
  if (!pending || !appState.controlUnlocked) return;

  const submit = $("submitControlCommand");
  submit.disabled = true;
  if (pending.localAction === "enable_motion") {
    enableMotionControl();
    $("controlConfirmDialog").close();
    appState.pendingControl = null;
    return;
  }
  setText("controlDialogFeedback", "正在提交一次性控制指令…");
  try {
    const response = await fetch("/api/control/command", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        request_key: createRequestKey(),
        category: pending.category,
        command: pending.command,
        argument: pending.argument,
        action_name: pending.actionName || "",
        confirmed: true,
      }),
    });
    const result = await response.json();
    if (!response.ok || !result.accepted) {
      throw new Error(result.error || `HTTP ${response.status}`);
    }
    $("controlConfirmDialog").close();
    const hint = $("controlLockHint");
    hint.className = "ok";
    hint.textContent = `控制请求 #${result.request_id} 已接受，等待机器人状态确认`;
    appState.pendingControl = null;
  } catch (error) {
    setText("controlDialogFeedback", `提交失败：${controlError(error.message)}`);
    submit.disabled = false;
  }
}

function renderSources(sources) {
  const strip = $("sourceStrip");
  strip.replaceChildren();
  for (const [key, label] of Object.entries(sourceLabels)) {
    const source = sources[key] || { status: "offline", age_ms: null };
    const item = document.createElement("div");
    item.className = "source-item";

    const name = document.createElement("span");
    name.textContent = label;

    const state = document.createElement("span");
    state.className = `state-chip ${source.status || "offline"}`;
    const age =
      source.age_ms === null || source.age_ms === undefined
        ? ""
        : ` · ${integer(source.age_ms)}ms`;
    state.textContent = `${statusLabels[source.status] || "未知"}${age}`;

    item.append(name, state);
    strip.append(item);
  }
}

function hex32(value) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) {
    return "--";
  }
  return `0x${(Number(value) >>> 0).toString(16).toUpperCase().padStart(8, "0")}`;
}

function decodeMotorState(value) {
  const raw = Number(value) >>> 0;
  if (raw === 0) return [];

  let knownMask = 0;
  const labels = [];
  motorStateFaults.forEach(([mask, label]) => {
    knownMask = (knownMask | mask) >>> 0;
    if ((raw & mask) !== 0) labels.push(label);
  });
  const unknown = (raw & ~knownMask) >>> 0;
  if (unknown !== 0) labels.push(`未知位 ${hex32(unknown)}`);
  return labels;
}

function maxMotorTemperature(joints, index) {
  let result = null;
  joints.forEach((joint) => {
    const value = Number(joint?.temperature_raw?.[index]);
    if (!Number.isFinite(value)) return;
    if (!result || value > result.value) {
      result = { value, name: joint.name || `motor ${integer(joint.index)}` };
    }
  });
  return result;
}

function renderDiagnostics(data) {
  const sources = data.sources || {};
  const joints = Array.isArray(data.joints) ? data.joints : [];
  const reserved = Array.isArray(data.reserved_motor_slots)
    ? data.reserved_motor_slots
    : [];
  const battery = data.battery || {};
  const mainboard = data.mainboard || {};
  const odometry = data.odometry || {};
  const control = data.control || {};
  const voice = data.voice || {};

  const sourceEntries = Object.entries(sourceLabels).map(([key, label]) => [
    key,
    label,
    sources[key] || { status: "offline", age_ms: null },
  ]);
  const onlineCount = sourceEntries.filter(([, , source]) => source.status === "online").length;
  const offlineSources = sourceEntries.filter(([, , source]) => source.status === "offline");
  const delayedSources = sourceEntries.filter(([, , source]) => source.status === "delayed");
  const lowState = sources.low_state || {};

  setText("diagnosticDdsState", data.dds_initialized ? "已初始化" : "初始化失败");
  setText("diagnosticSourceHealth", `${onlineCount}/${sourceEntries.length} 在线`);
  setText(
    "diagnosticLowStateAge",
    lowState.age_ms === null || lowState.age_ms === undefined
      ? "--"
      : `${integer(lowState.age_ms)} ms · ${statusLabels[lowState.status] || "未知"}`,
  );
  setText("diagnosticOdomError", hex32(odometry.error_code));

  const bodyFaults = joints.filter((joint) => (Number(joint.state_raw) >>> 0) !== 0);
  const reservedFaults = reserved.filter((motor) => (Number(motor.state_raw) >>> 0) !== 0);
  setText("diagnosticMotorFaultCount", String(bodyFaults.length));

  const shellTemp = maxMotorTemperature(joints, 0);
  const windingTemp = maxMotorTemperature(joints, 1);
  setText(
    "diagnosticMotorShellTemp",
    shellTemp ? `${integer(shellTemp.value)} · ${shellTemp.name}` : "--",
  );
  setText(
    "diagnosticMotorWindingTemp",
    windingTemp ? `${integer(windingTemp.value)} · ${windingTemp.name}` : "--",
  );

  const motorVoltages = joints
    .map((joint) => Number(joint.voltage_v))
    .filter((value) => Number.isFinite(value) && value > 0);
  setText(
    "diagnosticMotorVoltage",
    motorVoltages.length
      ? `${number(Math.min(...motorVoltages), 2)}–${number(Math.max(...motorVoltages), 2)} V`
      : "--",
  );

  const faultList = $("diagnosticMotorFaultList");
  faultList.replaceChildren();
  const allMotorFaults = [...bodyFaults, ...reservedFaults];
  if (allMotorFaults.length === 0) {
    const ok = document.createElement("div");
    ok.className = "diagnostic-fault ok";
    ok.textContent = "当前未发现电机已公开故障位";
    faultList.append(ok);
  } else {
    allMotorFaults.forEach((motor) => {
      const item = document.createElement("div");
      item.className = "diagnostic-fault error";
      const title = document.createElement("strong");
      title.textContent = `${motor.name || `motor ${integer(motor.index)}`} · ${hex32(motor.state_raw)}`;
      const detail = document.createElement("span");
      const labels = decodeMotorState(motor.state_raw);
      detail.textContent = labels.length ? labels.join(" / ") : "未知电机状态";
      item.append(title, detail);
      faultList.append(item);
    });
  }

  const reservedHint = $("diagnosticReservedMotorHint");
  const hasReservedTimeout = reservedFaults.some((motor) => {
    const raw = Number(motor.state_raw) >>> 0;
    return (raw & 0x40000000) !== 0 || (raw & 0x80000000) !== 0;
  });
  reservedHint.hidden = !hasReservedTimeout;
  reservedHint.textContent = hasReservedTimeout
    ? "扩展槽 29–34 出现 PC/电机断联超时时，若未安装灵巧手，请检查 App 外设配置中的灵巧手检测。"
    : "";

  const batteryCells = Array.isArray(battery.cell_voltage_raw)
    ? battery.cell_voltage_raw.map(Number).filter((value) => Number.isFinite(value) && value > 0)
    : [];
  const batteryTemps = Array.isArray(battery.temperature_raw)
    ? battery.temperature_raw.map(Number).filter(Number.isFinite)
    : [];
  setText(
    "diagnosticBatteryHealth",
    `${integer(battery.soc_pct)}% / ${integer(battery.soh_pct)}%`,
  );
  setText(
    "diagnosticBatteryCellRange",
    batteryCells.length
      ? `${integer(Math.min(...batteryCells))}–${integer(Math.max(...batteryCells))}`
      : "--",
  );
  setText(
    "diagnosticBatteryCellDelta",
    batteryCells.length
      ? integer(Math.max(...batteryCells) - Math.min(...batteryCells))
      : "--",
  );
  setText("diagnosticBatteryCurrent", integer(battery.current_raw));
  setText(
    "diagnosticBatteryTemp",
    batteryTemps.length ? integer(Math.max(...batteryTemps)) : "--",
  );
  setText(
    "diagnosticBatteryState",
    Array.isArray(battery.state_raw)
      ? battery.state_raw.map(hex32).join(" / ")
      : "--",
  );

  const mainboardTemps = Array.isArray(mainboard.temperature_raw)
    ? mainboard.temperature_raw.map(Number).filter(Number.isFinite)
    : [];
  setText(
    "diagnosticMainboardTemp",
    mainboardTemps.length ? integer(Math.max(...mainboardTemps)) : "--",
  );
  setText(
    "diagnosticMainboardFan",
    Array.isArray(mainboard.fan_state_raw)
      ? mainboard.fan_state_raw.map(integer).join(" / ")
      : "--",
  );
  setText(
    "diagnosticMainboardValue",
    Array.isArray(mainboard.value_raw)
      ? mainboard.value_raw.map((value) => number(value, 3)).join(" / ")
      : "--",
  );
  setText(
    "diagnosticMainboardState",
    Array.isArray(mainboard.state_raw)
      ? mainboard.state_raw.map(hex32).join(" / ")
      : "--",
  );

  const issues = [];
  if (!data.dds_initialized) {
    issues.push({ level: "error", text: `DDS 初始化失败：${data.dds_error || "请检查网卡与 SDK2 环境"}` });
  }
  if (offlineSources.length) {
    issues.push({
      level: "error",
      text: `数据源离线：${offlineSources.map(([, label]) => label).join(" / ")}`,
    });
  }
  if (delayedSources.length) {
    issues.push({
      level: "warning",
      text: `数据源延迟：${delayedSources.map(([, label]) => label).join(" / ")}`,
    });
  }
  if (bodyFaults.length) {
    const warningOnly = bodyFaults.every(
      (motor) => ((Number(motor.state_raw) >>> 0) & ~0x00020000) === 0,
    );
    issues.push({
      level: warningOnly ? "warning" : "error",
      text: `机身电机有 ${bodyFaults.length} 个 state_raw 非零`,
    });
  }
  if (reservedFaults.length) {
    issues.push({
      level: "warning",
      text: `扩展槽 29–34 有 ${reservedFaults.length} 个 state_raw 非零`,
    });
  }
  if ((Number(odometry.error_code) >>> 0) !== 0) {
    issues.push({
      level: "warning",
      text: `里程计 error_code 非零：${hex32(odometry.error_code)}`,
    });
  }
  if (control.initialization_error) {
    issues.push({ level: "error", text: `控制服务初始化异常：${control.initialization_error}` });
  }
  if (voice.initialization_error) {
    issues.push({ level: "warning", text: `语音服务初始化异常：${voice.initialization_error}` });
  }

  const issueList = $("diagnosticIssueList");
  issueList.replaceChildren();
  if (issues.length === 0) {
    const item = document.createElement("div");
    item.className = "diagnostic-issue ok";
    item.textContent = "当前可判定项未见异常";
    issueList.append(item);
  } else {
    issues.forEach((issue) => {
      const item = document.createElement("div");
      item.className = `diagnostic-issue ${issue.level}`;
      item.textContent = issue.text;
      issueList.append(item);
    });
  }

  const badge = $("diagnosticOverallBadge");
  const hasError = issues.some((issue) => issue.level === "error");
  badge.className = `state-chip ${hasError ? "offline" : issues.length ? "delayed" : "online"}`;
  badge.textContent = issues.length ? `${issues.length} 项需要检查` : "未发现已知故障";
  setText(
    "diagnosticOverallSummary",
    issues.length
      ? "以下项目来自当前实时遥测，请优先处理红色故障，再检查黄色告警。"
      : "DDS 数据源、电机已公开故障位、里程计及服务初始化状态当前未见异常。",
  );
}

function renderBattery(battery) {
  const soc = Math.max(0, Math.min(100, Number(battery.soc_pct) || 0));
  $("batteryRing").style.setProperty("--soc", soc);
  setText("batterySoc", `${integer(battery.soc_pct)}%`);
  setText("topBattery", `${integer(battery.soc_pct)}%`);
  setText("batterySoh", `${integer(battery.soh_pct)}%`);
  setText("batteryCycle", integer(battery.cycle));
  setText("batteryCurrent", integer(battery.current_raw));
  setText("batteryVersion", battery.version || "--");

  const temperatures = Array.isArray(battery.temperature_raw)
    ? battery.temperature_raw.filter(Number.isFinite)
    : [];
  setText(
    "batteryTemp",
    temperatures.length ? integer(Math.max(...temperatures)) : "--",
  );

  const cells = Array.isArray(battery.cell_voltage_raw)
    ? battery.cell_voltage_raw.filter((value) => Number(value) > 0)
    : [];
  setText(
    "batteryCells",
    cells.length
      ? `${integer(Math.min(...cells))}–${integer(Math.max(...cells))}`
      : "--",
  );
}

function renderOdometry(odometry) {
  renderAxes("positionAxes", odometry.position_m);
  renderAxes("velocityAxes", odometry.velocity_m_s);
  setText("bodyHeight", `${number(odometry.body_height_m)} m`);
  setText("yawSpeed", `${number(odometry.yaw_speed_rad_s)} rad/s`);
  setText("odomMode", integer(odometry.mode_raw));
  const error = Number(odometry.error_code);
  setText(
    "odomError",
    Number.isFinite(error) ? `0x${error.toString(16).toUpperCase()}` : "--",
  );
}

function renderAxes(containerId, values) {
  const container = $(containerId);
  const labels = ["X", "Y", "Z"];
  container.replaceChildren();
  labels.forEach((label, index) => {
    const item = document.createElement("div");
    item.className = "axis-value";
    const axis = document.createElement("span");
    axis.textContent = label;
    const value = document.createElement("strong");
    value.textContent = number(values?.[index]);
    item.append(axis, value);
    container.append(item);
  });
}

function renderJoints(joints) {
  const tbody = $("jointTableBody");
  const seen = new Set();

  joints.forEach((joint) => {
    const index = Number(joint.index);
    seen.add(index);
    let record = appState.jointRows.get(index);
    if (!record) {
      const row = document.createElement("tr");
      row.tabIndex = 0;
      row.setAttribute("role", "button");
      const cells = Array.from({ length: 9 }, () =>
        document.createElement("td"),
      );
      cells.forEach((cell) => row.append(cell));
      cells[1].className = "joint-name";
      tbody.append(row);
      record = { row, cells, joint: null };
      appState.jointRows.set(index, record);
    }

    record.joint = joint;
    record.row.dataset.jointName = joint.name || "";
    record.row.dataset.search =
      `${joint.index} ${joint.name || ""} ${joint.name_zh || ""}`.toLowerCase();
    record.row.dataset.state = Number(joint.state_raw) === 0 ? "zero" : "nonzero";

    const cells = record.cells;
    cells[0].textContent = String(joint.index);
    cells[1].replaceChildren();
    const zh = document.createElement("strong");
    zh.textContent = joint.name_zh || "--";
    const en = document.createElement("small");
    en.textContent = joint.name || "--";
    cells[1].append(zh, en);
    cells[2].textContent = integer(joint.mode_raw);
    cells[3].textContent = number(joint.q_rad);
    cells[4].textContent = number(joint.dq_rad_s);
    cells[5].textContent = number(joint.tau_est_nm);
    cells[6].textContent = arrayText(joint.temperature_raw, 0);
    cells[7].textContent = number(joint.voltage_v, 2);
    cells[8].replaceChildren();
    const state = document.createElement("span");
    const isZero = Number(joint.state_raw) === 0;
    state.className = `state-chip ${isZero ? "zero" : "nonzero"}`;
    state.textContent = integer(joint.state_raw);
    cells[8].append(state);
  });

  for (const [index, record] of appState.jointRows) {
    if (!seen.has(index)) {
      record.row.remove();
      appState.jointRows.delete(index);
    }
  }
  applyJointFilter();
}

function applyJointFilter() {
  const query = $("jointSearch").value.trim().toLowerCase();
  const filter = $("jointFilter").value;
  let visible = 0;
  for (const record of appState.jointRows.values()) {
    const matchesSearch = !query || record.row.dataset.search.includes(query);
    const matchesState =
      filter === "all" || record.row.dataset.state === filter;
    const show = matchesSearch && matchesState;
    record.row.hidden = !show;
    if (show) visible += 1;
  }
  $("emptyJoints").hidden = visible !== 0;
}

$("jointSearch").addEventListener("input", applyJointFilter);
$("jointFilter").addEventListener("change", applyJointFilter);
$("jointTableBody").addEventListener("keydown", (event) => {
  if ((event.key === "Enter" || event.key === " ") && event.target.matches("tr")) {
    event.preventDefault();
    event.target.click();
  }
});
$("ttsText").addEventListener("input", () => {
  setText("ttsCharCount", `${$("ttsText").value.length} / 240`);
  updateTtsButton();
});
$("ttsBackend").addEventListener("change", updateTtsBackendState);
updateTtsBackendState();
$("ttsForm").addEventListener("submit", submitTts);
$("enableAsr").addEventListener("click", () => setAsrEnabled(true));
$("disableAsr").addEventListener("click", () => setAsrEnabled(false));
$("llmModeSelect").addEventListener("change", handleLlmModeChange);
["customerLlmApiUrl", "customerLlmModel"].forEach((id) => {
  $(id).addEventListener("input", markLlmConfigDirty);
});
$("customerLlmApiKey").addEventListener("focus", () => {
  if ($("customerLlmApiKey").dataset.savedKeyMask === "true") {
    $("customerLlmApiKey").select();
  }
});
$("customerLlmApiKey").addEventListener("input", () => {
  if ($("customerLlmApiKey").value !== SAVED_API_KEY_MASK) {
    $("customerLlmApiKey").dataset.savedKeyMask = "false";
  }
  markLlmConfigDirty();
});
["customerRolePrompt", "customerWakeWord", "customerQaPairs"].forEach((id) => {
  $(id).addEventListener("input", markCustomerVoiceConfigDirty);
});
$("customerWakeEnabled").addEventListener("change", markCustomerVoiceConfigDirty);
$("customerTtsBackend").addEventListener("change", markCustomerVoiceConfigDirty);
$("saveCustomerVoiceConfig").addEventListener("click", saveCustomerVoiceConfig);
$("openCustomerConfigFile").addEventListener("click", openCustomerConfigFile);
$("reloadCustomerConfigFile").addEventListener("click", () => reloadCustomerConfigFile(false));
$("saveCustomerConfigFile").addEventListener("click", saveCustomerConfigFile);
$("closeCustomerConfigFile").addEventListener("click", () => $("customerConfigFileDialog").close());
$("llmChatForm").addEventListener("submit", submitLlmChat);
document.querySelectorAll("[data-voice-tab]").forEach((button) => {
  button.addEventListener("click", () => setVoiceTab(button.dataset.voiceTab));
  button.addEventListener("keydown", (event) => {
    if (!['ArrowLeft', 'ArrowRight'].includes(event.key)) return;
    event.preventDefault();
    const tabs = ["llm", "asr", "tts"];
    const direction = event.key === 'ArrowRight' ? 1 : -1;
    const index = tabs.indexOf(appState.voiceTab);
    setVoiceTab(tabs[(index + direction + tabs.length) % tabs.length], true);
  });
});
document.querySelectorAll("[data-control-tab]").forEach((button) => {
  button.addEventListener("click", () => setControlTab(button.dataset.controlTab));
  button.addEventListener("keydown", (event) => {
    if (!["ArrowLeft", "ArrowRight"].includes(event.key)) return;
    event.preventDefault();
    const tabs = ["mode", "motion", "arm"];
    const direction = event.key === "ArrowRight" ? 1 : -1;
    const index = tabs.indexOf(appState.controlTab);
    setControlTab(tabs[(index + direction + tabs.length) % tabs.length], true);
  });
});
document.querySelectorAll("[data-llm-source]").forEach((button) => {
  button.addEventListener("click", () => setLlmInputMode(button.dataset.llmSource, true));
});
$("llmMessage").addEventListener("input", () => updateLlmComposerState());
$("llmMessage").addEventListener("keydown", (event) => {
  if (event.key !== "Enter" || event.shiftKey || event.isComposing) return;
  event.preventDefault();
  $("llmChatForm").requestSubmit();
});
$("useLatestAsr").addEventListener("click", () => setLlmInputMode("asr"));
$("sendLatestAsrToLlm").addEventListener("click", () => {
  setVoiceTab("llm");
  setLlmInputMode("asr");
  $("sendLlmMessage").focus();
});
$("clearLlmMessage").addEventListener("click", () => {
  $("llmMessage").value = "";
  updateLlmComposerState();
  setLlmInputMode("text", true);
});
$("copyLlmResponse").addEventListener("click", copyLlmResponse);
$("speakLlmResponseNow").addEventListener("click", speakCurrentLlmResponse);
setVoiceTab("llm");
setControlTab("mode");
setLlmInputMode("text");
loadCustomerVoiceConfig();
function setVolumeFromPointer(event) {
  const slider = $("volumeSlider");
  const rect = slider.getBoundingClientRect();
  if (!rect.width) return;
  const ratio = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
  const min = Number(slider.min) || 0;
  const max = Number(slider.max) || 100;
  const step = Number(slider.step) || 1;
  const value = Math.round((min + ratio * (max - min)) / step) * step;
  slider.value = String(Math.max(min, Math.min(max, value)));
  setText("volumeValue", `${slider.value}%`);
}
$("volumeSlider").addEventListener("pointerdown", (event) => {
  event.preventDefault();
  appState.volumeDraft = true;
  appState.volumePointerId = event.pointerId;
  $("volumeSlider").setPointerCapture?.(event.pointerId);
  setVolumeFromPointer(event);
});
$("volumeSlider").addEventListener("pointermove", (event) => {
  if (!appState.volumeDraft || appState.volumePointerId !== event.pointerId) return;
  setVolumeFromPointer(event);
});
$("volumeSlider").addEventListener("pointerup", (event) => {
  if (appState.volumePointerId !== event.pointerId) return;
  setVolumeFromPointer(event);
  $("volumeSlider").releasePointerCapture?.(event.pointerId);
  appState.volumePointerId = null;
  applyVolume();
});
$("volumeSlider").addEventListener("input", () => {
  appState.volumeDraft = true;
  setText("volumeValue", `${$("volumeSlider").value}%`);
});
$("volumeSlider").addEventListener("change", applyVolume);
$("volumeSlider").addEventListener("pointercancel", () => {
  appState.volumeDraft = false;
  appState.volumePointerId = null;
});
$("applyVolume").addEventListener("click", applyVolume);
$("unlockControl").addEventListener("click", unlockControl);
$("lockControl").addEventListener("click", lockControl);
$("teachActionSelect").addEventListener("change", updateControlButtons);
document.querySelectorAll(".control-command").forEach((button) => {
  button.addEventListener("click", () => openControlConfirmation(button));
});
$("controlConfirmForm").addEventListener("submit", submitControl);
$("controlConfirmDialog").addEventListener("close", () => {
  appState.pendingControl = null;
});
$("cancelControlCommand").addEventListener("click", () => {
  appState.pendingControl = null;
  $("controlConfirmDialog").close();
});
$("enableMotionControl").addEventListener(
  "click",
  openMotionEnableConfirmation,
);
$("disableMotionControl").addEventListener("click", () => {
  disableMotionControl();
});
$("motionStopButton").addEventListener("click", () => {
  emergencyStopMotion();
});
document.querySelectorAll(".motion-speed-button").forEach((button) => {
  button.addEventListener("click", () => selectMotionPreset(button));
});

document.querySelectorAll(".motion-pad-button").forEach((button) => {
  button.addEventListener("pointerdown", (event) => {
    if (button.disabled || !appState.motionEnabled) return;
    event.preventDefault();
    button.setPointerCapture?.(event.pointerId);
    appState.pointerMotionKeys.set(
      event.pointerId,
      button.dataset.motionKey,
    );
    applyMotionInput();
  });
  const releasePointer = (event) => {
    if (!appState.pointerMotionKeys.has(event.pointerId)) return;
    event.preventDefault();
    appState.pointerMotionKeys.delete(event.pointerId);
    applyMotionInput();
  };
  button.addEventListener("pointerup", releasePointer);
  button.addEventListener("pointercancel", releasePointer);
});

function isTypingTarget(target) {
  return (
    target instanceof HTMLElement &&
    (target.isContentEditable ||
      ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName))
  );
}

window.addEventListener("keydown", (event) => {
  const key = event.key.toLowerCase();
  if (!["w", "a", "s", "d", "q", "e"].includes(key) ||
      isTypingTarget(event.target) ||
      !appState.motionEnabled ||
      !motionBaseReady()) {
    return;
  }
  event.preventDefault();
  if (!appState.keyboardMotionKeys.has(key)) {
    appState.keyboardMotionKeys.add(key);
    applyMotionInput();
  }
});

window.addEventListener("keyup", (event) => {
  const key = event.key.toLowerCase();
  if (!["w", "a", "s", "d", "q", "e"].includes(key) ||
      !appState.keyboardMotionKeys.has(key)) {
    return;
  }
  event.preventDefault();
  appState.keyboardMotionKeys.delete(key);
  applyMotionInput();
});

window.addEventListener("blur", () => {
  if (appState.motionEnabled) {
    emergencyStopMotion("窗口失焦，已自动停止运动");
  }
});

document.addEventListener("visibilitychange", () => {
  if (document.hidden && appState.motionEnabled) {
    emergencyStopMotion("页面进入后台，已自动停止运动");
  }
});

window.addEventListener("g1:workspace-change", (event) => {
  if (event.detail?.workspace !== "console" && appState.motionEnabled) {
    disableMotionControl("已离开控制区，运动已自动停止");
  }
});

window.addEventListener("g1:console-region-change", (event) => {
  if (event.detail?.region !== "control" && appState.motionEnabled) {
    disableMotionControl("已离开控制区，运动已自动停止");
  }
});

window.addEventListener("pagehide", () => {
  if (!appState.motionEnabled) return;
  fetch("/api/control/velocity", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      vx: 0,
      vy: 0,
      vyaw: 0,
      speed_mode: 0,
      active: false,
    }),
    keepalive: true,
  }).catch(() => {});
});

setInterval(() => {
  if (!appState.motionEnabled || !motionBaseReady()) return;
  const vector = currentMotionVector();
  if (vector.active) queueMotionRequest(vector);
}, 100);

setInterval(() => {
  const stale =
    appState.lastMessageAt > 0 && Date.now() - appState.lastMessageAt > 2000;
  if (stale) {
    $("staleBanner").hidden = false;
    setConnection("offline", "遥测数据过期", "保留最后一次数据");
  }
}, 500);

connect();
