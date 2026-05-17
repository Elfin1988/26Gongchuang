const state = {
  ports: [],
  txCount: 0,
  rxCount: 0,
  responseCount: 0,
  eventCount: 0,
  logAutoScroll: {
    protocol: true,
    raw: true,
  },
  unseenLogCount: {
    protocol: 0,
    raw: 0,
  },
  keyboardControl: {
    enabled: false,
    pressedKeys: new Set(),
    pressedOrder: [],
    activeChassisKey: null,
    activeStepperKey: null,
    activeServoKey: null,
    repeatTimer: null,
    lastChassisSignature: "",
    lastStepperSignature: "",
    lastServoSignature: "",
    servoTargetAngle: 0,
  },
  chassisSpeedSamples: [],
  chassisTimeScaleMs: 10000,
  chassisChartPaused: false,
  chassisPausedEndTimeMs: null,
  chassisRenderFramePending: false,
  chassisLastSampleUnixMs: null,
  chassisSampleIntervalsMs: [],
  chassisGridMode: "seconds",
  chassisHoverSampleIndex: null,
  chassisChartView: null,
  chassisWheelTrim: {
    mode: "scale",
    fl: 1,
    fr: 1,
    rl: 1,
    rr: 1,
  },
};

const CHASSIS_MAX_LINEAR_MM_S = 500;
const CHASSIS_MAX_WZ_DPS_X10 = 900;
const MODULE_CHASSIS = 0x10;
const CMD_CHASSIS_RPM_REPORT = 0x81;
const CMD_CHASSIS_SET_WHEEL_TRIM = 0x06;
const PWM_COMMAND_MAX = 100;
const KEYBOARD_REPEAT_MS = 150;
const MAX_CHASSIS_SPEED_SAMPLES = 3600;
const CHASSIS_TIME_SCALE_MIN_MS = 1000;
const CHASSIS_TIME_SCALE_MAX_MS = 180000;
const CHASSIS_MAX_SAMPLE_INTERVAL_HISTORY = 80;
const CHASSIS_TRIM_STORAGE_KEY = "chassisWheelTrim.v1";
const CHASSIS_TRIM_DEFAULT = Object.freeze({
  mode: "scale",
  fl: 1,
  fr: 1,
  rl: 1,
  rr: 1,
});

const SERVO_ANGLE_LIMITS = {
  1: 135,
  2: 135,
  3: 90,
};

const STEPPER_PRESETS = {
  low: { speed: 400, accel: 1200 },
  medium: { speed: 1200, accel: 2500 },
  high: { speed: 2200, accel: 4500 },
};

const SERVO_PRESETS = {
  home: 0,
  "elbow-strike": -135,
};

const KEYBOARD_CHASSIS_MAP = {
  w: { vxSign: 1, vySign: 0, label: "前进" },
  s: { vxSign: -1, vySign: 0, label: "后退" },
  a: { vxSign: 0, vySign: -1, label: "左移" },
  d: { vxSign: 0, vySign: 1, label: "右移" },
};

const KEYBOARD_STEPPER_MAP = {
  ArrowUp: { axisMask: 0x02, direction: 1, label: "Z轴正转" },
  ArrowDown: { axisMask: 0x02, direction: 0, label: "Z轴反转" },
  ArrowLeft: { axisMask: 0x01, direction: 1, label: "Y轴正转" },
  ArrowRight: { axisMask: 0x01, direction: 0, label: "Y轴反转" },
};

const KEYBOARD_SERVO_MAP = {
  q: { direction: 1, limit: 90, label: "放置舵机向 90°" },
  e: { direction: -1, limit: -90, label: "放置舵机向 -90°" },
};

const CHASSIS_SERIES = [
  { key: "fl", label: "FL", color: "#0f766e" },
  { key: "fr", label: "FR", color: "#d97706" },
  { key: "rl", label: "RL", color: "#2563eb" },
  { key: "rr", label: "RR", color: "#b42318" },
];

const elements = {
  portSelect: document.getElementById("port-select"),
  baudrate: document.getElementById("baudrate"),
  dataBits: document.getElementById("data-bits"),
  stopBits: document.getElementById("stop-bits"),
  parity: document.getElementById("parity"),
  deviceMeta: document.getElementById("device-meta"),
  connectionPill: document.getElementById("connection-pill"),
  txCount: document.getElementById("tx-count"),
  rxCount: document.getElementById("rx-count"),
  protocolLog: document.getElementById("protocol-log"),
  rawLog: document.getElementById("raw-log"),
  protocolJumpLatest: document.getElementById("protocol-jump-latest"),
  rawJumpLatest: document.getElementById("raw-jump-latest"),
  rawMode: document.getElementById("raw-mode"),
  rawData: document.getElementById("raw-data"),
  toastStack: document.getElementById("toast-stack"),
  keyboardToggleBtn: document.getElementById("keyboard-toggle-btn"),
  keyboardStatusPill: document.getElementById("keyboard-status-pill"),
  keyboardActiveText: document.getElementById("keyboard-active-text"),
  rpmFl: document.getElementById("rpm-fl"),
  rpmFr: document.getElementById("rpm-fr"),
  rpmRl: document.getElementById("rpm-rl"),
  rpmRr: document.getElementById("rpm-rr"),
  chassisSpeedCanvas: document.getElementById("chassis-speed-chart"),
  chassisSpeedFootnote: document.getElementById("chassis-speed-footnote"),
  chassisTimeScaleSlider: document.getElementById("time-scale-slider"),
  chassisTimeScaleValue: document.getElementById("time-scale-value"),
  chassisSampleRatePill: document.getElementById("sample-rate-pill"),
  chassisPauseButton: document.getElementById("toggle-speed-scroll"),
  chassisGridModeSecondsButton: document.getElementById("grid-mode-seconds"),
  chassisGridMode200msButton: document.getElementById("grid-mode-200ms"),
  chassisChartTooltip: document.getElementById("chassis-chart-tooltip"),
  closedLoopMode: document.getElementById("closed-loop-mode"),
  closedLoopStatus: document.getElementById("closed-loop-status"),
  closedLoopFl: document.getElementById("closed-loop-fl"),
  closedLoopFr: document.getElementById("closed-loop-fr"),
  closedLoopRl: document.getElementById("closed-loop-rl"),
  closedLoopRr: document.getElementById("closed-loop-rr"),
  chassisTrimSummary: document.getElementById("chassis-trim-summary"),
  keyboardTrimSummary: document.getElementById("keyboard-trim-summary"),
  chassisTrimModal: document.getElementById("chassis-trim-modal"),
  chassisTrimSubtitle: document.getElementById("chassis-trim-subtitle"),
  chassisTrimMode: document.getElementById("chassis-trim-mode"),
  chassisTrimFl: document.getElementById("chassis-trim-fl"),
  chassisTrimFr: document.getElementById("chassis-trim-fr"),
  chassisTrimRl: document.getElementById("chassis-trim-rl"),
  chassisTrimRr: document.getElementById("chassis-trim-rr"),
};

async function request(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });

  if (!response.ok) {
    let message = `请求失败: ${response.status}`;
    try {
      const body = await response.json();
      message = body.detail || JSON.stringify(body);
    } catch (_) {
      const text = await response.text();
      if (text) {
        message = text;
      }
    }
    throw new Error(message);
  }
  return response.json();
}

function showToast(message, type = "info", title = "") {
  const toast = document.createElement("section");
  toast.className = `toast ${type}`;
  const resolvedTitle =
    title ||
    {
      error: "操作失败",
      success: "操作成功",
      info: "提示",
    }[type] ||
    "提示";

  toast.innerHTML = `
    <p class="toast-title">${resolvedTitle}</p>
    <p class="toast-message">${message}</p>
  `;

  elements.toastStack.appendChild(toast);
  window.setTimeout(() => {
    toast.remove();
  }, 3600);
}

function parseTrimRatio(value) {
  const ratio = Number(value);
  if (!Number.isFinite(ratio)) {
    return 1;
  }
  return Math.min(2, Math.max(0, Number(ratio.toFixed(2))));
}

function parseTrimOffset(value) {
  const offset = Number(value);
  if (!Number.isFinite(offset)) {
    return 0;
  }
  return Math.min(1000, Math.max(-1000, Math.round(offset)));
}

function getTrimModeLabel(mode) {
  return mode === "offset" ? "固定数值" : "倍率";
}

function getTrimModeConfig(mode) {
  if (mode === "offset") {
    return {
      subtitle: "倍率模式下 1.00 表示不补偿；固定数值模式下 0 表示不补偿。",
      labels: { fl: "FL 固定值", fr: "FR 固定值", rl: "RL 固定值", rr: "RR 固定值" },
      min: "-1000",
      max: "1000",
      step: "1",
      defaultValue: "0",
    };
  }
  return {
    subtitle: "倍率模式下 1.00 表示不补偿；固定数值模式下 0 表示不补偿。",
    labels: { fl: "FL 比例", fr: "FR 比例", rl: "RL 比例", rr: "RR 比例" },
    min: "0",
    max: "2",
    step: "0.01",
    defaultValue: "1.00",
  };
}

function formatTrimSummary(trim = state.chassisWheelTrim) {
  if (trim.mode === "offset") {
    return `纠偏模式：固定数值 | FL ${trim.fl} / FR ${trim.fr} / RL ${trim.rl} / RR ${trim.rr}`;
  }
  return `纠偏模式：倍率 | FL ${trim.fl.toFixed(2)} / FR ${trim.fr.toFixed(2)} / RL ${trim.rl.toFixed(2)} / RR ${trim.rr.toFixed(2)}`;
}

function syncChassisTrimSummary() {
  if (elements.chassisTrimSummary) {
    elements.chassisTrimSummary.textContent = formatTrimSummary();
  }
  if (elements.keyboardTrimSummary) {
    elements.keyboardTrimSummary.textContent = formatTrimSummary();
  }
}

function syncChassisTrimInputs() {
  if (!elements.chassisTrimFl || !elements.chassisTrimMode) {
    return;
  }
  const mode = state.chassisWheelTrim.mode === "offset" ? "offset" : "scale";
  const config = getTrimModeConfig(mode);
  elements.chassisTrimMode.value = mode;
  if (elements.chassisTrimSubtitle) {
    elements.chassisTrimSubtitle.textContent = config.subtitle;
  }
  [
    ["fl", elements.chassisTrimFl, "chassis-trim-fl-label"],
    ["fr", elements.chassisTrimFr, "chassis-trim-fr-label"],
    ["rl", elements.chassisTrimRl, "chassis-trim-rl-label"],
    ["rr", elements.chassisTrimRr, "chassis-trim-rr-label"],
  ].forEach(([key, input, labelId]) => {
    const label = document.getElementById(labelId);
    if (label) {
      label.textContent = config.labels[key];
    }
    input.min = config.min;
    input.max = config.max;
    input.step = config.step;
  });
  if (mode === "offset") {
    elements.chassisTrimFl.value = String(state.chassisWheelTrim.fl);
    elements.chassisTrimFr.value = String(state.chassisWheelTrim.fr);
    elements.chassisTrimRl.value = String(state.chassisWheelTrim.rl);
    elements.chassisTrimRr.value = String(state.chassisWheelTrim.rr);
  } else {
    elements.chassisTrimFl.value = Number(state.chassisWheelTrim.fl).toFixed(2);
    elements.chassisTrimFr.value = Number(state.chassisWheelTrim.fr).toFixed(2);
    elements.chassisTrimRl.value = Number(state.chassisWheelTrim.rl).toFixed(2);
    elements.chassisTrimRr.value = Number(state.chassisWheelTrim.rr).toFixed(2);
  }
}

function loadChassisTrimFromStorage() {
  try {
    const raw = window.localStorage.getItem(CHASSIS_TRIM_STORAGE_KEY);
    if (!raw) {
      return;
    }
    const parsed = JSON.parse(raw);
    const mode = parsed.mode === "offset" ? "offset" : "scale";
    if (mode === "offset") {
      state.chassisWheelTrim = {
        mode,
        fl: parseTrimOffset(parsed.fl),
        fr: parseTrimOffset(parsed.fr),
        rl: parseTrimOffset(parsed.rl),
        rr: parseTrimOffset(parsed.rr),
      };
    } else {
      state.chassisWheelTrim = {
        mode: "scale",
        fl: parseTrimRatio(parsed.fl),
        fr: parseTrimRatio(parsed.fr),
        rl: parseTrimRatio(parsed.rl),
        rr: parseTrimRatio(parsed.rr),
      };
    }
  } catch (_) {
    state.chassisWheelTrim = { ...CHASSIS_TRIM_DEFAULT };
  }
}

function persistChassisTrim() {
  window.localStorage.setItem(CHASSIS_TRIM_STORAGE_KEY, JSON.stringify(state.chassisWheelTrim));
}

function getChassisTrimPayload() {
  return {
    trim_mode: state.chassisWheelTrim.mode === "offset" ? 1 : 0,
    fl_value: state.chassisWheelTrim.mode === "offset" ? state.chassisWheelTrim.fl : Math.round(state.chassisWheelTrim.fl * 100),
    fr_value: state.chassisWheelTrim.mode === "offset" ? state.chassisWheelTrim.fr : Math.round(state.chassisWheelTrim.fr * 100),
    rl_value: state.chassisWheelTrim.mode === "offset" ? state.chassisWheelTrim.rl : Math.round(state.chassisWheelTrim.rl * 100),
    rr_value: state.chassisWheelTrim.mode === "offset" ? state.chassisWheelTrim.rr : Math.round(state.chassisWheelTrim.rr * 100),
  };
}

function closeChassisTrimModal() {
  if (elements.chassisTrimModal) {
    elements.chassisTrimModal.hidden = true;
  }
}

function openChassisTrimModal() {
  syncChassisTrimInputs();
  if (elements.chassisTrimModal) {
    elements.chassisTrimModal.hidden = false;
  }
}

function readChassisTrimFromInputs() {
  const mode = elements.chassisTrimMode?.value === "offset" ? "offset" : "scale";
  if (mode === "offset") {
    return {
      mode,
      fl: parseTrimOffset(getNumber("chassis-trim-fl")),
      fr: parseTrimOffset(getNumber("chassis-trim-fr")),
      rl: parseTrimOffset(getNumber("chassis-trim-rl")),
      rr: parseTrimOffset(getNumber("chassis-trim-rr")),
    };
  }
  return {
    mode: "scale",
    fl: parseTrimRatio(getNumber("chassis-trim-fl")),
    fr: parseTrimRatio(getNumber("chassis-trim-fr")),
    rl: parseTrimRatio(getNumber("chassis-trim-rl")),
    rr: parseTrimRatio(getNumber("chassis-trim-rr")),
  };
}

async function applyChassisTrim(showSuccessToast = true) {
  await sendCommand(MODULE_CHASSIS, CMD_CHASSIS_SET_WHEEL_TRIM, getChassisTrimPayload());
  if (showSuccessToast) {
    showToast(`四个轮子的${getTrimModeLabel(state.chassisWheelTrim.mode)}纠偏已应用到底盘。`, "success", "纠偏已生效");
  }
}

function clampNumberInput(input, notify = false) {
  const rawValue = input.value;
  if (rawValue === "") {
    return;
  }

  const originalValue = Number(rawValue);
  let value = originalValue;
  if (!Number.isFinite(value)) {
    value = 0;
  }

  const minAttr = input.getAttribute("min");
  const maxAttr = input.getAttribute("max");
  const stepAttr = input.getAttribute("step");

  if (minAttr !== null) {
    value = Math.max(Number(minAttr), value);
  }
  if (maxAttr !== null) {
    value = Math.min(Number(maxAttr), value);
  }

  if (stepAttr && stepAttr !== "any") {
    const step = Number(stepAttr);
    if (Number.isFinite(step) && step > 0) {
      const decimals = stepAttr.includes(".") ? stepAttr.split(".")[1].length : 0;
      value = Number(value.toFixed(decimals));
    }
  }

  input.value = String(value);

  if (notify && value !== originalValue) {
    const label = input.closest("label")?.querySelector("span")?.textContent || "当前输入";
    showToast(`${label} 已自动限制为 ${input.min || "-inf"} ~ ${input.max || "+inf"} 范围内。`, "info", "数值已修正");
  }
}

function bindNumberLimits() {
  document.querySelectorAll("input[type='number'], input[data-number-input]").forEach((input) => {
    input.addEventListener("blur", () => clampNumberInput(input, true));
    input.addEventListener("change", () => clampNumberInput(input, true));
  });
}

function applyStepperPreset(presetKey) {
  const preset = STEPPER_PRESETS[presetKey];
  if (!preset) {
    return;
  }

  document.getElementById("stepper-speed").value = String(preset.speed);
  document.getElementById("stepper-accel").value = String(preset.accel);
  document.querySelectorAll("[data-stepper-preset]").forEach((button) => {
    button.classList.toggle("active", button.dataset.stepperPreset === presetKey);
  });
}

function syncServoAngleLimit() {
  const servoId = Number(document.getElementById("servo-id").value);
  const limit = SERVO_ANGLE_LIMITS[servoId] || 90;
  const angleInput = document.getElementById("servo-angle");
  angleInput.min = String(-limit);
  angleInput.max = String(limit);
  clampNumberInput(angleInput, false);
  syncServoPresetAvailability(limit);
}

function syncServoPresetAvailability(limit) {
  const currentAngle = Number(document.getElementById("servo-angle").value);
  document.querySelectorAll("[data-servo-preset]").forEach((button) => {
    const presetKey = button.dataset.servoPreset;
    const presetAngle = SERVO_PRESETS[presetKey];
    const enabled = Number.isFinite(presetAngle) && Math.abs(presetAngle) <= limit;
    button.disabled = !enabled;
    button.classList.toggle("active", enabled && currentAngle === presetAngle);
  });
}

function applyServoPreset(presetKey) {
  const presetAngle = SERVO_PRESETS[presetKey];
  if (presetAngle === undefined) {
    return;
  }

  const angleInput = document.getElementById("servo-angle");
  angleInput.value = String(presetAngle);
  clampNumberInput(angleInput, true);
  const limit = SERVO_ANGLE_LIMITS[Number(document.getElementById("servo-id").value)] || 90;
  syncServoPresetAvailability(limit);
}

function setConnectionState(connected, label) {
  elements.connectionPill.textContent = label;
  elements.connectionPill.classList.toggle("connected", connected);
}

function formatPortMeta(port) {
  if (!port) {
    return "尚未选择串口";
  }
  return [
    `设备: ${port.device || "-"}`,
    `描述: ${port.description || "-"}`,
    `厂商: ${port.manufacturer || "-"}`,
    `产品: ${port.product || "-"}`,
    `序列号: ${port.serial_number || "-"}`,
    `VID:PID: ${port.vid ?? "-"}:${port.pid ?? "-"}`,
  ].join("\n");
}

function refreshMetaBySelection() {
  const selected = state.ports.find((port) => port.device === elements.portSelect.value);
  elements.deviceMeta.textContent = formatPortMeta(selected);
}

function isNearBottom(container, threshold = 24) {
  return container.scrollHeight - container.scrollTop - container.clientHeight <= threshold;
}

function updateLogAutoScrollState(container) {
  const key = container.id === "protocol-log" ? "protocol" : "raw";
  state.logAutoScroll[key] = isNearBottom(container);
  if (state.logAutoScroll[key]) {
    state.unseenLogCount[key] = 0;
    updateJumpLatestButton(key);
  }
}

function scrollLogToBottom(container) {
  container.scrollTop = container.scrollHeight;
}

function updateJumpLatestButton(key) {
  const button = key === "protocol" ? elements.protocolJumpLatest : elements.rawJumpLatest;
  const count = state.unseenLogCount[key];
  if (count > 0) {
    button.hidden = false;
    button.textContent = count > 1 ? `有 ${count} 条新消息，点击跳到底部` : "有新消息，点击跳到底部";
  } else {
    button.hidden = true;
  }
}

function bindLogScrollBehavior() {
  [elements.protocolLog, elements.rawLog].forEach((container) => {
    container.addEventListener("scroll", () => updateLogAutoScrollState(container));
  });
  elements.protocolJumpLatest.addEventListener("click", () => {
    scrollLogToBottom(elements.protocolLog);
    state.logAutoScroll.protocol = true;
    state.unseenLogCount.protocol = 0;
    updateJumpLatestButton("protocol");
  });
  elements.rawJumpLatest.addEventListener("click", () => {
    scrollLogToBottom(elements.rawLog);
    state.logAutoScroll.raw = true;
    state.unseenLogCount.raw = 0;
    updateJumpLatestButton("raw");
  });
}

async function loadPorts() {
  const data = await request("/api/ports");
  state.ports = data.ports;
  elements.portSelect.innerHTML = "";
  if (!state.ports.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "未发现串口";
    elements.portSelect.appendChild(option);
  } else {
    for (const port of state.ports) {
      const option = document.createElement("option");
      option.value = port.device;
      option.textContent = `${port.device} - ${port.description || "未知"}`;
      elements.portSelect.appendChild(option);
    }
  }
  refreshMetaBySelection();
}

function appendLog(container, entry) {
  const key = container.id === "protocol-log" ? "protocol" : "raw";
  const shouldStickToLatest = state.logAutoScroll[key];
  const wrapper = document.createElement("article");
  wrapper.className = "log-entry";
  wrapper.innerHTML = entry;
  container.appendChild(wrapper);
  if (shouldStickToLatest) {
    scrollLogToBottom(container);
    state.unseenLogCount[key] = 0;
    updateJumpLatestButton(key);
  } else {
    state.unseenLogCount[key] += 1;
    updateJumpLatestButton(key);
  }
}

function shouldHideProtocolLogFrame(frame) {
  return Boolean(
    frame?.is_event &&
      frame.module === MODULE_CHASSIS &&
      frame.cmd === CMD_CHASSIS_RPM_REPORT
  );
}

function renderProtocolEvent(kind, frame, timestamp) {
  const badgeClass = kind === "TX" ? "tx" : "rx";
  const typeLabel = frame.is_event ? "事件" : (frame.is_response ? "响应" : "请求");
  const resultBlock = frame.result_name
    ? `<div>结果: <strong>${frame.result_name}</strong> (${frame.result_code})</div>`
    : "";
  const decoded = frame.decoded || {};
  const explanationLines = [];

  if (decoded.summary) {
    explanationLines.push(`<div>${decoded.summary}</div>`);
  }
  if (frame.is_response && decoded.request_module_name && decoded.request_cmd_name) {
    explanationLines.push(
      `<div>回复目标: <strong>${decoded.request_module_name}</strong> / <strong>${decoded.request_cmd_name}</strong></div>`
    );
  }
  if (frame.is_response && decoded.response_format) {
    explanationLines.push(`<div>应答格式: <span class="mono">${decoded.response_format}</span></div>`);
  }
  if (frame.is_response && decoded.result_description) {
    explanationLines.push(`<div>结果说明: ${decoded.result_description}</div>`);
  }

  Object.entries(decoded).forEach(([key, value]) => {
    if (
      [
        "summary",
        "result",
        "result_name",
        "result_description",
        "response_format",
        "request_module_name",
        "request_cmd_name",
      ].includes(key)
    ) {
      return;
    }
    explanationLines.push(`<div>${key}: <span class="mono">${value}</span></div>`);
  });

  const explanationBlock = explanationLines.length
    ? `
      <details class="explain-details">
        <summary>查看解释</summary>
        <div class="explain-block">${explanationLines.join("")}</div>
      </details>
    `
    : "";

  return `
    <div class="topline">
      <span class="badge ${badgeClass}">${kind}</span>
      <span>${timestamp}</span>
    </div>
    <div><strong>${frame.module_name}</strong> / <strong>${frame.command_name}</strong> / 序号 ${frame.seq}</div>
    <div>类型: ${typeLabel}</div>
    <div>标志: 需应答=${frame.need_ack} 响应=${frame.is_response} 事件=${frame.is_event} 错误=${frame.is_error}</div>
    ${resultBlock}
    ${explanationBlock}
    <div>负载: <span class="mono">${frame.payload_hex || "(空)"}</span></div>
    <div>帧: <span class="mono">${frame.frame_hex}</span></div>
  `;
}

function renderRawEvent(kind, hex, text, timestamp, mode = "") {
  const badgeClass = kind === "TX" ? "tx" : "rx";
  const modeLabel =
    mode === "TEXT" ? "文本" : mode === "HEX" ? "HEX" : mode;
  return `
    <div class="topline">
      <span class="badge ${badgeClass}">${kind}${modeLabel ? ` / ${modeLabel}` : ""}</span>
      <span>${timestamp}</span>
    </div>
    <div>HEX: <span class="mono">${hex || "(空)"}</span></div>
    <div>文本: <span class="mono">${text || "(空)"}</span></div>
  `;
}

function renderErrorEvent(message, timestamp) {
  return `
    <div class="topline">
      <span class="badge error">错误</span>
      <span>${timestamp}</span>
    </div>
    <div>${message}</div>
  `;
}

function getChassisProtocolValues() {
  return {
    vx_mm_s: Math.round(getNumber("chassis-vx") * CHASSIS_MAX_LINEAR_MM_S),
    vy_mm_s: Math.round(getNumber("chassis-vy") * CHASSIS_MAX_LINEAR_MM_S),
    wz_dps_x10: Math.round(getNumber("chassis-wz") * CHASSIS_MAX_WZ_DPS_X10),
    timeout_ms: getNumber("chassis-timeout"),
  };
}

function getChassisControlModeValue() {
  return getNumber("chassis-control-mode");
}

function setChassisFieldVisibility(inputId, visible) {
  const input = document.getElementById(inputId);
  const label = input?.closest("label");
  if (!label) {
    return;
  }
  label.classList.toggle("chassis-mode-hidden", !visible);
}

function syncChassisControlModeUi() {
  const isClosedLoop = getChassisControlModeValue() === 1;

  ["chassis-vx", "chassis-vy", "chassis-wz"].forEach((inputId) => {
    setChassisFieldVisibility(inputId, !isClosedLoop);
  });

  if (elements.chassisTrimSummary) {
    elements.chassisTrimSummary.classList.toggle("chassis-mode-hidden", isClosedLoop);
  }

  const trimButton = document.getElementById("chassis-open-trim");
  if (trimButton) {
    trimButton.classList.toggle("chassis-mode-hidden", isClosedLoop);
  }

  [
    "chassis-fl-target",
    "chassis-fr-target",
    "chassis-rl-target",
    "chassis-rr-target",
    "chassis-pid-mask",
    "chassis-kp",
    "chassis-ki",
    "chassis-kd",
  ].forEach((inputId) => {
    setChassisFieldVisibility(inputId, isClosedLoop);
  });

  const applyPidButton = document.getElementById("chassis-apply-pid");
  if (applyPidButton) {
    applyPidButton.classList.toggle("chassis-mode-hidden", !isClosedLoop);
  }
}

function getChassisWheelTargetValues() {
  return {
    fl_target_rpm: getNumber("chassis-fl-target"),
    fr_target_rpm: getNumber("chassis-fr-target"),
    rl_target_rpm: getNumber("chassis-rl-target"),
    rr_target_rpm: getNumber("chassis-rr-target"),
    timeout_ms: getNumber("chassis-timeout"),
  };
}

function getChassisPidValues() {
  return {
    wheel_mask: getNumber("chassis-pid-mask"),
    kp_x1000: getNumber("chassis-kp"),
    ki_x1000: getNumber("chassis-ki"),
    kd_x1000: getNumber("chassis-kd"),
  };
}

function getPwmCommandFromFactor(inputId) {
  return Math.round(getNumber(inputId) * PWM_COMMAND_MAX);
}

function normalizeKey(key) {
  if (typeof key !== "string") {
    return "";
  }
  if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(key)) {
    return key;
  }
  return key.toLowerCase();
}

function shouldIgnoreKeyboardEvent(target) {
  return Boolean(target?.closest("input, textarea, select, button"));
}

function updateKeyboardIndicators() {
  document.querySelectorAll("[data-key-indicator]").forEach((element) => {
    element.classList.toggle("active", state.keyboardControl.pressedKeys.has(element.dataset.keyIndicator));
  });
}

function updateKeyboardSummaryText() {
  if (!elements.keyboardActiveText) {
    return;
  }

  const chassisLabel = state.keyboardControl.activeChassisKey
    ? KEYBOARD_CHASSIS_MAP[state.keyboardControl.activeChassisKey]?.label || state.keyboardControl.activeChassisKey
    : "停止";
  const stepperLabel = state.keyboardControl.activeStepperKey
    ? KEYBOARD_STEPPER_MAP[state.keyboardControl.activeStepperKey]?.label || state.keyboardControl.activeStepperKey
    : "停止";
  const servoLabel = state.keyboardControl.activeServoKey
    ? KEYBOARD_SERVO_MAP[state.keyboardControl.activeServoKey]?.label || state.keyboardControl.activeServoKey
    : "停止";

  elements.keyboardActiveText.textContent = `底盘：${chassisLabel} / 步进：${stepperLabel} / 舵机：${servoLabel}`;
}

function updateKeyboardStatus() {
  if (!elements.keyboardStatusPill || !elements.keyboardToggleBtn || !elements.keyboardActiveText) {
    return;
  }

  const { enabled } = state.keyboardControl;
  elements.keyboardStatusPill.textContent = enabled ? "已启用" : "未启用";
  elements.keyboardStatusPill.classList.toggle("connected", enabled);
  elements.keyboardToggleBtn.textContent = enabled ? "停用键盘控制" : "启用键盘控制";
  updateKeyboardSummaryText();
}

function getKeyboardChassisValues() {
  const factorInput = document.getElementById("keyboard-chassis-factor");
  if (!factorInput) {
    return null;
  }
  const factor = getNumber("keyboard-chassis-factor");
  const speed = Math.round(factor * CHASSIS_MAX_LINEAR_MM_S);
  const key = state.keyboardControl.activeChassisKey;
  const mapping = key ? KEYBOARD_CHASSIS_MAP[key] : null;
  if (!mapping || speed <= 0) {
    return null;
  }
  return {
    vx_mm_s: mapping.vxSign * speed,
    vy_mm_s: mapping.vySign * speed,
    wz_dps_x10: 0,
    timeout_ms: Math.max(KEYBOARD_REPEAT_MS * 2, 250),
  };
}

function getKeyboardStepperValues() {
  const speedInput = document.getElementById("keyboard-stepper-speed");
  const accelInput = document.getElementById("keyboard-stepper-accel");
  if (!speedInput || !accelInput) {
    return null;
  }

  const key = state.keyboardControl.activeStepperKey;
  const mapping = key ? KEYBOARD_STEPPER_MAP[key] : null;
  if (!mapping) {
    return null;
  }
  return {
    axis_mask: mapping.axisMask,
    direction: mapping.direction,
    speed_sps: getNumber("keyboard-stepper-speed"),
    accel_sps2: getNumber("keyboard-stepper-accel"),
  };
}

function getKeyboardServoValues() {
  const stepInput = document.getElementById("keyboard-servo-step");
  if (!stepInput) {
    return null;
  }

  const key = state.keyboardControl.activeServoKey;
  const mapping = key ? KEYBOARD_SERVO_MAP[key] : null;
  if (!mapping) {
    return null;
  }

  const fineStep = getNumber("keyboard-servo-step");
  if (!(fineStep > 0)) {
    return null;
  }

  const nextAngle = Math.max(
    -90,
    Math.min(90, state.keyboardControl.servoTargetAngle + mapping.direction * fineStep)
  );
  state.keyboardControl.servoTargetAngle = nextAngle;

  return {
    servo_id: 3,
    angle_deg_x10: Math.round(nextAngle * 10),
    duration_ms: KEYBOARD_REPEAT_MS,
  };
}

async function stopKeyboardChassis(force = false) {
  if (!force && !state.keyboardControl.lastChassisSignature) {
    return;
  }
  state.keyboardControl.lastChassisSignature = "";
  try {
    await sendCommand(0x10, 0x02);
  } catch (_) {
    // sendCommand already surfaces the error
  }
}

async function stopKeyboardStepper(force = false) {
  if (!force && !state.keyboardControl.lastStepperSignature) {
    return;
  }
  state.keyboardControl.lastStepperSignature = "";
  try {
    await sendCommand(0x20, 0x02, { axis_mask: 0x03 });
  } catch (_) {
    // sendCommand already surfaces the error
  }
}

async function stopKeyboardServo(force = false) {
  if (!force && !state.keyboardControl.lastServoSignature) {
    return;
  }
  state.keyboardControl.lastServoSignature = "";
}

async function syncKeyboardCommands() {
  const chassisPayload = getKeyboardChassisValues();
  const chassisSignature = chassisPayload ? JSON.stringify(chassisPayload) : "";
  if (chassisPayload) {
    if (chassisSignature !== state.keyboardControl.lastChassisSignature) {
      state.keyboardControl.lastChassisSignature = chassisSignature;
      try {
        await sendCommand(0x10, 0x01, chassisPayload);
      } catch (_) {
        state.keyboardControl.lastChassisSignature = "";
      }
    }
  } else if (state.keyboardControl.lastChassisSignature) {
    await stopKeyboardChassis();
  }

  const stepperPayload = getKeyboardStepperValues();
  const stepperSignature = stepperPayload ? JSON.stringify(stepperPayload) : "";
  if (stepperPayload) {
    if (stepperSignature !== state.keyboardControl.lastStepperSignature) {
      state.keyboardControl.lastStepperSignature = stepperSignature;
      try {
        await sendCommand(0x20, 0x01, stepperPayload);
      } catch (_) {
        state.keyboardControl.lastStepperSignature = "";
      }
    }
  } else if (state.keyboardControl.lastStepperSignature) {
    await stopKeyboardStepper();
  }

  const servoPayload = getKeyboardServoValues();
  const servoSignature = servoPayload ? JSON.stringify(servoPayload) : "";
  if (servoPayload) {
    if (servoSignature !== state.keyboardControl.lastServoSignature) {
      state.keyboardControl.lastServoSignature = servoSignature;
      try {
        await sendCommand(0x30, 0x01, servoPayload);
      } catch (_) {
        state.keyboardControl.lastServoSignature = "";
      }
    }
  } else if (state.keyboardControl.lastServoSignature) {
    await stopKeyboardServo();
  }
}

function syncActiveKeyboardControls() {
  const order = state.keyboardControl.pressedOrder.filter((key) => state.keyboardControl.pressedKeys.has(key));
  state.keyboardControl.pressedOrder = order;
  state.keyboardControl.activeChassisKey =
    order.find((key) => Object.prototype.hasOwnProperty.call(KEYBOARD_CHASSIS_MAP, key)) || null;
  state.keyboardControl.activeStepperKey =
    order.find((key) => Object.prototype.hasOwnProperty.call(KEYBOARD_STEPPER_MAP, key)) || null;
  state.keyboardControl.activeServoKey =
    order.find((key) => Object.prototype.hasOwnProperty.call(KEYBOARD_SERVO_MAP, key)) || null;
  updateKeyboardIndicators();
  updateKeyboardStatus();
  void syncKeyboardCommands();
}

function clearKeyboardPressedState() {
  state.keyboardControl.pressedKeys.clear();
  state.keyboardControl.pressedOrder = [];
  state.keyboardControl.activeChassisKey = null;
  state.keyboardControl.activeStepperKey = null;
  state.keyboardControl.activeServoKey = null;
  updateKeyboardIndicators();
  updateKeyboardStatus();
}

async function releaseKeyboardControl() {
  clearKeyboardPressedState();
  await Promise.all([stopKeyboardChassis(), stopKeyboardStepper(), stopKeyboardServo()]);
}

function startKeyboardRepeatTimer() {
  stopKeyboardRepeatTimer();
  state.keyboardControl.repeatTimer = window.setInterval(() => {
    if (!state.keyboardControl.enabled) {
      return;
    }
    state.keyboardControl.lastChassisSignature = "";
    state.keyboardControl.lastStepperSignature = "";
    state.keyboardControl.lastServoSignature = "";
    void syncKeyboardCommands();
  }, KEYBOARD_REPEAT_MS);
}

function stopKeyboardRepeatTimer() {
  if (state.keyboardControl.repeatTimer) {
    window.clearInterval(state.keyboardControl.repeatTimer);
    state.keyboardControl.repeatTimer = null;
  }
}

async function setKeyboardControlEnabled(enabled) {
  if (state.keyboardControl.enabled === enabled) {
    return;
  }

  state.keyboardControl.enabled = enabled;
  if (enabled) {
    startKeyboardRepeatTimer();
    updateKeyboardStatus();
    await applyChassisTrim(false);
    showToast("键盘模式已启用，点击页面空白处后可直接使用 W/A/S/D 和方向键控制。", "success", "键盘控制");
    return;
  }

  stopKeyboardRepeatTimer();
  await releaseKeyboardControl();
  updateKeyboardStatus();
  showToast("键盘模式已停用，底盘和步进已发送停止指令。", "info", "键盘控制");
}

function bindKeyboardMode() {
  if (!elements.keyboardToggleBtn) {
    return;
  }

  updateKeyboardStatus();
  updateKeyboardIndicators();

  elements.keyboardToggleBtn.addEventListener("click", async () => {
    await setKeyboardControlEnabled(!state.keyboardControl.enabled);
  });

  window.addEventListener("keydown", (event) => {
    if (!state.keyboardControl.enabled || shouldIgnoreKeyboardEvent(event.target)) {
      return;
    }

    const key = normalizeKey(event.key);
    const isControlKey =
      Object.prototype.hasOwnProperty.call(KEYBOARD_CHASSIS_MAP, key) ||
      Object.prototype.hasOwnProperty.call(KEYBOARD_STEPPER_MAP, key) ||
      Object.prototype.hasOwnProperty.call(KEYBOARD_SERVO_MAP, key);
    if (!isControlKey) {
      return;
    }

    event.preventDefault();
    if (!state.keyboardControl.pressedKeys.has(key)) {
      state.keyboardControl.pressedKeys.add(key);
      state.keyboardControl.pressedOrder.unshift(key);
      syncActiveKeyboardControls();
    }
  });

  window.addEventListener("keyup", (event) => {
    if (!state.keyboardControl.enabled) {
      return;
    }

    const key = normalizeKey(event.key);
    const wasPressed = state.keyboardControl.pressedKeys.delete(key);
    if (!wasPressed) {
      return;
    }

    event.preventDefault();
    syncActiveKeyboardControls();
  });

  window.addEventListener("blur", () => {
    if (!state.keyboardControl.enabled) {
      return;
    }
    void releaseKeyboardControl();
  });

  document.addEventListener("visibilitychange", () => {
    if (document.hidden && state.keyboardControl.enabled) {
      void releaseKeyboardControl();
    }
  });
}

function getCurrentCommandRequest() {
  const activePanel = document.querySelector(".command-pane.active");
  if (!activePanel) {
    throw new Error("当前没有可复制的指令面板");
  }

  switch (activePanel.id) {
    case "command-pane-chassis":
      if (getChassisControlModeValue() === 1) {
        return {
          module: 0x10,
          cmd: 0x04,
          payload: getChassisWheelTargetValues(),
          needAck: true,
        };
      }
      return {
        module: 0x10,
        cmd: 0x01,
        payload: getChassisProtocolValues(),
        needAck: true,
      };
    case "command-pane-stepper":
      return {
        module: 0x20,
        cmd: 0x01,
        payload: {
          axis_mask: getNumber("stepper-axis"),
          direction: getNumber("stepper-direction"),
          speed_sps: getNumber("stepper-speed"),
          accel_sps2: getNumber("stepper-accel"),
        },
        needAck: true,
      };
    case "command-pane-servo":
      return {
        module: 0x30,
        cmd: 0x01,
        payload: {
          servo_id: getNumber("servo-id"),
          angle_deg_x10: Math.round(Number(document.getElementById("servo-angle").value) * 10),
          duration_ms: getNumber("servo-duration"),
        },
        needAck: true,
      };
    case "command-pane-vacuum":
      return {
        module: 0x40,
        cmd: 0x01,
        payload: {
          pump_on: getNumber("vacuum-pump"),
          valve_on: getNumber("vacuum-valve"),
          timeout_ms: getNumber("vacuum-timeout"),
        },
        needAck: true,
      };
    case "command-pane-friction":
      return getNumber("friction-direction") === -1
        ? {
            module: 0x50,
            cmd: 0x02,
            payload: {},
            needAck: true,
          }
        : {
            module: 0x50,
            cmd: 0x01,
            payload: {
              motor_mask: 0x03,
              direction: getNumber("friction-direction"),
              pwm_command_percent: getPwmCommandFromFactor("friction-pwm"),
            },
            needAck: true,
          };
    case "command-pane-conveyor":
      return {
        module: 0x60,
        cmd: 0x01,
        payload: {
          direction: getNumber("conveyor-direction"),
          pwm_command_percent: getPwmCommandFromFactor("conveyor-pwm"),
          timeout_ms: getNumber("conveyor-timeout"),
        },
        needAck: true,
      };
    default:
      throw new Error("当前面板暂不支持复制 HEX 指令");
  }
}

function bindTabs() {
  document.querySelectorAll(".tab").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".tab").forEach((tab) => tab.classList.remove("active"));
      document.querySelectorAll(".tab-content").forEach((panel) => panel.classList.remove("active"));
      button.classList.add("active");
      document.getElementById(`tab-${button.dataset.tab}`).classList.add("active");
    });
  });
}

function bindModeNavs() {
  document.querySelectorAll("[data-mode-nav]").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll("[data-mode-nav]").forEach((nav) => nav.classList.remove("active"));
      document.querySelectorAll(".control-pane").forEach((pane) => pane.classList.remove("active"));
      button.classList.add("active");
      document.getElementById(`control-pane-${button.dataset.modeNav}`).classList.add("active");
    });
  });
}

function bindCommandPanels() {
  document.querySelectorAll("[data-command-panel]").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll("[data-command-panel]").forEach((nav) => nav.classList.remove("active"));
      document.querySelectorAll(".command-pane").forEach((pane) => pane.classList.remove("active"));
      button.classList.add("active");
      document.getElementById(`command-pane-${button.dataset.commandPanel}`).classList.add("active");
    });
  });
}

async function sendCommand(module, cmd, payload = {}, needAck = true) {
  try {
    await request("/api/command", {
      method: "POST",
      body: JSON.stringify({ module, cmd, payload, needAck }),
    });
  } catch (error) {
    showToast(`发送命令失败：${error.message}`, "error");
    throw error;
  }
}

function getNumber(id) {
  const input = document.getElementById(id);
  if (
    input instanceof HTMLInputElement &&
    (input.type === "number" || input.hasAttribute("data-number-input"))
  ) {
    clampNumberInput(input);
  }
  return Number(input.value);
}

function formatChartTimeLabel(unixMs) {
  if (!Number.isFinite(unixMs)) {
    return "--:--:--";
  }
  const date = new Date(unixMs);
  return date.toLocaleTimeString("zh-CN", {
    hour12: false,
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
}

function formatTimeScaleLabel(durationMs) {
  const seconds = Math.max(1, Math.round(durationMs / 1000));
  if (seconds < 60) {
    return `${seconds} 秒`;
  }
  if (seconds % 60 === 0) {
    return `${seconds / 60} 分钟`;
  }
  return `${(seconds / 60).toFixed(1)} 分钟`;
}

function syncChassisTimeScaleUi() {
  if (elements.chassisTimeScaleSlider) {
    elements.chassisTimeScaleSlider.value = String(Math.round(state.chassisTimeScaleMs / 1000));
  }
  if (elements.chassisTimeScaleValue) {
    elements.chassisTimeScaleValue.textContent = formatTimeScaleLabel(state.chassisTimeScaleMs);
  }
}

function getChassisSampleRateHz() {
  if (!state.chassisSampleIntervalsMs.length) {
    return 0;
  }
  const total = state.chassisSampleIntervalsMs.reduce((sum, value) => sum + value, 0);
  if (total <= 0) {
    return 0;
  }
  return (state.chassisSampleIntervalsMs.length * 1000) / total;
}

function syncChassisSampleRateUi() {
  if (!elements.chassisSampleRatePill) {
    return;
  }
  const hz = getChassisSampleRateHz();
  elements.chassisSampleRatePill.textContent = hz > 0 ? `${hz.toFixed(1)} Hz` : "-- Hz";
}

function syncChassisPauseUi() {
  if (!elements.chassisPauseButton) {
    return;
  }
  elements.chassisPauseButton.textContent = state.chassisChartPaused ? "继续滚动" : "暂停滚动";
  elements.chassisPauseButton.classList.toggle("is-paused", state.chassisChartPaused);
}

function syncChassisGridModeUi() {
  if (elements.chassisGridModeSecondsButton) {
    elements.chassisGridModeSecondsButton.classList.toggle("active", state.chassisGridMode === "seconds");
  }
  if (elements.chassisGridMode200msButton) {
    elements.chassisGridMode200msButton.classList.toggle("active", state.chassisGridMode === "200ms");
  }
}

function setChassisChartPaused(paused) {
  state.chassisChartPaused = paused;
  if (paused) {
    state.chassisPausedEndTimeMs =
      state.chassisSpeedSamples.length > 0
        ? state.chassisSpeedSamples[state.chassisSpeedSamples.length - 1].unix_ms
        : null;
  } else {
    state.chassisPausedEndTimeMs = null;
  }
  syncChassisPauseUi();
  scheduleChassisChartRender();
}

function getChassisChartWindow(samples) {
  if (!samples.length) {
    return {
      visibleSamples: [],
      startTime: 0,
      endTime: 0,
      timeSpan: 1,
    };
  }

  const liveEndTime = samples[samples.length - 1].unix_ms;
  const endTime = state.chassisChartPaused && Number.isFinite(state.chassisPausedEndTimeMs)
    ? Math.min(state.chassisPausedEndTimeMs, liveEndTime)
    : liveEndTime;
  const startTime = endTime - state.chassisTimeScaleMs;
  const firstVisibleIndex = samples.findIndex((sample) => sample.unix_ms >= startTime);
  const startIndex = firstVisibleIndex <= 0 ? 0 : firstVisibleIndex - 1;
  const endExclusiveIndex = samples.findIndex((sample) => sample.unix_ms > endTime);
  const visibleSamples = samples.slice(startIndex, endExclusiveIndex < 0 ? samples.length : endExclusiveIndex);
  return {
    visibleSamples,
    startTime,
    endTime,
    timeSpan: Math.max(1, state.chassisTimeScaleMs),
  };
}

function getTimeTickStepMs(timeSpan) {
  if (state.chassisGridMode === "200ms" && timeSpan <= 15000) {
    return 200;
  }
  if (timeSpan <= 8000) {
    return 1000;
  }
  if (timeSpan <= 20000) {
    return 2000;
  }
  if (timeSpan <= 45000) {
    return 5000;
  }
  if (timeSpan <= 90000) {
    return 10000;
  }
  if (timeSpan <= 180000) {
    return 30000;
  }
  return 60000;
}

function decimateChassisSamples(samples, startTime, timeSpan, plotWidth) {
  const maxPoints = Math.max(180, Math.floor(plotWidth * 1.6));
  if (samples.length <= maxPoints) {
    return samples;
  }

  const decimated = [];
  let currentBucket = -1;
  let firstSampleInBucket = null;
  let lastSampleInBucket = null;

  const flushBucket = () => {
    if (!firstSampleInBucket) {
      return;
    }
    decimated.push(firstSampleInBucket);
    if (lastSampleInBucket && lastSampleInBucket !== firstSampleInBucket) {
      decimated.push(lastSampleInBucket);
    }
  };

  samples.forEach((sample) => {
    const bucket = Math.max(
      0,
      Math.min(
        Math.floor(plotWidth),
        Math.floor((((sample.unix_ms - startTime) / timeSpan) * plotWidth))
      )
    );

    if (bucket !== currentBucket) {
      flushBucket();
      currentBucket = bucket;
      firstSampleInBucket = sample;
      lastSampleInBucket = sample;
      return;
    }

    lastSampleInBucket = sample;
  });

  flushBucket();
  return decimated;
}

function findNearestChassisSampleIndex(samples, targetUnixMs) {
  if (!samples.length) {
    return -1;
  }

  let low = 0;
  let high = samples.length - 1;
  while (low <= high) {
    const mid = Math.floor((low + high) / 2);
    if (samples[mid].unix_ms < targetUnixMs) {
      low = mid + 1;
    } else if (samples[mid].unix_ms > targetUnixMs) {
      high = mid - 1;
    } else {
      return mid;
    }
  }

  const leftIndex = Math.max(0, high);
  const rightIndex = Math.min(samples.length - 1, low);
  return Math.abs(samples[leftIndex].unix_ms - targetUnixMs) <= Math.abs(samples[rightIndex].unix_ms - targetUnixMs)
    ? leftIndex
    : rightIndex;
}

function formatTooltipTimestamp(unixMs) {
  if (!Number.isFinite(unixMs)) {
    return "--";
  }
  const date = new Date(unixMs);
  const time = date.toLocaleTimeString("zh-CN", {
    hour12: false,
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
  const ms = `${date.getMilliseconds()}`.padStart(3, "0");
  return `${time}.${ms}`;
}

function hideChassisChartTooltip() {
  if (elements.chassisChartTooltip) {
    elements.chassisChartTooltip.hidden = true;
  }
}

function renderChassisChartTooltip(view) {
  const tooltip = elements.chassisChartTooltip;
  if (!tooltip) {
    return;
  }
  if (!view || view.hoverIndex == null || view.hoverIndex < 0 || view.hoverIndex >= view.visibleSamples.length) {
    tooltip.hidden = true;
    return;
  }

  const sample = view.visibleSamples[view.hoverIndex];
  const hoverX = view.toX(sample.unix_ms);
  const hoverY = Math.min(view.toY(sample.fl), view.toY(sample.fr), view.toY(sample.rl), view.toY(sample.rr));
  tooltip.innerHTML = `
    <p class="chart-tooltip-time">${formatTooltipTimestamp(sample.unix_ms)}</p>
    <div class="chart-tooltip-row"><span class="chart-tooltip-label fl">FL</span><strong class="chart-tooltip-value">${sample.fl} rpm</strong></div>
    <div class="chart-tooltip-row"><span class="chart-tooltip-label fr">FR</span><strong class="chart-tooltip-value">${sample.fr} rpm</strong></div>
    <div class="chart-tooltip-row"><span class="chart-tooltip-label rl">RL</span><strong class="chart-tooltip-value">${sample.rl} rpm</strong></div>
    <div class="chart-tooltip-row"><span class="chart-tooltip-label rr">RR</span><strong class="chart-tooltip-value">${sample.rr} rpm</strong></div>
  `;
  tooltip.hidden = false;
  tooltip.style.left = `${Math.max(88, Math.min(view.width - 88, hoverX))}px`;
  tooltip.style.top = `${Math.max(48, hoverY)}px`;
}

function scheduleChassisChartRender() {
  if (state.chassisRenderFramePending) {
    return;
  }
  state.chassisRenderFramePending = true;
  window.requestAnimationFrame(() => {
    state.chassisRenderFramePending = false;
    drawChassisSpeedChart();
  });
}

function drawEmptyChassisSpeedChart(message = "等待底盘速度数据") {
  const canvas = elements.chassisSpeedCanvas;
  if (!(canvas instanceof HTMLCanvasElement)) {
    return;
  }
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return;
  }

  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#fffaf3";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "rgba(31, 41, 51, 0.08)";
  ctx.strokeRect(0.5, 0.5, width - 1, height - 1);

  ctx.fillStyle = "#6b7280";
  ctx.font = "16px Segoe UI";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(message, width / 2, height / 2);
}

function drawChassisSpeedChart() {
  const canvas = elements.chassisSpeedCanvas;
  if (!(canvas instanceof HTMLCanvasElement)) {
    return;
  }
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return;
  }

  const samples = state.chassisSpeedSamples;
  if (!samples.length) {
    drawEmptyChassisSpeedChart();
    return;
  }

  const width = canvas.width;
  const height = canvas.height;
  const padding = { top: 24, right: 26, bottom: 42, left: 58 };
  const plotWidth = width - padding.left - padding.right;
  const plotHeight = height - padding.top - padding.bottom;
  const { visibleSamples, startTime, endTime, timeSpan } = getChassisChartWindow(samples);
  const plotSamples = decimateChassisSamples(visibleSamples, startTime, timeSpan, plotWidth);
  const values = visibleSamples.flatMap((sample) => [sample.fl, sample.fr, sample.rl, sample.rr]);
  const maxValue = Math.max(100, ...values);
  const roundedMax = Math.ceil(maxValue / 100) * 100;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#fffaf3";
  ctx.fillRect(0, 0, width, height);

  ctx.strokeStyle = "rgba(31, 41, 51, 0.10)";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = padding.top + (plotHeight * i) / 4;
    ctx.beginPath();
    ctx.moveTo(padding.left, y);
    ctx.lineTo(width - padding.right, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(31, 41, 51, 0.16)";
  ctx.beginPath();
  ctx.moveTo(padding.left, padding.top);
  ctx.lineTo(padding.left, height - padding.bottom);
  ctx.lineTo(width - padding.right, height - padding.bottom);
  ctx.stroke();

  ctx.fillStyle = "#6b7280";
  ctx.font = "12px Segoe UI";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let i = 0; i <= 4; i += 1) {
    const value = Math.round((roundedMax * (4 - i)) / 4);
    const y = padding.top + (plotHeight * i) / 4;
    ctx.fillText(`${value}`, padding.left - 8, y);
  }

  const toX = (unixMs) => padding.left + (((unixMs - startTime) / timeSpan) * plotWidth);
  const toY = (rpm) => padding.top + plotHeight - ((rpm / roundedMax) * plotHeight);
  const tickStepMs = getTimeTickStepMs(timeSpan);
  const shouldDrawPoints = plotSamples.length <= 180;
  const minorTickStepMs = state.chassisGridMode === "200ms" && tickStepMs >= 1000 ? 200 : null;

  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  if (minorTickStepMs) {
    const firstMinorTick = Math.ceil(startTime / minorTickStepMs) * minorTickStepMs;
    for (let tickTime = firstMinorTick; tickTime <= endTime; tickTime += minorTickStepMs) {
      if (tickTime % tickStepMs === 0) {
        continue;
      }
      const x = toX(tickTime);
      ctx.strokeStyle = "rgba(31, 41, 51, 0.04)";
      ctx.beginPath();
      ctx.moveTo(x, padding.top);
      ctx.lineTo(x, height - padding.bottom);
      ctx.stroke();
    }
  }
  const firstTick = Math.ceil(startTime / tickStepMs) * tickStepMs;
  for (let tickTime = firstTick; tickTime <= endTime; tickTime += tickStepMs) {
    const x = toX(tickTime);
    ctx.strokeStyle = "rgba(31, 41, 51, 0.08)";
    ctx.beginPath();
    ctx.moveTo(x, padding.top);
    ctx.lineTo(x, height - padding.bottom);
    ctx.stroke();
    ctx.fillStyle = "#6b7280";
    ctx.fillText(formatChartTimeLabel(tickTime), x, height - padding.bottom + 10);
  }

  ctx.fillStyle = "#6b7280";
  ctx.fillText(formatChartTimeLabel(startTime), padding.left, height - padding.bottom + 10);
  ctx.fillText(formatChartTimeLabel(endTime), width - padding.right, height - padding.bottom + 10);

  ctx.save();
  ctx.beginPath();
  ctx.rect(padding.left, padding.top, plotWidth, plotHeight);
  ctx.clip();
  CHASSIS_SERIES.forEach(({ key, color }) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    plotSamples.forEach((sample, index) => {
      const x = toX(sample.unix_ms);
      const y = toY(sample[key]);
      if (index === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();

    if (shouldDrawPoints) {
      ctx.fillStyle = color;
      plotSamples.forEach((sample) => {
        const x = toX(sample.unix_ms);
        const y = toY(sample[key]);
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fill();
      });
    }
  });

  if (state.chassisHoverSampleIndex != null && state.chassisHoverSampleIndex >= 0 && state.chassisHoverSampleIndex < visibleSamples.length) {
    const hoverSample = visibleSamples[state.chassisHoverSampleIndex];
    const hoverX = toX(hoverSample.unix_ms);
    ctx.strokeStyle = "rgba(31, 41, 51, 0.42)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(hoverX, padding.top);
    ctx.lineTo(hoverX, height - padding.bottom);
    ctx.stroke();

    CHASSIS_SERIES.forEach(({ key, color }) => {
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(hoverX, toY(hoverSample[key]), 4, 0, Math.PI * 2);
      ctx.fill();
    });
  }
  ctx.restore();

  state.chassisChartView = {
    visibleSamples,
    padding,
    plotWidth,
    plotHeight,
    width,
    height,
    toX,
    toY,
    hoverIndex: state.chassisHoverSampleIndex,
  };
  renderChassisChartTooltip(state.chassisChartView);
}

function updateChassisSpeedCards(sample) {
  if (!elements.rpmFl) {
    return;
  }
  elements.rpmFl.textContent = String(sample.fl);
  elements.rpmFr.textContent = String(sample.fr);
  elements.rpmRl.textContent = String(sample.rl);
  elements.rpmRr.textContent = String(sample.rr);
  const { visibleSamples } = getChassisChartWindow(state.chassisSpeedSamples);
  const hz = getChassisSampleRateHz();
  const pausedText = state.chassisChartPaused ? "，滚动已暂停" : "";
  elements.chassisSpeedFootnote.textContent =
    `最近上报时间：${sample.t}，时间轴：${formatTimeScaleLabel(state.chassisTimeScaleMs)}，频率约 ${hz > 0 ? hz.toFixed(1) : "--"} Hz，显示 ${visibleSamples.length} / ${state.chassisSpeedSamples.length} 个采样点${pausedText}。`;
}

function handleChassisRpmSample(sample) {
  if (Number.isFinite(state.chassisLastSampleUnixMs)) {
    const intervalMs = sample.unix_ms - state.chassisLastSampleUnixMs;
    if (intervalMs > 0) {
      state.chassisSampleIntervalsMs.push(intervalMs);
      if (state.chassisSampleIntervalsMs.length > CHASSIS_MAX_SAMPLE_INTERVAL_HISTORY) {
        state.chassisSampleIntervalsMs.splice(0, state.chassisSampleIntervalsMs.length - CHASSIS_MAX_SAMPLE_INTERVAL_HISTORY);
      }
    }
  }
  state.chassisLastSampleUnixMs = sample.unix_ms;
  state.chassisSpeedSamples.push(sample);
  if (state.chassisSpeedSamples.length > MAX_CHASSIS_SPEED_SAMPLES) {
    state.chassisSpeedSamples.splice(0, state.chassisSpeedSamples.length - MAX_CHASSIS_SPEED_SAMPLES);
  }
  syncChassisSampleRateUi();
  updateChassisSpeedCards(sample);
  if (state.chassisHoverSampleIndex != null) {
    const { visibleSamples } = getChassisChartWindow(state.chassisSpeedSamples);
    state.chassisHoverSampleIndex = visibleSamples.length ? visibleSamples.length - 1 : null;
  }
  if (!state.chassisChartPaused) {
    scheduleChassisChartRender();
  }
}

function handleChassisClosedLoopSample(sample) {
  if (!elements.closedLoopMode) {
    return;
  }

  const modeText = sample.mode === 1 ? "闭环" : "开环";
  const statusFlags = Number(sample.status_flags || 0);
  const statusText = [];
  if (statusFlags & 0x01) {
    statusText.push("超时");
  }
  if (statusFlags & 0x02) {
    statusText.push("急停");
  }

  elements.closedLoopMode.textContent = modeText;
  elements.closedLoopStatus.textContent = statusText.length ? statusText.join(" / ") : "运行中";
  elements.closedLoopFl.textContent = `${sample.fl_target_rpm} / ${sample.fl_pwm}`;
  elements.closedLoopFr.textContent = `${sample.fr_target_rpm} / ${sample.fr_pwm}`;
  elements.closedLoopRl.textContent = `${sample.rl_target_rpm} / ${sample.rl_pwm}`;
  elements.closedLoopRr.textContent = `${sample.rr_target_rpm} / ${sample.rr_pwm}`;
}

function bindActions() {
  const chassisControlMode = document.getElementById("chassis-control-mode");
  if (chassisControlMode) {
    chassisControlMode.addEventListener("change", () => {
      syncChassisControlModeUi();
    });
  }

  document.getElementById("refresh-ports").addEventListener("click", loadPorts);
  elements.portSelect.addEventListener("change", refreshMetaBySelection);
  document.getElementById("servo-id").addEventListener("change", syncServoAngleLimit);
  document.getElementById("servo-angle").addEventListener("input", syncServoAngleLimit);

  document.querySelectorAll("[data-stepper-preset]").forEach((button) => {
    button.addEventListener("click", () => applyStepperPreset(button.dataset.stepperPreset));
  });

  document.querySelectorAll("[data-servo-preset]").forEach((button) => {
    button.addEventListener("click", () => applyServoPreset(button.dataset.servoPreset));
  });

  document.getElementById("connect-btn").addEventListener("click", async () => {
    try {
      await request("/api/connect", {
        method: "POST",
        body: JSON.stringify({
          port: elements.portSelect.value,
          baudrate: Number(elements.baudrate.value),
          dataBits: Number(elements.dataBits.value),
          parity: elements.parity.value,
          stopBits: Number(elements.stopBits.value),
        }),
      });
      showToast("串口已打开，可以开始联调。", "success", "连接成功");
    } catch (error) {
      showToast(`打开串口失败：${error.message}`, "error");
    }
  });

  document.getElementById("disconnect-btn").addEventListener("click", async () => {
    await request("/api/disconnect", { method: "POST" });
    showToast("串口已关闭。", "info", "连接已断开");
  });

  document.querySelector("[data-command='ping']").addEventListener("click", () => sendCommand(0x00, 0x01));
  document.querySelector("[data-command='estop']").addEventListener("click", () => sendCommand(0x00, 0x03));
  document.querySelector("[data-command='clear-estop']").addEventListener("click", () => sendCommand(0x00, 0x04));

  document.getElementById("chassis-send").addEventListener("click", async () => {
    if (getChassisControlModeValue() === 1) {
      await sendCommand(0x10, 0x04, getChassisWheelTargetValues());
      return;
    }
    await applyChassisTrim(false);
    await sendCommand(0x10, 0x01, getChassisProtocolValues());
  });
  document.getElementById("chassis-open-trim").addEventListener("click", openChassisTrimModal);
  document.getElementById("keyboard-open-trim").addEventListener("click", openChassisTrimModal);
  document.getElementById("chassis-apply-mode").addEventListener("click", () =>
    sendCommand(0x10, 0x03, { mode: getChassisControlModeValue() })
  );
  document.getElementById("chassis-apply-pid").addEventListener("click", () =>
    sendCommand(0x10, 0x05, getChassisPidValues())
  );
  document.getElementById("chassis-stop").addEventListener("click", () => sendCommand(0x10, 0x02));
  document.getElementById("chassis-trim-close").addEventListener("click", closeChassisTrimModal);
  document.querySelectorAll("[data-modal-close='chassis-trim']").forEach((element) => {
    element.addEventListener("click", closeChassisTrimModal);
  });
  document.getElementById("chassis-trim-mode").addEventListener("change", () => {
    const nextMode = elements.chassisTrimMode?.value === "offset" ? "offset" : "scale";
    state.chassisWheelTrim = nextMode === "offset"
      ? { mode: "offset", fl: 0, fr: 0, rl: 0, rr: 0 }
      : { ...CHASSIS_TRIM_DEFAULT };
    syncChassisTrimInputs();
  });
  document.getElementById("chassis-trim-reset").addEventListener("click", () => {
    const mode = elements.chassisTrimMode?.value === "offset" ? "offset" : "scale";
    state.chassisWheelTrim = mode === "offset"
      ? { mode: "offset", fl: 0, fr: 0, rl: 0, rr: 0 }
      : { ...CHASSIS_TRIM_DEFAULT };
    syncChassisTrimInputs();
  });
  document.getElementById("chassis-trim-save").addEventListener("click", async () => {
    state.chassisWheelTrim = readChassisTrimFromInputs();
    persistChassisTrim();
    syncChassisTrimSummary();
    syncChassisTrimInputs();
    await applyChassisTrim(true);
    closeChassisTrimModal();
  });
  window.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && elements.chassisTrimModal && !elements.chassisTrimModal.hidden) {
      closeChassisTrimModal();
    }
  });

  document.getElementById("stepper-send").addEventListener("click", () =>
    sendCommand(0x20, 0x01, {
      axis_mask: getNumber("stepper-axis"),
      direction: getNumber("stepper-direction"),
      speed_sps: getNumber("stepper-speed"),
      accel_sps2: getNumber("stepper-accel"),
    })
  );
  document.getElementById("stepper-stop").addEventListener("click", () =>
    sendCommand(0x20, 0x02, {
      axis_mask: getNumber("stepper-axis"),
    })
  );

  document.getElementById("servo-send").addEventListener("click", () =>
    sendCommand(0x30, 0x01, {
      servo_id: getNumber("servo-id"),
      angle_deg_x10: Math.round(Number(document.getElementById("servo-angle").value) * 10),
      duration_ms: getNumber("servo-duration"),
    })
  );

  document.getElementById("vacuum-send").addEventListener("click", () =>
    sendCommand(0x40, 0x01, {
      pump_on: getNumber("vacuum-pump"),
      valve_on: getNumber("vacuum-valve"),
      timeout_ms: getNumber("vacuum-timeout"),
    })
  );
  document.getElementById("vacuum-stop").addEventListener("click", () => sendCommand(0x40, 0x02));

  document.getElementById("friction-send").addEventListener("click", () =>
    getNumber("friction-direction") === -1
      ? sendCommand(0x50, 0x02)
      : sendCommand(0x50, 0x01, {
          motor_mask: 0x03,
          direction: getNumber("friction-direction"),
          pwm_command_percent: getPwmCommandFromFactor("friction-pwm"),
        })
  );
  document.getElementById("friction-stop").addEventListener("click", () => sendCommand(0x50, 0x02));

  document.getElementById("conveyor-send").addEventListener("click", () =>
    sendCommand(0x60, 0x01, {
      direction: getNumber("conveyor-direction"),
      pwm_command_percent: getPwmCommandFromFactor("conveyor-pwm"),
      timeout_ms: getNumber("conveyor-timeout"),
    })
  );
  document.getElementById("conveyor-stop").addEventListener("click", () => sendCommand(0x60, 0x02));

  document.getElementById("raw-send-btn").addEventListener("click", async () => {
    try {
      await request("/api/raw-send", {
        method: "POST",
        body: JSON.stringify({
          mode: elements.rawMode.value,
          data: elements.rawData.value,
        }),
      });
      showToast("原始数据已发送。", "success", "发送成功");
    } catch (error) {
      showToast(`发送失败：${error.message}`, "error");
    }
  });

  document.getElementById("copy-command-hex-btn").addEventListener("click", async () => {
    try {
      const command = getCurrentCommandRequest();
      const result = await request("/api/command-preview", {
        method: "POST",
        body: JSON.stringify(command),
      });
      await navigator.clipboard.writeText(result.frame.frame_hex);
      showToast(`已复制 HEX 指令：${result.frame.frame_hex}`, "success", "复制成功");
    } catch (error) {
      showToast(`复制 HEX 指令失败：${error.message}`, "error");
    }
  });

  document.getElementById("clear-logs").addEventListener("click", () => {
    elements.protocolLog.innerHTML = "";
    elements.rawLog.innerHTML = "";
    state.logAutoScroll.protocol = true;
    state.logAutoScroll.raw = true;
    state.unseenLogCount.protocol = 0;
    state.unseenLogCount.raw = 0;
    updateJumpLatestButton("protocol");
    updateJumpLatestButton("raw");
  });

  const clearSpeedButton = document.getElementById("clear-speed-chart");
  if (clearSpeedButton) {
    clearSpeedButton.addEventListener("click", () => {
      state.chassisSpeedSamples = [];
      state.chassisSampleIntervalsMs = [];
      state.chassisLastSampleUnixMs = null;
      state.chassisPausedEndTimeMs = null;
      state.chassisHoverSampleIndex = null;
      state.chassisChartView = null;
      syncChassisSampleRateUi();
      hideChassisChartTooltip();
      if (elements.rpmFl) {
        elements.rpmFl.textContent = "--";
        elements.rpmFr.textContent = "--";
        elements.rpmRl.textContent = "--";
        elements.rpmRr.textContent = "--";
        elements.chassisSpeedFootnote.textContent = "等待底盘轮速事件上报。";
      }
      drawEmptyChassisSpeedChart("曲线已清空，等待新数据");
    });
  }

  if (elements.chassisTimeScaleSlider) {
    elements.chassisTimeScaleSlider.addEventListener("input", () => {
      const seconds = Number(elements.chassisTimeScaleSlider.value);
      const clampedSeconds = Math.min(
        CHASSIS_TIME_SCALE_MAX_MS / 1000,
        Math.max(CHASSIS_TIME_SCALE_MIN_MS / 1000, Number.isFinite(seconds) ? seconds : 10)
      );
      state.chassisTimeScaleMs = clampedSeconds * 1000;
      syncChassisTimeScaleUi();
      scheduleChassisChartRender();
      if (state.chassisSpeedSamples.length) {
        updateChassisSpeedCards(state.chassisSpeedSamples[state.chassisSpeedSamples.length - 1]);
      }
    });
  }

  if (elements.chassisGridModeSecondsButton) {
    elements.chassisGridModeSecondsButton.addEventListener("click", () => {
      state.chassisGridMode = "seconds";
      syncChassisGridModeUi();
      scheduleChassisChartRender();
    });
  }

  if (elements.chassisGridMode200msButton) {
    elements.chassisGridMode200msButton.addEventListener("click", () => {
      state.chassisGridMode = "200ms";
      syncChassisGridModeUi();
      scheduleChassisChartRender();
    });
  }

  if (elements.chassisPauseButton) {
    elements.chassisPauseButton.addEventListener("click", () => {
      setChassisChartPaused(!state.chassisChartPaused);
    });
  }

  if (elements.chassisSpeedCanvas) {
    elements.chassisSpeedCanvas.addEventListener("mousemove", (event) => {
      const view = state.chassisChartView;
      const canvas = elements.chassisSpeedCanvas;
      if (!view || !(canvas instanceof HTMLCanvasElement) || !view.visibleSamples.length) {
        return;
      }
      const rect = canvas.getBoundingClientRect();
      const scaleX = canvas.width / rect.width;
      const scaleY = canvas.height / rect.height;
      const canvasX = (event.clientX - rect.left) * scaleX;
      const canvasY = (event.clientY - rect.top) * scaleY;
      const inPlotX = canvasX >= view.padding.left && canvasX <= view.padding.left + view.plotWidth;
      const inPlotY = canvasY >= view.padding.top && canvasY <= view.padding.top + view.plotHeight;
      if (!inPlotX || !inPlotY) {
        state.chassisHoverSampleIndex = null;
        hideChassisChartTooltip();
        scheduleChassisChartRender();
        return;
      }

      const ratio = (canvasX - view.padding.left) / view.plotWidth;
      const { startTime, endTime } = getChassisChartWindow(state.chassisSpeedSamples);
      const targetUnixMs = startTime + ratio * Math.max(1, endTime - startTime);
      const hoverIndex = findNearestChassisSampleIndex(view.visibleSamples, targetUnixMs);
      if (hoverIndex !== state.chassisHoverSampleIndex) {
        state.chassisHoverSampleIndex = hoverIndex;
        scheduleChassisChartRender();
      }
    });

    elements.chassisSpeedCanvas.addEventListener("mouseleave", () => {
      state.chassisHoverSampleIndex = null;
      hideChassisChartTooltip();
      scheduleChassisChartRender();
    });
  }
}

function connectWebSocket() {
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(`${protocol}://${location.host}/ws`);

  socket.addEventListener("open", () => {
    socket.send("hello");
  });

  socket.addEventListener("message", (event) => {
    const payload = JSON.parse(event.data);
    switch (payload.type) {
      case "connection":
        setConnectionState(payload.connected, payload.connected ? "串口已连接" : "未连接");
        if (payload.connected) {
          applyChassisTrim(false).catch(() => {});
        }
        break;
      case "protocol_tx":
        state.txCount += 1;
        elements.txCount.textContent = state.txCount;
        appendLog(elements.protocolLog, renderProtocolEvent("TX", payload.frame, payload.timestamp));
        break;
      case "protocol_rx":
        state.rxCount += 1;
        elements.rxCount.textContent = state.rxCount;
        if (!shouldHideProtocolLogFrame(payload.frame)) {
          appendLog(elements.protocolLog, renderProtocolEvent("RX", payload.frame, payload.timestamp));
        }
        break;
      case "response_rx":
        state.responseCount += 1;
        break;
      case "event_rx":
        state.eventCount += 1;
        break;
      case "chassis_rpm_event":
        handleChassisRpmSample(payload.sample);
        break;
      case "chassis_closed_loop_event":
        handleChassisClosedLoopSample(payload.sample);
        break;
      case "raw_tx":
        appendLog(
          elements.rawLog,
          renderRawEvent("TX", payload.hex, payload.text, payload.timestamp, payload.display_mode.toUpperCase())
        );
        break;
      case "raw_rx":
        appendLog(elements.rawLog, renderRawEvent("RX", payload.hex, payload.text, payload.timestamp));
        break;
      case "error":
        appendLog(elements.protocolLog, renderErrorEvent(payload.message, payload.timestamp));
        appendLog(elements.rawLog, renderErrorEvent(payload.message, payload.timestamp));
        showToast(payload.message, "error", "串口错误");
        break;
      default:
        break;
    }
  });

  socket.addEventListener("close", () => {
    setConnectionState(false, "WebSocket 已断开");
    window.setTimeout(connectWebSocket, 1500);
  });
}

async function init() {
  loadChassisTrimFromStorage();
  bindTabs();
  bindModeNavs();
  bindCommandPanels();
  bindNumberLimits();
  bindLogScrollBehavior();
  bindKeyboardMode();
  bindActions();
  syncServoAngleLimit();
  applyStepperPreset("medium");
  syncChassisTrimSummary();
  syncChassisTrimInputs();
  syncChassisControlModeUi();
  syncChassisTimeScaleUi();
  syncChassisSampleRateUi();
  syncChassisPauseUi();
  syncChassisGridModeUi();
  drawEmptyChassisSpeedChart();
  await loadPorts();
  const status = await request("/api/status");
  setConnectionState(status.connected, status.connected ? "串口已连接" : "未连接");
  if (status.connected) {
    applyChassisTrim(false).catch(() => {});
  }

  document.querySelectorAll("[data-mode-nav]").forEach((nav) => nav.classList.remove("active"));
  document.querySelectorAll(".control-pane").forEach((pane) => pane.classList.remove("active"));
  const serialNav = document.querySelector('[data-mode-nav="serial"]');
  const serialPane = document.getElementById("control-pane-serial");
  if (serialNav && serialPane) {
    serialNav.classList.add("active");
    serialPane.classList.add("active");
  }

  document.querySelectorAll("[data-command-panel]").forEach((nav) => nav.classList.remove("active"));
  document.querySelectorAll(".command-pane").forEach((pane) => pane.classList.remove("active"));
  const chassisNav = document.querySelector('[data-command-panel="chassis"]');
  const chassisPane = document.getElementById("command-pane-chassis");
  if (chassisNav && chassisPane) {
    chassisNav.classList.add("active");
    chassisPane.classList.add("active");
  }

  connectWebSocket();
}

init().catch((error) => {
  showToast(`初始化失败：${error.message}`, "error");
});
