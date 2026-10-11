let lastBackendEvent = null;
import {
  createInputState,
  reduceKeyEvent,
  reduceGamepad,
  validateGamepad,
  selectGamepadCandidate,
  reduceBlur,
  reduceVisibilityChange,
  reduceArmState,
  clearStopLatch,
  clearStopRequest,
  latchStop,
  InputClient,
  selectInputDevice as selectInputSource
} from './input_reducer.js';

import { ArmScene } from './arm_scene.js';
import { canAcknowledgeStartupTimeout, deriveDashboardPresentation, deriveWorkflowSteps, finiteNumber as finiteValue, j1GaugePresentation, jointGaugePresentation, jointPositionPresentation, jointVelocityText, serialPortStatus, checkTravelSpan, formatJointHealth, validateConnectionSettings, applyConnectionSettings, draftNumber } from './presentation.js';
import { PreviewPlayer, validateModel, figureEight, followAxis } from './presets.js';
const badgeMode = document.getElementById('badge-mode');
const badgeConnection = document.getElementById('badge-connection');
const badgeState = document.getElementById('badge-state');
const badgeJ1Source = document.getElementById('badge-j1-source');
const badgeProfile = document.getElementById('badge-profile');
const appTitleTag = document.getElementById('app-title-tag');
const btnStop = document.getElementById('btn-stop');

const selectPort = document.getElementById('select-port');
const selectInputDevice = document.getElementById('select-input-device');
const btnRefreshPorts = document.getElementById('btn-refresh-ports');
const inputPortPath = document.getElementById('input-port-path');
const portStatus = document.getElementById('serial-port-status');

const alertBanner = document.getElementById('alert-banner');
const stepConnect = document.getElementById('step-connect');
const stepCheck = document.getElementById('step-check');
const stepZero = document.getElementById('step-zero');
const stepJog = document.getElementById('step-jog');
const workflowHelp = document.getElementById('workflow-help');
const motionIndicator = document.getElementById('motion-indicator');
const j1PosGauge = document.getElementById('j1-pos-gauge');
const j1EffortBar = document.getElementById('j1-effort-bar');
const j1Gauge = {
  container: document.getElementById('j1-pos-gauge-container'),
  status: document.getElementById('j1-gauge-status'),
  zeroMarker: document.getElementById('j1-zero-marker'),
  zeroLabel: document.getElementById('j1-zero-label'),
  minimum: document.getElementById('j1-limit-min'),
  maximum: document.getElementById('j1-limit-max'),
  effortTrack: document.getElementById('j1-effort-track')
};
const btnJogJ1Neg = document.getElementById('btn-jog-j1-neg');
const btnJogJ1Pos = document.getElementById('btn-jog-j1-pos');
const btnClearLog = document.getElementById('btn-clear-log');

let alertHideTimeout = null;
let stateAlert = null;
let transientAlert = null;
let visibleAlert = null;
function renderAlert() {
  if (!alertBanner) return;
  const next = stateAlert || transientAlert;
  if (!next) {
    visibleAlert = null;
    alertBanner.style.display = 'none';
    return;
  }
  if (visibleAlert?.message === next.message && visibleAlert?.type === next.type) return;
  visibleAlert = next;
  alertBanner.textContent = next.message;
  alertBanner.className = `alert-banner ${next.type}`;
  alertBanner.style.display = 'flex';
}

function setStateAlert(alert) {
  stateAlert = alert || null;
  renderAlert();
}

function showAlert(message, type = 'info', durationMs = 0) {
  if (!alertBanner) return;
  if (alertHideTimeout) clearTimeout(alertHideTimeout);
  transientAlert = { message, type };
  renderAlert();
  alertHideTimeout = durationMs > 0 ? setTimeout(() => {
    transientAlert = null;
    alertHideTimeout = null;
    renderAlert();
  }, durationMs) : null;
}

let knownPorts = null;

function updatePortOptions(ports, defaultPort) {
  if (!Array.isArray(ports)) return;
  const currentVal = selectPort.value;
  const portsJson = JSON.stringify(ports);
  if (portsJson === JSON.stringify(knownPorts) && selectPort.options.length > 0) {
    return;
  }
  knownPorts = ports;

  selectPort.innerHTML = '';
  const placeholder = document.createElement('option');
  placeholder.value = '';
  placeholder.textContent = isDemoMode === true ? 'Simulated motor — no USB adapter required'
    : ports.length === 0 ? 'No USB adapter detected' : 'Choose the motor adapter';
  selectPort.appendChild(placeholder);
  let selectedValue = '';
  for (const p of ports) {
    if (!p || typeof p.path !== 'string' || !p.path) continue;
    const opt = document.createElement('option');
    const path = p.path;
    opt.value = path;
    opt.dataset.channelD = p.channel_d === true ? 'true' : 'false';
    const label = typeof p.label === 'string' ? p.label : path;
    opt.textContent = label;
    if (path === currentVal) selectedValue = path;
    else if (!selectedValue && defaultPort && path === defaultPort) selectedValue = path;
    selectPort.appendChild(opt);
  }
  selectPort.value = selectedValue;
}

function updatePortStatus(ports, error = null) {
  const records = Array.isArray(ports) ? ports.filter(Boolean) : [];
  const message = serialPortStatus(records, error, isDemoMode === true);
  if (!portStatus) return message;
  portStatus.dataset.state = isDemoMode === true ? 'demo' : error ? 'error'
    : records.length === 0 ? 'missing'
      : records.some(port => port.channel_d === true) ? 'ready' : 'other';
  portStatus.textContent = message;
  return message;
}

btnRefreshPorts.addEventListener('click', async () => {
  try {
    btnRefreshPorts.disabled = true;
    const res = await fetch('/api/ports');
    if (res.ok) {
      const data = await res.json();
      updatePortOptions(data.ports, data.default);
      updatePortStatus(data.ports, data.error);
      const ports = Array.isArray(data.ports) ? data.ports.filter(Boolean) : [];
      const channelDListed = ports.some(port => port.channel_d === true);
      const summary = data.error ? `Adapter scan failed: ${data.error}`
        : ports.length === 0 ? 'No USB adapters found; Channel D not listed.'
          : `${ports.length} USB adapter${ports.length === 1 ? '' : 's'} listed; Channel D ${channelDListed ? 'found' : 'not identified'}.`;
      logEvent(summary);
    } else {
      updatePortStatus(null, `Port refresh failed (HTTP ${res.status})`);
      logEvent(`Failed to refresh ports: HTTP ${res.status}`, 'error');
    }
  } catch (err) {
    updatePortStatus(null, `Port refresh failed: ${err.message}`);
    logEvent(`Failed to refresh ports: ${err.message}`, 'error');
  } finally {
    btnRefreshPorts.disabled = false;
  }
});
const selectProfile = document.getElementById('select-profile');
const groupThreeSupport = document.getElementById('group-three-support');
const chkThreeSupport = document.getElementById('chk-three-support');
const inputBaud = document.getElementById('input-baud');
const inputId = document.getElementById('input-id');
const btnConnect = document.getElementById('btn-connect');
const btnDisconnect = document.getElementById('btn-disconnect');
const btnCheck = document.getElementById('btn-check');
const btnAck = document.getElementById('btn-ack');
const timeoutRecovery = document.getElementById('timeout-recovery');
const chkAck = document.getElementById('chk-ack-fault');

const selectJoint = document.getElementById('select-joint');
const travelSpanWarning = document.getElementById('travel-span-warning');
const btnPreviewMin = document.getElementById('btn-preview-min');
const btnPreviewMax = document.getElementById('btn-preview-max');
const jointsCalSummary = document.getElementById('joints-cal-summary');
const presetsCardSub = document.getElementById('presets-card-sub');
const noteFollowPreview = document.getElementById('note-follow-preview');
const lblFollowPreview = document.getElementById('lbl-follow-preview');
const controlsCardTitle = document.getElementById('controls-card-title');
const controlsCardSub = document.getElementById('controls-card-sub');
const sectionZeroTitle = document.getElementById('section-zero-title');
const sectionZeroSub = document.getElementById('section-zero-sub');
const lblCalAngle = document.getElementById('lbl-cal-angle');
const calHelpText = document.getElementById('cal-help-text');
const labelJogJ1 = document.getElementById('label-jog-j1');
const labelJogJ2 = document.getElementById('label-jog-j2');
const labelJogJ3 = document.getElementById('label-jog-j3');
const kbdJogHelp = document.getElementById('kbd-jog-help');
const xboxJogHelp = document.getElementById('xbox-jog-help');
const gaugeJointTitle = document.getElementById('gauge-joint-title');
const hudJ1 = document.getElementById('hud-j1');
const hudJ2 = document.getElementById('hud-j2');
const hudJ3 = document.getElementById('hud-j3');

const inputCalAngle = document.getElementById('input-cal-angle');
const selectCalDir = document.getElementById('select-cal-dir');
const inputCalLower = document.getElementById('input-cal-lower');
const inputCalUpper = document.getElementById('input-cal-upper');
const btnCalibrate = document.getElementById('btn-calibrate');
const btnZeroJ1 = document.getElementById('btn-zero-j1');

const inputKp = document.getElementById('input-kp');
const inputKd = document.getElementById('input-kd');
const inputMaxRpm = document.getElementById('input-max-rpm');
const inputEffort = document.getElementById('input-effort');
const btnConfigure = document.getElementById('btn-configure');

const chkEnableArm = document.getElementById('chk-enable-arm');
const btnArm = document.getElementById('btn-arm');
const btnRelease = document.getElementById('btn-release');

const sliderJogSpeed = document.getElementById('slider-jog-speed');
const lblJogSpeed = document.getElementById('lbl-jog-speed');
const gamepadStatus = document.getElementById('gamepad-status');

const telLifecycle = document.getElementById('tel-lifecycle');
const telReason = document.getElementById('tel-reason');
const telFeedbackAge = document.getElementById('tel-feedback-age');
const telPendingOp = document.getElementById('tel-pending-op');
const telStopConfirmed = document.getElementById('tel-stop-confirmed');

const telJ1Source = document.getElementById('tel-j1-source');
const telJ1Pos = document.getElementById('tel-j1-pos');
const telJ1Vel = document.getElementById('tel-j1-vel');
const telJ1Torque = document.getElementById('tel-j1-torque');
const telJ1Mode = document.getElementById('tel-j1-mode');
const telJ1Volt = document.getElementById('tel-j1-volt');
const telJ1Temp = document.getElementById('tel-j1-temp');
const telJ1Errors = document.getElementById('tel-j1-errors');
const telJ1StopResult = document.getElementById('tel-j1-stop-result');
const telJ1HealthMsg = document.getElementById('tel-j1-health-msg');

const telJ2Source = document.getElementById('tel-j2-source');
const telHeadingJ2 = document.getElementById('tel-heading-j2');
const telJ2Pos = document.getElementById('tel-j2-pos');
const telJ2Vel = document.getElementById('tel-j2-vel');
const telJ2Torque = document.getElementById('tel-j2-torque');
const telJ2Mode = document.getElementById('tel-j2-mode');
const telJ2Volt = document.getElementById('tel-j2-volt');
const telJ2Temp = document.getElementById('tel-j2-temp');
const telJ2Errors = document.getElementById('tel-j2-errors');
const telJ2StopResult = document.getElementById('tel-j2-stop-result');
const telJ2HealthMsg = document.getElementById('tel-j2-health-msg');

const telJ3Source = document.getElementById('tel-j3-source');
const telHeadingJ3 = document.getElementById('tel-heading-j3');
const telJ3Pos = document.getElementById('tel-j3-pos');
const telJ3Vel = document.getElementById('tel-j3-vel');
const telJ3Torque = document.getElementById('tel-j3-torque');
const telJ3Mode = document.getElementById('tel-j3-mode');
const telJ3Volt = document.getElementById('tel-j3-volt');
const telJ3Temp = document.getElementById('tel-j3-temp');
const telJ3Errors = document.getElementById('tel-j3-errors');
const telJ3StopResult = document.getElementById('tel-j3-stop-result');
const telJ3HealthMsg = document.getElementById('tel-j3-health-msg');
const telCardJ1 = document.getElementById('tel-card-j1');
const telCardJ2 = document.getElementById('tel-card-j2');
const telCardJ3 = document.getElementById('tel-card-j3');

const eventLogBox = document.getElementById('event-log');
const btnResetCam = document.getElementById('btn-reset-cam');
const inputState = createInputState();
const inputClient = new InputClient();
const canvasContainer = document.getElementById('canvas-container');
const armScene = new ArmScene(canvasContainer);
const preview = new PreviewPlayer();
const followPreview = document.getElementById('chk-follow-preview');
let measuredJ1 = null, appliedSettings = null, observedPose = [0, 0, 0];
let followMotion = false, followBlocked = false;

let currentProfileId = 'arm_j1';
let selectedJoint = 'J1';
let setupBusy = false;
let setupGeneration = 0;
const editedJointDrafts = new Set();
let isRangePreview = false;
let rangePreviewAngle = 0;
const perJointSettings = {
  J1: { kp: 30.0, kd: 3.0, max_rpm: 5.0, effort_limit_nm: 4.0, known_angle_deg: 0.0, direction: 1, lower_deg: -90.0, upper_deg: 90.0 },
  J2: { kp: 15.0, kd: 3.0, max_rpm: 0.5, effort_limit_nm: 1.0, known_angle_deg: 0.0, direction: 1, lower_deg: -90.0, upper_deg: 90.0 },
  J3: { kp: 15.0, kd: 3.0, max_rpm: 0.5, effort_limit_nm: 1.0, known_angle_deg: 0.0, direction: 1, lower_deg: -90.0, upper_deg: 90.0 }
};

let isDemoMode = null;
let modeKnown = false;
let isConnected = false;
let ownsMotor = false;
let isArmed = false;
let connectionKnown = false;
let armedKnown = false;
let statePollController = null;
let statePollGeneration = 0;
let statePollUnavailable = false;
let lastKnownState = null;
let lastStateData = null;
let freshFeedback = false;
let lastStateReceivedAt = 0;

function logEvent(msg, level = 'info') {
  const entry = document.createElement('div');
  entry.className = `event-entry ${level}`;
  const timestamp = new Date().toLocaleTimeString();
  entry.textContent = `[${timestamp}] ${msg}`;
  eventLogBox.appendChild(entry);
  eventLogBox.scrollTop = eventLogBox.scrollHeight;

  while (eventLogBox.children.length > 100) {
    eventLogBox.removeChild(eventLogBox.firstChild);
  }
}
async function ensureAction(action, values = {}) {
  const disconnectedProfileSelection = action === 'select_profile' && !isConnected;
  if (!ownsMotor && action !== 'connect' && !disconnectedProfileSelection) {
    throw new Error('This tab is observing; connect here after the current owner disconnects.');
  }
  return inputClient.requestAction(action, values);
}

async function initSession() {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3000);
  try {
    const saved = sessionStorage.getItem('arm-session');
    if (saved) {
      const check = await fetch('/api/session-status', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ session: saved }),
        signal: controller.signal
      });
      if (check.ok) {
        const resume = await check.json();
        inputClient.setSession(saved);
        ownsMotor = resume.controls_motor === true;
        if (Number.isSafeInteger(resume.last_sequence)) inputClient.seq = Math.max(0, resume.last_sequence + 1);
        logEvent(ownsMotor ? 'Controlling tab session resumed.' : 'Observer session resumed; motor controls are disabled.');
        updateActionControls();
        return true;
      }
      if (check.status === 400) sessionStorage.removeItem('arm-session');
      else throw new Error(`HTTP ${check.status}; saved session retained for retry`);
    }
    const resp = await fetch('/api/session', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({}),
      signal: controller.signal
    });
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const data = await resp.json();
    if (typeof data.session !== 'string' || !data.session) throw new Error('Invalid session response');
    inputClient.setSession(data.session);
    sessionStorage.setItem('arm-session', data.session);
    ownsMotor = false;
    logEvent('Observer session created. Connect explicitly to take motor ownership.');
    updateActionControls();
    return true;
  } catch (err) {
    logEvent(`Session connection error: ${err.message}`, 'error');
    setTimeout(initSession, 1500);
    return false;
  } finally {
    clearTimeout(timeout);
  }
}
async function pollState() {
  if (statePollController) return;
  const generation = statePollGeneration;

  const controller = new AbortController();
  statePollController = controller;
  const timeoutId = setTimeout(() => controller.abort(), 2000);

  try {
    const resp = await fetch('/api/state', {
      signal: controller.signal
    });
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const data = await resp.json();
    if (generation === statePollGeneration && statePollController === controller) {
      if (statePollUnavailable) {
        ownsMotor = false;
        updateActionControls();
        if (!await initSession()) throw new Error('Session recovery failed; retrying');
        logEvent('Dashboard server connection restored; session checked.');
      }
      statePollUnavailable = false;
      updateDashboardState(data);
    }
  } catch (err) {
    if (generation === statePollGeneration) {
      if (!statePollUnavailable) {
        logEvent(`Dashboard server is not responding (${err.name === 'AbortError' ? 'response timed out' : err.message}). Check the terminal running ./start_arm.sh.`, 'error');
      }
      statePollUnavailable = true;
      updateActionControls();
      updateStatusPresentation();
    }
  } finally {
    clearTimeout(timeoutId);
    if (statePollController === controller) statePollController = null;
  }
}

function updateProfileUI() {
  const isThreeJoint = currentProfileId === 'arm_j1_j2_j3';
  document.getElementById('connection-note').textContent = isThreeJoint
    ? 'Motor IDs 0 / 1 / 2 · 6 Mbaud. Every motor must return fresh feedback before movement.'
    : 'Motor ID 0 · 6 Mbaud. Connection is confirmed only by fresh J1 feedback.';
  document.getElementById('input-device-label').textContent = isThreeJoint ? 'How to control the arm' : 'How to control J1';
  document.getElementById('enable-support-label').textContent = isThreeJoint
    ? 'Enable movement · All links supported, travel clear, E-stop available'
    : 'Enable movement · Base secured, travel clear, E-stop available';
  document.getElementById('footer-profile-help').textContent = isThreeJoint
    ? 'Three motors on one bus, J1 ID 0 / J2 ID 1 / J3 ID 2. Stop releases torque on all joints.'
    : 'Single physical J1 at ID 0; J2/J3 virtual.';
  btnArm.textContent = isThreeJoint ? 'Enable all 3 joints' : 'Enable J1 movement';
  if (badgeProfile) {
    badgeProfile.textContent = isThreeJoint
      ? 'Profile: 3-Joint Daisychain (IDs 0, 1, 2)'
      : 'Profile: J1 only (1 motor · ID 0)';
  }
  if (appTitleTag) {
    appTitleTag.textContent = isThreeJoint
      ? 'IM6014 Workbench · 3-Joint Daisychain'
      : 'IM6014 Workbench · J1 Only';
  }
  if (groupThreeSupport) {
    groupThreeSupport.style.display = (isThreeJoint && isDemoMode !== true) ? 'block' : 'none';
  }
  if (presetsCardSub) {
    presetsCardSub.textContent = isThreeJoint ? 'Presets: virtual preview only' : 'J2/J3 virtual';
  }
  if (controlsCardTitle) {
    controlsCardTitle.textContent = isThreeJoint ? 'Connect and move 3-joint daisychain' : 'Connect and move arm';
  }
  if (controlsCardSub) {
    controlsCardSub.textContent = isThreeJoint ? '3 motors · IDs 0, 1, 2' : 'One motor · ID 0';
  }
  if (lblFollowPreview) {
    lblFollowPreview.textContent = isThreeJoint
      ? 'Path follow disabled for 3-joint daisychain (presets remain virtual preview)'
      : 'After Enable, let the physical J1 motor follow this path';
  }
  if (noteFollowPreview) {
    noteFollowPreview.textContent = isThreeJoint
      ? 'In 3-joint daisychain mode, presets remain virtual preview. Physical path follow is disabled to prevent uncoordinated motion.'
      : 'Presets move only the colored preview. If path follow is checked, J1 moves automatically after Enable; press STOP, Esc, or controller B to stop it.';
  }
  if (isThreeJoint) {
    followPreview.checked = false;
    followPreview.disabled = true;
  } else {
    followPreview.disabled = isArmed;
  }
  if (labelJogJ1) labelJogJ1.textContent = isThreeJoint ? 'J1 · Base Yaw (ID 0)' : 'J1 · Physical Base Yaw (ID 0)';
  if (labelJogJ2) labelJogJ2.textContent = isThreeJoint ? 'J2 · Shoulder Pitch (ID 1)' : 'J2 · Virtual Shoulder';
  if (labelJogJ3) labelJogJ3.textContent = isThreeJoint ? 'J3 · Elbow Pitch (ID 2)' : 'J3 · Virtual Elbow';
  if (hudJ1) hudJ1.textContent = isThreeJoint ? 'J1 · Base Yaw · ID 0' : 'J1 · physical when connected · ID 0';
  if (hudJ2) hudJ2.textContent = isThreeJoint ? 'J2 · Shoulder Pitch · ID 1' : 'J2 Shoulder · Virtual Simulation';
  if (hudJ3) hudJ3.textContent = isThreeJoint ? 'J3 · Elbow Pitch · ID 2' : 'J3 Elbow · Virtual Simulation';
  if (kbdJogHelp) {
    kbdJogHelp.innerHTML = isThreeJoint
      ? '<strong>Keyboard:</strong> A / D move J1 · W / S move J2 · Q / E move J3 · Esc stops.'
      : '<strong>Keyboard:</strong> A / D move J1 · W / S move preview J2 · Q / E move preview J3 · Esc stops.';
  }
  if (xboxJogHelp) {
    xboxJogHelp.innerHTML = isThreeJoint
      ? '<strong>Xbox:</strong> Left stick X moves J1 · left Y moves J2 · right Y moves J3 · B stops.'
      : '<strong>Xbox:</strong> Left stick X moves J1 · left Y and right Y move preview J2/J3 · B stops.';
  }
  if (telHeadingJ2) telHeadingJ2.textContent = isThreeJoint ? 'Joint 2 · Shoulder (ID 1)' : 'Joint 2 · Shoulder (Virtual)';
  if (telHeadingJ3) telHeadingJ3.textContent = isThreeJoint ? 'Joint 3 · Elbow (ID 2)' : 'Joint 3 · Elbow (Virtual)';
  if (telCardJ2) telCardJ2.className = `control-section ${isThreeJoint ? 'telemetry-card-physical' : 'telemetry-card-virtual'}`;
  if (telCardJ3) telCardJ3.className = `control-section ${isThreeJoint ? 'telemetry-card-physical' : 'telemetry-card-virtual'}`;
}

function updateSceneLimitsForSelectedJoint(currentAngleDeg = null) {
  const lower = draftNumber(inputCalLower.value);
  const upper = draftNumber(inputCalUpper.value);
  if (Number.isFinite(lower) && Number.isFinite(upper) && lower < upper) {
    armScene.setJointLimits(selectedJoint, lower, upper, currentAngleDeg);
  } else {
    armScene.clearJointLimits();
  }
}

function onLimitDraftInput() {
  editedJointDrafts.add(selectedJoint);
  const lower = draftNumber(inputCalLower.value);
  const upper = draftNumber(inputCalUpper.value);
  perJointSettings[selectedJoint].lower_deg = lower;
  perJointSettings[selectedJoint].upper_deg = upper;
  updateParameterWarnings();

  const spanWarn = checkTravelSpan(lower, upper);
  if (spanWarn) {
    travelSpanWarning.style.display = 'block';
    travelSpanWarning.textContent = spanWarn;
  } else {
    travelSpanWarning.style.display = 'none';
    travelSpanWarning.textContent = '';
  }

  const currentJointIdx = selectedJoint === 'J1' ? 0 : selectedJoint === 'J2' ? 1 : 2;
  const currentJoint = lastStateData?.joints?.[currentJointIdx];
  const curPos = finiteValue(currentJoint?.position_rad);
  const curDeg = curPos !== null ? curPos * 180 / Math.PI : draftNumber(inputCalAngle.value) || 0;

  updateSceneLimitsForSelectedJoint(curDeg);
  updateJointGauge(curPos, finiteValue(currentJoint?.torque_nm), {
    lower_deg: lower,
    upper_deg: upper,
    effort_limit_nm: perJointSettings[selectedJoint].effort_limit_nm
  }, selectedJoint);
}

function onSelectedJointChanged() {
  selectedJoint = selectJoint.value;
  isRangePreview = false;

  if (sectionZeroTitle) sectionZeroTitle.textContent = `${selectedJoint} pose reference & travel`;
  if (lblCalAngle) lblCalAngle.textContent = `${selectedJoint} angle at this pose (°)`;
  if (btnZeroJ1) btnZeroJ1.textContent = `Set ${selectedJoint} zero here (0°)`;
  if (gaugeJointTitle) gaugeJointTitle.textContent = `Travel gauge · Selected joint (${selectedJoint})`;

  if (selectedJoint === 'J1') {
    if (calHelpText) calHelpText.innerHTML = 'Before Connect, place J1 at the angle entered above (0° means your zero pose). Connect records that fresh encoder reference without moving the motor. Set zero here changes it while stopped. J1 travel cap is −90° to +90°.';
  } else {
    if (calHelpText) calHelpText.innerHTML = `Before Connect, position ${selectedJoint} at the entered angle. Connect records a fresh reference without moving it. Verify your physical travel limits and support the links; CAD cannot prove clearance.`;
  }

  const s = perJointSettings[selectedJoint];
  inputKp.value = s.kp;
  inputKd.value = s.kd;
  inputMaxRpm.value = s.max_rpm;
  inputEffort.value = s.effort_limit_nm;
  inputCalAngle.value = s.known_angle_deg;
  selectCalDir.value = s.direction;
  inputCalLower.value = s.lower_deg;
  inputCalUpper.value = s.upper_deg;

  onLimitDraftInput();
  updateParameterWarnings();
}

function updateJointGauge(positionRad, torque, applied, jointName = 'J1') {
  const gauge = jointGaugePresentation({
    positionRad,
    lowerDeg: applied?.lower_deg,
    upperDeg: applied?.upper_deg,
    connected: isConnected,
    fresh: freshFeedback,
    jointName
  });
  if (j1Gauge.container) j1Gauge.container.hidden = !gauge.visible;
  if (j1Gauge.status) j1Gauge.status.textContent = gauge.status;
  if (j1Gauge.minimum) j1Gauge.minimum.textContent = gauge.lowerLabel;
  if (j1Gauge.maximum) j1Gauge.maximum.textContent = gauge.upperLabel;
  if (j1Gauge.zeroLabel) j1Gauge.zeroLabel.textContent = gauge.zeroLabel;
  if (j1Gauge.zeroMarker) j1Gauge.zeroMarker.style.left = `${gauge.zeroPercent}%`;
  if (j1PosGauge) {
    j1PosGauge.style.left = `${gauge.visible ? gauge.fillLeftPercent : gauge.zeroPercent}%`;
    j1PosGauge.style.width = `${gauge.visible ? gauge.fillWidthPercent : 0}%`;
  }

  const effortVisible = isConnected && torque !== null;
  if (j1Gauge.effortTrack) j1Gauge.effortTrack.hidden = !effortVisible;
  if (!j1EffortBar) return;
  if (!effortVisible) {
    j1EffortBar.style.width = '0%';
    return;
  }
  const maxEffort = applied?.effort_limit_nm ?? 1.0;
  const effortPct = Math.min(100, (Math.abs(torque) / maxEffort) * 100);
  j1EffortBar.style.width = `${effortPct.toFixed(1)}%`;
  j1EffortBar.style.background = effortPct >= 85 ? 'var(--accent-rose)'
    : effortPct >= 60 ? 'var(--accent-amber)' : 'var(--accent-emerald)';
}

function updateActionControls() {
  const latest = lastStateData?.latest;
  const readyStop = Boolean(!statePollUnavailable && freshFeedback && lastStateData?.stop_confirmed === true
    && lastStateData?.state === 'Ready · Stop' && latest?.mode === 0
    && (latest?.timeout === false || latest?.timeout === 0));
  const canAcknowledgeTimeout = !statePollUnavailable && canAcknowledgeStartupTimeout({
    connected: isConnected,
    owner: ownsMotor,
    fresh: freshFeedback,
    stopConfirmed: lastStateData?.stop_confirmed,
    latest,
    joints: lastStateData?.joints,
    stopLatched: inputState.stopLatched,
    armed: isArmed
  });
  const hasPort = Boolean(inputPortPath?.value.trim() || selectPort.value);
  const canConnect = isDemoMode === true || hasPort;
  selectPort.disabled = isDemoMode === true || isConnected;
  if (inputPortPath) inputPortPath.disabled = selectPort.disabled;
  if (selectProfile) selectProfile.disabled = isConnected;
  if (chkThreeSupport) chkThreeSupport.disabled = isConnected;
  btnConnect.disabled = setupBusy || statePollUnavailable || isConnected || !inputClient.session || !canConnect;
  btnDisconnect.disabled = setupBusy || !ownsMotor || !isConnected;
  btnCheck.disabled = setupBusy || statePollUnavailable || !ownsMotor || !isConnected || isArmed;
  btnAck.disabled = setupBusy || !canAcknowledgeTimeout || !chkAck.checked;
  if (timeoutRecovery) timeoutRecovery.hidden = !isConnected || !String(lastStateData?.state || '').startsWith('Timeout');
  if (!isConnected || !String(lastStateData?.state || '').startsWith('Timeout')) chkAck.checked = false;
  const selectedPhysical = lastStateData?.joints?.find(joint => joint.name === selectedJoint)?.configured === true;
  btnCalibrate.disabled = setupBusy || !selectedPhysical || !ownsMotor || !isConnected || isArmed || !readyStop;
  btnZeroJ1.disabled = btnCalibrate.disabled;
  btnConfigure.disabled = setupBusy || !selectedPhysical || !ownsMotor || !isConnected || isArmed || !readyStop;
  btnArm.disabled = setupBusy || !isConnected || isArmed
    || !ownsMotor || !lastStateData?.calibrated || !readyStop
    || !chkEnableArm.checked || inputState.stopLatched;
  if (!setupBusy) chkEnableArm.checked = isArmed;
  chkEnableArm.disabled = setupBusy || !ownsMotor || !isConnected || !lastStateData?.calibrated || statePollUnavailable;
  selectJoint.disabled = setupBusy || isArmed;
  for (const input of [inputCalAngle, selectCalDir, inputCalLower, inputCalUpper, inputKp, inputKd, inputMaxRpm, inputEffort]) input.disabled = setupBusy || isArmed;
  if (btnPreviewMin) btnPreviewMin.disabled = isArmed;
  if (btnPreviewMax) btnPreviewMax.disabled = isArmed;
  btnRelease.disabled = !ownsMotor || !isConnected || !isArmed;
  if (selectInputDevice) {
    const gamepadOption = selectInputDevice.querySelector('option[value="gamepad"]');
    if (gamepadOption) gamepadOption.disabled = !inputState.gamepadConnected;
    if (document.activeElement !== selectInputDevice) selectInputDevice.value = inputState.activeDevice;
    selectInputDevice.disabled = !ownsMotor || isArmed || inputState.stopLatched;
  }
}

function updateStatusPresentation(data = lastStateData) {
  const presentation = deriveDashboardPresentation({
    profileId: currentProfileId,
    serverUnavailable: statePollUnavailable,
    demo: isDemoMode === true,
    connectionKnown, connected: isConnected, owner: ownsMotor, armedKnown, armed: isArmed,
    fresh: freshFeedback, timeout: String(data?.state || lastKnownState || '').startsWith('Timeout'),
    timeoutAcknowledgementAvailable: !statePollUnavailable && canAcknowledgeStartupTimeout({
      connected: isConnected,
      owner: ownsMotor,
      fresh: freshFeedback,
      stopConfirmed: data?.stop_confirmed,
      latest: data?.latest,
      joints: data?.joints,
      stopLatched: inputState.stopLatched,
      armed: isArmed
    }),
    stopLatched: inputState.stopLatched, stopReason: (String(data?.state).startsWith('Fault') || String(data?.state).startsWith('Stopped')) ? data.reason : inputState.stopReason || data?.reason,
    motionActive: Boolean(inputState.motionEnergized || inputState.isEnergized || followMotion),
    axes: inputState.effectiveAxes, source: inputState.activeDevice,
    followSelected: Boolean(preview.selected && followPreview.checked && currentProfileId === 'arm_j1'),
    followActive: followMotion, followBlocked,
    calibrated: Boolean(data?.calibrated)
  });
  setStateAlert(presentation.alert);
  if (motionIndicator) {
    motionIndicator.textContent = presentation.motion.text;
    motionIndicator.style.color = presentation.motion.color;
  }
  if (workflowHelp) workflowHelp.textContent = presentation.guidance;
}

followPreview.addEventListener('change', () => updateStatusPresentation());

function markFeedbackStale() {
  const elapsed = lastStateReceivedAt ? (performance.now() - lastStateReceivedAt) / 1000 : 0;
  const reportedAge = finiteValue(lastStateData?.feedback_age_s);
  const age = reportedAge === null ? null : reportedAge + elapsed;
  freshFeedback = false;
  badgeConnection.textContent = isConnected ? 'Connected · stale feedback' : 'Disconnected';
  badgeConnection.className = `badge ${isConnected ? 'badge-fault' : 'badge-disconnected'}`;
  telFeedbackAge.textContent = age === null ? 'STALE · no recent state response'
    : `STALE · ${age.toFixed(3)} s since last motor reply`;
  telStopConfirmed.textContent = 'Stale — cached result only';
  badgeJ1Source.textContent = 'STALE feedback';
  telJ1Source.textContent = 'STALE feedback · cached';
  const cachedJoints = lastStateData?.joints || [];
  const cachedSelJoint = cachedJoints[selectedJoint === 'J1' ? 0 : selectedJoint === 'J2' ? 1 : 2];
  updateJointGauge(finiteValue(cachedSelJoint?.position_rad), finiteValue(cachedSelJoint?.torque_nm), cachedSelJoint?.settings || lastStateData?.settings, selectedJoint);
  updateStatusPresentation(lastStateData);
  updateActionControls();
  if (ownsMotor && isArmed && !inputState.stopLatched) void stopNow('state_poll_stale');
}

function expireStateFreshness() {
  if (freshFeedback && lastStateReceivedAt && performance.now() - lastStateReceivedAt > 1000) {
    markFeedbackStale();
  }
}

function updateDashboardState(data) {
  lastStateData = data;
  lastStateReceivedAt = performance.now();
  modeKnown = typeof data.physical_enabled === 'boolean';
  isDemoMode = modeKnown ? !data.physical_enabled : null;
  const portList = Array.isArray(data.ports) ? data.ports : null;
  if (portList) {
    updatePortOptions(portList, data.default_port);
    updatePortStatus(portList, data.port_error);
  }
  connectionKnown = typeof data.connected === 'boolean';
  isConnected = connectionKnown ? data.connected : false;

  if (typeof data.profile_id === 'string' && data.profile_id && data.profile_id !== currentProfileId) {
    currentProfileId = data.profile_id;
    if (selectProfile.value !== currentProfileId) selectProfile.value = currentProfileId;
    updateProfileUI();
  }

  const feedbackAge = finiteValue(data.feedback_age_s);
  freshFeedback = Boolean(isConnected && feedbackAge !== null && feedbackAge >= 0
    && feedbackAge <= 0.1 && data.latest && typeof data.latest === 'object');
  const wasArmed = isArmed;
  if (wasArmed && !freshFeedback && ownsMotor && !inputState.stopLatched) {
    void stopNow('feedback_stale');
  }
  armedKnown = typeof data.armed === 'boolean';
  if (armedKnown && !inputState.stopLatched) {
    isArmed = data.armed && ownsMotor;
    reduceArmState(inputState, isArmed);
  } else if (inputState.stopLatched || !armedKnown) {
    isArmed = false;
    reduceArmState(inputState, false);
    if (wasArmed && !inputState.stopLatched) void stopNow('arm_status_unknown');
  }
  lastKnownState = typeof data.state === 'string' && data.state ? data.state : null;
  if (String(lastKnownState).startsWith('Stopped') || String(lastKnownState).startsWith('Fault')) {
    latchStop(inputState, data.reason || lastKnownState);
    clearStopRequest(inputState);
    isArmed = false;
    reduceArmState(inputState, false);
  }
  if (!modeKnown) {
    badgeMode.textContent = 'Mode Awaiting Status';
    badgeMode.className = 'badge';
  } else if (isDemoMode) {
    badgeMode.textContent = 'Demo Mode (Simulated)';
    badgeMode.className = 'badge badge-mode-demo';
  } else {
    badgeMode.textContent = 'Physical Option';
    badgeMode.className = 'badge badge-mode-physical';
  }

  badgeConnection.textContent = !connectionKnown ? 'Status Awaiting'
    : !isConnected ? 'Disconnected' : freshFeedback ? 'Connected · fresh feedback' : 'Connected · stale feedback';
  badgeConnection.className = `badge ${!isConnected ? 'badge-disconnected' : freshFeedback ? 'badge-connected' : 'badge-fault'}`;

  const isMoving = Boolean(inputState.motionEnergized || inputState.isEnergized || followMotion);
  badgeState.textContent = inputState.stopLatched ? 'STOPPED · Enable when ready'
    : !armedKnown ? 'Awaiting status'
      : (isArmed ? (isMoving ? 'JOGGING' : 'ARMED · READY TO JOG')
        : (!ownsMotor && data.armed === true ? 'ARMED · OBSERVER'
          : (String(lastKnownState).startsWith('Fault') ? 'FAULT' : 'Disarmed')));
  badgeState.className = `badge ${inputState.stopLatched ? 'badge-fault' : (isArmed ? (isMoving ? 'badge-jogging' : 'badge-armed') : (String(lastKnownState).startsWith('Fault') ? 'badge-fault' : ''))}`;
  telLifecycle.textContent = lastKnownState || 'Awaiting status';
  telReason.textContent = typeof data.reason === 'string' && data.reason ? data.reason : 'Awaiting status';
  telFeedbackAge.textContent = !isConnected ? 'No active connection'
    : feedbackAge === null ? 'No reply received'
      : `${freshFeedback ? 'Fresh' : 'STALE'} · ${feedbackAge.toFixed(3)} s since last motor reply`;
  telStopConfirmed.textContent = !isConnected ? 'Disconnected — see final Stop result below'
    : !freshFeedback ? 'Stale — cached result only' : data.stop_confirmed === true ? 'Confirmed'
      : data.stop_confirmed === false ? 'Not confirmed' : 'Awaiting status';
  updateActionControls();

  if (stepConnect && stepCheck && stepZero && stepJog) {
    const steps = deriveWorkflowSteps({
      connected: isConnected,
      fresh: freshFeedback,
      stopConfirmed: data.stop_confirmed,
      motorState: data.state,
      latest: data.latest,
      calibrated: data.calibrated,
      armed: isArmed,
      stopLatched: inputState.stopLatched
    });
    stepConnect.className = `step-node ${steps.connect}`;
    stepCheck.className = `step-node ${steps.check}`;
    stepZero.className = `step-node ${steps.zero}`;
    stepJog.className = `step-node ${steps.jog}`;
  }

  updateStatusPresentation(data);
  const applied = data.settings;
  appliedSettings = applied;
  followPreview.disabled = isArmed || currentProfileId === 'arm_j1_j2_j3';
  telPendingOp.textContent = inputClient.ordinaryRun ? 'Processing…' : 'None';

  const joints = Array.isArray(data.joints) ? data.joints : [];
  const [j1, j2, j3] = joints;

  for (const j of joints) {
    if (j?.name && j.settings && perJointSettings[j.name] && !editedJointDrafts.has(j.name)) {
      Object.assign(perJointSettings[j.name], j.settings);
    }
  }

  const selIndex = selectedJoint === 'J1' ? 0 : selectedJoint === 'J2' ? 1 : 2;
  const selJoint = joints[selIndex];
  const curJointApplied = selJoint?.settings || applied || perJointSettings[selectedJoint];
  if (curJointApplied) {
    document.getElementById('applied-settings').textContent = `Applied (${selectedJoint}): Kp ${curJointApplied.kp}, Kd ${curJointApplied.kd}, ${curJointApplied.max_rpm} rpm, ${curJointApplied.effort_limit_nm} Nm; travel ${curJointApplied.lower_deg}° to ${curJointApplied.upper_deg}°.`;
  }

  const positionElements = [telJ1Pos, telJ2Pos, telJ3Pos];
  const positionDisplays = [j1, j2, j3].map(jointPositionPresentation);
  const positions = positionDisplays.map((display, index) => {
    positionElements[index].textContent = display.text;
    return display.radians;
  });

  for (const [element, joint, includeRpm] of [[telJ1Vel, j1, true], [telJ2Vel, j2, true], [telJ3Vel, j3, true]]) {
    element.textContent = jointVelocityText(joint, includeRpm);
  }

  const j1Health = formatJointHealth(j1);
  const j1Source = j1?.source;
  const sourceText = j1Source === 'physical' ? (freshFeedback ? 'Fresh encoder feedback' : 'STALE encoder feedback')
    : j1Source === 'demo' ? (freshFeedback ? 'Fresh simulated feedback' : 'STALE simulated feedback')
      : 'Awaiting feedback source';
  badgeJ1Source.textContent = sourceText;
  telJ1Source.textContent = isConnected ? j1Health.text : 'Awaiting feedback';
  if (telJ1HealthMsg) {
    if (isConnected && j1Health.status === 'missing') {
      telJ1HealthMsg.style.display = 'block';
      telJ1HealthMsg.textContent = j1Health.text;
    } else {
      telJ1HealthMsg.style.display = 'none';
    }
  }
  const torque1 = finiteValue(j1?.torque_nm);
  const volt1 = finiteValue(j1?.voltage);
  const dTemp1 = finiteValue(j1?.driver_temp);
  const wTemp1 = finiteValue(j1?.winding_temp);
  telJ1Torque.textContent = !isConnected ? 'Disconnected' : torque1 === null ? 'Awaiting status'
    : `${torque1.toFixed(2)} Nm${freshFeedback ? '' : ' · cached'}`;
  telJ1Mode.textContent = Number.isInteger(j1?.mode) ? String(j1.mode) : 'Awaiting status';
  telJ1Volt.textContent = volt1 === null ? 'Awaiting status' : `${volt1.toFixed(1)} V`;
  telJ1Temp.textContent = `${dTemp1 === null ? 'Awaiting status' : `${dTemp1.toFixed(1)}°C`} / ${wTemp1 === null ? 'Awaiting status' : `${wTemp1.toFixed(1)}°C`}`;
  if (Number.isInteger(j1?.errors) && Number.isInteger(j1?.warnings)) {
    const faulted = j1.errors !== 0 || j1.warnings !== 0;
    telJ1Errors.textContent = faulted ? `Errors 0x${j1.errors.toString(16)} / warnings ${j1.warnings}` : 'None reported';
    telJ1Errors.className = `telemetry-val ${faulted ? 'error' : 'ok'}`;
  } else {
    telJ1Errors.textContent = 'Awaiting status';
    telJ1Errors.className = 'telemetry-val';
  }
  telJ1StopResult.textContent = (isConnected ? '' : 'Last session: ') + (j1?.stop_result || (j1?.stop_confirmed ? 'Confirmed' : (data?.stop_confirmed ? 'Confirmed' : 'not attempted')));

  const isThreeJoint = currentProfileId === 'arm_j1_j2_j3';
  if (!isThreeJoint) {
    telJ2Source.textContent = 'Virtual simulation';
    telJ2Torque.textContent = '-';
    telJ2Mode.textContent = '-';
    telJ2Volt.textContent = '-';
    telJ2Temp.textContent = '- / -';
    telJ2Errors.textContent = 'Virtual simulation';
    telJ2Errors.className = 'telemetry-val';
    telJ2StopResult.textContent = 'N/A (virtual)';
    if (telJ2HealthMsg) telJ2HealthMsg.style.display = 'none';

    telJ3Source.textContent = 'Virtual simulation';
    telJ3Torque.textContent = '-';
    telJ3Mode.textContent = '-';
    telJ3Volt.textContent = '-';
    telJ3Temp.textContent = '- / -';
    telJ3Errors.textContent = 'Virtual simulation';
    telJ3Errors.className = 'telemetry-val';
    telJ3StopResult.textContent = 'N/A (virtual)';
    if (telJ3HealthMsg) telJ3HealthMsg.style.display = 'none';
  } else {
    const j2Health = formatJointHealth(j2);
    telJ2Source.textContent = isConnected ? j2Health.text : 'Awaiting feedback';
    if (telJ2HealthMsg) {
      if (isConnected && j2Health.status === 'missing') {
        telJ2HealthMsg.style.display = 'block';
        telJ2HealthMsg.textContent = j2Health.text;
      } else {
        telJ2HealthMsg.style.display = 'none';
      }
    }
    const torque2 = finiteValue(j2?.torque_nm);
    const volt2 = finiteValue(j2?.voltage);
    const dTemp2 = finiteValue(j2?.driver_temp);
    const wTemp2 = finiteValue(j2?.winding_temp);
    telJ2Torque.textContent = !isConnected ? 'Disconnected' : torque2 === null ? 'Awaiting status' : `${torque2.toFixed(2)} Nm`;
    telJ2Mode.textContent = Number.isInteger(j2?.mode) ? String(j2.mode) : 'Awaiting status';
    telJ2Volt.textContent = volt2 === null ? 'Awaiting status' : `${volt2.toFixed(1)} V`;
    telJ2Temp.textContent = `${dTemp2 === null ? 'Awaiting status' : `${dTemp2.toFixed(1)}°C`} / ${wTemp2 === null ? 'Awaiting status' : `${wTemp2.toFixed(1)}°C`}`;
    if (Number.isInteger(j2?.errors) && Number.isInteger(j2?.warnings)) {
      const faulted = j2.errors !== 0 || j2.warnings !== 0;
      telJ2Errors.textContent = faulted ? `Errors 0x${j2.errors.toString(16)} / warnings ${j2.warnings}` : 'None reported';
      telJ2Errors.className = `telemetry-val ${faulted ? 'error' : 'ok'}`;
    } else {
      telJ2Errors.textContent = 'Awaiting status';
      telJ2Errors.className = 'telemetry-val';
    }
    telJ2StopResult.textContent = (isConnected ? '' : 'Last session: ') + (j2?.stop_result || (j2?.stop_confirmed ? 'Confirmed' : 'not attempted'));

    const j3Health = formatJointHealth(j3);
    telJ3Source.textContent = isConnected ? j3Health.text : 'Awaiting feedback';
    if (telJ3HealthMsg) {
      if (isConnected && j3Health.status === 'missing') {
        telJ3HealthMsg.style.display = 'block';
        telJ3HealthMsg.textContent = j3Health.text;
      } else {
        telJ3HealthMsg.style.display = 'none';
      }
    }
    const torque3 = finiteValue(j3?.torque_nm);
    const volt3 = finiteValue(j3?.voltage);
    const dTemp3 = finiteValue(j3?.driver_temp);
    const wTemp3 = finiteValue(j3?.winding_temp);
    telJ3Torque.textContent = !isConnected ? 'Disconnected' : torque3 === null ? 'Awaiting status' : `${torque3.toFixed(2)} Nm`;
    telJ3Mode.textContent = Number.isInteger(j3?.mode) ? String(j3.mode) : 'Awaiting status';
    telJ3Volt.textContent = volt3 === null ? 'Awaiting status' : `${volt3.toFixed(1)} V`;
    telJ3Temp.textContent = `${dTemp3 === null ? 'Awaiting status' : `${dTemp3.toFixed(1)}°C`} / ${wTemp3 === null ? 'Awaiting status' : `${wTemp3.toFixed(1)}°C`}`;
    if (Number.isInteger(j3?.errors) && Number.isInteger(j3?.warnings)) {
      const faulted = j3.errors !== 0 || j3.warnings !== 0;
      telJ3Errors.textContent = faulted ? `Errors 0x${j3.errors.toString(16)} / warnings ${j3.warnings}` : 'None reported';
      telJ3Errors.className = `telemetry-val ${faulted ? 'error' : 'ok'}`;
    } else {
      telJ3Errors.textContent = 'Awaiting status';
      telJ3Errors.className = 'telemetry-val';
    }
    telJ3StopResult.textContent = (isConnected ? '' : 'Last session: ') + (j3?.stop_result || (j3?.stop_confirmed ? 'Confirmed' : 'not attempted'));
  }

  if (jointsCalSummary) {
    const summaryParts = [
      `J1 (ID 0): ${j1?.calibrated ? 'Calibrated' : 'Uncalibrated'}${positions[0] !== null ? ' [' + (positions[0] * 180 / Math.PI).toFixed(1) + '°]' : ''}`,
      isThreeJoint
        ? `J2 (ID 1): ${j2?.calibrated ? 'Calibrated' : 'Uncalibrated'}${positions[1] !== null ? ' [' + (positions[1] * 180 / Math.PI).toFixed(1) + '°]' : ''}`
        : 'J2: Virtual',
      isThreeJoint
        ? `J3 (ID 2): ${j3?.calibrated ? 'Calibrated' : 'Uncalibrated'}${positions[2] !== null ? ' [' + (positions[2] * 180 / Math.PI).toFixed(1) + '°]' : ''}`
        : 'J3: Virtual'
    ];
    jointsCalSummary.textContent = summaryParts.join(' · ');
  }

  updateJointGauge(positions[selIndex], finiteValue(selJoint?.torque_nm), curJointApplied, selectedJoint);

  updateSceneLimitsForSelectedJoint(positions[selIndex] !== null ? positions[selIndex] * 180 / Math.PI : null);

  measuredJ1 = positions[0];
  if (positions.every(position => position !== null)) observedPose = positions;

  if (!preview.selected) {
    if (isRangePreview) {
      const previewPose = [
        Number.isFinite(positions[0]) ? positions[0] : (observedPose[0] || 0),
        Number.isFinite(positions[1]) ? positions[1] : (observedPose[1] || 0),
        Number.isFinite(positions[2]) ? positions[2] : (observedPose[2] || 0)
      ];
      previewPose[selIndex] = rangePreviewAngle;
      armScene.updateJoints(...previewPose);
    } else if (positions.some(position => position !== null)) {
      const q1 = Number.isFinite(positions[0]) ? positions[0] : (observedPose[0] || 0);
      const q2 = Number.isFinite(positions[1]) ? positions[1] : (observedPose[1] || 0);
      const q3 = Number.isFinite(positions[2]) ? positions[2] : (observedPose[2] || 0);
      armScene.updateJoints(q1, q2, q3);
    }
  }

  if (Array.isArray(data.events) && data.events.length) {
    const lastEvent = data.events[data.events.length - 1];
    if (lastEvent && lastEvent !== lastBackendEvent) {
      lastBackendEvent = lastEvent;
      logEvent(lastEvent);
    }
  }
}
function invalidateStatePoll() {
  statePollGeneration++;
  if (statePollController) {
    statePollController.abort();
    statePollController = null;
  }
}

function applyLocalStop(reason) {
  setupGeneration++;
  chkEnableArm.checked = false;
  isRangePreview = false;
  preview.clear();
  armScene.showPath(null);
  document.getElementById('preview-status').textContent = 'Live / disconnected pose';
  if (observedPose.every(Number.isFinite)) armScene.updateJoints(...observedPose);
  followMotion = false;
  latchStop(inputState, reason);
  clearStopRequest(inputState);
  isArmed = false;
  invalidateStatePoll();
  badgeState.textContent = 'STOPPED · Enable when ready';
  badgeState.className = 'badge badge-fault';
  telStopConfirmed.textContent = 'Stop requested';
  updateActionControls();
  updateStatusPresentation(lastStateData);
}

async function stopNow(reason, { keepalive = false, log = true } = {}) {
  applyLocalStop(reason);
  if (log) logEvent(`STOP / DISARM requested: ${reason}`, 'error');
  const result = await inputClient.triggerStop(reason, { keepalive });
  if (!result.ok && inputClient.session) {
    telStopConfirmed.textContent = 'Stop not confirmed';
    logEvent(`STOP request failed: ${result.error || result.reason || 'unknown error'}`, 'error');
  }
  return result;
}

function syncClientStopLatch() {
  if (inputClient.stopLatched && !inputState.stopLatched) {
    const reason = (String(lastStateData?.state).startsWith('Fault') || String(lastStateData?.state).startsWith('Stopped'))
      ? lastStateData.reason : inputClient.lastFailureReason || 'Connection request failed; check the dashboard terminal and USB connection.';
    applyLocalStop(reason);
  }
}

function serviceReducerStop() {
  if (!inputState.stopRequested) return;
  const reason = inputState.stopReason || 'input_stop';
  clearStopRequest(inputState);
  if (!ownsMotor && reason !== 'gamepad_b_stop') {
    clearStopLatch(inputState);
    return;
  }
  void stopNow(reason);
}

function readBounded(input, label, minimum, maximum) {
  if (!input || input.value.trim() === '') throw new Error(`${label} is required`);
  const value = Number(input.value);
  if (!Number.isFinite(value)) throw new Error(`${label} must be a finite number`);
  if (value < minimum || value > maximum) logEvent(`Warning: ${label} is outside the recommended ${minimum}–${maximum} range.`, 'warn');
  return value;
}

btnStop.addEventListener('click', () => { void stopNow('operator_stop_button'); });

selectPort.addEventListener('change', () => {
  if (inputPortPath) inputPortPath.value = '';
  updateActionControls();
});
inputPortPath?.addEventListener('input', updateActionControls);

selectInputDevice?.addEventListener('change', () => {
  const selected = selectInputDevice.value;
  if (!selectInputSource(inputState, selected)) {
    selectInputDevice.value = inputState.activeDevice;
    showAlert('Choose an input while disarmed, with both sticks centered and all jog keys released.', 'warn', 5000);
    return;
  }
  logEvent(`${selected === 'gamepad' ? 'Xbox controller' : 'Keyboard and on-screen buttons'} selected. Keep controls neutral through Enable.`);
  document.getElementById('input-status').textContent = `${selected === 'gamepad' ? 'Xbox controller' : 'Keyboard and buttons'} selected`;
  updateActionControls();
});

selectProfile?.addEventListener('change', async () => {
  if (isConnected) {
    selectProfile.value = currentProfileId;
    showAlert('Disconnect before changing arm profile.', 'warn', 4000);
    return;
  }
  const newProfile = selectProfile.value;
  try {
    const res = await ensureAction('select_profile', { profile_id: newProfile });
    if (res.status === 'complete' || res.status === 'ok') {
      currentProfileId = newProfile;
      updateParameterWarnings();
      logEvent(`Profile selected: ${newProfile}`);
      updateProfileUI();
      updateActionControls();
    } else {
      selectProfile.value = currentProfileId;
      logEvent(`Select profile rejected: ${res.reason || res.status}`, 'error');
    }
  } catch (err) {
    selectProfile.value = currentProfileId;
    logEvent(`Failed to switch profile: ${err.message}`, 'error');
  }
});

async function refreshSetupState() {
  invalidateStatePoll();
  const response = await fetch('/api/state', { signal: AbortSignal.timeout(2000) });
  if (!response.ok) throw new Error(`Motor status request failed (HTTP ${response.status}).`);
  const data = await response.json();
  updateDashboardState(data);
  return data;
}

function captureConnectionSettings(profileId) {
  const current = perJointSettings[selectedJoint];
  for (const [key, input] of Object.entries({ kp: inputKp, kd: inputKd, max_rpm: inputMaxRpm,
    effort_limit_nm: inputEffort, known_angle_deg: inputCalAngle, lower_deg: inputCalLower, upper_deg: inputCalUpper })) {
    if (!input.value.trim()) throw new Error(`${selectedJoint} ${key} is required.`);
    current[key] = Number(input.value);
  }
  current.direction = Number(selectCalDir.value);
  const names = profileId === 'arm_j1_j2_j3' ? ['J1', 'J2', 'J3'] : ['J1'];
  const entries = names.map(joint => ({ joint, settings: { ...perJointSettings[joint] } }));
  validateConnectionSettings(entries);
  for (const { joint } of entries) editedJointDrafts.add(joint);
  return entries;
}

function checkedSetupAction(generation) {
  return async (name, values) => {
    if (generation !== setupGeneration) throw new Error('Setup cancelled by Stop.');
    const result = await ensureAction(name, values);
    if (generation !== setupGeneration) throw new Error('Setup cancelled by Stop.');
    if (result.status !== 'complete') throw new Error(`${name}: ${result.reason || result.status}`);
    return result;
  };
}

btnConnect.addEventListener('click', async () => {
  if (setupBusy) return;
  const profileId = selectProfile.value || currentProfileId;
  const demo = isDemoMode === true;
  const supportConfirmed = chkThreeSupport?.checked === true;
  if (!demo && profileId === 'arm_j1_j2_j3' && !supportConfirmed) {
    showAlert('Support all arm links, then confirm mechanical support before connecting.', 'warn', 5000);
    return;
  }
  let entries;
  try { entries = captureConnectionSettings(profileId); }
  catch (error) { showAlert(error.message, 'error', 8000); logEvent(error.message, 'error'); return; }
  const generation = ++setupGeneration;
  const action = checkedSetupAction(generation);
  setupBusy = true;
  updateActionControls();
  logEvent('Connecting and applying the entered settings and current-pose references. Motors stay stopped.');
  try {
    await action('connect', { demo, port: inputPortPath?.value.trim() || selectPort.value.trim(),
      baud: Number(inputBaud.value), id: Number(inputId.value), profile_id: profileId, support_confirmed: supportConfirmed });
    ownsMotor = true;
    const data = await refreshSetupState();
    const timeout = data.joints?.some(joint => joint.configured && (joint.timeout === true || joint.timeout === 1));
    await applyConnectionSettings(action, entries, timeout);
    clearStopLatch(inputState);
    await refreshSetupState();
    logEvent('Connected. Settings and pose references applied. Check Enable movement to jog.');
  } catch (error) {
    logEvent(`Connect setup stopped: ${error.message}`, 'error');
    await stopNow(`Connect setup failed: ${error.message}`);
  } finally {
    setupBusy = false;
    updateActionControls();
  }
});

btnDisconnect.addEventListener('click', async () => {
  logEvent('Disconnecting...');
  try {
    const res = await ensureAction('disconnect', {});
    if (res.status === 'complete') ownsMotor = false;
    logEvent(`Disconnect result: ${res.status}`);
    updateActionControls();
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Disconnect rejected: ${err.message}`, 'error');
  }
});

btnCheck.addEventListener('click', async () => {
  logEvent('Running health check...');
  try {
    const res = await ensureAction('check', {});
    logEvent(`Check result: ${res.status}`);
    if (res.status === 'complete') {
      clearStopLatch(inputState);
      isArmed = false;
      updateActionControls();
    }
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Check rejected: ${err.message}`, 'error');
  }
});

btnAck.addEventListener('click', async () => {
  if (!chkAck.checked) {
    logEvent('Fault acknowledgment blocked: confirm the fault cause was resolved first', 'error');
    return;
  }
  logEvent('Acknowledging fault...');
  try {
    const res = await ensureAction('ack', { supported: chkAck.checked });
    logEvent(`Ack result: ${res.status}`);
    if (res.status === 'complete') chkAck.checked = false;
    updateActionControls();
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Ack rejected: ${err.message}`, 'error');
  }
});

btnZeroJ1.addEventListener('click', () => {
  if (btnCalibrate.disabled) return;
  inputCalAngle.value = '0';
  btnCalibrate.click();
});

btnCalibrate.addEventListener('click', async () => {
  const targetJoint = selectedJoint;
  try {
    const isJ1 = targetJoint === 'J1';
    const angle = readBounded(inputCalAngle, `${targetJoint} angle at this pose`, isJ1 ? -90 : -3600, isJ1 ? 90 : 3600);
    const dir = Number(selectCalDir.value);
    const lower = readBounded(inputCalLower, 'Lower limit', isJ1 ? -90 : -3600, isJ1 ? 90 : 3600);
    const upper = readBounded(inputCalUpper, 'Upper limit', isJ1 ? -90 : -3600, isJ1 ? 90 : 3600);
    if (![1, -1].includes(dir) || lower >= upper || angle < lower || angle > upper) {
      throw new Error(`${targetJoint} limits must have lower < upper and contain the measured angle.`);
    }
    if (isJ1 && (lower < -90 || upper > 90)) {
      throw new Error('J1 applied travel must stay within −90° to +90° wire cap. Save larger values as a draft.');
    }
    logEvent(`Setting ${targetJoint} software zero: measured angle=${angle}°, dir=${dir}, limits=[${lower}°, ${upper}°]...`);
    const res = await ensureAction('calibrate', {
      joint: targetJoint,
      known_angle_deg: angle,
      direction: dir,
      lower_deg: lower,
      upper_deg: upper
    });
    logEvent(`Calibrate ${targetJoint} result: ${res.status}`);
    if (res.status === 'complete') {
      perJointSettings[targetJoint].known_angle_deg = angle;
      perJointSettings[targetJoint].direction = dir;
      perJointSettings[targetJoint].lower_deg = lower;
      perJointSettings[targetJoint].upper_deg = upper;
      const checked = await ensureAction('check', {});
      if (checked.status === 'complete') clearStopLatch(inputState);
    }
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Calibrate ${targetJoint} rejected: ${err.message}`, 'error');
  }
});

btnConfigure.addEventListener('click', async () => {
  const targetJoint = selectedJoint;
  try {
    const kp = readBounded(inputKp, 'Kp', 0, 30);
    const kd = readBounded(inputKd, 'Kd', 0, 6);
    const maxRpm = readBounded(inputMaxRpm, 'Maximum RPM', 0.1, 100.0);
    const effort = readBounded(inputEffort, 'Effort limit', 0.1, 15.0);
    logEvent(`Configuring ${targetJoint} PD gains: Kp=${kp}, Kd=${kd}, MaxRPM=${maxRpm}, Effort=${effort}Nm...`);
    const res = await ensureAction('configure', {
      joint: targetJoint,
      kp,
      kd,
      max_rpm: maxRpm,
      effort_limit_nm: effort
    });
    logEvent(`Configure ${targetJoint} result: ${res.status}`);
    if (res.status === 'complete') {
      perJointSettings[targetJoint].kp = kp;
      perJointSettings[targetJoint].kd = kd;
      perJointSettings[targetJoint].max_rpm = maxRpm;
      perJointSettings[targetJoint].effort_limit_nm = effort;
      const checked = await ensureAction('check', {});
      if (checked.status === 'complete') clearStopLatch(inputState);
    }
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Configure ${targetJoint} rejected: ${err.message}`, 'error');
  }
});

btnPreviewMin?.addEventListener('click', () => {
  if (isArmed) { logEvent('Stop movement before previewing a travel limit.', 'warn'); return; }
  const lower = draftNumber(inputCalLower.value);
  if (!Number.isFinite(lower)) return;
  const targetRad = ((lower % 360) * Math.PI) / 180;
  preview.clear();
  armScene.showPath(null);
  followPreview.checked = false;
  followMotion = false;
  isRangePreview = true;
  rangePreviewAngle = targetRad;

  const previewPose = [...observedPose];
  const idx = selectedJoint === 'J1' ? 0 : selectedJoint === 'J2' ? 1 : 2;
  previewPose[idx] = targetRad;
  armScene.updateJoints(...previewPose);

  logEvent(`Previewing ${selectedJoint} minimum limit: ${lower}° (visual only, motors not commanded)`);
  const statusElem = document.getElementById('preview-status');
  if (statusElem) statusElem.textContent = `Preview ${selectedJoint} min limit (${lower}°) · visual only`;
});

btnPreviewMax?.addEventListener('click', () => {
  if (isArmed) { logEvent('Stop movement before previewing a travel limit.', 'warn'); return; }
  const upper = draftNumber(inputCalUpper.value);
  if (!Number.isFinite(upper)) return;
  const targetRad = ((upper % 360) * Math.PI) / 180;
  preview.clear();
  armScene.showPath(null);
  followPreview.checked = false;
  followMotion = false;
  isRangePreview = true;
  rangePreviewAngle = targetRad;

  const previewPose = [...observedPose];
  const idx = selectedJoint === 'J1' ? 0 : selectedJoint === 'J2' ? 1 : 2;
  previewPose[idx] = targetRad;
  armScene.updateJoints(...previewPose);

  logEvent(`Previewing ${selectedJoint} maximum limit: ${upper}° (visual only, motors not commanded)`);
  const statusElem = document.getElementById('preview-status');
  if (statusElem) statusElem.textContent = `Preview ${selectedJoint} max limit (${upper}°) · visual only`;
});

chkAck.addEventListener('change', updateActionControls);
chkEnableArm.addEventListener('change', () => {
  if (chkEnableArm.checked) void enableMovement();
  else void stopNow('Enable movement unchecked');
});

async function enableMovement() {
  if (setupBusy || !chkEnableArm.checked || !ownsMotor || !isConnected || isArmed) return;
  if (inputState.keysDown.size || inputState.gamepadAxes.some(axis => Math.abs(axis) > 0.01)) {
    chkEnableArm.checked = false;
    showAlert('Release all jog keys and center the controller sticks, then check Enable movement.', 'warn', 6000);
    return;
  }
  const generation = ++setupGeneration;
  const action = checkedSetupAction(generation);
  setupBusy = true;
  updateActionControls();
  try {
    await action('check', {});
    clearStopLatch(inputState);
    isRangePreview = false;
    if (currentProfileId === 'arm_j1_j2_j3') {
      preview.clear();
      armScene.showPath(null);
      document.getElementById('preview-status').textContent = 'Live / disconnected pose';
    }
    await action('arm', { supported: true });
    await inputClient.submitJog(false, [0, 0, 0], inputState.activeDevice);
    await refreshSetupState();
    document.activeElement?.blur();
    logEvent('Movement enabled. Use the jog buttons, keyboard, or selected Xbox controller.');
  } catch (error) {
    logEvent(`Enable failed: ${error.message}`, 'error');
    await stopNow(`Enable failed: ${error.message}`);
  } finally {
    setupBusy = false;
    updateActionControls();
  }
}

btnArm.addEventListener('click', () => { void enableMovement(); });

btnRelease.addEventListener('click', () => {
  void stopNow('operator_release_mode_0');
});

btnResetCam.addEventListener('click', () => {
  armScene.resetCamera();
});

sliderJogSpeed.addEventListener('input', () => {
  lblJogSpeed.textContent = `${sliderJogSpeed.value}%`;
});
function focusedInEditor() {
  const element = document.activeElement;
  return ['INPUT', 'TEXTAREA', 'SELECT'].includes(element?.tagName)
    || Boolean(element?.isContentEditable);
}

window.addEventListener('keydown', (e) => {
  const isEditing = focusedInEditor();
  reduceKeyEvent(inputState, { type: 'keydown', code: e.code || ({a:'KeyA',d:'KeyD',w:'KeyW',s:'KeyS',q:'KeyQ',e:'KeyE',Escape:'Escape'}[e.key]), isEditing });

  if (e.code === 'Escape') {
    e.preventDefault();
    void stopNow('keyboard_escape');
  }
});

window.addEventListener('keyup', (e) => {
  const isEditing = focusedInEditor();
  reduceKeyEvent(inputState, { type: 'keyup', code: e.code || ({a:'KeyA',d:'KeyD',w:'KeyW',s:'KeyS',q:'KeyQ',e:'KeyE',Escape:'Escape'}[e.key]), isEditing });
});

window.addEventListener('blur', () => {
  reduceBlur(inputState);
  if (ownsMotor && isArmed && !inputState.stopLatched) void stopNow('window_blur');
});

document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    reduceVisibilityChange(inputState, true);
    if (ownsMotor && !inputState.stopLatched) void stopNow('document_hidden');
  }
});

window.addEventListener('pagehide', () => {
  reduceBlur(inputState);
  if(ownsMotor)void stopNow('pagehide', { keepalive: true, log: false });
  else if(inputClient.session)void fetch('/api/retire', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({session:inputClient.session}),keepalive:true}).catch(()=>{});
});

window.addEventListener('gamepadconnected', () => {
  pollGamepad();
});

window.addEventListener('gamepaddisconnected', () => {
  pollGamepad();
});
function pollGamepad() {
  if (typeof navigator.getGamepads !== 'function') {
    gamepadStatus.textContent = 'This browser cannot read controllers. Open this address in Chrome or Firefox.';
    gamepadStatus.dataset.state = 'unsupported';
    reduceGamepad(inputState, null);
    serviceReducerStop();
    return;
  }
  const gamepads = Array.from(navigator.getGamepads() || []);
  const pad = selectGamepadCandidate(gamepads, inputState.lastGamepadIdentity);
  if (pad) {
    const validation = validateGamepad(pad);
    reduceGamepad(inputState, pad);
    gamepadStatus.textContent = validation.valid
      ? `Standard gamepad connected: ${pad.id.slice(0, 48)}. Jog J1 with left stick. Press B to Stop.`
      : `Selected controller cannot jog: ${validation.reason}. Select a standard mapping after Stop/Check.`;
    gamepadStatus.dataset.state = validation.valid ? 'ready' : 'error';
  } else {
    const unsupported = gamepads.find(candidate => candidate && candidate.connected === true);
    const reason = unsupported ? validateGamepad(unsupported).reason : null;
    gamepadStatus.textContent = reason
      ? `No supported standard controller found: ${reason}. Use a standard-mapped controller in Chrome or Firefox.`
      : 'Controller not visible yet. Focus this page and press a controller button. If needed, open this address in Chrome or Firefox.';
    gamepadStatus.dataset.state = reason ? 'error' : 'idle';
    reduceGamepad(inputState, null);
  }
  serviceReducerStop();
}

function jogLoop() {
  expireStateFreshness();
  pollGamepad();
  syncClientStopLatch();
  serviceReducerStop();
  if (!ownsMotor || !isArmed || inputState.stopLatched || !inputClient.session) {
    followMotion = false;
    followBlocked = false;
    document.getElementById('input-status').textContent = 'Disarmed · controls inactive';
    if (preview.selected && followPreview.checked) {
      document.getElementById('preview-status').textContent = 'Virtual preview · J1 following disarmed';
    }
    btnJogJ1Neg?.classList.remove('jog-active');
    btnJogJ1Pos?.classList.remove('jog-active');
    return;
  }

  const enteredScale = Number(sliderJogSpeed.value);
  if (!sliderJogSpeed.value.trim() || !Number.isFinite(enteredScale)) { void stopNow('invalid_jog_multiplier'); return; }
  const speedScale = Math.max(0, Math.min(1, enteredScale / 100));
  let axes = inputState.effectiveAxes.map(axis => axis * speedScale);

  if (btnJogJ1Neg) {
    const isA = inputState.keysDown.has('KeyA') || (axes[0] < -0.05);
    btnJogJ1Neg.classList.toggle('jog-active', Boolean(isArmed && isA));
  }
  if (btnJogJ1Pos) {
    const isD = inputState.keysDown.has('KeyD') || (axes[0] > 0.05);
    btnJogJ1Pos.classList.toggle('jog-active', Boolean(isArmed && isD));
  }

  followBlocked = false;
  if (preview.selected && followPreview.checked && currentProfileId === 'arm_j1') {
    const follow = freshFeedback && inputState.neutralLatched
      ? followAxis(preview.q[0], measuredJ1, appliedSettings?.lower_deg, appliedSettings?.upper_deg)
      : { axis: 0, blocked: true };
    axes = [follow.axis * speedScale, 0, 0];
    followBlocked = follow.blocked;
    document.getElementById('preview-status').textContent = !freshFeedback
      ? 'J1 follow paused: feedback is stale'
      : !inputState.neutralLatched
        ? 'J1 follow paused: release keyboard and center both sticks once after Enable'
        : follow.blocked
          ? 'J1 follow paused: target is outside applied limits or feedback is unavailable'
          : 'Fresh J1 feedback · following within applied travel and rate limits';
  } else if (currentProfileId === 'arm_j1_j2_j3') {
    followMotion = false;
  }
  const motionIntent = axes.some(axis => Math.abs(axis) > 1e-4);
  followMotion = Boolean(preview.selected && followPreview.checked && motionIntent && !followBlocked && currentProfileId === 'arm_j1');
  document.getElementById('input-status').textContent = preview.selected && followPreview.checked && currentProfileId === 'arm_j1'
    ? followMotion ? 'Automatic preview follow · J1'
      : followBlocked ? 'Preview follow paused · J1' : 'Preview follow ready · J1'
    : motionIntent ? `Jogging · ${inputState.activeDevice}` : 'Holding position';
  inputClient.submitJog(motionIntent, axes, inputState.activeDevice)
    .catch(error => {
      const alreadyStopped = inputState.stopLatched;
      syncClientStopLatch();
      if (!alreadyStopped) logEvent(`Jog input failed and Stop was requested: ${error.message}`, 'error');
    });
}
initSession();
setInterval(pollState, 200);
setInterval(jogLoop, 50);

function updateParameterWarnings() {
  const declaration = document.getElementById('connection-declarations');
  if (declaration) {
    const names = currentProfileId === 'arm_j1_j2_j3' ? ['J1', 'J2', 'J3'] : ['J1'];
    declaration.textContent = 'Connect declares current poses: ' + names.map(name => {
      const s = perJointSettings[name];
      return `${name} ${Number.isFinite(s.known_angle_deg) ? s.known_angle_deg + '°' : 'angle required'} (travel ${s.lower_deg}° to ${s.upper_deg}°)`;
    }).join(' · ') + '. Confirm every motor matches its entered pose before Connect.';
  }
  const warnings = [];
  const isJ1 = selectedJoint === 'J1';
  const lowerCap = isJ1 ? -90 : -360;
  const upperCap = isJ1 ? 90 : 360;

  for (const [input, label, low, high] of [
    [sliderJogSpeed, 'Jog multiplier (%)', 0, 100],
    [inputKp, 'Kp', 0, 30],
    [inputKd, 'Kd', 0, 6],
    [inputMaxRpm, 'RPM', 0, isJ1 ? 5 : 0.5],
    [inputEffort, 'Effort (Nm)', 0, 15],
    [inputCalLower, `${selectedJoint} lower angle`, lowerCap, upperCap],
    [inputCalUpper, `${selectedJoint} upper angle`, lowerCap, upperCap]
  ]) {
    if (input.value.trim() && (!Number.isFinite(Number(input.value)) || Number(input.value) < low || Number(input.value) > high)) {
      warnings.push(`${label}: outside the recommended ${low}–${high} range.`);
    }
  }

  const spanWarn = checkTravelSpan(draftNumber(inputCalLower.value), draftNumber(inputCalUpper.value));
  if (spanWarn) {
    travelSpanWarning.style.display = 'block';
    travelSpanWarning.textContent = spanWarn;
  } else {
    travelSpanWarning.style.display = 'none';
    travelSpanWarning.textContent = '';
  }

  document.getElementById('parameter-warning').textContent = warnings.length
    ? 'WARNING — outside bench or travel recommendations. Physical command limits currently still apply.\n' + warnings.join('\n')
    : '';
}

for (const input of [sliderJogSpeed, inputKp, inputKd, inputMaxRpm, inputEffort]) {
  input.addEventListener('input', updateParameterWarnings);
}
inputCalLower.addEventListener('input', onLimitDraftInput);
inputCalUpper.addEventListener('input', onLimitDraftInput);
inputCalAngle.addEventListener('input', () => { editedJointDrafts.add(selectedJoint); perJointSettings[selectedJoint].known_angle_deg = draftNumber(inputCalAngle.value); updateParameterWarnings(); });
selectCalDir.addEventListener('change', () => { editedJointDrafts.add(selectedJoint); perJointSettings[selectedJoint].direction = Number(selectCalDir.value); });
inputKp.addEventListener('input', () => { editedJointDrafts.add(selectedJoint); perJointSettings[selectedJoint].kp = draftNumber(inputKp.value); updateParameterWarnings(); });
inputKd.addEventListener('input', () => { editedJointDrafts.add(selectedJoint); perJointSettings[selectedJoint].kd = draftNumber(inputKd.value); updateParameterWarnings(); });
inputMaxRpm.addEventListener('input', () => { editedJointDrafts.add(selectedJoint); perJointSettings[selectedJoint].max_rpm = draftNumber(inputMaxRpm.value); updateParameterWarnings(); });
inputEffort.addEventListener('input', () => { editedJointDrafts.add(selectedJoint); perJointSettings[selectedJoint].effort_limit_nm = draftNumber(inputEffort.value); updateParameterWarnings(); });
selectJoint.addEventListener('change', onSelectedJointChanged);

for (const button of document.querySelectorAll('[data-jog]')) {
  let held = false;
  button.addEventListener('pointerdown', event => {
    event.preventDefault(); if (!isArmed || held) return;
    held = true; button.setPointerCapture(event.pointerId);
    reduceKeyEvent(inputState, { type: 'keydown', code: button.dataset.jog, isEditing: false });
  });
  const release = () => { if (!held) return; held = false; reduceKeyEvent(inputState, { type: 'keyup', code: button.dataset.jog, isEditing: false }); };
  button.addEventListener('pointerup', release);
  button.addEventListener('pointercancel', release);
  button.addEventListener('lostpointercapture', release);
}

document.getElementById('btn-save-draft').addEventListener('click', async () => {
  const targetJoint = selectedJoint;
  const status = document.getElementById('draft-status');
  try {
    if ([inputKp, inputKd, inputMaxRpm, inputEffort, inputCalAngle, inputCalLower, inputCalUpper].some(input => !input.value.trim())) {
      throw new Error('Draft fields cannot be blank');
    }
    const values = {
      kp: draftNumber(inputKp.value),
      kd: draftNumber(inputKd.value),
      max_rpm: draftNumber(inputMaxRpm.value),
      effort_limit_nm: draftNumber(inputEffort.value),
      known_angle_deg: draftNumber(inputCalAngle.value),
      direction: Number(selectCalDir.value),
      lower_deg: draftNumber(inputCalLower.value),
      upper_deg: draftNumber(inputCalUpper.value)
    };
    if (!Object.values(values).every(Number.isFinite)) throw new Error('Draft values must be finite numbers');
    const response = await fetch('/api/draft', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ session: inputClient.session, joint: targetJoint, values }),
      signal: AbortSignal.timeout(1000)
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.reason || 'Draft rejected');
    status.textContent = `Draft for ${targetJoint} saved in backend memory. Not applied to the motor.`;
  } catch (error) {
    status.textContent = error.message;
  }
});

async function selectPreview(kind) {
  try {
    validateModel(armScene.model);
    if (isArmed) await stopNow('new_preview_selected');
    isRangePreview = false;
    preview.select(kind, preview.selected ? preview.q : observedPose);
    armScene.showPath(kind === 'figure8' ? Array.from({length: 201}, (_, i) => figureEight(i / 20)) : null);
    updateStatusPresentation(lastStateData);
    document.getElementById('preview-status').textContent = `${kind === 'figure8' ? 'Figure-8' : kind === 'ready' ? 'Ready pose' : 'Calibration pose'} · virtual preview`;
  } catch (error) {
    logEvent(error.message, 'error');
  }
}
document.getElementById('btn-ready').addEventListener('click', () => selectPreview('ready'));
document.getElementById('btn-cal-pose').addEventListener('click', () => selectPreview('calibration'));
document.getElementById('btn-figure8').addEventListener('click', () => selectPreview('figure8'));
document.getElementById('btn-live-view').addEventListener('click', async () => {
  if (isArmed) await stopNow('live_view_selected');
  preview.clear();
  isRangePreview = false;
  followPreview.checked = false;
  armScene.showPath(null);
  document.getElementById('preview-status').textContent = 'Live / disconnected pose';
  if (observedPose.every(p => Number.isFinite(p))) {
    armScene.updateJoints(...observedPose);
  }
});

let previewFrame = performance.now();
function animatePreview(now) {
  const dt = (now - previewFrame) / 1000;
  previewFrame = now;
  if (preview.selected) {
    if (!(followPreview.checked && isConnected && !freshFeedback)) {
      try { preview.step(dt); } catch (error) { logEvent(error.message, 'error'); }
    }
    const q = [...preview.q];
    if (followPreview.checked && isConnected && currentProfileId === 'arm_j1' && Number.isFinite(measuredJ1)) {
      q[0] = measuredJ1;
    }
    armScene.updateJoints(...q);
  }
  requestAnimationFrame(animatePreview);
}

if (btnClearLog) {
  btnClearLog.addEventListener('click', () => {
    eventLogBox.innerHTML = '';
    logEvent('Log cleared.');
  });
}

updateProfileUI();
onSelectedJointChanged();
requestAnimationFrame(animatePreview);
