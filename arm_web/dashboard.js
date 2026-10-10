let lastBackendEvent = null;
import {
  createInputState,
  reduceKeyEvent,
  reduceGamepad,
  validateGamepad,
  reduceBlur,
  reduceVisibilityChange,
  reduceArmState,
  clearStopLatch,
  clearStopRequest,
  latchStop,
  InputClient
} from './input_reducer.js';

import { ArmScene } from './arm_scene.js';
import { PreviewPlayer, validateModel, figureEight, followAxis } from './presets.js';
const badgeMode = document.getElementById('badge-mode');
const badgeConnection = document.getElementById('badge-connection');
const badgeState = document.getElementById('badge-state');
const badgeJ1Source = document.getElementById('badge-j1-source');
const btnStop = document.getElementById('btn-stop');

const selectPort = document.getElementById('select-port');
const btnRefreshPorts = document.getElementById('btn-refresh-ports');

const alertBanner = document.getElementById('alert-banner');
const stepConnect = document.getElementById('step-connect');
const stepCheck = document.getElementById('step-check');
const stepZero = document.getElementById('step-zero');
const stepJog = document.getElementById('step-jog');
const motionIndicator = document.getElementById('motion-indicator');
const j1PosGauge = document.getElementById('j1-pos-gauge');
const j1EffortBar = document.getElementById('j1-effort-bar');
const btnJogJ1Neg = document.getElementById('btn-jog-j1-neg');
const btnJogJ1Pos = document.getElementById('btn-jog-j1-pos');
const btnClearLog = document.getElementById('btn-clear-log');

let alertHideTimeout = null;
function showAlert(message, type = 'info', durationMs = 0) {
  if (!alertBanner) return;
  if (alertHideTimeout) {
    clearTimeout(alertHideTimeout);
    alertHideTimeout = null;
  }
  alertBanner.textContent = message;
  alertBanner.className = `alert-banner ${type}`;
  alertBanner.style.display = 'flex';
  if (durationMs > 0) {
    alertHideTimeout = setTimeout(hideAlert, durationMs);
  }
}

function hideAlert() {
  if (!alertBanner) return;
  alertBanner.style.display = 'none';
  if (alertHideTimeout) {
    clearTimeout(alertHideTimeout);
    alertHideTimeout = null;
  }
}

let knownPorts = [];

function updatePortOptions(ports, defaultPort) {
  if (!Array.isArray(ports)) return;
  const currentVal = selectPort.value;
  const portsJson = JSON.stringify(ports);
  if (portsJson === JSON.stringify(knownPorts) && selectPort.options.length > 0) {
    return;
  }
  knownPorts = ports;

  selectPort.innerHTML = '';
  if (ports.length === 0) {
    const opt = document.createElement('option');
    opt.value = '';
    opt.textContent = 'No serial ports detected';
    selectPort.appendChild(opt);
    return;
  }

  let selected = false;
  for (const p of ports) {
    const opt = document.createElement('option');
    const dev = typeof p === 'string' ? p : p.device;
    const label = typeof p === 'string' ? p : (p.label || p.device);
    opt.value = dev;
    opt.textContent = label;
    if (dev === currentVal || (!selected && dev === defaultPort)) {
      opt.selected = true;
      selected = true;
    }
    selectPort.appendChild(opt);
  }
  if (!selected && selectPort.options.length > 0) {
    selectPort.selectedIndex = 0;
  }
}

btnRefreshPorts.addEventListener('click', async () => {
  try {
    btnRefreshPorts.disabled = true;
    const res = await fetch('/api/ports');
    if (res.ok) {
      const data = await res.json();
      updatePortOptions(data.ports, data.default);
      logEvent(`Refreshed serial ports (${data.ports.length} found)`);
    } else {
      logEvent(`Failed to refresh ports: HTTP ${res.status}`, 'error');
    }
  } catch (err) {
    logEvent(`Failed to refresh ports: ${err.message}`, 'error');
  } finally {
    btnRefreshPorts.disabled = false;
  }
});
const inputBaud = document.getElementById('input-baud');
const inputId = document.getElementById('input-id');
const btnConnect = document.getElementById('btn-connect');
const btnDisconnect = document.getElementById('btn-disconnect');
const btnCheck = document.getElementById('btn-check');
const btnAck = document.getElementById('btn-ack');
const chkAck = document.getElementById('chk-ack-fault');

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

const telJ2Pos = document.getElementById('tel-j2-pos');
const telJ2Vel = document.getElementById('tel-j2-vel');
const telJ3Pos = document.getElementById('tel-j3-pos');
const telJ3Vel = document.getElementById('tel-j3-vel');
const eventLogBox = document.getElementById('event-log');
const btnResetCam = document.getElementById('btn-reset-cam');
const inputState = createInputState();
const inputClient = new InputClient();
const canvasContainer = document.getElementById('canvas-container');
const armScene = new ArmScene(canvasContainer);
const preview=new PreviewPlayer();
const followPreview=document.getElementById('chk-follow-preview');
let measuredJ1=null, appliedSettings=null, observedPose=[0,0,0];

let isDemoMode = null;
let modeKnown = false;
let isConnected = false;
let ownsMotor = false;
let isArmed = false;
let connectionKnown = false;
let armedKnown = false;
let statePollController = null;
let statePollGeneration = 0;
let lastKnownState = null;

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
async function claimOwnership() {
  if (!inputClient.session) return;
  try {
    const res = await fetch('/api/claim', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ session: inputClient.session })
    });
    if (res.ok) {
      ownsMotor = true;
      updateActionControls();
    }
  } catch (e) {}
}

async function ensureAction(action, values = {}) {
  if (!ownsMotor) await claimOwnership();
  return inputClient.requestAction(action, values);
}

async function initSession() {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3000);
  try {
    const saved = localStorage.getItem('arm-session') || sessionStorage.getItem('arm-session');
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
        localStorage.setItem('arm-session', saved);
        ownsMotor = resume.controls_motor === true;
        if (Number.isSafeInteger(resume.last_sequence)) inputClient.seq = Math.max(0, resume.last_sequence + 1);
        if (!ownsMotor && isConnected) void claimOwnership();
        updateActionControls();
        return;
      }
      localStorage.removeItem('arm-session');
      sessionStorage.removeItem('arm-session');
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
    localStorage.setItem('arm-session', data.session);
    sessionStorage.setItem('arm-session', data.session);
    ownsMotor = true;
    logEvent('Dashboard connected.');
    if (isConnected) void claimOwnership();
    updateActionControls();
  } catch (err) {
    logEvent(`Session connection error: ${err.message}`, 'error');
    setTimeout(initSession, 1500);
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
    if (generation === statePollGeneration && statePollController === controller) updateDashboardState(data);
  } catch (err) {
    if (generation === statePollGeneration && err.name === 'AbortError') {
      logEvent('State poll timed out (deadline 2s)', 'error');
    } else if (generation === statePollGeneration) {
      logEvent(`State poll failed: ${err.message}`, 'error');
    }
  } finally {
    clearTimeout(timeoutId);
    if (statePollController === controller) statePollController = null;
  }
}

function finiteValue(value) {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}

function showPosition(element, joint) {
  const radians = finiteValue(joint?.position_rad);
  if (radians === null) {
    element.textContent = 'Awaiting known angle';
    return null;
  }
  element.textContent = `${radians.toFixed(3)} rad (${(radians * 180 / Math.PI).toFixed(2)}°)`;
  return radians;
}

function showVelocity(element, joint, includeRpm = false) {
  const radiansPerSecond = finiteValue(joint?.velocity_rad_s);
  if (radiansPerSecond === null) {
    element.textContent = 'Awaiting status';
    return;
  }
  const rpm = radiansPerSecond * 60 / (2 * Math.PI);
  element.textContent = includeRpm
    ? `${radiansPerSecond.toFixed(3)} rad/s (${rpm.toFixed(2)} rpm)`
    : `${radiansPerSecond.toFixed(3)} rad/s`;
}

function updateActionControls() {
  btnConnect.disabled = isConnected;
  btnDisconnect.disabled = !isConnected;
  btnCheck.disabled = !isConnected || isArmed;
  btnAck.disabled = !isConnected || isArmed || !chkAck.checked;
  btnCalibrate.disabled = !isConnected || isArmed;
  btnZeroJ1.disabled = btnCalibrate.disabled;
  btnConfigure.disabled = !isConnected || isArmed;
  btnArm.disabled = !isConnected || isArmed
    || !chkEnableArm.checked || inputState.stopLatched;
  btnRelease.disabled = !isConnected || !isArmed;
}

function updateDashboardState(data) {
  modeKnown = typeof data.physical_enabled === 'boolean';
  isDemoMode = modeKnown ? !data.physical_enabled : null;
  const portList = Array.isArray(data.ports) ? data.ports : (Array.isArray(data.available_ports) ? data.available_ports : null);
  if (portList) {
    updatePortOptions(portList, data.default_port);
  } else if (!selectPort.value && typeof data.default_port === 'string') {
    const opt = document.createElement('option');
    opt.value = data.default_port;
    opt.textContent = data.default_port;
    selectPort.appendChild(opt);
  }
  connectionKnown = typeof data.connected === 'boolean';
  isConnected = connectionKnown ? data.connected : false;
  const wasArmed = isArmed;
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

  badgeConnection.textContent = !connectionKnown ? 'Status Awaiting' : (isConnected ? 'Connected' : 'Disconnected');
  badgeConnection.className = `badge ${isConnected ? 'badge-connected' : 'badge-disconnected'}`;

  const isMoving = Boolean(inputState.motionEnergized || inputState.isEnergized);
  badgeState.textContent = inputState.stopLatched ? 'STOP LATCHED · Check required'
    : !armedKnown ? 'Awaiting status'
      : (isArmed ? (isMoving ? 'JOGGING' : 'ARMED · READY TO JOG')
        : (String(lastKnownState).startsWith('Fault') ? 'FAULT' : 'Disarmed'));
  badgeState.className = `badge ${inputState.stopLatched ? 'badge-fault' : (isArmed ? (isMoving ? 'badge-jogging' : 'badge-armed') : (String(lastKnownState).startsWith('Fault') ? 'badge-fault' : ''))}`;
  telLifecycle.textContent = lastKnownState || 'Awaiting status';
  telReason.textContent = typeof data.reason === 'string' && data.reason ? data.reason : 'Awaiting status';
  const feedbackAge = finiteValue(data.feedback_age_s);
  telFeedbackAge.textContent = feedbackAge !== null && feedbackAge >= 0 ? `${feedbackAge.toFixed(3)} s` : 'Awaiting status';
  telStopConfirmed.textContent = !isConnected ? 'Disconnected — see final Stop result below' : data.stop_confirmed === true ? 'Confirmed'
    : data.stop_confirmed === false ? 'Not confirmed' : 'Awaiting status';
  updateActionControls();

  // Workflow Stepper & Guidance
  if (stepConnect && stepCheck && stepZero && stepJog) {
    const isConn = Boolean(isConnected);
    const isTimeout = isConn && String(data.state).startsWith('Timeout');
    const isCalib = Boolean(data.calibrated);
    const isArm = Boolean(isArmed);

    stepConnect.className = `step-node ${isConn ? 'completed' : 'active'}`;
    if (!isConn) {
      stepCheck.className = 'step-node';
      stepZero.className = 'step-node';
      stepJog.className = 'step-node';
    } else {
      stepCheck.className = `step-node ${isTimeout || inputState.stopLatched ? 'active' : 'completed'}`;
      if (isTimeout) {
        stepZero.className = 'step-node';
        stepJog.className = 'step-node';
      } else {
        stepZero.className = `step-node ${isCalib ? 'completed' : 'active'}`;
        stepJog.className = `step-node ${!isCalib ? '' : isArm ? 'completed active' : 'active'}`;
      }
    }
  }

  // Contextual Alert Banner
  if (inputState.stopLatched) {
    showAlert(`🛑 Stop Latched: ${data.reason || 'Check required to clear Stop'}. Click "Health Check" to resume.`, 'error');
  } else if (String(data.state).startsWith('Timeout')) {
    showAlert('⚠️ Startup Watchdog Timeout: Confirm safety checkbox, then click "Acknowledge Timeout".', 'warn');
  } else if (isArmed) {
    if (isMoving) {
      showAlert('⚡ J1 Moving: Hold A/D or click −/+ to jog. Release keys to hold position.', 'info');
    } else {
      showAlert('⚡ Motor Armed & Ready: Press A/D or click −/+ to jog J1 directly. Esc stops instantly.', 'success');
    }
  } else if (alertBanner && alertBanner.style.display !== 'none') {
    if (alertBanner.className.includes('error') && !inputState.stopLatched) {
      hideAlert();
    } else if (alertBanner.className.includes('warn') && !String(data.state).startsWith('Timeout')) {
      hideAlert();
    }
  }

  const help=document.getElementById('workflow-help');
  help.textContent=!isConnected ? '1. Select serial port and Connect (Channel D adapter prioritized).'
    : String(data.state).startsWith('Timeout') ? 'Confirm the secured-motor checkbox, then Acknowledge Timeout. The motor stays in Stop.'
    : !data.calibrated ? '2. Set the current angle (0 means this pose is your zero), then Use measured angle for J1.'
    : isArmed ? 'Press A/D, click −/+, or use controller stick to jog J1. Release to hold position; Stop removes torque. J2/J3 are virtual.'
    : inputState.stopLatched ? 'Press Check to clear Stop, then confirm your setup and Enable jogging.'
    : '3. Confirm your setup, then Enable jogging. Start with short movements.';
  const applied=data.settings;
  appliedSettings=applied;
  followPreview.disabled=isArmed;
  telPendingOp.textContent=inputClient.ordinaryRun ? 'Processing…' : 'None';
  if(applied) document.getElementById('applied-settings').textContent=`Applied: Kp ${applied.kp}, Kd ${applied.kd}, ${applied.max_rpm} rpm, ${applied.effort_limit_nm} Nm; travel ${applied.lower_deg}° to ${applied.upper_deg}°.`;
  const joints = Array.isArray(data.joints) ? data.joints : [];
  const [j1, j2, j3] = joints;
  const positions = [showPosition(telJ1Pos, j1), showPosition(telJ2Pos, j2), showPosition(telJ3Pos, j3)];
  showVelocity(telJ1Vel, j1, true);
  showVelocity(telJ2Vel, j2);
  showVelocity(telJ3Vel, j3);

  const source = j1?.source;
  const sourceText = source === 'physical' ? 'Encoder feedback'
    : source === 'demo' ? 'Simulated feedback'
      : 'Awaiting feedback source';
  badgeJ1Source.textContent = sourceText;
  telJ1Source.textContent = sourceText;
  const torque = finiteValue(j1?.torque_nm);
  const voltage = finiteValue(j1?.voltage);
  const driverTemp = finiteValue(j1?.driver_temp);
  const windingTemp = finiteValue(j1?.winding_temp);
  telJ1Torque.textContent = torque === null ? 'Awaiting status' : `${torque.toFixed(2)} Nm`;
  telJ1Mode.textContent = Number.isInteger(j1?.mode) ? String(j1.mode) : 'Awaiting status';
  telJ1Volt.textContent = voltage === null ? 'Awaiting status' : `${voltage.toFixed(1)} V`;
  telJ1Temp.textContent = `${driverTemp === null ? 'Awaiting status' : `${driverTemp.toFixed(1)}°C`} / ${windingTemp === null ? 'Awaiting status' : `${windingTemp.toFixed(1)}°C`}`;
  if (Number.isInteger(j1?.errors) && Number.isInteger(j1?.warnings)) {
    const faulted = j1.errors !== 0 || j1.warnings !== 0;
    telJ1Errors.textContent = faulted ? `Errors 0x${j1.errors.toString(16)} / warnings ${j1.warnings}` : 'None reported';
    telJ1Errors.className = `telemetry-val ${faulted ? 'error' : 'ok'}`;
  } else {
    telJ1Errors.textContent = 'Awaiting status';
    telJ1Errors.className = 'telemetry-val';
  }

  // Visual Position Gauge
  if (j1PosGauge && Number.isFinite(positions[0])) {
    const lowerDeg = applied?.lower_deg ?? -90;
    const upperDeg = applied?.upper_deg ?? 90;
    const range = upperDeg - lowerDeg;
    const curDeg = positions[0] * 180 / Math.PI;
    const pct = Math.max(0, Math.min(100, ((curDeg - lowerDeg) / (range || 1)) * 100));
    if (pct >= 50) {
      j1PosGauge.style.left = '50%';
      j1PosGauge.style.width = `${Math.min(50, pct - 50)}%`;
    } else {
      j1PosGauge.style.left = `${pct}%`;
      j1PosGauge.style.width = `${50 - pct}%`;
    }
  }

  // Visual Effort / Torque Bar
  if (j1EffortBar && torque !== null) {
    const maxEffort = applied?.effort_limit_nm ?? 1.0;
    const effortPct = Math.min(100, (Math.abs(torque) / maxEffort) * 100);
    j1EffortBar.style.width = `${effortPct.toFixed(1)}%`;
    if (effortPct >= 85) {
      j1EffortBar.style.background = 'var(--accent-rose)';
    } else if (effortPct >= 60) {
      j1EffortBar.style.background = 'var(--accent-amber)';
    } else {
      j1EffortBar.style.background = 'var(--accent-emerald)';
    }
  }

  // Visual Motion Indicator
  if (motionIndicator) {
    if (!isArmed) {
      motionIndicator.textContent = 'Disarmed';
      motionIndicator.style.color = 'var(--text-muted)';
    } else if (!isMoving) {
      motionIndicator.textContent = 'Holding position';
      motionIndicator.style.color = 'var(--accent-cyan)';
    } else {
      const axis0 = inputState.effectiveAxes[0];
      if (axis0 > 0.01) {
        motionIndicator.textContent = 'J1 Jogging CW ▶';
        motionIndicator.style.color = 'var(--accent-emerald)';
      } else if (axis0 < -0.01) {
        motionIndicator.textContent = '◀ J1 Jogging CCW';
        motionIndicator.style.color = 'var(--accent-emerald)';
      } else {
        motionIndicator.textContent = 'Jogging Virtual Joints';
        motionIndicator.style.color = 'var(--accent-amber)';
      }
    }
  }

  measuredJ1=positions[0];
  if(positions.every(position=>position!==null))observedPose=positions;
  if(!preview.selected && positions.every(position=>position!==null))armScene.updateJoints(...positions);
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
  preview.stop();
  latchStop(inputState, reason);
  clearStopRequest(inputState);
  isArmed = false;
  invalidateStatePoll();
  badgeState.textContent = 'STOP LATCHED · Check required';
  badgeState.className = 'badge badge-fault';
  telStopConfirmed.textContent = 'Stop requested';
  updateActionControls();
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
    applyLocalStop('client_request_failure');
  }
}

function serviceReducerStop() {
  if (!inputState.stopRequested) return;
  const reason = inputState.stopReason || 'input_stop';
  clearStopRequest(inputState);
  if(ownsMotor || reason==='gamepad_b_stop')void stopNow(reason);
}

function readBounded(input, label, minimum, maximum) {
  if (!input || input.value.trim() === '') throw new Error(`${label} is required`);
  const value = Number(input.value);
  if (!Number.isFinite(value)) throw new Error(`${label} must be a finite number`);
  if (value < minimum || value > maximum) logEvent(`Warning: ${label} is outside the recommended ${minimum}–${maximum} range.`, 'warn');
  return value;
}

btnStop.addEventListener('click', () => { void stopNow('operator_stop_button'); });

btnConnect.addEventListener('click', async () => {
  const port = selectPort.value.trim();
  const baud = parseInt(inputBaud.value, 10);
  const id = parseInt(inputId.value, 10);
  const demo = typeof isDemoMode === 'boolean' ? isDemoMode : false;

  logEvent(`Connecting (port=${port}, baud=${baud}, id=${id}, demo=${demo})...`);
  try {
    const res = await ensureAction('connect', {
      demo,
      port,
      baud,
      id
    });
    ownsMotor = res.status === 'complete';
    logEvent(`Connect result: ${res.status}${res.reason ? ' (' + res.reason + ')' : ''}`);
    updateActionControls();
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Connect rejected: ${err.message}`, 'error');
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
  try {
    const angle = readBounded(inputCalAngle, 'Measured current angle', -90, 90);
    const dir = Number(selectCalDir.value);
    const lower = readBounded(inputCalLower, 'Lower limit', -90, 90);
    const upper = readBounded(inputCalUpper, 'Upper limit', -90, 90);
    if (![1, -1].includes(dir) || lower < -90 || upper > 90 || lower >= upper || angle < lower || angle > upper) {
      throw new Error('Applied travel must stay within −90° to +90° and contain the measured angle. Save larger values as a draft.');
    }
    logEvent(`Setting software zero: measured angle=${angle}°, dir=${dir}, limits=[${lower}°, ${upper}°]...`);
    const res = await ensureAction('calibrate', {
      known_angle_deg: angle,
      direction: dir,
      lower_deg: lower,
      upper_deg: upper
    });
    logEvent(`Calibrate result: ${res.status}`);
    if (res.status === 'complete') {
      const checked = await ensureAction('check', {});
      if (checked.status === 'complete') clearStopLatch(inputState);
    }
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Calibrate rejected: ${err.message}`, 'error');
  }
});

btnConfigure.addEventListener('click', async () => {
  try {
    const kp = readBounded(inputKp, 'Kp', 0, 30);
    const kd = readBounded(inputKd, 'Kd', 0, 6);
    const maxRpm = readBounded(inputMaxRpm, 'Maximum RPM', 0.1, 1.0);
    const effort = readBounded(inputEffort, 'Effort limit', 0.1, 1.0);
    logEvent(`Configuring PD gains: Kp=${kp}, Kd=${kd}, MaxRPM=${maxRpm}, Effort=${effort}Nm...`);
    const res = await ensureAction('configure', {
      kp,
      kd,
      max_rpm: maxRpm,
      effort_limit_nm: effort
    });
    logEvent(`Configure result: ${res.status}`);
    if (res.status === 'complete') {
      const checked = await ensureAction('check', {});
      if (checked.status === 'complete') clearStopLatch(inputState);
    }
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Configure rejected: ${err.message}`, 'error');
  }
});

chkAck.addEventListener('change', updateActionControls);
chkEnableArm.addEventListener('change', updateActionControls);

btnArm.addEventListener('click', async () => {
  if (!chkEnableArm.checked || inputState.stopLatched || !isConnected || isArmed) {
    logEvent('Arm blocked: confirm the arm, clear Stop with Check, and verify connected status', 'error');
    updateActionControls();
    return;
  }
  logEvent('Requesting jog authority; motor is armed and ready to jog.', 'info');
  try {
    const res = await ensureAction('arm', { supported: chkEnableArm.checked });
    logEvent(`Arm result: ${res.status}${res.reason ? ' (' + res.reason + ')' : ''}`);
    if (res.status === 'complete') await inputClient.submitJog(false, [0, 0, 0], inputState.activeDevice);
    chkEnableArm.checked = false;
    document.activeElement?.blur();
    updateActionControls();
  } catch (err) {
    syncClientStopLatch();
    logEvent(`Arm rejected: ${err.message}`, 'error');
  }
});

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
  reduceKeyEvent(inputState, { type: 'keydown', code: e.code || ({' ':'Space',a:'KeyA',d:'KeyD',w:'KeyW',s:'KeyS',q:'KeyQ',e:'KeyE',Escape:'Escape'}[e.key]), isEditing });

  if (e.code === 'Escape') {
    e.preventDefault();
    void stopNow('keyboard_escape');
  } else if (e.code === 'Space' && !isEditing) {
    e.preventDefault();
  }
});

window.addEventListener('keyup', (e) => {
  const isEditing = focusedInEditor();
  reduceKeyEvent(inputState, { type: 'keyup', code: e.code || ({' ':'Space',a:'KeyA',d:'KeyD',w:'KeyW',s:'KeyS',q:'KeyQ',e:'KeyE',Escape:'Escape'}[e.key]), isEditing });
});

window.addEventListener('blur', () => {
  reduceBlur(inputState);
});

document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    reduceVisibilityChange(inputState, true);
    if(ownsMotor)void stopNow('document_hidden');
  }
});

window.addEventListener('pagehide', () => {
  reduceBlur(inputState);
  if(ownsMotor)void stopNow('pagehide', { keepalive: true, log: false });
  else if(inputClient.session)void fetch('/api/retire', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({session:inputClient.session}),keepalive:true}).catch(()=>{});
});

window.addEventListener('gamepadconnected', event => {
  reduceGamepad(inputState, event.gamepad);
  serviceReducerStop();
});

window.addEventListener('gamepaddisconnected', () => {
  reduceGamepad(inputState, null);
  serviceReducerStop();
});
function pollGamepad() {
  if (typeof navigator.getGamepads !== 'function') {
    gamepadStatus.textContent = 'This browser cannot read controllers. Open this address in Chrome or Firefox.';
    reduceGamepad(inputState, null);
    serviceReducerStop();
    return;
  }
  const gamepads = Array.from(navigator.getGamepads() || []);
  const pad = gamepads.find(candidate => candidate && candidate.index === 0 && candidate.connected === true) || null;
  if (pad) {
    const validation = validateGamepad(pad);
    reduceGamepad(inputState, pad);
    gamepadStatus.textContent = validation.valid
      ? `Xbox connected: ${pad.id.slice(0, 48)}. Jog J1 with left stick. Press B to Stop.`
      : `Controller cannot jog: ${validation.reason}. Use a standard Xbox controller in Chrome or Firefox.`;
    gamepadStatus.style.color = validation.valid ? '#166534' : '#b91c1c';
  } else {
    gamepadStatus.textContent = 'Controller not visible yet. Focus this page and press A on your Xbox controller. If needed, open this address in Chrome or Firefox.';
    gamepadStatus.style.color = '#64748b';
    reduceGamepad(inputState, null);
  }
  serviceReducerStop();
}

function jogLoop() {
  pollGamepad();
  syncClientStopLatch();
  serviceReducerStop();
  if (!ownsMotor || !isArmed || inputState.stopLatched || !inputClient.session) return;

  const isMoving = Boolean(inputState.motionEnergized || inputState.isEnergized);
  document.getElementById('input-status').textContent = isMoving ? `Jogging · ${inputState.activeDevice}` : 'Holding position';
  const enteredScale = Number(sliderJogSpeed.value);
  if (!sliderJogSpeed.value.trim() || !Number.isFinite(enteredScale)) { void stopNow('invalid_jog_multiplier'); return; }
  const speedScale = Math.max(0,Math.min(1,enteredScale/100));
  const axes = inputState.effectiveAxes.map(axis => axis * speedScale);

  if (btnJogJ1Neg) {
    const isA = inputState.keysDown.has('KeyA') || (axes[0] < -0.05);
    btnJogJ1Neg.classList.toggle('jog-active', Boolean(isArmed && isA));
  }
  if (btnJogJ1Pos) {
    const isD = inputState.keysDown.has('KeyD') || (axes[0] > 0.05);
    btnJogJ1Pos.classList.toggle('jog-active', Boolean(isArmed && isD));
  }

  if(preview.selected && followPreview.checked){
    const follow=followAxis(preview.q[0],measuredJ1,appliedSettings?.lower_deg,appliedSettings?.upper_deg);
    axes[0] = isMoving ? follow.axis*speedScale : 0;
    axes[1]=axes[2]=0;
    document.getElementById('preview-status').textContent=follow.blocked ? 'J1 follow paused: target outside applied limits / feedback unavailable' : 'J1 measured; J2/J3 path preview · Jogging to follow';
  }
  inputClient.submitJog(inputState.effectiveDeadman, axes, inputState.activeDevice)
    .catch(error => {
      const alreadyStopped=inputState.stopLatched;
      syncClientStopLatch();
      if(!alreadyStopped) logEvent(`Jog input failed and Stop was requested: ${error.message}`, 'error');
    });
}
initSession();
setInterval(pollState, 200);
setInterval(jogLoop, 50);

function updateParameterWarnings() {
  const warnings=[];
  for (const [input,label,low,high] of [[sliderJogSpeed,'Jog multiplier (%)',0,100],[inputKp,'Kp',0,30],[inputKd,'Kd',0,6],[inputMaxRpm,'RPM',0,1],[inputEffort,'Effort (Nm)',0,1],[inputCalLower,'Lower angle',-90,90],[inputCalUpper,'Upper angle',-90,90]]) {
    if (input.value.trim() && (!Number.isFinite(Number(input.value)) || Number(input.value)<low || Number(input.value)>high)) warnings.push(`${label}: outside the recommended ${low}–${high} range.`);
  }
  document.getElementById('parameter-warning').textContent=warnings.length?'WARNING — outside bench or wire-management recommendations. Physical command limits currently still apply.\n'+warnings.join('\n'):'';
}
for (const input of [inputKp,inputKd,inputMaxRpm,inputEffort,inputCalLower,inputCalUpper]) input.addEventListener('input',updateParameterWarnings);
updateParameterWarnings();
for (const button of document.querySelectorAll('[data-jog]')) {
  let held=false;
  button.addEventListener('pointerdown',event=>{
    event.preventDefault(); if(!isArmed||held)return;
    held=true;button.setPointerCapture(event.pointerId);
    reduceKeyEvent(inputState,{type:'keydown',code:button.dataset.jog,isEditing:false});
  });
  const release=()=>{if(!held)return;held=false;reduceKeyEvent(inputState,{type:'keyup',code:button.dataset.jog,isEditing:false});};
  button.addEventListener('pointerup',release);button.addEventListener('pointercancel',release);button.addEventListener('lostpointercapture',release);
}

document.getElementById('btn-save-draft').addEventListener('click',async()=>{
  const status=document.getElementById('draft-status');
  try {
    if([inputKp,inputKd,inputMaxRpm,inputEffort,inputCalAngle,inputCalLower,inputCalUpper].some(input=>!input.value.trim()))throw new Error('Draft fields cannot be blank');
    const values={kp:Number(inputKp.value),kd:Number(inputKd.value),max_rpm:Number(inputMaxRpm.value),effort_limit_nm:Number(inputEffort.value),known_angle_deg:Number(inputCalAngle.value),direction:Number(selectCalDir.value),lower_deg:Number(inputCalLower.value),upper_deg:Number(inputCalUpper.value)};
    if(!Object.values(values).every(Number.isFinite))throw new Error('Draft values must be finite numbers');
    const response=await fetch('/api/draft',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({session:inputClient.session,values}),signal:AbortSignal.timeout(1000)});
    const result=await response.json();if(!response.ok)throw new Error(result.reason||'Draft rejected');
    status.textContent='Draft saved in backend memory. Not applied to the motor.';
  } catch(error){status.textContent=error.message;}
});

async function selectPreview(kind){
  try {
    validateModel(armScene.model);
    if(isArmed)await stopNow('new_preview_selected');
    preview.select(kind,preview.selected ? preview.q : observedPose);
    armScene.showPath(kind==='figure8'?Array.from({length:201},(_,i)=>figureEight(i/20)):null);
    document.getElementById('preview-status').textContent=`${kind==='figure8'?'Figure-8':kind==='ready'?'Ready pose':'Calibration pose'} · virtual preview`;
  }catch(error){logEvent(error.message,'error');}
}
document.getElementById('btn-ready').addEventListener('click',()=>selectPreview('ready'));
document.getElementById('btn-cal-pose').addEventListener('click',()=>selectPreview('calibration'));
document.getElementById('btn-figure8').addEventListener('click',()=>selectPreview('figure8'));
document.getElementById('btn-live-view').addEventListener('click',async()=>{if(isArmed)await stopNow('live_view_selected');preview.clear();followPreview.checked=false;armScene.showPath(null);document.getElementById('preview-status').textContent='Live / disconnected pose';});
let previewFrame=performance.now();
function animatePreview(now){
  const dt=(now-previewFrame)/1000;previewFrame=now;
  if(preview.selected){
    try {preview.step(dt);}catch(error){logEvent(error.message,'error');}
    const q=[...preview.q];
    if(followPreview.checked&&isConnected&&Number.isFinite(measuredJ1))q[0]=measuredJ1;
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

requestAnimationFrame(animatePreview);
