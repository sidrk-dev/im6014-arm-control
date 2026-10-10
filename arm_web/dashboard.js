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
import { canAcknowledgeStartupTimeout, deriveDashboardPresentation, deriveWorkflowSteps, finiteNumber as finiteValue, j1GaugePresentation, jointPositionPresentation, jointVelocityText, serialPortStatus } from './presentation.js';
import { PreviewPlayer, validateModel, figureEight, followAxis } from './presets.js';
const badgeMode = document.getElementById('badge-mode');
const badgeConnection = document.getElementById('badge-connection');
const badgeState = document.getElementById('badge-state');
const badgeJ1Source = document.getElementById('badge-j1-source');
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
const inputBaud = document.getElementById('input-baud');
const inputId = document.getElementById('input-id');
const btnConnect = document.getElementById('btn-connect');
const btnDisconnect = document.getElementById('btn-disconnect');
const btnCheck = document.getElementById('btn-check');
const btnAck = document.getElementById('btn-ack');
const timeoutRecovery = document.getElementById('timeout-recovery');
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
let followMotion=false, followBlocked=false;

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
  if (!ownsMotor && action !== 'connect') throw new Error('This tab is observing; connect here after the current owner disconnects.');
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

function updateJ1Gauges(positionRad, torque, applied) {
  const gauge = j1GaugePresentation({
    positionRad, lowerDeg: applied?.lower_deg, upperDeg: applied?.upper_deg,
    connected: isConnected, fresh: freshFeedback
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
    armed: isArmed
  });
  const hasPort = Boolean(inputPortPath?.value.trim() || selectPort.value);
  const canConnect = isDemoMode === true || hasPort;
  selectPort.disabled = isDemoMode === true || isConnected;
  if (inputPortPath) inputPortPath.disabled = selectPort.disabled;
  btnConnect.disabled = statePollUnavailable || isConnected || !inputClient.session || !canConnect;
  btnDisconnect.disabled = !ownsMotor || !isConnected;
  btnCheck.disabled = statePollUnavailable || !ownsMotor || !isConnected || isArmed;
  btnAck.disabled = !canAcknowledgeTimeout || !chkAck.checked;
  if (timeoutRecovery) timeoutRecovery.hidden = !canAcknowledgeTimeout;
  if (!canAcknowledgeTimeout) chkAck.checked = false;
  btnCalibrate.disabled = !ownsMotor || !isConnected || isArmed || !readyStop;
  btnZeroJ1.disabled = btnCalibrate.disabled;
  btnConfigure.disabled = !ownsMotor || !isConnected || isArmed || !readyStop;
  btnArm.disabled = !isConnected || isArmed
    || !ownsMotor || !lastStateData?.calibrated || !readyStop
    || !chkEnableArm.checked || inputState.stopLatched;
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
      armed: isArmed
    }),
    stopLatched: inputState.stopLatched, stopReason: inputState.stopReason || data?.reason,
    motionActive: Boolean(inputState.motionEnergized || inputState.isEnergized || followMotion),
    axes: inputState.effectiveAxes, source: inputState.activeDevice,
    followSelected: Boolean(preview.selected && followPreview.checked), followActive: followMotion, followBlocked,
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
  const cachedJ1 = lastStateData?.joints?.[0];
  updateJ1Gauges(finiteValue(cachedJ1?.position_rad), finiteValue(cachedJ1?.torque_nm), lastStateData?.settings);
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
  badgeState.textContent = inputState.stopLatched ? 'STOP LATCHED · Check required'
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
  const applied=data.settings;
  appliedSettings=applied;
  followPreview.disabled=isArmed;
  telPendingOp.textContent=inputClient.ordinaryRun ? 'Processing…' : 'None';
  if(applied) document.getElementById('applied-settings').textContent=`Applied: Kp ${applied.kp}, Kd ${applied.kd}, ${applied.max_rpm} rpm, ${applied.effort_limit_nm} Nm; travel ${applied.lower_deg}° to ${applied.upper_deg}°.`;
  const joints = Array.isArray(data.joints) ? data.joints : [];
  const [j1, j2, j3] = joints;
  const positionElements = [telJ1Pos, telJ2Pos, telJ3Pos];
  const positionDisplays = [j1, j2, j3].map(jointPositionPresentation);
  const positions = positionDisplays.map((display, index) => {
    positionElements[index].textContent = display.text;
    return display.radians;
  });
  for (const [element, joint, includeRpm] of [[telJ1Vel, j1, true], [telJ2Vel, j2, false], [telJ3Vel, j3, false]]) {
    element.textContent = jointVelocityText(joint, includeRpm);
  }

  const source = j1?.source;
  const sourceText = source === 'physical' ? (freshFeedback ? 'Fresh encoder feedback' : 'STALE encoder feedback')
    : source === 'demo' ? (freshFeedback ? 'Fresh simulated feedback' : 'STALE simulated feedback')
      : 'Awaiting feedback source';
  badgeJ1Source.textContent = sourceText;
  telJ1Source.textContent = sourceText;
  const torque = finiteValue(j1?.torque_nm);
  const voltage = finiteValue(j1?.voltage);
  const driverTemp = finiteValue(j1?.driver_temp);
  const windingTemp = finiteValue(j1?.winding_temp);
  telJ1Torque.textContent = !isConnected ? 'Disconnected' : torque === null ? 'Awaiting status'
    : `${torque.toFixed(2)} Nm${freshFeedback ? '' : ' · cached'}`;
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

  updateJ1Gauges(positions[0], torque, applied);


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
  followMotion = false;
  latchStop(inputState, reason);
  clearStopRequest(inputState);
  isArmed = false;
  invalidateStatePoll();
  badgeState.textContent = 'STOP LATCHED · Check required';
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
    applyLocalStop('client_request_failure');
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

btnConnect.addEventListener('click', async () => {
  const port = inputPortPath?.value.trim() || selectPort.value.trim();
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
    const angle = readBounded(inputCalAngle, 'J1 angle at this pose', -90, 90);
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
  if (!ownsMotor || !chkEnableArm.checked || inputState.stopLatched || !isConnected || isArmed
    || !freshFeedback || !lastStateData?.calibrated || lastStateData?.state !== 'Ready · Stop'
    || lastStateData?.stop_confirmed !== true) {
    logEvent('Arm blocked: this tab must own a calibrated connection with fresh Ready/Stop feedback; clear Stop with Check and confirm Enable.', 'error');
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
  const speedScale = Math.max(0,Math.min(1,enteredScale/100));
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
  if (preview.selected && followPreview.checked) {
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
  }
  const motionIntent = axes.some(axis => Math.abs(axis) > 1e-4);
  followMotion = Boolean(preview.selected && followPreview.checked && motionIntent && !followBlocked);
  document.getElementById('input-status').textContent = preview.selected && followPreview.checked
    ? followMotion ? 'Automatic preview follow · J1'
      : followBlocked ? 'Preview follow paused · J1' : 'Preview follow ready · J1'
    : motionIntent ? `Jogging · ${inputState.activeDevice}` : 'Holding position';
  inputClient.submitJog(motionIntent, axes, inputState.activeDevice)
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
for (const input of [sliderJogSpeed,inputKp,inputKd,inputMaxRpm,inputEffort,inputCalLower,inputCalUpper]) input.addEventListener('input',updateParameterWarnings);
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
    updateStatusPresentation(lastStateData);
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
    if (!(followPreview.checked && isConnected && !freshFeedback)) {
      try {preview.step(dt);}catch(error){logEvent(error.message,'error');}
    }
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
