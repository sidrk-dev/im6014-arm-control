const STOP_KEYS = 'Stop, Esc, or controller B';
const MOTION_COLORS = {
  muted: 'var(--text-muted)', cyan: 'var(--accent-cyan)',
  emerald: 'var(--accent-emerald)', amber: 'var(--accent-amber)'
};

export function finiteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}

export function jointPositionPresentation(joint) {
  const radians = finiteNumber(joint?.position_rad);
  return {
    radians,
    text: radians === null ? 'Awaiting known angle'
      : `${radians.toFixed(3)} rad (${(radians * 180 / Math.PI).toFixed(2)}°)`
  };
}

export function jointVelocityText(joint, includeRpm = false) {
  const radiansPerSecond = finiteNumber(joint?.velocity_rad_s);
  if (radiansPerSecond === null) return 'Awaiting status';
  const rpm = radiansPerSecond * 60 / (2 * Math.PI);
  return includeRpm
    ? `${radiansPerSecond.toFixed(3)} rad/s (${rpm.toFixed(2)} rpm)`
    : `${radiansPerSecond.toFixed(3)} rad/s`;
}

export function j1GaugePresentation({ positionRad, lowerDeg, upperDeg, connected, fresh }) {
  const validBounds = Number.isFinite(lowerDeg) && Number.isFinite(upperDeg) && lowerDeg < upperDeg;
  const validPosition = Number.isFinite(positionRad);
  const visible = connected === true && validBounds && validPosition;
  if (!validBounds) {
    return {
      visible: false, status: 'Applied travel limits unavailable', lowerLabel: 'Lower limit unavailable',
      upperLabel: 'Upper limit unavailable', zeroLabel: 'Zero anchor', zeroPercent: 50,
      fillLeftPercent: 50, fillWidthPercent: 0
    };
  }
  const zeroDeg = Math.max(lowerDeg, Math.min(upperDeg, 0));
  const span = upperDeg - lowerDeg;
  const zeroPercent = ((zeroDeg - lowerDeg) / span) * 100;
  const angleDeg = validPosition ? positionRad * 180 / Math.PI : zeroDeg;
  const positionPercent = Math.max(0, Math.min(100, ((angleDeg - lowerDeg) / span) * 100));
  const formatLimit = value => (value > 0 ? '+' : '') + Number(value.toFixed(1)) + '°';
  return {
    visible,
    status: !connected ? 'Disconnected' : !validPosition ? 'J1 feedback unavailable'
      : fresh ? 'Fresh measured feedback' : 'STALE · cached feedback',
    lowerLabel: formatLimit(lowerDeg),
    upperLabel: formatLimit(upperDeg),
    zeroLabel: zeroDeg === 0 ? '0° (Zero)' : zeroDeg === lowerDeg ? 'Zero anchor · lower limit' : 'Zero anchor · upper limit',
    zeroPercent,
    fillLeftPercent: visible ? Math.min(zeroPercent, positionPercent) : zeroPercent,
    fillWidthPercent: visible ? Math.abs(positionPercent - zeroPercent) : 0
  };
}

export function serialPortStatus(ports, error = null, demo = false) {
  if (demo === true) return 'Demo mode uses a simulated J1 motor; no USB adapter is needed.';
  const records = Array.isArray(ports) ? ports.filter(Boolean) : [];
  const count = records.length;
  const channelDListed = records.some(port => port.channel_d === true);
  if (error) {
    return `Could not list USB adapters (${error}). Reconnect the adapter, then select Refresh.`;
  }
  if (!count) {
    return 'No USB adapter found. Reconnect the USB-RS485 adapter, then select Refresh.';
  }
  if (channelDListed) {
    return 'Channel D adapter found and marked Recommended. Select it, connect, then confirm fresh J1 feedback.';
  }
  const countText = count === 1 ? '1 USB adapter found' : `${count} USB adapters found`;
  return `${countText}, but Channel D was not identified. Select only the adapter wired to J1, then connect and check for fresh feedback.`;
}

export function canAcknowledgeStartupTimeout(state = {}) {
  const timeout = state.latest?.timeout === true || state.latest?.timeout === 1;
  return state.connected === true && state.owner === true && state.fresh === true
    && state.stopConfirmed === true && state.latest?.mode === 0 && timeout
    && state.armed !== true && state.stopLatched !== true;
}

export function deriveWorkflowSteps(state = {}) {
  const connected = state.connected === true;
  if (!connected) return { connect: 'active', check: '', zero: '', jog: '' };

  const fresh = state.fresh === true;
  const stoppedAndChecked = fresh && state.stopConfirmed === true
    && state.motorState === 'Ready · Stop' && state.latest?.mode === 0
    && (state.latest?.timeout === false || state.latest?.timeout === 0);
  const checked = state.stopLatched !== true
    && (stoppedAndChecked || (fresh && state.armed === true));
  if (!checked) return { connect: 'completed', check: 'active', zero: '', jog: '' };

  const calibrated = state.calibrated === true;
  return {
    connect: 'completed',
    check: 'completed',
    zero: calibrated ? 'completed' : 'active',
    jog: !calibrated ? '' : state.armed === true ? 'completed active' : 'active'
  };
}

export function deriveDashboardPresentation(state) {
  const connectionKnown = state.connectionKnown === true;
  const connected = state.connected === true;
  const owner = state.owner === true;
  const armedKnown = state.armedKnown === true;
  const armed = connectionKnown && connected && owner && armedKnown && state.armed === true;
  const fresh = state.fresh === true;
  const stopLatched = state.stopLatched === true;
  const timeout = state.timeout === true;
  const timeoutAcknowledgementAvailable = state.timeoutAcknowledgementAvailable === true;
  const followSelected = state.followSelected === true;
  const followActive = state.followActive === true;
  const followBlocked = state.followBlocked === true;
  const axes = Array.isArray(state.axes) ? state.axes : [0, 0, 0];
  const moving = state.motionActive === true;
  const source = state.source === 'gamepad' ? 'the selected controller' : 'keyboard or on-screen controls';
  let alert = null;

  if (state.serverUnavailable === true) {
    alert = {
      type: 'error',
      message: 'Dashboard server is not responding. Check the terminal running ./start_arm.sh. Motor feedback cannot be confirmed; waiting to reconnect.'
    };
  } else if (stopLatched) {
    const recovery = !connected
      ? 'Reconnect this session, then Check motor.'
      : !owner
        ? 'Use the controlling tab to Check motor; this tab cannot clear Stop.'
        : 'Check motor to clear Stop, then enable movement again.';
    alert = { type: 'error', message: 'STOP pressed' + (state.stopReason ? ': ' + state.stopReason : '') + '. ' + recovery };
  } else if (connectionKnown && connected && !owner) {
    const condition = timeout ? ' A startup timeout is active.' : !fresh ? ' Feedback is stale.' : '';
    alert = {
      type: timeout || !fresh ? 'warn' : 'info',
      message: 'Another tab controls this motor.' + condition
        + ' Use that tab to check or disconnect; this tab cannot move it.'
    };
  } else if (connectionKnown && connected && !fresh) {
    alert = {
      type: 'error',
      message: 'No recent motor reply. Movement is locked. Check USB, motor power, ID 0, and Channel D, then select Check motor.'
    };
  } else if (connectionKnown && connected && timeout) {
    alert = {
      type: 'warn',
      message: timeoutAcknowledgementAvailable
        ? 'Startup timeout. The motor stays stopped. Confirm it is secure, then select Acknowledge timeout.'
        : 'Startup timeout reported. Movement stays locked until fresh feedback confirms Stop in mode 0.'
    };
  } else if (armed && followSelected) {
    const followState = followActive
      ? 'Preview follow is active for J1 and follows automatically.'
      : followBlocked
        ? 'Preview follow is selected for J1 but paused by its safety gate.'
        : 'Preview follow is selected for J1 and ready to follow the preview automatically.';
    alert = {
      type: 'info',
      message: followState + ' Once active, releasing keys does not pause it. ' + STOP_KEYS + ' to stop automatic following.'
    };
  } else if (armed && moving) {
    alert = Math.abs(axes[0]) > 0.01
      ? { type: 'info', message: 'J1 is jogging from ' + source + '. Release the selected controls to hold position; ' + STOP_KEYS + ' to stop.' }
      : { type: 'info', message: 'Virtual J2/J3 are moving in the preview. Release the selected controls to hold the preview; Stop, Esc, or controller B to stop motor authority.' };
  } else if (armed) {
    alert = { type: 'success', message: 'J1 is ready. Use A/D, the on-screen buttons, or the selected Xbox stick. Release to hold. STOP or Esc stops the motor.' };
  }

  let motion;
  if (state.serverUnavailable === true) {
    motion = { text: 'Feedback unavailable', tone: 'muted' };
  } else if (!armed) {
    motion = { text: followSelected ? 'Disarmed · preview follow selected' : 'Disarmed', tone: 'muted' };
  } else if (followSelected) {
    motion = followActive
      ? { text: 'Following preview on J1', tone: 'emerald' }
      : { text: state.followBlocked ? 'Preview follow paused · J1' : 'Preview follow ready · J1', tone: 'amber' };
  } else if (!moving) {
    motion = { text: 'Holding position', tone: 'cyan' };
  } else if (axes[0] > 0.01) {
    motion = { text: 'J1 Jogging CW ▶', tone: 'emerald' };
  } else if (axes[0] < -0.01) {
    motion = { text: '◀ J1 Jogging CCW', tone: 'emerald' };
  } else {
    motion = { text: 'Jogging Virtual Joints', tone: 'amber' };
  }

  let guidance;
  if (state.serverUnavailable === true) {
    guidance = 'Check the terminal running ./start_arm.sh. Waiting for the dashboard server to respond.';
  } else if (!connected) {
    guidance = stopLatched
      ? 'Reconnect the controlling session, then Check motor.'
      : state.demo === true
        ? 'Select Connect to start simulated J1. No USB adapter is needed.'
      : 'Select the USB adapter, then Connect. If none appears, reconnect it and select Refresh.';
  } else if (!owner) {
    guidance = 'Another tab controls this motor. Use that tab to check or disconnect; this tab cannot move it.';
  } else if (!fresh) {
    guidance = 'No recent motor reply; movement is locked. Check USB, motor power, ID 0, and Channel D, then select Check motor.';
  } else if (stopLatched) {
    guidance = 'Select Check motor to clear Stop, then confirm your setup and enable movement.';
  } else if (timeout) {
    guidance = timeoutAcknowledgementAvailable
      ? 'The motor stays stopped. Confirm it is secure, then select Acknowledge timeout.'
      : 'The motor stays stopped. Select Check motor; acknowledgement appears only after fresh Stop feedback in mode 0.';
  } else if (!state.calibrated) {
    guidance = 'Point J1 at the pose you want to call 0°, then select Set J1 zero here. If you know its angle, enter it and select Use entered angle. Neither action moves the motor.';
  } else if (armed && followSelected) {
    guidance = followActive
      ? 'Preview follow is active: it automatically follows the preview on J1 using fresh measured feedback. Releasing keys does not pause it; press Stop, Esc, or controller B to stop. J2/J3 remain virtual.'
      : followBlocked
        ? 'Preview follow is paused by its safety gate. Confirm fresh J1 feedback, applied travel limits, and neutral input. Once active, releasing keys does not pause it; press Stop, Esc, or controller B to stop. J2/J3 remain virtual.'
        : 'Preview follow is selected and ready to follow the preview on J1 using fresh measured feedback. Releasing keys does not pause it; press Stop, Esc, or controller B to stop. J2/J3 remain virtual.';
  } else if (armed) {
    guidance = 'Move J1 with A/D, the on-screen buttons, or the Xbox stick. Release to hold. Press STOP, Esc, or B to stop the motor. J2/J3 are preview only.';
  } else if (followSelected) {
    guidance = 'Preview follow is selected. After Enable and the fresh-neutral gate, it automatically follows the preview on J1. Releasing keys does not pause it; press Stop, Esc, or controller B to stop. J2/J3 remain virtual.';
  } else {
    guidance = 'Check the space around J1, confirm the setup is secure, then enable movement. Start with a short tap.';
  }

  motion.color = MOTION_COLORS[motion.tone];
  return { alert, motion, guidance };
}
