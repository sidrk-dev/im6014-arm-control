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

export function serialPortStatus(ports, error = null) {
  const records = Array.isArray(ports) ? ports.filter(Boolean) : [];
  const count = records.length;
  const countText = count === 1 ? '1 serial port detected.' : count ? count + ' serial ports detected.' : 'No serial ports detected.';
  const channelDListed = records.some(port => port.channel_d === true);
  const channelDText = count
    ? channelDListed ? 'Recommended Channel D is listed.' : 'Recommended Channel D is missing.'
    : '';
  if (error) {
    return error + '. ' + countText + (channelDText ? ' ' + channelDText : '')
      + ' Replug the USB adapter, then Refresh. Port listing is passive and does not open or probe devices.';
  }
  if (!count) {
    return 'No serial ports detected. Replug the USB adapter, then Refresh. Port listing is passive and does not open or probe devices.';
  }
  return countText + ' ' + channelDText
    + (channelDListed ? ' No device was opened.' : ' Replug the USB adapter, then Refresh.')
    + ' Enumeration is passive and does not open or probe devices.';
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
  const followSelected = state.followSelected === true;
  const followActive = state.followActive === true;
  const followBlocked = state.followBlocked === true;
  const axes = Array.isArray(state.axes) ? state.axes : [0, 0, 0];
  const moving = state.motionActive === true;
  const source = state.source === 'gamepad' ? 'the selected controller' : 'keyboard or on-screen controls';
  let alert = null;

  if (stopLatched) {
    const recovery = !owner
      ? 'Use the owner tab to run Health Check; this tab cannot clear the Stop latch.'
      : !connected
        ? 'Reconnect this owner session before running Health Check.'
        : 'Run Health Check here to clear the Stop latch, then Enable jogging again.';
    alert = { type: 'error', message: 'Stop latched' + (state.stopReason ? ': ' + state.stopReason : '') + '. ' + recovery };
  } else if (connectionKnown && connected && !owner) {
    const condition = timeout ? ' A startup timeout is active.' : !fresh ? ' Feedback is stale.' : '';
    alert = {
      type: timeout || !fresh ? 'warn' : 'info',
      message: 'Observer: another tab owns this connection.' + condition
        + ' Use the owner tab to run Health Check or disconnect; this tab cannot operate it.'
    };
  } else if (connectionKnown && connected && !fresh) {
    alert = {
      type: 'error',
      message: 'STALE FEEDBACK — values below are cached. Reconnect the USB adapter, Refresh and select Channel D, verify motor power and ID 0, then run Health Check.'
    };
  } else if (connectionKnown && connected && timeout) {
    alert = {
      type: 'warn',
      message: 'Startup watchdog timeout. Confirm the safety checkbox, then click Acknowledge Timeout. The motor stays in Stop.'
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
    alert = { type: 'success', message: 'Motor armed and ready. Press A/D, click −/+, or use the selected controller to jog J1. Esc stops instantly.' };
  }

  let motion;
  if (!armed) {
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
  if (!connected) {
    guidance = stopLatched
      ? 'Reconnect this owner session before running Health Check.'
      : '1. Select a detected serial port or enter a custom path, then Connect. Port listing is passive; adapter identity may remain unverified.';
  } else if (!owner) {
    guidance = 'Observer mode: another tab owns this connection. Use the owner tab for Health Check or disconnect; this tab cannot take control.';
  } else if (!fresh) {
    guidance = 'Feedback is stale; telemetry is cached and motion controls are locked. Reconnect the USB adapter, Refresh and select Channel D, verify power and ID 0, then run Health Check.';
  } else if (stopLatched) {
    guidance = 'Run Health Check to clear Stop, then confirm your setup and Enable jogging.';
  } else if (timeout) {
    guidance = 'Confirm the secured-motor checkbox, then Acknowledge Timeout. The motor stays in Stop.';
  } else if (!state.calibrated) {
    guidance = '2. Set the current angle (0 means this pose is your zero), then Use measured angle for J1.';
  } else if (armed && followSelected) {
    guidance = followActive
      ? 'Preview follow is active: it automatically follows the preview on J1 using fresh measured feedback. Releasing keys does not pause it; press Stop, Esc, or controller B to stop. J2/J3 remain virtual.'
      : followBlocked
        ? 'Preview follow is paused by its safety gate. Confirm fresh J1 feedback, applied travel limits, and neutral input. Once active, releasing keys does not pause it; press Stop, Esc, or controller B to stop. J2/J3 remain virtual.'
        : 'Preview follow is selected and ready to follow the preview on J1 using fresh measured feedback. Releasing keys does not pause it; press Stop, Esc, or controller B to stop. J2/J3 remain virtual.';
  } else if (armed) {
    guidance = 'Press A/D, click −/+, or use the selected controller stick to jog J1. Release or center controls to hold position; Stop removes torque. J2/J3 are virtual.';
  } else if (followSelected) {
    guidance = 'Preview follow is selected. After Enable and the fresh-neutral gate, it automatically follows the preview on J1. Releasing keys does not pause it; press Stop, Esc, or controller B to stop. J2/J3 remain virtual.';
  } else {
    guidance = '3. Confirm your setup, then Enable jogging. Start with short movements.';
  }

  motion.color = MOTION_COLORS[motion.tone];
  return { alert, motion, guidance };
}
