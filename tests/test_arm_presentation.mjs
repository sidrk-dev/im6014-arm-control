import assert from 'node:assert/strict';
import { deriveDashboardPresentation, finiteNumber, j1GaugePresentation, jointPositionPresentation, jointVelocityText, serialPortStatus } from '../arm_web/presentation.js';

function state(overrides = {}) {
  return {
    connectionKnown: true,
    connected: true,
    owner: true,
    armedKnown: true,
    armed: false,
    fresh: true,
    calibrated: true,
    timeout: false,
    stopLatched: false,
    motionActive: false,
    axes: [0, 0, 0],
    source: 'keyboard',
    followSelected: false,
    followActive: false,
    followBlocked: false,
    ...overrides
  };
}

function testPreviewFollowIsVisibleAndClearAboutStopping() {
  const presentation = deriveDashboardPresentation(state({
    armed: true,
    motionActive: true,
    source: 'gamepad',
    followSelected: true,
    followActive: true
  }));
  assert.equal(presentation.motion.text, 'Following preview on J1');
  assert.match(presentation.alert.message, /Preview follow is active/i);
  assert.match(presentation.alert.message, /Releasing keys does not pause it/i);
  assert.match(presentation.alert.message, /Stop, Esc, or controller B/i);
  assert.match(presentation.guidance, /automatically follows the preview on J1/i);
  assert.match(presentation.guidance, /Releasing keys does not pause it/i);
  assert.match(presentation.guidance, /Stop, Esc, or controller B/i);
}

function testPreviewFollowPauseAndDisarmAreDistinct() {
  const paused = deriveDashboardPresentation(state({
    armed: true,
    followSelected: true,
    followBlocked: true
  }));
  assert.equal(paused.motion.text, 'Preview follow paused · J1');
  assert.match(paused.alert.message, /Releasing keys does not pause it/i);

  const armed = deriveDashboardPresentation(state({ armed: true }));
  assert.equal(armed.alert.type, 'success');
  const disarmed = deriveDashboardPresentation(state());
  const disconnected = deriveDashboardPresentation(state({ connected: false, armed: true }));
  assert.equal(disarmed.alert, null, 'disarm clears the armed success banner');
  assert.equal(disconnected.alert, null, 'disconnect clears the armed success banner');
}

function testObserverHealthCheckGuidanceNamesOwnerTab() {
  const presentation = deriveDashboardPresentation(state({ owner: false }));
  assert.match(presentation.alert.message, /Observer: another tab owns this connection/i);
  assert.match(presentation.alert.message, /Use the owner tab to run Health Check/i);
  assert.doesNotMatch(presentation.alert.message, /click.*Health Check/i);
  assert.match(presentation.guidance, /Use the owner tab/i);
}

function testTelemetryPresentationFormatsFiniteValues() {
  const position = jointPositionPresentation({ position_rad: 0 });
  assert.equal(position.radians, 0);
  assert.equal(position.text, '0.000 rad (0.00°)');
  assert.equal(finiteNumber(Infinity), null);
  assert.equal(jointPositionPresentation({ position_rad: null }).text, 'Awaiting known angle');
  assert.equal(jointVelocityText({ velocity_rad_s: 1 }, true), '1.000 rad/s (9.55 rpm)');
}

function testAsymmetricGaugeUsesAppliedZeroAnchorAndMarksCachedFeedback() {
  const gauge = j1GaugePresentation({
    positionRad: Math.PI / 4,
    lowerDeg: 0,
    upperDeg: 90,
    connected: true,
    fresh: false
  });
  assert.equal(gauge.visible, true);
  assert.equal(gauge.lowerLabel, '0°');
  assert.equal(gauge.upperLabel, '+90°');
  assert.equal(gauge.zeroLabel, '0° (Zero)');
  assert.equal(gauge.zeroPercent, 0);
  assert.equal(gauge.fillLeftPercent, 0);
  assert.equal(gauge.fillWidthPercent, 50);
  assert.match(gauge.status, /cached/i);

  const positiveBounds = j1GaugePresentation({ positionRad: Math.PI / 4, lowerDeg: 10, upperDeg: 90, connected: true, fresh: true });
  assert.equal(positiveBounds.zeroLabel, 'Zero anchor · lower limit');
  assert.equal(positiveBounds.zeroPercent, 0);

  const unavailable = j1GaugePresentation({
    positionRad: 0,
    lowerDeg: 0,
    upperDeg: 90,
    connected: false,
    fresh: false
  });
  assert.equal(unavailable.visible, false, 'disconnected feedback hides the gauge');
  assert.equal(unavailable.fillWidthPercent, 0);
}

function testLegacyPortsSayChannelDMissingWithoutHidingEnumeration() {
  const ports = Array.from({ length: 32 }, (_, index) => ({
    path: '/dev/ttyS' + index,
    channel_d: false
  }));
  const status = serialPortStatus(ports);
  assert.match(status, /32 serial ports detected/i);
  assert.match(status, /Recommended Channel D is missing/i);
  assert.match(status, /Replug.*Refresh/i);
  assert.match(status, /passive/i);
  assert.match(status, /does not open or probe/i);

  const found = serialPortStatus([{ path: '/dev/serial/by-id/recommended', channel_d: true }]);
  assert.match(found, /Recommended Channel D is listed/i);
  assert.match(serialPortStatus([]), /No serial ports detected/i);
}

testPreviewFollowIsVisibleAndClearAboutStopping();
testPreviewFollowPauseAndDisarmAreDistinct();
testObserverHealthCheckGuidanceNamesOwnerTab();
testLegacyPortsSayChannelDMissingWithoutHidingEnumeration();
testAsymmetricGaugeUsesAppliedZeroAnchorAndMarksCachedFeedback();
testTelemetryPresentationFormatsFiniteValues();
console.log('Dashboard presentation behavior tests passed.');
