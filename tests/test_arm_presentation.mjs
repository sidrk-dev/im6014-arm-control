import assert from 'node:assert/strict';
import { canAcknowledgeStartupTimeout, deriveDashboardPresentation, deriveWorkflowSteps, finiteNumber, j1GaugePresentation, jointPositionPresentation, jointVelocityText, serialPortStatus } from '../arm_web/presentation.js';

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
  assert.match(presentation.alert.message, /Another tab controls this motor/i);
  assert.match(presentation.alert.message, /Use that tab to check or disconnect/i);
  assert.doesNotMatch(presentation.alert.message, /click.*Health Check/i);
  assert.match(presentation.guidance, /Use that tab/i);
}

function testServerFailureNamesTheRecoveryAction() {
  const failed = deriveDashboardPresentation(state({ serverUnavailable: true, connected: false }));
  assert.equal(failed.alert.type, 'error');
  assert.match(failed.alert.message, /server is not responding/i);
  assert.match(failed.guidance, /terminal running.*start_arm\.sh/i);
  const restored = deriveDashboardPresentation(state({ serverUnavailable: false, connected: false }));
  assert.equal(restored.alert, null);
  assert.match(restored.guidance, /USB adapter/i);
}

function testDisconnectedStopAndDemoGivePossibleNextActions() {
  const stopped = deriveDashboardPresentation(state({ connected: false, owner: false, stopLatched: true }));
  assert.match(stopped.alert.message, /Reconnect this session/i);
  assert.doesNotMatch(stopped.alert.message, /controlling tab/i);
  const demo = deriveDashboardPresentation(state({ connected: false, demo: true }));
  assert.match(demo.guidance, /Select Connect.*simulated J1/i);
  assert.match(serialPortStatus([], null, true), /no USB adapter is needed/i);
}

function testOperatorGuidanceUsesShortActionableSteps() {
  const disconnected = deriveDashboardPresentation(state({ connected: false }));
  assert.match(disconnected.guidance, /Select the USB adapter.*Connect/i);
  assert.match(disconnected.guidance, /Refresh/i);

  const calibration = deriveDashboardPresentation(state({ calibrated: false }));
  assert.match(calibration.guidance, /Point J1.*0°/i);
  assert.match(calibration.guidance, /Use entered angle/i);
  assert.match(calibration.guidance, /Neither action moves the motor/i);

  const timeout = deriveDashboardPresentation(state({ timeout: true }));
  assert.match(timeout.guidance, /stays stopped/i);
  assert.match(timeout.guidance, /Check motor/i);
  assert.match(timeout.guidance, /fresh Stop feedback in mode 0/i);

  const timeoutReady = deriveDashboardPresentation(state({
    timeout: true,
    timeoutAcknowledgementAvailable: true
  }));
  assert.match(timeoutReady.guidance, /Acknowledge timeout/i);

  const ready = deriveDashboardPresentation(state({ armed: true }));
  assert.match(ready.alert.message, /Use A\/D/i);
  assert.match(ready.guidance, /Release to hold/i);
  assert.match(ready.guidance, /J2\/J3 are preview only/i);
}

function testWorkflowProgressTracksFreshCheckThroughJogging() {
  const readyStop = {
    connected: true,
    fresh: true,
    stopConfirmed: true,
    motorState: 'Ready · Stop',
    latest: { mode: 0, timeout: false },
    calibrated: false,
    armed: false,
    stopLatched: false
  };
  assert.deepEqual(deriveWorkflowSteps({ connected: false }), {
    connect: 'active', check: '', zero: '', jog: ''
  });
  assert.deepEqual(deriveWorkflowSteps({ ...readyStop, fresh: false }), {
    connect: 'completed', check: 'active', zero: '', jog: ''
  });
  assert.deepEqual(deriveWorkflowSteps({ ...readyStop, latest: { mode: 0, timeout: true } }), {
    connect: 'completed', check: 'active', zero: '', jog: ''
  });
  assert.deepEqual(deriveWorkflowSteps(readyStop), {
    connect: 'completed', check: 'completed', zero: 'active', jog: ''
  });
  assert.deepEqual(deriveWorkflowSteps({ ...readyStop, calibrated: true }), {
    connect: 'completed', check: 'completed', zero: 'completed', jog: 'active'
  });
  assert.deepEqual(deriveWorkflowSteps({
    ...readyStop,
    stopConfirmed: false,
    motorState: 'Running',
    latest: { mode: 1, timeout: false },
    calibrated: true,
    armed: true
  }), {
    connect: 'completed', check: 'completed', zero: 'completed', jog: 'completed active'
  });
  assert.deepEqual(deriveWorkflowSteps({ ...readyStop, stopLatched: true }), {
    connect: 'completed', check: 'active', zero: '', jog: ''
  });
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

function testMissingAdapterGuidanceExplainsRefreshAndCustomPath() {
  const status = serialPortStatus([]);
  assert.match(status, /No USB adapter found/i);
  assert.match(status, /USB-RS485.*Refresh/i);
  assert.doesNotMatch(status, /Using a different adapter/i);
  assert.doesNotMatch(status, /Listing alone/i);

  const found = serialPortStatus([{ path: '/dev/serial/by-id/recommended', channel_d: true }]);
  assert.match(found, /Channel D.*marked Recommended/i);
  const otherAdapter = serialPortStatus([{ path: '/dev/ttyUSB0', channel_d: false }]);
  assert.match(otherAdapter, /Channel D was not identified/i);
  assert.match(otherAdapter, /only the adapter wired to J1/i);
}

function testTimeoutAcknowledgementRequiresFreshConfirmedStoppedOwner() {
  const ready = {
    connected: true,
    owner: true,
    fresh: true,
    stopConfirmed: true,
    latest: { mode: 0, timeout: true },
    armed: false
  };
  assert.equal(canAcknowledgeStartupTimeout(ready), true);
  assert.equal(canAcknowledgeStartupTimeout({ ...ready, stopConfirmed: false }), false);
  assert.equal(canAcknowledgeStartupTimeout({ ...ready, fresh: false }), false);
  assert.equal(canAcknowledgeStartupTimeout({ ...ready, latest: { mode: 1, timeout: true } }), false);
  assert.equal(canAcknowledgeStartupTimeout({ ...ready, armed: true }), false);
  assert.equal(canAcknowledgeStartupTimeout({ ...ready, owner: false }), false);
}

testPreviewFollowIsVisibleAndClearAboutStopping();
testPreviewFollowPauseAndDisarmAreDistinct();
testObserverHealthCheckGuidanceNamesOwnerTab();
testServerFailureNamesTheRecoveryAction();
testDisconnectedStopAndDemoGivePossibleNextActions();
testOperatorGuidanceUsesShortActionableSteps();
testWorkflowProgressTracksFreshCheckThroughJogging();
testMissingAdapterGuidanceExplainsRefreshAndCustomPath();
testTimeoutAcknowledgementRequiresFreshConfirmedStoppedOwner();
testAsymmetricGaugeUsesAppliedZeroAnchorAndMarksCachedFeedback();
testTelemetryPresentationFormatsFiniteValues();
console.log('Dashboard presentation behavior tests passed.');
