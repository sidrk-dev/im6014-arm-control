import assert from 'node:assert/strict';
import {
  canAcknowledgeStartupTimeout,
  deriveDashboardPresentation,
  deriveWorkflowSteps,
  finiteNumber,
  jointGaugePresentation,
  jointPositionPresentation,
  jointVelocityText,
  checkTravelSpan,
  formatJointHealth
} from '../arm_web/presentation.js';

function baseState(overrides = {}) {
  return {
    profileId: 'arm_j1_j2_j3',
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

function testThreeJointPresentationAlertAndGuidance() {
  const armedState = deriveDashboardPresentation(baseState({ armed: true }));
  assert.equal(armedState.alert.type, 'success');
  assert.match(armedState.alert.message, /Use A\/D \(J1\), W\/S \(J2\), Q\/E \(J3\)/i);
  assert.match(armedState.guidance, /A\/D \(J1\), W\/S \(J2\), Q\/E \(J3\)/i);
  assert.match(armedState.guidance, /stop all motors/i);

  const movingState = deriveDashboardPresentation(baseState({
    armed: true,
    motionActive: true,
    axes: [0, 0.5, 0]
  }));
  assert.match(movingState.motion.text, /Jogging 3-joint arm/i);
}

function testThreeJointPresetsPathFollowDisabled() {
  const threeJointWithFollow = deriveDashboardPresentation(baseState({
    profileId: 'arm_j1_j2_j3',
    armed: true,
    followSelected: true,
    followActive: false
  }));
  assert.doesNotMatch(threeJointWithFollow.alert?.message || '', /Preview follow is active/i);
  assert.doesNotMatch(threeJointWithFollow.guidance, /automatically follows the preview/i);
}

function testTravelSpanWarning() {
  assert.equal(checkTravelSpan(-90, 90), null);
  assert.equal(checkTravelSpan(0, 360), null);
  const warning = checkTravelSpan(-200, 200);
  assert.notEqual(warning, null);
  assert.match(warning, /exceeds one full turn \(360°\)/i);
  assert.match(warning, /Save finite values as draft/i);

  assert.equal(checkTravelSpan(100, -100), null);
  assert.equal(checkTravelSpan(NaN, 100), null);
  assert.equal(checkTravelSpan(0, Infinity), null);
}

function testJointHealthFormattingAndMissingIdActionableGuidance() {
  const missingJoint = formatJointHealth({ id: 1, name: 'J2', health: 'missing' });
  assert.equal(missingJoint.status, 'missing');
  assert.match(missingJoint.text, /Motor ID 1 missing/i);
  assert.match(missingJoint.text, /Set IDs 0\/1\/2 one at a time using vendor tool before connecting/i);
  assert.match(missingJoint.text, /check 485 wiring & DC power/i);

  const staleJoint = formatJointHealth({ id: 2, name: 'J3', health: 'stale', feedback_age_s: 0.25 });
  assert.equal(staleJoint.status, 'stale');
  assert.match(staleJoint.text, /STALE · cached feedback/i);
  assert.match(staleJoint.text, /0\.250s age/i);

  const faultedJoint = formatJointHealth({ id: 0, name: 'J1', health: 'fault', errors: 2, warnings: 0 });
  assert.equal(faultedJoint.status, 'fault');
  assert.match(faultedJoint.text, /Motor fault/i);

  const healthyJoint = formatJointHealth({ id: 0, name: 'J1', health: 'healthy', latest: {} });
  assert.equal(healthyJoint.status, 'healthy');
  assert.match(healthyJoint.text, /Fresh measured feedback/i);
}

function testPartialCalibrationMeasuredAngleVisibility() {
  const j1Calibrated = jointPositionPresentation({ calibrated: true, position_rad: Math.PI / 18 });
  assert.notEqual(j1Calibrated.radians, null);
  assert.match(j1Calibrated.text, /0\.175 rad/i);
  assert.match(j1Calibrated.text, /10\.00°/i);

  const j2Uncalibrated = jointPositionPresentation({ calibrated: false, position_rad: null });
  assert.equal(j2Uncalibrated.radians, null);
  assert.equal(j2Uncalibrated.text, 'Awaiting known angle');
}

function testJointGaugeWithCustomJointNames() {
  const j2Gauge = jointGaugePresentation({
    positionRad: 0.5,
    lowerDeg: -180,
    upperDeg: 180,
    connected: true,
    fresh: true,
    jointName: 'J2'
  });
  assert.equal(j2Gauge.visible, true);
  assert.equal(j2Gauge.lowerLabel, '-180°');
  assert.equal(j2Gauge.upperLabel, '+180°');
  assert.equal(j2Gauge.zeroLabel, '0° (Zero)');
  assert.equal(j2Gauge.zeroPercent, 50);

  const j3GaugeDisconnected = jointGaugePresentation({
    positionRad: 0.5,
    lowerDeg: -90,
    upperDeg: 90,
    connected: false,
    fresh: false,
    jointName: 'J3'
  });
  assert.equal(j3GaugeDisconnected.visible, false);
}

function testWorkflowStepsWithThreeJoints() {
  const allCalibrated = {
    connected: true,
    fresh: true,
    stopConfirmed: true,
    motorState: 'Ready · Stop',
    latest: { mode: 0, timeout: false },
    calibrated: true,
    armed: false,
    stopLatched: false
  };
  const steps = deriveWorkflowSteps(allCalibrated);
  assert.equal(steps.connect, 'completed');
  assert.equal(steps.check, 'completed');
  assert.equal(steps.zero, 'completed');
  assert.equal(steps.jog, 'active');

  const partialCalibrated = { ...allCalibrated, calibrated: false };
  const stepsPartial = deriveWorkflowSteps(partialCalibrated);
  assert.equal(stepsPartial.connect, 'completed');
  assert.equal(stepsPartial.check, 'completed');
  assert.equal(stepsPartial.zero, 'active');
  assert.equal(stepsPartial.jog, '');
}

testThreeJointPresentationAlertAndGuidance();
testThreeJointPresetsPathFollowDisabled();
testTravelSpanWarning();
testJointHealthFormattingAndMissingIdActionableGuidance();
testPartialCalibrationMeasuredAngleVisibility();
testJointGaugeWithCustomJointNames();
testWorkflowStepsWithThreeJoints();
console.log('Three-joint UI behavior tests passed.');

const timeoutState = {
  connected: true, owner: true, fresh: true, stopConfirmed: true,
  armed: false, stopLatched: false, latest: {mode: 0, timeout: false},
  joints: [0, 1, 2].map(id => ({configured: true, stop_confirmed: true,
    latest: {mode: 0, timeout: id === 1}}))
};
assert.equal(canAcknowledgeStartupTimeout(timeoutState), true);
assert.equal(canAcknowledgeStartupTimeout({...timeoutState, fresh: false}), false);
timeoutState.joints[2].latest.mode = 1;
assert.equal(canAcknowledgeStartupTimeout(timeoutState), false);
timeoutState.joints[2].latest.mode = 0;
timeoutState.joints[2].stop_confirmed = false;
assert.equal(canAcknowledgeStartupTimeout(timeoutState), false);
console.log('Per-joint timeout acknowledgment gates passed.');
