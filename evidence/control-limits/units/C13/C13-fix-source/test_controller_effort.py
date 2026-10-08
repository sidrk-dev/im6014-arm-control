"""Acceptance and unit tests for C13 combined effort prediction and measured monitoring."""

from fractions import Fraction
import math
import unittest

from im6014_codec import decode_command
from joint_contract import (
    DecisionCode,
    JointSpec,
    MotorHealth,
    State,
    Target,
    WriteResult,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget
from joint_reference import (
    map_decoded_command,
    predict_joint_effort,
    prepare_reference_joint,
)
from test_controller import DeterministicClock, FakeStation, HEALTHY_STOP, SOURCE


_ROUNDED_CANONICAL_EFFORT = 0.33977850723047515


def _exact_effort(target, sample):
    return (
        Fraction(target.tau_nm)
        + Fraction(target.kp)
        * (Fraction(target.q_rad) - Fraction(sample.q_rad))
        + Fraction(target.kd)
        * (Fraction(target.dq_rad_s) - Fraction(sample.dq_rad_s))
    )


class EffortLimitTests(unittest.TestCase):
    def setUp(self):
        self.clock = DeterministicClock(10.0)
        self.spec1 = JointSpec(
            "J1",
            0,
            JointMap(1.0, 1, 0.0),
            -2.0,
            2.0,
            5.0,     # v_max_rad_s
            50.0,    # a_max_rad_s2
            1.0,     # max_lead_rad
            10.0,    # max_effort_nm
            "fixture: effort test J1",
        )
        self.spec2 = JointSpec(
            "J2",
            1,
            JointMap(1.0, 1, 0.0),
            -2.0,
            2.0,
            5.0,
            50.0,
            1.0,
            15.0,    # max_effort_nm
            "fixture: effort test J2",
        )
        self.specs = (self.spec1, self.spec2)
        self.sample1 = JointSample(0.0, 0.0, 0.0)
        self.sample2 = JointSample(0.0, 0.0, 0.0)

    def _armed_station(self, max_reference_gap_s=1.0):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=max_reference_gap_s)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("effort-tester")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, owner

    def _single_joint_spec(self, max_effort_nm):
        return JointSpec(
            "J",
            0,
            JointMap(1.0, 1, 0.0),
            -2.0,
            2.0,
            5.0,
            50.0,
            1.0,
            max_effort_nm,
            "fixture: exact effort boundary",
        )

    def _armed_single_joint(self, spec, sample):
        core = JointCore((spec,), clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("effort-tester")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, sample, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, owner

    def _submit_single_joint(self, station, owner, sequence, goal):
        target = Target(
            sequence,
            (goal,),
            window=station.issue_window(1.0),
            owner=owner,
        )
        return target, station.submit(target)

    def _admitted_rounding_goal(self):
        spec = self._single_joint_spec(_ROUNDED_CANONICAL_EFFORT)
        core, station, owner = self._armed_single_joint(
            spec, JointSample(0.0, 0.0, 0.0)
        )
        goal = JointTarget(0.0, 0.2, 0.1, 1.0, 0.1)
        _, decision = self._submit_single_joint(station, owner, 1, goal)
        self.assertTrue(decision.accepted)
        return core, station, owner, goal

    def _rounding_feedback(self, core):
        sample = JointSample(-0.1, -0.2, 0.0)
        return core.receive(
            0,
            sample,
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False),
        )

    def test_effort_limit_requires_finite_positive_real_value(self):
        mapping = JointMap(1.0, 1, 0.0)
        for val in (True, False, 0, 0.0, -1.0, math.nan, math.inf, -math.inf, 10**400, "10.0"):
            with self.subTest(val=val), self.assertRaises(ValueError):
                JointSpec(
                    "J1", 0, mapping, -2.0, 2.0, 5.0, 50.0, 1.0, val,
                    "fixture: invalid effort limit input",
                )

        spec = JointSpec("J1", 0, mapping, -2.0, 2.0, 5.0, 50.0, 1.0, 12.5, "fixture: effort limit")
        self.assertEqual(spec.max_effort_nm, 12.5)

    def test_combined_sum_trips_when_each_term_individually_under_peak(self):
        # max_effort_nm is 10.0
        # tau_pred = tau_ff + kp*(q_ref - q_meas) + kd*(dq_ref - dq_meas)
        # Choose terms: tau_ff = 4.0, kp*(q - q_meas) = 4.0, kd*(dq - dq_meas) = 4.0
        # Each term alone is 4.0 <= 10.0, but total is 12.0 > 10.0
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        # q_meas=0.0, dq_meas=0.0
        target = Target(
            1,
            (
                JointTarget(q_rad=0.4, dq_rad_s=4.0, tau_nm=4.0, kp=10.0, kd=1.0),
                JointTarget(q_rad=0.0, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
            ),
            window=window,
            owner=owner,
        )
        decision = station.submit(target)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)

        # Ensure no term clipping occurred: individual terms are under peak
        self.assertLessEqual(abs(4.0), self.spec1.max_effort_nm)
        self.assertLessEqual(abs(10.0 * 0.4), self.spec1.max_effort_nm)
        self.assertLessEqual(abs(1.0 * 4.0), self.spec1.max_effort_nm)

    def test_opposing_and_cancelling_terms(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        # Opposing terms: tau_ff = 8.0, kp*(q - q_meas) = -6.0 -> net is ~2.0 <= 10.0
        # q_meas = 0.0, q = -0.6, kp = 10.0 -> kp*(q - q_meas) = -6.0
        cancelling_target = Target(
            1,
            (
                JointTarget(q_rad=-0.6, dq_rad_s=0.0, tau_nm=8.0, kp=10.0, kd=0.0),
                JointTarget(q_rad=0.0, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
            ),
            window=window,
            owner=owner,
        )
        decision = station.submit(cancelling_target)
        self.assertTrue(decision.accepted)

        # Opposing terms that do NOT cancel sufficiently:
        # tau_ff = -8.0, kp*(q - q_meas) = -5.0 -> net is -13.0, |-13.0| > 10.0
        tripping_target = Target(
            2,
            (
                JointTarget(q_rad=-0.5, dq_rad_s=0.0, tau_nm=-8.0, kp=10.0, kd=0.0),
                JointTarget(q_rad=0.0, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
            ),
            window=window,
            owner=owner,
        )
        decision2 = station.submit(tripping_target)
        self.assertFalse(decision2.accepted)
        self.assertIs(decision2.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)

    def test_negative_mapping_and_units(self):
        # reduction=3.0, direction=-1, zero_output_rad=0.25
        mapping = JointMap(reduction=3.0, direction=-1, zero_output_rad=0.25)
        spec = JointSpec(
            "J_neg", 0, mapping, -2.0, 2.0, 5.0, 50.0, 1.0, 10.0, "fixture: neg mapping effort"
        )
        jt = JointTarget(q_rad=0.1, dq_rad_s=0.2, tau_nm=3.0, kp=9.0, kd=0.9)

        # encode and map back
        code, actual, packet = prepare_reference_joint(spec, jt)
        self.assertIsNone(code)
        self.assertIsNotNone(actual)
        self.assertIsNotNone(packet)

        sample = JointSample(actual.q_rad, actual.dq_rad_s, 0.0)
        pred_effort = predict_joint_effort(actual, sample)
        self.assertEqual(pred_effort, actual.tau_nm)
        self.assertLessEqual(abs(pred_effort), spec.max_effort_nm)

    def test_encoded_gains_boundary_uses_quantized_wire_values(self):
        # Test that predict_joint_effort uses actual quantized wire gains,
        # not the raw float gains before encoding.
        mapping = JointMap(1.0, 1, 0.0)
        spec = JointSpec("J", 0, mapping, -2.0, 2.0, 5.0, 50.0, 1.0, 20.0, "fixture: quant test")

        kp_continuous = 1.234567
        jt = JointTarget(q_rad=0.1, dq_rad_s=0.0, tau_nm=0.0, kp=kp_continuous, kd=0.0)
        sample = JointSample(q_rad=0.0, dq_rad_s=0.0, tau_nm=0.0)

        code, actual, packet = prepare_reference_joint(spec, jt)
        self.assertIsNone(code)
        decoded = decode_command(packet)
        decoded_target = map_decoded_command(mapping, decoded)
        self.assertEqual(actual, decoded_target)
        self.assertNotEqual(actual.kp, jt.kp)
        predicted = predict_joint_effort(actual, sample)
        expected = actual.kp * actual.q_rad
        self.assertEqual(predicted, expected)

    def test_target_admission_enforces_peak_prediction(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        # Submit target exceeding spec1 limit (10.0 Nm):
        # tau_ff = 12.0 Nm
        target = Target(
            1,
            (
                JointTarget(q_rad=0.0, dq_rad_s=0.0, tau_nm=12.0, kp=0.0, kd=0.0),
                JointTarget(q_rad=0.0, dq_rad_s=0.0, tau_nm=0.0, kp=0.0, kd=0.0),
            ),
            window=window,
            owner=owner,
        )
        decision = station.submit(target)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)
        self.assertIsNone(core.snapshot().admitted_target)

    def test_canonical_exact_effort_cannot_round_down_at_admission(self):
        goal = JointTarget(0.1, 0.2, 0.1, 1.0, 0.1)
        sample = JointSample(-0.1, -0.2, 0.0)
        wide_spec = self._single_joint_spec(100.0)
        code, actual, packet = prepare_reference_joint(wide_spec, goal)
        self.assertIsNone(code)
        self.assertIsNotNone(actual)
        self.assertEqual(packet.hex(), "feee900014001a00ce1900005000200024fb021b")

        exact_effort = _exact_effort(actual, sample)
        self.assertEqual(predict_joint_effort(actual, sample), _ROUNDED_CANONICAL_EFFORT)
        self.assertGreater(exact_effort, Fraction(_ROUNDED_CANONICAL_EFFORT))

        spec = self._single_joint_spec(_ROUNDED_CANONICAL_EFFORT)
        core, station, owner = self._armed_single_joint(spec, sample)
        before = core.snapshot()
        _, decision = self._submit_single_joint(station, owner, 1, goal)

        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)
        self.assertEqual(core.snapshot().last_sequence, before.last_sequence)
        self.assertEqual(core.snapshot().admitted_target, before.admitted_target)

    def test_canonical_exact_effort_cannot_round_toward_zero_at_negative_bound(self):
        goal = JointTarget(-0.1, -0.2, -0.1, 1.0, 0.1)
        sample = JointSample(0.1, 0.2, 0.0)
        wide_spec = self._single_joint_spec(100.0)
        code, actual, _ = prepare_reference_joint(wide_spec, goal)
        self.assertIsNone(code)
        self.assertIsNotNone(actual)

        exact_effort = _exact_effort(actual, sample)
        self.assertEqual(predict_joint_effort(actual, sample), -_ROUNDED_CANONICAL_EFFORT)
        self.assertLess(exact_effort, -Fraction(_ROUNDED_CANONICAL_EFFORT))

        spec = self._single_joint_spec(_ROUNDED_CANONICAL_EFFORT)
        core, station, owner = self._armed_single_joint(spec, sample)
        _, decision = self._submit_single_joint(station, owner, 1, goal)

        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)
        self.assertIsNone(core.snapshot().admitted_target)

    def test_canonical_effort_equality_is_inclusive_for_both_signs(self):
        wide_spec = self._single_joint_spec(100.0)
        for torque in (1.0, -1.0):
            with self.subTest(torque=torque):
                goal = JointTarget(0.0, 0.0, torque, 0.0, 0.0)
                code, actual, _ = prepare_reference_joint(wide_spec, goal)
                self.assertIsNone(code)
                self.assertIsNotNone(actual)
                spec = self._single_joint_spec(abs(actual.tau_nm))
                sample = JointSample(actual.q_rad, actual.dq_rad_s, 0.0)
                self.assertEqual(abs(_exact_effort(actual, sample)), Fraction(spec.max_effort_nm))

                _, station, owner = self._armed_single_joint(spec, sample)
                _, decision = self._submit_single_joint(station, owner, 1, goal)
                self.assertTrue(decision.accepted)

    def test_positive_cadence_preparation_uses_exact_effort_boundary(self):
        core, _, _, _ = self._admitted_rounding_goal()
        self.clock.value += 0.5
        self.assertTrue(self._rounding_feedback(core).accepted)

        self.assertIsNone(core.prepare_refresh())
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.FAULT)
        self.assertTrue(snapshot.faults[0].effort_limit)

    def test_h0_cached_reference_uses_exact_effort_boundary(self):
        core, _, _, _ = self._admitted_rounding_goal()
        self.clock.value += 0.5
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        self.assertTrue(
            core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE).accepted
        )
        self.assertTrue(self._rounding_feedback(core).accepted)

        self.assertIsNone(core.prepare_refresh())
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.FAULT)
        self.assertTrue(snapshot.faults[0].effort_limit)

    def test_final_write_gate_rechecks_exact_effort_after_feedback_drift(self):
        core, _, _, _ = self._admitted_rounding_goal()
        self.clock.value += 0.5
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertTrue(self._rounding_feedback(core).accepted)

        self.assertFalse(core.claim_dispatch(plan.plan_id, 0))
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.FAULT)
        self.assertTrue(snapshot.faults[0].effort_limit)

    def test_rejected_incoming_goal_leaves_watermark_and_prior_admitted_goal_intact(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        valid_target = Target(
            1,
            (
                JointTarget(q_rad=0.1, dq_rad_s=0.0, tau_nm=2.0, kp=1.0, kd=0.1),
                JointTarget(q_rad=0.1, dq_rad_s=0.0, tau_nm=2.0, kp=1.0, kd=0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(valid_target).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        self.assertEqual(core.snapshot().admitted_target, valid_target)

        # Rejected target 2 due to effort violation
        invalid_target = Target(
            2,
            (
                JointTarget(q_rad=0.1, dq_rad_s=0.0, tau_nm=25.0, kp=1.0, kd=0.1),
                JointTarget(q_rad=0.1, dq_rad_s=0.0, tau_nm=2.0, kp=1.0, kd=0.1),
            ),
            window=window,
            owner=owner,
        )
        decision = station.submit(invalid_target)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)

        # Sequence watermark and admitted target intact; state remains ARMED
        snap = core.snapshot()
        self.assertEqual(snap.last_sequence, 1)
        self.assertEqual(snap.admitted_target, valid_target)
        self.assertIs(snap.state, State.ARMED)
        self.assertFalse(snap.fault_latched)

    def test_changed_feedback_after_preparation_invalidates_dispatch_claim(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        # Admitted target has tau_ff = 8.0, kd = 2.0. With dq_meas = 0.0, effort = 8.0 <= 10.0
        target = Target(
            1,
            (
                JointTarget(q_rad=0.0, dq_rad_s=0.0, tau_nm=8.0, kp=0.0, kd=2.0),
                JointTarget(q_rad=0.0, dq_rad_s=0.0, tau_nm=0.0, kp=0.0, kd=0.0),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # Now before dispatch, observation feedback changes: dq_meas becomes -2.0 rad/s
        # kd * (dq_ref - dq_meas) = 2.0 * (0.0 - (-2.0)) = 4.0
        # Total predicted effort = 8.0 + 4.0 = 12.0 > 10.0
        self.clock.value += 0.01
        self.assertTrue(
            core.receive(
                0,
                JointSample(0.0, -2.0, 0.0),
                MotorHealth(mode=1, errors=0, warnings=0, timeout=False),
            ).accepted
        )

        # claim_dispatch re-evaluates effort with the fresh feedback and rejects
        claimed = core.claim_dispatch(plan.plan_id, 0)
        self.assertFalse(claimed)

        # Rejection latches effort_limit and faults controller
        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertTrue(snap.faults[0].effort_limit)
        self.assertEqual(snap.faults[0].name, "J1")

    def test_independent_measured_effort_monitoring_trips_immediately(self):
        # 1. receive() trips when abs(tau_nm) > max_effort_nm
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("measured-tester")
        self.assertTrue(core.begin_check().accepted)

        # J1 receives tau_nm = 10.5 > 10.0
        decision = core.receive(0, JointSample(0.0, 0.0, 10.5), HEALTHY_STOP)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)

        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertTrue(snap.faults[0].effort_limit)
        self.assertEqual(snap.faults[0].name, "J1")

        # 2. Measured effort exact equality is allowed
        self.assertTrue(station.stop().accepted)
        owner2 = station.claim("recovery-tester")
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        # J1 receives exact limit tau_nm = 10.0 -> accepted
        self.assertTrue(core.receive(0, JointSample(0.0, 0.0, 10.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, JointSample(0.0, 0.0, -15.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner2).accepted)
        self.assertIs(core.snapshot().state, State.ARMED)

        # 3. receive() with negative effort exceeding limit trips
        decision_neg = core.receive(0, JointSample(0.0, 0.0, -10.1), HEALTHY_STOP)
        self.assertFalse(decision_neg.accepted)
        self.assertIs(decision_neg.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)
        self.assertTrue(core.snapshot().faults[0].effort_limit)

    def test_public_receive_effort_fault_prevents_arm(self):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("arm-after-effort-fault")

        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, JointSample(0.0, 0.0, 5.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, JointSample(0.0, 0.0, 5.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)

        decision = core.receive(0, JointSample(0.0, 0.0, 11.0), HEALTHY_STOP)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)
        self.assertTrue(core.snapshot().faults[0].effort_limit)
        self.assertIs(core.snapshot().state, State.FAULT)

        arm_decision = station.arm(owner)
        self.assertFalse(arm_decision.accepted)
        self.assertIs(arm_decision.code, DecisionCode.INVALID_STATE)

    def test_per_joint_attribution(self):
        core, _, _ = self._armed_station()

        # J1 is within limit (5.0 <= 10.0), J2 exceeds limit (20.0 > 15.0)
        decision = core.receive(1, JointSample(0.0, 0.0, 20.0), HEALTHY_STOP)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)

        snap = core.snapshot()
        self.assertEqual(len(snap.faults), 1)
        self.assertEqual(snap.faults[0].name, "J2")
        self.assertEqual(snap.faults[0].motor_id, 1)
        self.assertTrue(snap.faults[0].effort_limit)
        self.assertEqual(snap.affected_joints, ("J2",))

    def test_invalid_arithmetic_overflow_fails_closed(self):
        # Huge numbers that overflow float arithmetic
        target = JointTarget(q_rad=0.0, dq_rad_s=0.0, tau_nm=1e308, kp=1e308, kd=1e308)
        sample = JointSample(q_rad=1e308, dq_rad_s=1e308, tau_nm=0.0)

        with self.assertRaises((OverflowError, ValueError)):
            predict_joint_effort(target, sample)

    def test_cause_union_and_fresh_healthy_stop_recovery(self):
        core, station, _ = self._armed_station()

        # Trip position limit AND effort limit on J1
        decision = core.receive(
            0,
            JointSample(3.0, 0.0, 11.0),
            HEALTHY_STOP,
        )
        self.assertFalse(decision.accepted)

        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.faults[0].position_limit)
        self.assertTrue(snap.faults[0].effort_limit)

        # Disconnect and reconnect does not clear the fault
        self.assertTrue(core.disconnect().accepted)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertIs(core.snapshot().state, State.CONNECTED)
        self.assertTrue(core.snapshot().fault_latched)

        # begin_check without acknowledge_fault fails
        dec_check = core.begin_check()
        self.assertFalse(dec_check.accepted)
        self.assertIs(dec_check.code, DecisionCode.FAULT_ACK_REQUIRED)

        # Proper recovery with acknowledged stop check inside bounds
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.assertTrue(core.receive(0, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertEqual(core.snapshot().faults, ())
        self.assertFalse(core.snapshot().fault_latched)


if __name__ == "__main__":
    unittest.main()
