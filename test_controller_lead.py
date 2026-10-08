
from fractions import Fraction
import math
import random
import unittest

from im6014_codec import decode_command, raw_to_position
from joint_contract import (
    DecisionCode,
    JointSpec,
    JointTrajectoryReference,
    MotorHealth,
    State,
    Target,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget, OutputSample
from joint_reference import shape_reference_joint
from test_controller import DEFAULT_PROTECTION, DeterministicClock, FakeStation, HEALTHY_STOP


class ReferenceLeadTests(unittest.TestCase):
    def setUp(self):
        self.clock = DeterministicClock(10.0)
        self.spec1 = JointSpec(
            name="J1",
            motor_id=0,
            mapping=JointMap(1.0, 1, 0.0),
            q_min_rad=-2.0,
            q_max_rad=2.0,
            v_max_rad_s=2.0,
            a_max_rad_s2=10.0,
            max_lead_rad=0.5,
            max_effort_nm=50.0,
            limits_provenance="fixture: lead test J1",
            protection=DEFAULT_PROTECTION,
        )
        self.spec2 = JointSpec(
            name="J2",
            motor_id=1,
            mapping=JointMap(1.0, 1, 0.0),
            q_min_rad=-2.0,
            q_max_rad=2.0,
            v_max_rad_s=2.0,
            a_max_rad_s2=10.0,
            max_lead_rad=0.5,
            max_effort_nm=50.0,
            limits_provenance="fixture: lead test J2",
            protection=DEFAULT_PROTECTION,
        )
        self.specs = (self.spec1, self.spec2)
        self.sample1 = JointSample(0.0, 0.0, 0.0)
        self.sample2 = JointSample(0.0, 0.0, 0.0)

    def _armed_station(self, max_reference_gap_s=1.0):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=max_reference_gap_s)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("lead-tester")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, owner

    def assert_all_constraints_certificate(self, previous, current, spec, q_meas):
        h = current.sample_time_s - previous.sample_time_s
        h_exact = Fraction(h)
        self.assertGreaterEqual(current.q_rad, spec.q_min_rad)
        self.assertLessEqual(current.q_rad, spec.q_max_rad)
        v_max = Fraction(spec.v_max_rad_s)
        self.assertLessEqual(abs(current.position_step_velocity_rad_s), v_max)
        self.assertLessEqual(abs(Fraction(current.encoded_velocity_rad_s)), v_max)
        if h > 0.0:
            a_max = Fraction(spec.a_max_rad_s2)
            prev_v_pos = Fraction(previous.position_step_velocity_rad_s)
            prev_v_enc = Fraction(previous.encoded_velocity_rad_s)
            curr_v_enc = Fraction(current.encoded_velocity_rad_s)
            self.assertLessEqual(
                abs(current.position_step_velocity_rad_s - prev_v_pos),
                a_max * h_exact,
            )
            self.assertLessEqual(
                abs(curr_v_enc - prev_v_enc),
                a_max * h_exact,
            )
        lead_exact = Fraction(spec.max_lead_rad)
        q_meas_exact = Fraction(q_meas)
        curr_q_exact = Fraction(current.q_rad)
        self.assertGreaterEqual(curr_q_exact, q_meas_exact - lead_exact)
        self.assertLessEqual(curr_q_exact, q_meas_exact + lead_exact)

    def assert_decoded_reference_certificate(
        self, previous, emitted, reference, packet, goal, spec, q_meas
    ):
        self.assertTrue(reference.is_emitted)
        self.assertEqual(reference.requested_goal, goal)
        self.assertEqual(reference.emitted_target, emitted)
        self.assertEqual(reference.packet, packet)
        self.assertIs(type(packet), bytes)

        decoded = decode_command(packet)
        decoded_joint = spec.mapping.from_output(
            OutputSample(decoded["position_rad"], decoded["rpm"], decoded["torque_nm"])
        )
        self.assertEqual(decoded_joint.q_rad, emitted.q_rad)
        self.assertEqual(decoded_joint.dq_rad_s, emitted.dq_rad_s)

        h = reference.sample_time_s - previous.sample_time_s
        h_exact = Fraction(h)
        self.assertGreater(h_exact, 0)
        q_previous = Fraction(previous.q_rad)
        q_emitted = Fraction(decoded_joint.q_rad)
        q_measured = Fraction(q_meas)
        lead = Fraction(spec.max_lead_rad)
        v_max = Fraction(spec.v_max_rad_s)
        a_step = Fraction(spec.a_max_rad_s2) * h_exact
        position_velocity = (q_emitted - q_previous) / h_exact

        self.assertEqual(reference.position_step_velocity_rad_s, position_velocity)
        self.assertGreaterEqual(q_emitted, Fraction(spec.q_min_rad))
        self.assertLessEqual(q_emitted, Fraction(spec.q_max_rad))
        self.assertGreaterEqual(q_emitted, q_measured - lead)
        self.assertLessEqual(q_emitted, q_measured + lead)
        self.assertLessEqual(abs(position_velocity), v_max)
        self.assertLessEqual(abs(Fraction(decoded_joint.dq_rad_s)), v_max)
        self.assertLessEqual(
            abs(position_velocity - Fraction(previous.position_step_velocity_rad_s)),
            a_step,
        )
        self.assertLessEqual(
            abs(Fraction(decoded_joint.dq_rad_s) - Fraction(previous.encoded_velocity_rad_s)),
            a_step,
        )

    def test_lead_limit_requires_finite_positive_real_value(self):
        for val in (True, False, 0, 0.0, -0.5, math.nan, math.inf, -math.inf, 10**400, "0.5"):
            with self.subTest(val=val), self.assertRaises(ValueError):
                JointSpec(
                    "J1", 0, JointMap(1.0, 1, 0.0), -2.0, 2.0, 2.0, 10.0, val, 50.0,
                    "fixture: invalid lead limit input",
                    DEFAULT_PROTECTION,
                )

    def test_exact_lead_equality_and_outside_bounds(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        target_exact_upper = Target(
            1,
            (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target_exact_upper).accepted)

        target_exact_lower = Target(
            2,
            (JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target_exact_lower).accepted)

        target_outside_upper = Target(
            3,
            (JointTarget(0.5001, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        dec_upper = station.submit(target_outside_upper)
        self.assertFalse(dec_upper.accepted)
        self.assertIs(dec_upper.code, DecisionCode.LEAD_OUT_OF_BOUNDS)

        target_outside_lower = Target(
            4,
            (JointTarget(-0.5001, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        dec_lower = station.submit(target_outside_lower)
        self.assertFalse(dec_lower.accepted)
        self.assertIs(dec_lower.code, DecisionCode.LEAD_OUT_OF_BOUNDS)

    def test_parent_exact_admission_reproduction(self):
        clock = DeterministicClock(0.0)
        spec = JointSpec(
            "J", 0, JointMap(1.0, 1, 0.0), -1.0, 1.0, 1.0, 10.0, 0.2, 50.0,
            "fixture: exact lead admission regression",
            DEFAULT_PROTECTION,
        )
        core = JointCore((spec,), clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("exact-lead-parent")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, JointSample(0.1, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)

        window = station.issue_window(1.0)
        valid = Target(1, (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),), window, owner)
        self.assertTrue(station.submit(valid).accepted)
        before = core.snapshot()

        q = 0.1 + 0.2
        self.assertGreater(Fraction(q) - Fraction(0.1), Fraction(0.2))
        outside = Target(2, (JointTarget(q, 0.0, 0.0, 1.0, 0.1),), window, owner)
        decision = station.submit(outside)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.LEAD_OUT_OF_BOUNDS)
        after = core.snapshot()
        self.assertEqual(after.admitted_target, before.admitted_target)
        self.assertEqual(after.last_sequence, before.last_sequence)
        self.assertIs(after.state, State.ARMED)

    def test_exact_binary_admission_boundaries_are_inclusive(self):
        clock = DeterministicClock(0.0)
        spec = JointSpec(
            "J", 0, JointMap(1.0, 1, 0.0), -1.0, 1.0, 1.0, 10.0, 0.25, 50.0,
            "fixture: exact binary lead equality",
            DEFAULT_PROTECTION,
        )
        core = JointCore((spec,), clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("exact-lead-equality")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, JointSample(0.125, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        window = station.issue_window(1.0)

        upper = Target(1, (JointTarget(0.375, 0.0, 0.0, 1.0, 0.1),), window, owner)
        lower = Target(2, (JointTarget(-0.125, 0.0, 0.0, 1.0, 0.1),), window, owner)
        self.assertEqual(Fraction(0.375) - Fraction(0.125), Fraction(0.25))
        self.assertEqual(Fraction(0.125) - Fraction(-0.125), Fraction(0.25))
        self.assertTrue(station.submit(upper).accepted)
        self.assertTrue(station.submit(lower).accepted)

    def test_healthy_rejected_goal_preservation(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        t1 = Target(
            1,
            (JointTarget(0.2, 0.0, 0.0, 1.0, 0.1), JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(t1).accepted)
        snap1 = core.snapshot()
        self.assertEqual(snap1.admitted_target, t1)
        self.assertEqual(snap1.last_sequence, 1)

        t2 = Target(
            2,
            (JointTarget(0.8, 0.0, 0.0, 1.0, 0.1), JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        rej = station.submit(t2)
        self.assertFalse(rej.accepted)
        self.assertIs(rej.code, DecisionCode.LEAD_OUT_OF_BOUNDS)

        snap2 = core.snapshot()
        self.assertEqual(snap2.admitted_target, t1)
        self.assertEqual(snap2.last_sequence, 1)

        self.clock.value += 0.05
        plan = station.refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(plan.target, t1)
        self.assertIs(core.snapshot().state, State.MOVING)

    def test_measured_drift_between_admit_prepare_claim(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        t = Target(
            1,
            (JointTarget(0.4, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(t).accepted)

        self.clock.value += 0.05
        self.assertTrue(
            core.receive(0, JointSample(-0.2, 0.0, 0.0), MotorHealth(1, 0, 0, False, 25.0, 30.0)).accepted
        )
        self.assertTrue(
            core.receive(1, JointSample(0.0, 0.0, 0.0), MotorHealth(1, 0, 0, False, 25.0, 30.0)).accepted
        )

        plan = station.refresh()
        self.assertIsNotNone(plan)
        ref = core.snapshot().confirmed_references[0]
        self.assertLessEqual(ref.q_rad, 0.3)
        self.assertGreaterEqual(ref.q_rad, -0.7)
        self.assert_all_constraints_certificate(
            JointTrajectoryReference(0.0, Fraction(0), 0.0, 10.0, False),
            ref,
            self.spec1,
            -0.2,
        )

        self.clock.value += 0.05
        t2 = Target(
            2,
            (JointTarget(0.2, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
            window=station.issue_window(1.0),
            owner=owner,
        )
        self.assertTrue(station.submit(t2).accepted)
        plan2 = core.prepare_refresh()
        self.assertIsNotNone(plan2)

        self.assertTrue(
            core.receive(0, JointSample(0.9, 0.0, 0.0), MotorHealth(1, 0, 0, False, 25.0, 30.0)).accepted
        )
        self.assertFalse(core.claim_dispatch(plan2.plan_id, 0))
        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertTrue(snap.faults[0].constraint_fault)

    def test_simultaneous_speed_accel_position_lead_satisfiable_and_infeasible(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        t_sat = Target(
            1,
            (JointTarget(0.3, 1.0, 0.0, 1.0, 0.1), JointTarget(0.3, 1.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(t_sat).accepted)
        self.clock.value += 0.05
        plan_sat = station.refresh()
        self.assertIsNotNone(plan_sat)
        ref_sat = core.snapshot().confirmed_references[0]
        self.assert_all_constraints_certificate(
            JointTrajectoryReference(0.0, Fraction(0), 0.0, 10.0, False),
            ref_sat,
            self.spec1,
            0.0,
        )

        anchor_goal = JointTarget(0.3, 1.0, 0.0, 1.0, 0.1)
        stop_anchor = JointTrajectoryReference(
            0.0, Fraction(0), 0.0, self.clock.value, False
        )
        code, anchor_target, anchor_fast, anchor_packet = shape_reference_joint(
            self.spec1, anchor_goal, stop_anchor, JointSample(0.0, 0.0, 0.0), self.clock.value + 0.2, 1.0
        )
        self.assertIsNone(code)
        self.assertTrue(anchor_fast.is_emitted)
        self.assertEqual(anchor_fast.requested_goal, anchor_goal)
        self.assertEqual(anchor_fast.emitted_target, anchor_target)
        self.assertEqual(anchor_fast.packet, anchor_packet)
        self.assertGreater(anchor_fast.position_step_velocity_rad_s, Fraction(1))

        next_now = anchor_fast.sample_time_s + 0.05
        h_exact = Fraction(next_now - anchor_fast.sample_time_s)
        accel_lower = max(
            -Fraction(self.spec1.v_max_rad_s),
            Fraction(anchor_fast.position_step_velocity_rad_s)
            - Fraction(self.spec1.a_max_rad_s2) * h_exact,
        )
        reachable_position_lower = (
            Fraction(anchor_fast.q_rad) + accel_lower * h_exact
        )
        drifted_q_meas = anchor_fast.q_rad - 0.6
        lead_position_upper = (
            Fraction(drifted_q_meas) + Fraction(self.spec1.max_lead_rad)
        )
        self.assertGreater(reachable_position_lower, lead_position_upper)

        code, emitted, ref, pkt = shape_reference_joint(
            self.spec1,
            anchor_goal,
            anchor_fast,
            JointSample(drifted_q_meas, 0.0, 0.0),
            next_now,
            1.0,
        )
        self.assertIs(code, DecisionCode.CONSTRAINT_VIOLATION)
        self.assertIsNone(emitted)
        self.assertIsNone(ref)
        self.assertIsNone(pkt)

    def test_h0_refresh_inside_and_outside_lead(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        target = Target(
            1,
            (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1), JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)

        self.assertIsNone(core.prepare_refresh())
        self.assertIs(core.snapshot().state, State.FAULT)

        self.assertTrue(station.stop().accepted)
        owner2 = station.claim("h0-tester")
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.clock.value += 0.1
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.clock.value += 0.5
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner2).accepted)

        w2 = station.issue_window(1.0)
        t_emit = Target(
            2,
            (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1), JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)),
            window=w2,
            owner=owner2,
        )
        self.assertTrue(station.submit(t_emit).accepted)
        self.clock.value += 0.05
        plan1 = station.refresh()
        self.assertIsNotNone(plan1)

        plan_h0 = core.prepare_refresh()
        self.assertIsNotNone(plan_h0)
        self.assertEqual(plan_h0.joint_targets[0].q_rad, plan1.joint_targets[0].q_rad)

        self.assertTrue(
            core.receive(0, JointSample(0.8, 0.0, 0.0), MotorHealth(1, 0, 0, False, 25.0, 30.0)).accepted
        )
        self.assertIsNone(core.prepare_refresh())
        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.faults[0].constraint_fault)

    def _quarter_cache(self, max_lead):
        mapping = JointMap(math.ldexp(raw_to_position(1), 2), 1, 0.0)
        spec = JointSpec(
            "J", 0, mapping, -1.0, 1.0, 1.0, 4.0, max_lead, 50.0,
            "fixture: exact cached lead reference",
            DEFAULT_PROTECTION,
        )
        goal = JointTarget(0.25, 0.0, 0.0, 0.0, 0.0)
        anchor = JointTrajectoryReference(0.25, Fraction(0), 0.0, 0.0, False)
        code, actual, emitted, packet = shape_reference_joint(
            spec, goal, anchor, JointSample(0.25, 0.0, 0.0), 0.25, 0.5
        )
        self.assertIsNone(code)
        self.assertEqual(actual.q_rad, 0.25)
        self.assertTrue(emitted.is_emitted)
        self.assertEqual(emitted.requested_goal, goal)
        self.assertEqual(emitted.emitted_target, actual)
        self.assertEqual(emitted.packet, packet)
        return spec, goal, emitted, actual, packet

    def test_parent_exact_h0_replay_reproduction(self):
        spec, goal, emitted_ref, actual, _ = self._quarter_cache(0.36)
        self.assertEqual(-0.11 + spec.max_lead_rad, actual.q_rad)
        self.assertGreater(
            Fraction(actual.q_rad) - Fraction(-0.11), Fraction(spec.max_lead_rad)
        )

        code, repeated_target, repeated_ref, repeated_packet = shape_reference_joint(
            spec, goal, emitted_ref, JointSample(-0.11, 0.0, 0.0), emitted_ref.sample_time_s, 0.5
        )
        self.assertIs(code, DecisionCode.CONSTRAINT_VIOLATION)
        self.assertIsNone(repeated_target)
        self.assertIsNone(repeated_ref)
        self.assertIsNone(repeated_packet)

    def test_exact_binary_h0_lead_equality_reuses_cached_command(self):
        spec, goal, emitted_ref, actual, packet = self._quarter_cache(0.375)
        self.assertEqual(Fraction(actual.q_rad) - Fraction(-0.125), Fraction(0.375))

        code, repeated_target, repeated_ref, repeated_packet = shape_reference_joint(
            spec, goal, emitted_ref, JointSample(-0.125, 0.0, 0.0), emitted_ref.sample_time_s, 0.5
        )
        self.assertIsNone(code)
        self.assertEqual(repeated_target, actual)
        self.assertEqual(repeated_ref, emitted_ref)
        self.assertEqual(repeated_packet, packet)

    def test_missing_and_stale_feedback_handling(self):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("stale-tester")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)

        w = station.issue_window(1.0)
        t1 = Target(
            1,
            (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1), JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)),
            window=w,
            owner=owner,
        )
        self.assertTrue(station.submit(t1).accepted)

        self.clock.value += 0.6
        t2 = Target(
            2,
            (JointTarget(0.2, 0.0, 0.0, 1.0, 0.1), JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)),
            window=station.issue_window(1.0),
            owner=owner,
        )
        rej = station.submit(t2)
        self.assertFalse(rej.accepted)
        self.assertIs(rej.code, DecisionCode.STALE_FEEDBACK)
        stale = core.snapshot()
        self.assertIs(stale.state, State.FAULT)
        self.assertTrue(stale.faults[0].stale)
        self.assertIsNone(stale.current_owner)
        self.assertIsNone(stale.admitted_target)
        self.assertEqual(stale.last_sequence, 1)
        self.assertIsNone(core.prepare_refresh())
        self.assertIsNone(station.refresh())
        after_stale = core.snapshot()
        self.assertIsNone(after_stale.admitted_target)
        self.assertEqual(after_stale.dispatch_attempts, (None, None))
        self.assertEqual(after_stale.dispatched_records, (None, None))

    def test_final_gate_clock_failure_withdraws_authority_without_write(self):
        for case, bad_time, error_type in (
            ("nonfinite", math.nan, ValueError),
            ("regressing", 0.0, ValueError),
            ("callback", None, RuntimeError),
        ):
            with self.subTest(case=case):
                self.clock.value = 10.0
                core, station, owner = self._armed_station()
                target = Target(
                    1,
                    (
                        JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                        JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                    ),
                    window=station.issue_window(1.0),
                    owner=owner,
                )
                self.assertTrue(station.submit(target).accepted)
                self.clock.value += 0.05
                plan = core.prepare_refresh()
                self.assertIsNotNone(plan)
                before = core.snapshot()
                writes_before = tuple(station.writes)
                good_time = self.clock.value
                original_clock = core._clock

                if case == "callback":
                    def broken_clock():
                        raise RuntimeError("fixture clock callback failed")

                    core._clock = broken_clock
                else:
                    self.clock.value = bad_time
                try:
                    with self.assertRaises(error_type):
                        core.claim_dispatch(plan.plan_id, 0)
                finally:
                    core._clock = original_clock
                    self.clock.value = good_time

                after = core.snapshot()
                self.assertIs(after.state, State.STOPPED)
                self.assertIsNone(after.current_owner)
                self.assertIsNone(after.admitted_target)
                self.assertIsNone(after.prepared_plan)
                self.assertIsNone(core._dispatch_claim)
                self.assertEqual(after.last_sequence, before.last_sequence)
                self.assertEqual(after.confirmed_references, before.confirmed_references)
                self.assertEqual(after.dispatched_records, before.dispatched_records)
                self.assertEqual(after.dispatch_attempts, before.dispatch_attempts)
                self.assertEqual(after.joints, before.joints)
                self.assertEqual(after.feedback_ages_s, before.feedback_ages_s)
                self.assertEqual(tuple(station.writes), writes_before)
                self.assertFalse(core.claim_dispatch(plan.plan_id, 0))
                self.assertEqual(tuple(station.writes), writes_before)

    def test_negative_direction_and_wire_quantization(self):
        mapping = JointMap(reduction=3.0, direction=-1, zero_output_rad=0.25)
        spec = JointSpec(
            "J_neg",
            0,
            mapping,
            -2.0,
            2.0,
            2.0,
            10.0,
            0.25,
            50.0,
            "fixture: negative direction lead test",
            DEFAULT_PROTECTION,
        )
        clock = DeterministicClock(10.0)
        core = JointCore((spec,), clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("neg-tester")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, JointSample(0.125, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)

        w = station.issue_window(1.0)
        t_bound = Target(1, (JointTarget(0.375, 0.0, 0.0, 1.0, 0.1),), window=w, owner=owner)
        self.assertTrue(station.submit(t_bound).accepted)

        clock.value += 0.05
        plan = station.refresh()
        self.assertIsNotNone(plan)
        ref = core.snapshot().confirmed_references[0]
        self.assertGreaterEqual(ref.q_rad, -0.125)
        self.assertLessEqual(ref.q_rad, 0.375)
        decoded = decode_command(plan.packets[0])
        mapped_back = mapping.from_output(OutputSample(
            decoded["position_rad"], decoded["rpm"], decoded["torque_nm"]
        ))
        self.assertEqual(ref.q_rad, mapped_back.q_rad)

    def test_progression_over_multistep_and_conservative_stop_on_too_small_cadence(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        target = Target(
            1,
            (JointTarget(0.4, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)

        for step in range(5):
            self.clock.value += 0.05
            previous = core.snapshot().confirmed_references[0]
            prev_q = previous.q_rad
            self.assertTrue(
                core.receive(0, JointSample(prev_q, 0.0, 0.0), MotorHealth(1, 0, 0, False, 25.0, 30.0)).accepted
            )
            self.assertTrue(
                core.receive(1, JointSample(0.0, 0.0, 0.0), MotorHealth(1, 0, 0, False, 25.0, 30.0)).accepted
            )
            next_window = station.issue_window(1.0)
            next_target = Target(
                step + 2,
                target.joints,
                window=next_window,
                owner=owner,
            )
            self.assertTrue(station.submit(next_target).accepted)
            plan = station.refresh()
            self.assertIsNotNone(plan)
            current = core.snapshot().confirmed_references[0]
            self.assert_all_constraints_certificate(previous, current, self.spec1, prev_q)
            self.assertGreater(current.q_rad, prev_q)

        self.clock.value += 1e-10
        self.assertIsNone(core.prepare_refresh())
        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.faults[0].constraint_fault)

    def test_attributed_fault_union_and_recovery(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        t_bad = Target(
            1,
            (JointTarget(0.8, 0.0, 0.0, 1.0, 0.1), JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        dec = station.submit(t_bad)
        self.assertFalse(dec.accepted)
        self.assertIs(dec.code, DecisionCode.LEAD_OUT_OF_BOUNDS)

        t_good = Target(
            1,
            (JointTarget(0.4, 0.0, 0.0, 1.0, 0.1), JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(t_good).accepted)
        self.clock.value += 0.01

        self.assertTrue(
            core.receive(0, JointSample(-1.0, 0.0, 0.0), MotorHealth(1, 0, 0, False, 25.0, 30.0)).accepted
        )
        self.assertTrue(
            core.receive(1, JointSample(0.0, 0.0, 0.0), MotorHealth(1, 0, 0, False, 25.0, 30.0)).accepted
        )

        self.assertIsNone(core.prepare_refresh())
        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertEqual(snap.affected_joints, ("J1",))
        self.assertTrue(snap.faults[0].constraint_fault)

        self.assertTrue(station.stop().accepted)
        new_owner = station.claim("recovery-client")
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.clock.value += 0.1
        self.assertTrue(core.receive(0, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.clock.value += 0.5
        self.assertTrue(core.receive(0, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(new_owner).accepted)
        self.assertIs(core.snapshot().state, State.ARMED)
        self.assertFalse(core.snapshot().fault_latched)

    def test_deterministic_seeded_fuzz_four_constraint_invariants(self):
        rng = random.Random(12345)
        reductions = (0.75, 1.25, 1.5, 2.25, 3.5)
        cadences = (1.0 / 64.0, 1.0 / 32.0)
        directions = set()
        used_reductions = set()
        exercised_reversal = 0
        successful_references = 0
        infeasible_cases = 0

        for iteration in range(50):
            reduction = reductions[iteration % len(reductions)]
            direction = 1 if iteration % 2 == 0 else -1
            directions.add(direction)
            used_reductions.add(reduction)
            zero = rng.uniform(-0.25, 0.25)
            q_min = rng.uniform(-3.0, -2.0)
            q_max = rng.uniform(2.0, 3.0)
            v_max = rng.uniform(0.75, 1.5)
            a_max = rng.uniform(256.0, 512.0)
            lead = rng.uniform(0.55, 0.75)
            mapping = JointMap(reduction, direction, zero)
            spec = JointSpec(
                "J_fuzz", 0, mapping, q_min, q_max, v_max, a_max, lead, 100.0,
                "fixture: seeded emitted-reference certificate",
                DEFAULT_PROTECTION,
            )
            cadence = rng.choice(cadences)
            goal_sign = rng.choice((-1, 1))
            goal = JointTarget(goal_sign * 0.45, 0.0, 0.0, 1.0, 0.1)
            previous = JointTrajectoryReference(
                0.0, Fraction(0), 0.0, 10.0, False
            )
            now = previous.sample_time_s + cadence

            code, emitted, reference, packet = shape_reference_joint(
                spec, goal, previous, JointSample(0.0, 0.0, 0.0), now, 1.0
            )
            with self.subTest(iteration=iteration, phase="advance"):
                self.assertIsNone(code)
                self.assertIsNotNone(emitted)
                self.assertIsNotNone(reference)
                self.assertIsNotNone(packet)
                self.assertGreater(
                    goal_sign * (emitted.q_rad - previous.q_rad), 0.0
                )
                self.assert_decoded_reference_certificate(
                    previous, emitted, reference, packet, goal, spec, 0.0
                )
            successful_references += 1

            previous = reference
            reverse_goal = JointTarget(-goal_sign * 0.45, 0.0, 0.0, 1.0, 0.1)
            now = previous.sample_time_s + cadence
            code, emitted, reference, packet = shape_reference_joint(
                spec, reverse_goal, previous, JointSample(previous.q_rad, 0.0, 0.0), now, 1.0
            )
            with self.subTest(iteration=iteration, phase="reverse"):
                self.assertIsNone(code)
                self.assertIsNotNone(emitted)
                self.assertIsNotNone(reference)
                self.assertIsNotNone(packet)
                self.assertLess(
                    goal_sign * (emitted.q_rad - previous.q_rad), 0.0
                )
                self.assert_decoded_reference_certificate(
                    previous, emitted, reference, packet, reverse_goal, spec,
                    previous.q_rad,
                )
            successful_references += 1
            exercised_reversal += 1

            previous = reference
            impossible_measurement = previous.q_rad + 1.0
            now = previous.sample_time_s + cadence
            code, emitted, reference, packet = shape_reference_joint(
                spec, reverse_goal, previous, JointSample(impossible_measurement, 0.0, 0.0), now, 1.0
            )
            with self.subTest(iteration=iteration, phase="infeasible"):
                self.assertIs(code, DecisionCode.CONSTRAINT_VIOLATION)
                self.assertIsNone(emitted)
                self.assertIsNone(reference)
                self.assertIsNone(packet)
                self.assertGreater(
                    Fraction(impossible_measurement) - Fraction(spec.max_lead_rad),
                    Fraction(previous.q_rad)
                    + Fraction(spec.v_max_rad_s)
                    * Fraction(now - previous.sample_time_s),
                )
            infeasible_cases += 1

        self.assertEqual(directions, {-1, 1})
        self.assertEqual(used_reductions, set(reductions))
        self.assertEqual(successful_references, 100)
        self.assertEqual(exercised_reversal, 50)
        self.assertEqual(infeasible_cases, 50)


if __name__ == "__main__":
    unittest.main()
