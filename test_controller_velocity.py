import math
import unittest
from unittest.mock import patch

import joint_controller
from im6014_codec import decode_command, raw_to_position, raw_to_rpm
from joint_controller import (
    DecisionCode,
    JointCore,
    JointSpec,
    MotorHealth,
    PositionLimits,
    Reference,
    SourceIdentity,
    State,
    Target,
)
from joint_mapping import JointMap, JointSample, JointTarget, OutputSample
from test_controller import DeterministicClock, FakeStation


SOURCE = SourceIdentity("fixture", "controller-velocity", 1)
STOP = MotorHealth(0, 0, 0, False)


def armed_core(specs, seeds, *, clock=None, max_step=0.02, feedback_max_age=1.0):
    if clock is None:
        clock = DeterministicClock(0.0)
    core = JointCore(
        specs,
        clock=clock,
        feedback_max_age_s=feedback_max_age,
        max_reference_step_s=max_step,
    )
    assert core.begin_connect()
    assert core.connected(SOURCE)
    assert core.begin_check()
    for spec, position in zip(specs, seeds):
        assert core.receive(spec.motor_id, JointSample(position, 0.0, 0.0), STOP)
    assert core.checked()
    assert core.arm(owner="test").accepted
    return core, FakeStation(core), clock


def target(core, clock, sequence, joints, *, created_at_s=None, expires_at_s=100.0):
    if created_at_s is None:
        created_at_s = clock.value
    return Target(
        sequence,
        tuple(joints),
        created_at_s=created_at_s,
        expires_at_s=expires_at_s,
        ownership=core.snapshot().ownership,
    )


class ReferenceVelocityTests(unittest.TestCase):
    def test_velocity_and_step_limits_are_required_finite_and_positive(self):
        mapping = JointMap(1, 1, 0.0)
        limits = PositionLimits(-1.0, 1.0)
        for value in (True, False, 0, -1.0, math.nan, math.inf, -math.inf, 10**400, "1"):
            with self.subTest(value=repr(value)), self.assertRaises(ValueError):
                JointSpec("J1", 0, mapping, limits, value)
            with self.subTest(step=value), self.assertRaises(ValueError):
                JointCore(
                    (JointSpec("J1", 0, mapping, limits, 1.0),),
                    max_reference_step_s=value,
                )
        with self.assertRaises(TypeError):
            JointSpec("J1", 0, mapping, limits)
        with self.assertRaises(TypeError):
            JointCore((JointSpec("J1", 0, mapping, limits, 1.0),))

    def test_arm_rejects_measured_seed_outside_codec_range_without_grant(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-20_000_000.0, 20_000_000.0), 1.0
        )
        core = JointCore((spec,), max_reference_step_s=0.02)
        self.assertTrue(core.begin_connect())
        self.assertTrue(core.connected(SOURCE))
        self.assertTrue(core.begin_check())
        self.assertTrue(core.receive(0, JointSample(10_000_000.0, 0.0, 0.0), STOP))
        self.assertTrue(core.checked())

        decision = core.arm(owner="test")

        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.INVALID_ARGUMENT)
        self.assertIs(core.snapshot().state, State.READY)
        self.assertIsNone(core.snapshot().ownership)
        self.assertEqual(core.snapshot().ownership_generation, 0)
        self.assertIsNone(core._reference_positions)
        self.assertIsNone(core._last_reference_at_s)

    def test_nonzero_measured_seed_per_joint_steps_reversal_and_feedforward_clipping(self):
        specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0), 0.4),
            JointSpec("J2", 1, JointMap(1, -1, 0.0), PositionLimits(-2.0, 2.0), 0.8),
        )
        core, station, clock = armed_core(specs, (0.2, -0.3), max_step=0.2)
        self.assertEqual(core._reference_positions, (0.2, -0.3))
        first = target(core, clock, 0, (
            JointTarget(1.0, 1.0, 0.0, 1.0, 0.1),
            JointTarget(-1.0, -1.5, 0.0, 1.0, 0.1),
        ))
        clock.value = 0.1
        self.assertTrue(station.submit(first))
        issued = station.consume()
        self.assertIsInstance(issued, Reference)
        self.assertEqual(issued.requested, first)
        self.assertGreater(issued.joints[0].q_rad, 0.2)
        self.assertLessEqual(issued.joints[0].q_rad - 0.2, 0.04)
        self.assertLess(issued.joints[1].q_rad, -0.3)
        self.assertGreaterEqual(issued.joints[1].q_rad + 0.3, -0.08)
        self.assertLessEqual(issued.joints[0].dq_rad_s, 0.4)
        self.assertGreaterEqual(issued.joints[0].dq_rad_s, -0.4)
        self.assertLessEqual(issued.joints[1].dq_rad_s, 0.8)
        self.assertGreaterEqual(issued.joints[1].dq_rad_s, -0.8)

        reverse = target(core, clock, 1, (
            JointTarget(-1.0, -1.0, 0.0, 1.0, 0.1),
            JointTarget(1.0, 1.5, 0.0, 1.0, 0.1),
        ))
        clock.value = 0.2
        self.assertTrue(station.submit(reverse))
        reversed_reference = station.consume()
        self.assertLess(reversed_reference.joints[0].q_rad, issued.joints[0].q_rad)
        self.assertGreaterEqual(
            reversed_reference.joints[0].q_rad - issued.joints[0].q_rad, -0.04
        )
        self.assertGreater(reversed_reference.joints[1].q_rad, issued.joints[1].q_rad)
        self.assertLessEqual(
            reversed_reference.joints[1].q_rad - issued.joints[1].q_rad, 0.08
        )

    def test_zero_elapsed_is_stationary_but_same_tick_streaming_advances(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.02)
        first = target(core, clock, 0, (JointTarget(0.5, 2.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(first))
        at_seed = station.consume()
        self.assertIsNotNone(at_seed)
        self.assertEqual(at_seed.joints[0].q_rad, 0.0)
        self.assertLessEqual(abs(at_seed.joints[0].dq_rad_s), 1.0)
        self.assertEqual((at_seed.accepted_at_s, at_seed.issued_at_s), (0.0, 0.0))

        clock.value = 0.01
        streamed = target(core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(streamed))
        advanced = station.consume()
        self.assertEqual(advanced.accepted_at_s, advanced.issued_at_s)
        self.assertGreater(advanced.joints[0].q_rad, at_seed.joints[0].q_rad)
        self.assertLessEqual(advanced.joints[0].q_rad, 0.01)

        clock.value = 0.02
        next_request = target(core, clock, 2, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(next_request))
        next_reference = station.consume()
        self.assertGreater(next_reference.joints[0].q_rad, advanced.joints[0].q_rad)
        self.assertLessEqual(next_reference.joints[0].q_rad - advanced.joints[0].q_rad, 0.01)

    def test_acceptance_queue_and_idle_time_never_bank_more_than_step_cap(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2, 2), 1.0),)
        core, station, clock = armed_core(
            specs, (0.0,), max_step=0.02, feedback_max_age=20.0
        )
        delayed = target(
            core, clock, 0, (JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=20.0,
        )
        clock.value = 0.5
        self.assertTrue(station.submit(delayed))
        clock.value = 0.6
        issued = station.consume()
        self.assertEqual(issued.accepted_at_s, 0.5)
        self.assertLessEqual(issued.joints[0].q_rad, 0.02)

        clock.value = 10.0
        idle_target = target(core, clock, 1, (JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(idle_target))
        idle_issue = station.consume()
        self.assertLessEqual(idle_issue.joints[0].q_rad - issued.joints[0].q_rad, 0.02)

    def test_absent_expired_invalid_replacement_and_clock_regression_emit_nothing(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0),)
        core, station, clock = armed_core(specs, (0.0,))
        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])

        accepted = target(core, clock, 0, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),), expires_at_s=0.1)
        self.assertTrue(station.submit(accepted))
        pending_before = core.snapshot().pending_target
        history_before = (core._reference_positions, core._last_reference_at_s)
        invalid = target(core, clock, 1, (JointTarget(2.0, 0.0, 0.0, 1.0, 0.1),))
        self.assertEqual(station.submit(invalid).code, DecisionCode.POSITION_LIMIT)
        self.assertEqual(core.snapshot().pending_target, pending_before)
        self.assertEqual((core._reference_positions, core._last_reference_at_s), history_before)
        self.assertEqual(station.writes, [])

        clock.value = 0.1
        self.assertIsNone(station.consume())
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 0)
        self.assertEqual(station.writes, [])

        newer = target(core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),), expires_at_s=1.0)
        self.assertTrue(station.submit(newer))
        clock.value = 0.05
        with self.assertRaises(ValueError):
            station.consume()
        clock.value = 0.1
        self.assertEqual(core.snapshot().pending_target, newer)
        self.assertEqual(station.writes, [])

    def test_failed_second_joint_projection_keeps_pending_and_history_atomically(self):
        specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0),
            JointSpec("J2", 1, JointMap(1, -1, 0.0), PositionLimits(-1, 1), 1.0),
        )
        core, station, clock = armed_core(specs, (0.0, 0.0))
        request = target(core, clock, 0, (
            JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),
            JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(request))
        clock.value = 0.01
        original = joint_controller.project_position_raw
        calls = 0

        def fail_second(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise ValueError("injected second-joint projection failure")
            return original(*args, **kwargs)

        history_before = (core._reference_positions, core._last_reference_at_s)
        with patch.object(joint_controller, "project_position_raw", side_effect=fail_second):
            self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])
        self.assertEqual(core.snapshot().pending_target, request)
        self.assertEqual((core._reference_positions, core._last_reference_at_s), history_before)
        self.assertIs(core.snapshot().state, State.ARMED)

    def test_projection_time_passing_target_deadline_discards_without_release(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.1)
        request = target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=0.1,
        )
        self.assertTrue(station.submit(request))
        clock.value = 0.05
        history_before = (core._reference_positions, core._last_reference_at_s)
        ownership_before = core.snapshot().ownership
        calls_before = clock.calls
        original = joint_controller.project_position_raw

        def pass_deadline_during_projection(*args, **kwargs):
            projected = original(*args, **kwargs)
            clock.value = 0.2
            return projected

        with patch.object(
            joint_controller,
            "project_position_raw",
            side_effect=pass_deadline_during_projection,
        ):
            self.assertIsNone(station.consume())

        self.assertEqual(clock.calls - calls_before, 2)
        self.assertEqual(station.writes, [])
        self.assertIsNone(core._pending_target)
        self.assertEqual(core._last_sequence, request.sequence)
        self.assertEqual((core._reference_positions, core._last_reference_at_s), history_before)
        self.assertIs(core._state, State.ARMED)
        self.assertIs(core._ownership, ownership_before)

    def test_projection_time_staling_feedback_revokes_without_release(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0),)
        core, station, clock = armed_core(
            specs, (0.0,), max_step=0.1, feedback_max_age=0.1
        )
        request = target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=1.0,
        )
        self.assertTrue(station.submit(request))
        clock.value = 0.05
        calls_before = clock.calls
        generation_before = core._ownership_generation
        original = joint_controller.project_position_raw

        def stale_feedback_during_projection(*args, **kwargs):
            projected = original(*args, **kwargs)
            clock.value = 0.2
            return projected

        with patch.object(
            joint_controller,
            "project_position_raw",
            side_effect=stale_feedback_during_projection,
        ):
            self.assertIsNone(station.consume())

        self.assertEqual(clock.calls - calls_before, 2)
        self.assertEqual(station.writes, [])
        self.assertIsNone(core._pending_target)
        self.assertEqual(core._last_sequence, request.sequence)
        self.assertIs(core._state, State.FAULT)
        self.assertTrue(core._faults["J1"].stale)
        self.assertIsNone(core._ownership)
        self.assertEqual(core._ownership_generation, generation_before + 1)
        self.assertIsNone(core._reference_positions)
        self.assertIsNone(core._last_reference_at_s)

    def test_projection_compute_delay_stamps_final_release_time(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.1)
        request = target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=1.0,
        )
        self.assertTrue(station.submit(request))
        clock.value = 0.05
        calls_before = clock.calls
        original = joint_controller.project_position_raw

        def small_compute_delay(*args, **kwargs):
            projected = original(*args, **kwargs)
            clock.value = 0.06
            return projected

        with patch.object(
            joint_controller,
            "project_position_raw",
            side_effect=small_compute_delay,
        ):
            issued = station.consume()

        self.assertIsNotNone(issued)
        self.assertIs(issued.requested, request)
        self.assertEqual(issued.issued_at_s, 0.06)
        self.assertEqual(core._last_reference_at_s, 0.06)
        self.assertEqual(core._reference_positions, tuple(joint.q_rad for joint in issued.joints))
        self.assertEqual(clock.calls - calls_before, 2)
        self.assertEqual(len(station.writes), 1)

        next_request = target(
            core, clock, 2, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            expires_at_s=1.0,
        )
        self.assertTrue(station.submit(next_request))
        clock.value = 0.07
        next_issue = station.consume()
        self.assertIsNotNone(next_issue)
        self.assertLessEqual(
            next_issue.joints[0].q_rad - issued.joints[0].q_rad,
            specs[0].max_velocity_rad_s * (next_issue.issued_at_s - issued.issued_at_s),
        )
        self.assertGreater(next_issue.joints[0].q_rad, issued.joints[0].q_rad)
        self.assertEqual(next_issue.issued_at_s, 0.07)
        self.assertEqual(core._last_reference_at_s, 0.07)
        self.assertEqual(len(station.writes), 2)

    def test_clock_regression_during_projection_preserves_history_and_writes_nothing(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.1)
        request = target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=1.0,
        )
        self.assertTrue(station.submit(request))
        clock.value = 0.05
        history_before = (core._reference_positions, core._last_reference_at_s)
        pending_before = core._pending_target
        ownership_before = core._ownership
        sequence_before = core._last_sequence
        original = joint_controller.project_position_raw

        def regress_clock_during_projection(*args, **kwargs):
            projected = original(*args, **kwargs)
            clock.value = 0.04
            return projected

        with patch.object(
            joint_controller,
            "project_position_raw",
            side_effect=regress_clock_during_projection,
        ):
            with self.assertRaises(ValueError):
                station.consume()

        self.assertEqual(station.writes, [])
        self.assertIs(core._pending_target, pending_before)
        self.assertEqual(core._last_sequence, sequence_before)
        self.assertEqual((core._reference_positions, core._last_reference_at_s), history_before)
        self.assertIs(core._state, State.ARMED)
        self.assertIs(core._ownership, ownership_before)

    def test_subgrid_zero_elapsed_corridor_is_retained_until_feasible(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 0.1),)
        core, station, clock = armed_core(specs, (0.0001,), max_step=0.02)
        request = target(core, clock, 0, (JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(request))
        history_before = (core._reference_positions, core._last_reference_at_s)
        self.assertIsNone(station.consume())
        self.assertEqual(core.snapshot().pending_target, request)
        self.assertEqual((core._reference_positions, core._last_reference_at_s), history_before)
        self.assertEqual(station.writes, [])

        clock.value = 0.01
        later = station.consume()
        self.assertIsNotNone(later)
        self.assertEqual(later.requested, request)
        self.assertIsNone(core.snapshot().pending_target)

    def test_lifecycle_rearm_seeds_new_measured_pose_and_requires_new_grant(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 0.5),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.1)
        old_grant = core.snapshot().ownership
        self.assertTrue(station.stop())
        self.assertIsNone(core._reference_positions)
        self.assertTrue(core.begin_check())
        self.assertTrue(core.receive(0, JointSample(0.5, 0.0, 0.0), STOP))
        self.assertTrue(core.checked())
        new_grant = core.arm(owner="test")
        self.assertTrue(new_grant.accepted)
        self.assertIsNot(new_grant.ownership, old_grant)
        self.assertEqual(core._reference_positions, (0.5,))
        self.assertEqual(core._last_reference_at_s, 0.0)

        stale_grant_target = Target(
            0, (JointTarget(0.8, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=10.0, ownership=old_grant,
        )
        self.assertEqual(station.submit(stale_grant_target).code, DecisionCode.INVALID_OWNERSHIP)
        current = target(core, clock, 0, (JointTarget(0.8, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(current))
        clock.value = 0.1
        issued = station.consume()
        self.assertLessEqual(issued.joints[0].q_rad - 0.5, 0.05)
        self.assertGreaterEqual(issued.joints[0].q_rad, 0.5)

    def test_inward_codec_grid_projection_for_position_speed_reductions_and_directions(self):
        for reduction in (1, 3, 6):
            for direction in (-1, 1):
                with self.subTest(reduction=reduction, direction=direction):
                    mapping = JointMap(reduction, direction, -direction * 0.37)
                    position_code = 100
                    q_grid = mapping.from_output(
                        OutputSample(raw_to_position(position_code), 0.0, 0.0)
                    ).q_rad
                    upper = math.nextafter(q_grid, -math.inf)
                    speed_code = direction * 2000
                    speed_grid = mapping.from_output(
                        OutputSample(0.0, raw_to_rpm(speed_code), 0.0)
                    ).dq_rad_s
                    max_velocity = math.nextafter(abs(speed_grid), 0.0)
                    limits = PositionLimits(-1.0, upper)
                    spec = JointSpec("J1", 0, mapping, limits, max_velocity)
                    core, station, clock = armed_core((spec,), (0.0,), max_step=1.0)

                    for speed_sign in (-1, 1):
                        sequence = 0 if speed_sign < 0 else 1
                        request = target(core, clock, sequence, (
                            JointTarget(upper, speed_sign * 2 * max_velocity, 0.0, 1.0, 0.1),
                        ))
                        self.assertTrue(station.submit(request))
                        clock.value += 0.1
                        reference = station.consume()
                        self.assertIsNotNone(reference)
                        self.assertLessEqual(reference.joints[0].q_rad, upper)
                        self.assertGreaterEqual(reference.joints[0].q_rad, limits.minimum_rad)
                        self.assertLessEqual(abs(reference.joints[0].dq_rad_s), max_velocity)
                        packet = station.writes[-1]
                        decoded = decode_command(packet)
                        packet_joint = mapping.from_output(
                            OutputSample(decoded["position_rad"], decoded["rpm"], decoded["torque_nm"])
                        )
                        self.assertLessEqual(packet_joint.q_rad, upper)
                        self.assertGreaterEqual(packet_joint.q_rad, limits.minimum_rad)
                        self.assertLessEqual(abs(packet_joint.dq_rad_s), max_velocity)
                        self.assertEqual(packet_joint.q_rad, reference.joints[0].q_rad)
                        self.assertEqual(packet_joint.dq_rad_s, reference.joints[0].dq_rad_s)
                        self.assertNotEqual(decoded["raw_position"], position_code)
                        self.assertNotEqual(decoded["raw_rpm"], speed_sign * speed_code)


if __name__ == "__main__":
    unittest.main()
