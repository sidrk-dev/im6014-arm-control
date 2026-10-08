"""Targeted unit and acceptance tests for C09 inclusive joint position bounds."""

import math
import random
import struct
import unittest
from dataclasses import replace
from fractions import Fraction
from unittest.mock import patch

from im6014_codec import (
    POSITION_MAX,
    POSITION_MIN,
    RAW_POSITION_MAX,
    RAW_POSITION_MIN,
    RATIO,
    RPM_MAX,
    RPM_MIN,
    command,
    command_with_raw_position,
    crc32,
    decode_command,
    position_to_raw,
    project_position_raw,
    raw_to_position,
)
from joint_contract import (
    CommunicationFailure,
    DecisionCode,
    IntentWindow,
    JointDispatchAttempt,
    JointDispatchedRecord,
    JointFault,
    JointSpec,
    JointStatus,
    JointTrajectoryReference,
    MotorHealth,
    OwnerToken,
    RefreshPlan,
    Snapshot,
    SourceIdentity,
    State,
    Target,
    WriteResult,
    owner_tokens_match,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget, OutputSample
from joint_reference import (
    ReferenceBatchResult,
    ReferenceFailure,
    prepare_reference_joint,
    shape_reference_joint,
)
from test_controller import DeterministicClock, FakeStation, HEALTHY_STOP, SAMPLE

SOURCE = SourceIdentity("fixture", "limits-test", 1)


def _with_command_crc(packet):
    return packet[:16] + struct.pack("<I", crc32(packet[:16]))


class PositionLimitTests(unittest.TestCase):
    def setUp(self):
        self.clock = DeterministicClock(0.0)
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), -1.0, 2.0, 100.0, "fixture: software position limits J1"),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), -0.5, 0.5, 100.0, "fixture: software position limits J2"),
        )
        self.valid_sample_j1 = JointSample(0.1, 0.0, 0.0)
        self.valid_sample_j2 = JointSample(0.0, 0.0, 0.0)

    def _armed_station(self, connection_id="tester"):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim(connection_id)
        # Check with valid within-bounds observations
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, self.valid_sample_j1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.valid_sample_j2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, owner

    @staticmethod
    def _project(spec, wanted_q):
        return project_position_raw(
            wanted_q,
            spec.q_min_rad,
            spec.q_max_rad,
            lambda output_q: spec.mapping.from_output(
                OutputSample(output_q, 0.0, 0.0)
            ).q_rad,
            spec.mapping.direction,
        )

    def test_exact_lower_and_upper_requests_accepted(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        # Exact lower on J1 (-1.0), exact upper on J2 (0.5)
        t1 = Target(
            1,
            (
                JointTarget(-1.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        dec1 = station.submit(t1)
        self.assertTrue(dec1.accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)

        station.writes.clear()
        self.clock.value += 0.05
        plan = station.refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(len(station.writes), 2)
        self.assertEqual(station.writes, list(plan.packets))
        self.assertIs(core.snapshot().state, State.MOVING)

        # Exact upper on J1 (2.0), exact lower on J2 (-0.5)
        t2 = Target(
            2,
            (
                JointTarget(2.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        dec2 = station.submit(t2)
        self.assertTrue(dec2.accepted)
        self.assertEqual(core.snapshot().last_sequence, 2)

    def test_one_representable_step_outside_bounds_rejected(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        prior_target = Target(
            1,
            (
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(prior_target).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)

        # One step below lower bound on J1 (-1.0001 < -1.0)
        below = Target(
            2,
            (
                JointTarget(-1.0001, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        dec_below = station.submit(below)
        self.assertFalse(dec_below.accepted)
        self.assertEqual(dec_below.code, DecisionCode.POSITION_OUT_OF_BOUNDS)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        # One step above upper bound on J2 (0.5001 > 0.5)
        above = Target(
            3,
            (
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.5001, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        dec_above = station.submit(above)
        self.assertFalse(dec_above.accepted)
        self.assertEqual(dec_above.code, DecisionCode.POSITION_OUT_OF_BOUNDS)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

    def test_negative_direction_and_nonzero_zero_reduction_map(self):
        specs_mapped = (
            JointSpec(
                "J1",
                0,
                JointMap(reduction=3.0, direction=-1, zero_output_rad=0.25),
                -1.0,
                1.0,
                100.0,
                "fixture: software limits negative direction",
            ),
            JointSpec(
                "J2",
                1,
                JointMap(reduction=2.0, direction=1, zero_output_rad=-0.1),
                -0.5,
                0.5,
                100.0,
                "fixture: software limits positive direction",
            ),
        )
        core = JointCore(specs_mapped, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("mapped-client")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)

        window = station.issue_window(1.0)
        # Exact lower on J1 (-1.0), exact upper on J2 (0.5)
        target = Target(
            1,
            (
                JointTarget(-1.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        station.writes.clear()
        self.clock.value += 0.05
        plan = station.refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(len(station.writes), 2)
        self.assertEqual(station.writes, list(plan.packets))

        # Verify decoded command positions stay within bounds
        dec_pkt_0 = decode_command(station.writes[0])
        decoded_sample_0 = specs_mapped[0].mapping.from_output(
            OutputSample(dec_pkt_0["position_rad"], 0.0, 0.0)
        )
        self.assertGreaterEqual(decoded_sample_0.q_rad, specs_mapped[0].q_min_rad)
        self.assertLessEqual(decoded_sample_0.q_rad, specs_mapped[0].q_max_rad)

        # Target outside bounds with negative direction is rejected
        outside = Target(
            2,
            (
                JointTarget(1.05, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        rej = station.submit(outside)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.POSITION_OUT_OF_BOUNDS)

    def test_zero_width_interval_and_no_representable_safe_command(self):
        # Case A: zero-width interval on exact representable grid point
        raw_code = 1000
        grid_pos = raw_to_position(raw_code)
        spec_zero_width = JointSpec(
            "J1",
            0,
            JointMap(1.0, 1, 0.0),
            grid_pos,
            grid_pos,
            10.0,
            "fixture: exact grid point interval",
        )
        found_code = self._project(spec_zero_width, grid_pos)
        self.assertEqual(found_code, raw_code)

        # Case B: interval strictly between two representable grid points
        raw0 = 2000
        raw1 = 2001
        pos0 = raw_to_position(raw0)
        pos1 = raw_to_position(raw1)
        mid = (pos0 + pos1) / 2.0
        # Interval narrower than one raw step containing no integer code
        spec_empty_grid = JointSpec(
            "J1",
            0,
            JointMap(1.0, 1, 0.0),
            mid - 1e-9,
            mid + 1e-9,
            10.0,
            "fixture: empty grid interval",
        )
        found_none = self._project(spec_empty_grid, mid)
        self.assertIsNone(found_none)

        # Submission on spec with no representable command is rejected
        core = JointCore((spec_empty_grid,), clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("zero-client")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, JointSample(mid, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)

        w = station.issue_window(1.0)
        t = Target(1, (JointTarget(mid, 0.0, 0.0, 1.0, 0.1),), window=w, owner=owner)
        dec = station.submit(t)
        self.assertFalse(dec.accepted)
        self.assertEqual(dec.code, DecisionCode.POSITION_OUT_OF_BOUNDS)

    def test_malformed_spec_and_fault_config_rejected(self):
        mapping = JointMap(1.0, 1, 0.0)

        # q_min_rad > q_max_rad
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, 1.0, 0.0, 10.0, "fixture: invalid min max")

        # non-numeric / bool types
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, True, 1.0, 10.0, "fixture: bool min")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, 0.0, False, 10.0, "fixture: bool max")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, "0.0", 1.0, 10.0, "fixture: str min")

        # infinite / NaN
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, float("-inf"), 1.0, 10.0, "fixture: neg inf min")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, 0.0, float("inf"), 10.0, "fixture: inf max")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, float("nan"), 1.0, 10.0, "fixture: nan min")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, 0, 10**400, 10.0, "fixture: overflowing max")

        # v_max_rad_s <= 0 or not finite
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, 0.0, "fixture: zero v_max")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, -10.0, "fixture: negative v_max")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, float("inf"), "fixture: inf v_max")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, float("nan"), "fixture: nan v_max")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, "10.0", "fixture: str v_max")

        # invalid provenance
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, 10.0, "")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, 10.0, "   ")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, 10.0, 123)
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, 10.0, "x" * 257)

        with self.assertRaises(TypeError):
            JointSpec("J1", 0, mapping, -1.0, 1.0)
        with self.assertRaises(TypeError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, 10.0)
        with self.assertRaises(TypeError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, 10.0, provenance="legacy alias")

        # JointFault position_limit type validation
        with self.assertRaises(ValueError):
            JointFault("J1", 0, position_limit="bad")
        with self.assertRaises(ValueError):
            JointFault("J1", 0, speed_limit="bad")
        with self.assertRaises(ValueError):
            JointFault("J1", 0, constraint_fault="bad")
        jf = JointFault("J1", 0, position_limit=True, speed_limit=True, constraint_fault=True)
        self.assertTrue(jf.position_limit)
        self.assertTrue(jf.speed_limit)
        self.assertTrue(jf.constraint_fault)
        self.assertFalse(hasattr(jf, "position_out_of_bounds"))

    def test_multi_joint_invalid_last_joint_cannot_replace_goal_or_watermark(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        prior_target = Target(
            1,
            (
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(prior_target).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)

        # Target 2: Joint 0 valid (0.2 within [-1.0, 2.0]), Joint 1 invalid (10.0 outside [-0.5, 0.5])
        invalid_target = Target(
            2,
            (
                JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
                JointTarget(10.0, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        decision = station.submit(invalid_target)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.code, DecisionCode.POSITION_OUT_OF_BOUNDS)

        # Atomically preserves admitted target and watermark sequence
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

    def test_measured_outside_bounds_at_admission_and_final_write_gate(self):
        # 1. Measured outside at receive during checking latches fault
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertTrue(core.begin_check().accepted)
        # J1 upper bound is 2.0; receive 2.5
        rej = core.receive(0, JointSample(2.5, 0.0, 0.0), HEALTHY_STOP)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.POSITION_OUT_OF_BOUNDS)
        self.assertTrue(core.snapshot().faults[0].position_limit)
        self.assertIs(core.snapshot().state, State.FAULT)

        # 2. Final write gate rechecks current measured bounds
        core2, station2, owner2 = self._armed_station("gate-client")
        w2 = station2.issue_window(1.0)
        t2 = Target(
            1,
            (
                JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            ),
            window=w2,
            owner=owner2,
        )
        self.assertTrue(station2.submit(t2).accepted)
        self.clock.value += 0.05
        plan = core2.prepare_refresh()
        self.assertIsNotNone(plan)

        # Before claim_dispatch, feedback reports J2 position exceeds upper limit 0.5
        core2.receive(1, JointSample(0.6, 0.0, 0.0), HEALTHY_STOP)
        # claim_dispatch must fail, latch position_limit fault, and deny write
        claimed = core2.claim_dispatch(plan.plan_id, 0)
        self.assertFalse(claimed)
        joint2_fault = next(
            fault for fault in core2.snapshot().faults if fault.motor_id == self.specs[1].motor_id
        )
        self.assertTrue(joint2_fault.position_limit)
        self.assertIs(core2.snapshot().state, State.FAULT)

    def test_unhealthy_outside_feedback_preserves_both_fault_causes(self):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertTrue(core.begin_check().accepted)
        unhealthy = MotorHealth(mode=0, errors=0x04, warnings=2, timeout=True)

        decision = core.receive(0, JointSample(2.5, 0.0, 0.0), unhealthy)

        self.assertEqual(decision.code, DecisionCode.POSITION_OUT_OF_BOUNDS)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))
        fault = core.snapshot().faults[0]
        self.assertEqual(fault.errors, 0x04)
        self.assertEqual(fault.warnings, 2)
        self.assertTrue(fault.timeout)
        self.assertTrue(fault.position_limit)

    def test_attributed_fault_and_recovery(self):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("recovery-tester")
        self.assertTrue(core.begin_check().accepted)
        # Induce position_limit fault on J1
        core.receive(0, JointSample(3.0, 0.0, 0.0), HEALTHY_STOP)
        self.assertTrue(core.snapshot().faults[0].position_limit)
        self.assertFalse(hasattr(core.snapshot().faults[0], "position_out_of_bounds"))

        # Submitting new target does NOT clear fault
        w = station.issue_window(1.0)
        t = Target(1, (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)), window=w, owner=owner)
        dec = station.submit(t)
        self.assertFalse(dec.accepted)
        self.assertTrue(core.snapshot().faults[0].position_limit)

        # arm fails while faulted
        self.assertFalse(station.arm(owner).accepted)

        # Recovery attempt with measured position still out of bounds fails
        self.assertTrue(station.stop().accepted)
        owner2 = station.claim("recovery-tester-2")
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.assertFalse(core.receive(0, JointSample(3.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertFalse(core.checked().accepted)

        # Proper recovery with acknowledged stop check inside bounds succeeds
        self.assertTrue(station.stop().accepted)
        owner3 = station.claim("recovery-tester-3")
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.assertTrue(core.receive(0, self.valid_sample_j1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(
            1, JointSample(self.specs[1].q_max_rad, 0.0, 0.0), HEALTHY_STOP
        ).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner3).accepted)
        self.assertEqual(core.snapshot().faults, ())
        self.assertFalse(core.snapshot().fault_latched)

    def test_codec_rounding_overshoot_produces_inward_actual_safe_endpoint(self):
        spec = JointSpec("J1", 0, JointMap(1.0, 1, 0.0), -0.1, 0.1, 10.0, "fixture: rounding overshoot test")
        target = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)

        requested_output = spec.mapping.to_output(target)
        requested_raw = position_to_raw(requested_output.q_rad)
        self.assertEqual(requested_raw, 6606)
        self.assertGreater(raw_to_position(requested_raw), spec.q_max_rad)

        code, actual_target, packet = prepare_reference_joint(spec, target)
        self.assertIsNone(code)
        self.assertIsNotNone(actual_target)
        self.assertIsNotNone(packet)

        # Actual reference must be strictly <= spec.q_max_rad
        self.assertLessEqual(actual_target.q_rad, spec.q_max_rad)
        self.assertGreaterEqual(actual_target.q_rad, spec.q_min_rad)

        dec = decode_command(packet)
        self.assertEqual(dec["raw_position"], 6605)
        self.assertLess(raw_to_position(dec["raw_position"]), spec.q_max_rad)
        self.assertEqual(dec["position_rad"], raw_to_position(dec["raw_position"]))

        requested = decode_command(command(
            spec.motor_id,
            mode=1,
            rpm=requested_output.rpm,
            kd=requested_output.kd,
            torque=requested_output.tau_nm,
            watchdog=True,
            position=requested_output.q_rad,
            kp=requested_output.kp,
        ))
        for field in ("raw_rpm", "raw_torque", "raw_kp", "raw_kd"):
            self.assertEqual(dec[field], requested[field])
        self.assertEqual(actual_target.q_rad, spec.mapping.from_output(
            OutputSample(dec["position_rad"], dec["rpm"], dec["torque_nm"])
        ).q_rad)
        self.assertEqual(packet, command_with_raw_position(
            spec.motor_id,
            6605,
            mode=1,
            rpm=requested_output.rpm,
            kd=requested_output.kd,
            torque=requested_output.tau_nm,
            watchdog=True,
            kp=requested_output.kp,
        ))

    def test_repeated_safe_refresh_preserves_canonical_packets_and_inward_reference(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        # Use codec-grid goals and a positive cadence with representable motion.
        endpoint_j1 = raw_to_position(position_to_raw(1.0))
        endpoint_j2 = raw_to_position(position_to_raw(0.4))
        target = Target(
            1,
            (
                JointTarget(endpoint_j1, 0.0, 0.0, 1.0, 0.1),
                JointTarget(endpoint_j2, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)

        # Multiple refreshes within window emit canonical packets consistently
        for step in range(5):
            self.clock.value += 0.01
            station.writes.clear()
            plan = station.refresh()
            self.assertIsNotNone(plan)
            self.assertEqual(len(station.writes), 2)
            self.assertEqual(station.writes, list(plan.packets))
            self.assertEqual(core.snapshot().last_sequence, 1)
            self.assertIs(core.snapshot().state, State.MOVING)

    def test_im6014_codec_decode_command_and_helpers(self):
        pkt = command(
            2,
            mode=1,
            position=1.5,
            rpm=10.0,
            torque=0.5,
            kp=5.0,
            kd=1.0,
            watchdog=True,
        )
        dec = decode_command(pkt)
        self.assertEqual(dec["motor_id"], 2)
        self.assertEqual(dec["mode"], 1)
        self.assertTrue(dec["watchdog"])
        self.assertEqual(dec["raw_position"], position_to_raw(1.5))
        self.assertAlmostEqual(dec["position_rad"], 1.5, delta=1e-4)
        self.assertEqual(dec["raw_rpm"], 135)
        self.assertEqual(dec["rpm"], 135 / 64 / RATIO * 60)
        self.assertEqual(dec["raw_torque"], 101)
        self.assertEqual(dec["torque_nm"], 101 / 2560 * RATIO)

        # Corrupt CRC
        corrupt = bytearray(pkt)
        corrupt[-1] ^= 0xFF
        with self.assertRaises(ValueError):
            decode_command(bytes(corrupt))

        # Invalid length
        with self.assertRaises(ValueError):
            decode_command(pkt[:19])

        # Stop commands are valid canonical packets.
        stop_pkt = command(2, mode=0)
        decoded_stop = decode_command(stop_pkt)
        self.assertEqual(decoded_stop["mode"], 0)
        self.assertEqual(decoded_stop["raw_position"], 0)
        no_watchdog_stop = decode_command(command(2, mode=0, watchdog=False))
        self.assertEqual(no_watchdog_stop["mode"], 0)
        self.assertFalse(no_watchdog_stop["watchdog"])

        for byte_index, value in ((0, 0), (1, 0), (3, 1)):
            malformed = bytearray(pkt)
            malformed[byte_index] = value
            with self.subTest(byte_index=byte_index), self.assertRaises(ValueError):
                decode_command(_with_command_crc(malformed))

        for field, update in (
            ("reserved mode bits", lambda raw: raw.__setitem__(2, raw[2] | 0x20)),
            ("invalid individual ID", lambda raw: raw.__setitem__(2, (raw[2] & 0xF0) | 0x0F)),
            ("missing FOC watchdog", lambda raw: raw.__setitem__(2, raw[2] & 0x7F)),
        ):
            malformed = bytearray(pkt)
            update(malformed)
            with self.subTest(field=field), self.assertRaises(ValueError):
                decode_command(_with_command_crc(malformed))

        malformed_stop = bytearray(stop_pkt)
        malformed_stop[4] = 1
        with self.assertRaises(ValueError):
            decode_command(_with_command_crc(malformed_stop))

        # Raw conversions
        self.assertEqual(position_to_raw(raw_to_position(0)), 0)
        self.assertEqual(position_to_raw(raw_to_position(100)), 100)
        self.assertEqual(position_to_raw(raw_to_position(-100)), -100)
        self.assertEqual(position_to_raw(raw_to_position(RAW_POSITION_MAX)), RAW_POSITION_MAX)
        self.assertEqual(position_to_raw(raw_to_position(RAW_POSITION_MIN)), RAW_POSITION_MIN)

    def test_decoder_accepts_canonical_input_endpoints_and_rejects_raw_values_outside_them(self):
        for position, rpm, torque, kp, kd in (
            (POSITION_MIN, RPM_MIN, -3.5, 0.0, 0.0),
            (POSITION_MAX, RPM_MAX, 3.5, 30.0, 6.0),
        ):
            with self.subTest(position=position, rpm=rpm, torque=torque, kp=kp, kd=kd):
                packet = command(
                    0, mode=1, position=position, rpm=rpm,
                    torque=torque, kp=kp, kd=kd, watchdog=True,
                )
                decoded = decode_command(packet)
                self.assertEqual(decoded["raw"], packet.hex())
                expected_position_raw = RAW_POSITION_MIN if position == POSITION_MIN else RAW_POSITION_MAX
                expected_rpm_raw = -32768 if rpm == RPM_MIN else 32767
                self.assertEqual(decoded["raw_position"], expected_position_raw)
                self.assertEqual(decoded["raw_rpm"], expected_rpm_raw)
                if kd == 6.0:
                    self.assertGreater(decoded["kd"], 6.0)

        endpoint_fields = (
            ("feedforward", 4, command(0, mode=1, torque=-3.5), command(0, mode=1, torque=3.5)),
            ("kp", 12, command(0, mode=1, kp=0.0), command(0, mode=1, kp=30.0)),
            ("kd", 14, command(0, mode=1, kd=0.0), command(0, mode=1, kd=6.0)),
        )
        for field, offset, low_packet, high_packet in endpoint_fields:
            low_raw = struct.unpack_from("<h", low_packet, offset)[0]
            high_raw = struct.unpack_from("<h", high_packet, offset)[0]
            invalid_raw_values = (
                (f"{field} below image", low_raw - 1),
                (f"{field} at raw minimum", -32768),
                (f"{field} above image", high_raw + 1),
                (f"{field} at raw maximum", 32767),
            )
            for name, raw_value in invalid_raw_values:
                malformed = bytearray(command(0, mode=1))
                struct.pack_into("<h", malformed, offset, raw_value)
                with self.subTest(name=name), self.assertRaises(ValueError):
                    decode_command(_with_command_crc(malformed))

    def test_actual_reference_matches_every_canonical_wire_field(self):
        mapping = JointMap(3.0, -1, 0.25)
        spec = JointSpec(
            "J1", 0, mapping, -2.0, 2.0, 10.0, "fixture: canonical reference test"
        )
        requested = JointTarget(0.321, 0.123, 0.321, 1.234, 0.123)

        code, actual, packet = prepare_reference_joint(spec, requested)

        self.assertIsNone(code)
        self.assertIsNotNone(actual)
        decoded = decode_command(packet)
        expected = mapping.from_output(OutputSample(
            decoded["position_rad"], decoded["rpm"], decoded["torque_nm"]
        ))
        self.assertEqual(actual.q_rad, expected.q_rad)
        self.assertEqual(actual.dq_rad_s, expected.dq_rad_s)
        self.assertEqual(actual.tau_nm, expected.tau_nm)
        self.assertEqual(actual.kp, decoded["kp"] * mapping.reduction ** 2)
        self.assertEqual(actual.kd, decoded["kd"] * mapping.reduction ** 2)

    def test_unencodable_mapped_request_is_invalid_not_projected_into_range(self):
        spec = JointSpec(
            "J1", 0, JointMap(1.0, 1, 0.0), -1e8, 1e8, 1e8,
            "fixture: wide software position limits",
        )
        code, actual, packet = prepare_reference_joint(
            spec, JointTarget(1e7, 0.0, 0.0, 1.0, 0.1)
        )
        self.assertIs(code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(actual)
        self.assertIsNone(packet)

    def test_mapping_overflow_is_invalid_argument(self):
        spec = JointSpec(
            "J1", 0, JointMap(1e-308, 1, 0.0), -1.0, 1.0, 10.0,
            "fixture: mapping overflow limits",
        )
        code, actual, packet = prepare_reference_joint(
            spec, JointTarget(0.0, 0.0, 0.0, 0.0, 0.0)
        )
        self.assertIs(code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(actual)
        self.assertIsNone(packet)

    def test_refresh_plan_requires_exact_count_canonical_packet_bytes(self):
        core, station, owner = self._armed_station()
        target = Target(
            1,
            (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
             JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)),
            window=station.issue_window(1.0),
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(len(plan.packets), len(plan.joint_targets))
        self.assertTrue(all(type(packet) is bytes for packet in plan.packets))

        with self.assertRaises(TypeError):
            RefreshPlan(
                plan.plan_id, target, plan.prepared_at_s, plan.joint_targets
            )
        with self.assertRaises(ValueError):
            replace(plan, packets=plan.packets[:-1])
        with self.assertRaises(ValueError):
            replace(plan, packets=list(plan.packets))
        with self.assertRaises(ValueError):
            replace(
                plan,
                packets=tuple(command(spec.motor_id, mode=0) for spec in self.specs),
            )

    def test_prepare_failure_only_attributes_the_joint_with_position_failure(self):
        core, station, owner = self._armed_station()
        target = Target(
            1,
            (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
             JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)),
            window=station.issue_window(1.0),
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        self.clock.value += 0.05

        with patch(
            "joint_controller.prepare_complete_reference",
            return_value=ReferenceBatchResult(
                None,
                None,
                None,
                (ReferenceFailure(DecisionCode.POSITION_OUT_OF_BOUNDS, 1),),
            ),
        ):
            self.assertIsNone(core.prepare_refresh())

        snapshot = core.snapshot()
        self.assertEqual(snapshot.affected_joints, ("J2",))
        self.assertTrue(snapshot.faults[0].position_limit)

    def test_generic_prepare_failure_does_not_mislabel_every_joint_position_limited(self):
        core, station, owner = self._armed_station()
        target = Target(
            1,
            (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
             JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)),
            window=station.issue_window(1.0),
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        self.clock.value += 0.05

        with patch(
            "joint_controller.prepare_complete_reference",
            return_value=ReferenceBatchResult(
                None,
                None,
                None,
                (ReferenceFailure(DecisionCode.INVALID_ARGUMENT, None),),
            ),
        ):
            self.assertIsNone(core.prepare_refresh())

        snapshot = core.snapshot()
        self.assertEqual(snapshot.state, State.STOPPED)
        self.assertFalse(snapshot.fault_latched)
        self.assertEqual(snapshot.faults, ())
        self.assertIsNone(snapshot.admitted_target)

    def test_owner_matching_uses_exact_type_and_boundary_records_reject_subclasses(self):
        class EqualAnyOwner(OwnerToken):
            def __eq__(self, other):
                return True

        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("owner-test")
        self.assertTrue(station.check().accepted)
        impostor = EqualAnyOwner("wrong-epoch", owner.generation, "other-client")

        self.assertTrue(owner_tokens_match(owner, OwnerToken(
            owner.epoch, owner.generation, owner.connection_id
        )))
        self.assertFalse(owner_tokens_match(impostor, owner))
        self.assertEqual(core.arm(impostor).code, DecisionCode.INVALID_OWNER)

        valid_joint = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        window = IntentWindow(1, 0.0, 1.0)
        with self.assertRaises(ValueError):
            Target(1, (valid_joint,), window, impostor)
        with self.assertRaises(ValueError):
            JointDispatchedRecord(1, valid_joint, 0.0, impostor)
        with self.assertRaises(ValueError):
            JointDispatchAttempt(
                1, 1, valid_joint, WriteResult.COMPLETE, 0.0, impostor
            )

        for field in ("latest_owner", "current_owner"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                Snapshot(
                    State.DISCONNECTED,
                    ("J1",),
                    (JointStatus(),),
                    None,
                    False,
                    (None,),
                    **{field: impostor},
                )

        fresh = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        self.assertEqual(fresh.arm(impostor).code, DecisionCode.INVALID_STATE)

    def test_late_failed_outcomes_settle_after_disconnect_and_allow_new_check(self):
        for result in (
            WriteResult.FAILED,
            WriteResult.PARTIAL,
            WriteResult.PREWRITE_DEADLINE,
        ):
            with self.subTest(result=result):
                clock = DeterministicClock(0.0)
                spec = JointSpec(
                    "J1", 0, JointMap(1.0, 1, 0.0), -2.0, 2.0, 10.0,
                    "fixture: late receipt limits",
                )
                core = JointCore((spec,), clock=clock, max_reference_gap_s=1.0)
                station = FakeStation(core)
                self.assertTrue(station.connect().accepted)
                owner = station.claim("late-receipt")
                self.assertTrue(station.check().accepted)
                self.assertTrue(station.arm(owner).accepted)

                first = Target(
                    1,
                    (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),),
                    station.issue_window(1.0),
                    owner,
                )
                self.assertTrue(station.submit(first).accepted)
                clock.value += 0.05
                first_plan = core.prepare_refresh()
                self.assertTrue(core.claim_dispatch(first_plan.plan_id, 0))
                self.assertTrue(core.dispatch_result(
                    first_plan.plan_id, 0, WriteResult.COMPLETE
                ).accepted)

                second = Target(
                    2,
                    (JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),),
                    station.issue_window(1.0),
                    owner,
                )
                self.assertTrue(station.submit(second).accepted)
                clock.value += 0.05
                second_plan = core.prepare_refresh()
                self.assertTrue(core.claim_dispatch(second_plan.plan_id, 0))
                self.assertTrue(core.disconnect().accepted)
                self.assertFalse(core.begin_connect().accepted)

                receipt = core.dispatch_result(second_plan.plan_id, 0, result)
                self.assertFalse(receipt.accepted)
                snapshot = core.snapshot()
                self.assertIs(snapshot.state, State.DISCONNECTED)
                self.assertIsNone(snapshot.source)
                self.assertIsNone(snapshot.current_owner)
                self.assertEqual(snapshot.dispatched_records[0].sequence, 1)
                self.assertEqual(snapshot.dispatch_attempts[0].sequence, 2)
                self.assertIs(snapshot.dispatch_attempts[0].result, result)
                self.assertTrue(snapshot.fault_latched)

                self.assertTrue(station.connect().accepted)
                new_owner = station.claim("new-owner")
                self.assertNotEqual(owner.generation, new_owner.generation)
                self.assertTrue(station.check(acknowledge_fault=True).accepted)
                self.assertTrue(station.arm(new_owner).accepted)
                self.assertIs(core.snapshot().state, State.ARMED)

    def test_complete_write_with_bad_end_clock_after_disconnect_keeps_full_record(self):
        class EndClock(DeterministicClock):
            fail = False

            def __call__(self):
                self.calls += 1
                return math.nan if self.fail else self.value

        clock = EndClock(0.0)
        spec = JointSpec(
            "J1", 0, JointMap(1.0, 1, 0.0), -2.0, 2.0, 10.0,
            "fixture: late complete limits",
        )
        core = JointCore((spec,), clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("late-complete")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)
        target = Target(
            1,
            (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),),
            station.issue_window(1.0),
            owner,
        )
        self.assertTrue(station.submit(target).accepted)
        clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        self.assertTrue(core.disconnect().accepted)

        clock.fail = True
        receipt = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)
        self.assertFalse(receipt.accepted)
        clock.fail = False
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.DISCONNECTED)
        self.assertIsNone(snapshot.source)
        self.assertEqual(snapshot.dispatch_attempts[0].result, WriteResult.COMPLETE)
        self.assertEqual(snapshot.dispatched_records[0].sequence, 1)
        self.assertEqual(
            snapshot.faults[0].communication_error, CommunicationFailure.CLOCK_FAILURE
        )

        self.assertTrue(station.connect().accepted)
        new_owner = station.claim("late-complete-new-owner")
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        self.assertTrue(station.arm(new_owner).accepted)
        self.assertIs(core.snapshot().state, State.ARMED)

    def test_seeded_extreme_mappings_keep_reference_and_packet_congruent(self):
        rng = random.Random(9009)
        for index in range(40):
            reduction = 10 ** rng.uniform(-6, 6)
            direction = rng.choice((-1, 1))
            zero = rng.uniform(-2.5, 2.5)
            output_q = rng.uniform(-1.0, 1.0)
            q = direction * (output_q - zero) / reduction
            half_width = abs(raw_to_position(4)) / reduction
            spec = JointSpec(
                f"J{index}",
                index % 15,
                JointMap(reduction, direction, zero),
                q - half_width,
                q + half_width,
                1e8,
                "fixture: seeded software limits",
            )
            output_velocity = rng.uniform(-0.5, 0.5)
            output_torque = rng.uniform(-0.1, 0.1)
            requested = JointTarget(
                q,
                direction * output_velocity / reduction,
                direction * output_torque * reduction,
                2.0 * reduction ** 2,
                0.3 * reduction ** 2,
            )

            code, actual, packet = prepare_reference_joint(spec, requested)

            with self.subTest(index=index, reduction=reduction, direction=direction):
                self.assertIsNone(code)
                self.assertIsNotNone(actual)
                self.assertIsNotNone(packet)
                decoded = decode_command(packet)
                self.assertGreaterEqual(actual.q_rad, spec.q_min_rad)
                self.assertLessEqual(actual.q_rad, spec.q_max_rad)
                expected = spec.mapping.from_output(OutputSample(
                    decoded["position_rad"], decoded["rpm"], decoded["torque_nm"]
                ))
                self.assertEqual(actual.q_rad, expected.q_rad)
                self.assertEqual(actual.dq_rad_s, expected.dq_rad_s)
                self.assertEqual(actual.tau_nm, expected.tau_nm)
                self.assertEqual(actual.kp, decoded["kp"] * reduction ** 2)
                self.assertEqual(actual.kd, decoded["kd"] * reduction ** 2)
                self.assertEqual(
                    decoded["raw_position"], self._project(spec, requested.q_rad)
                )

                mapped = spec.mapping.to_output(requested)
                request_decoded = decode_command(command(
                    spec.motor_id,
                    mode=1,
                    rpm=mapped.rpm,
                    kd=mapped.kd,
                    torque=mapped.tau_nm,
                    watchdog=True,
                    position=mapped.q_rad,
                    kp=mapped.kp,
                ))
                for field in ("raw_rpm", "raw_torque", "raw_kp", "raw_kd"):
                    self.assertEqual(decoded[field], request_decoded[field])



class ReferenceSpeedTests(unittest.TestCase):
    def setUp(self):
        self.clock = DeterministicClock(10.0)
        self.spec1 = JointSpec(
            "J1",
            0,
            JointMap(1.0, 1, 0.0),
            -2.0,
            2.0,
            1.0,  # v_max_rad_s = 1.0 rad/s
            "fixture: speed limit test J1",
        )
        self.spec2 = JointSpec(
            "J2",
            1,
            JointMap(1.0, 1, 0.0),
            -1.0,
            1.0,
            2.0,  # v_max_rad_s = 2.0 rad/s
            "fixture: speed limit test J2",
        )
        self.specs = (self.spec1, self.spec2)
        self.sample1 = JointSample(0.0, 0.0, 0.0)
        self.sample2 = JointSample(0.0, 0.0, 0.0)

    def _armed_station(self, max_reference_gap_s=1.0):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=max_reference_gap_s)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("speed-tester")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, owner

    def assert_reference_speed_certificate(self, previous, current, spec):
        h = current.sample_time_s - previous.sample_time_s
        self.assertGreater(h, 0.0)
        exact_position_step_velocity = (
            Fraction(current.q_rad) - Fraction(previous.q_rad)
        ) / Fraction(h)
        self.assertEqual(
            current.position_step_velocity_rad_s,
            exact_position_step_velocity,
        )
        self.assertIsInstance(current.position_step_velocity_rad_s, Fraction)
        exact_speed_limit = Fraction(spec.v_max_rad_s)
        self.assertLessEqual(abs(exact_position_step_velocity), exact_speed_limit)
        self.assertLessEqual(
            abs(Fraction(current.encoded_velocity_rad_s)), exact_speed_limit
        )

    def test_checked_sample_initializes_unemitted_stop_reference(self):
        core, station, owner = self._armed_station()
        snapshot = core.snapshot()
        self.assertEqual(len(snapshot.confirmed_references), 2)
        ref0 = snapshot.confirmed_references[0]
        self.assertIsNotNone(ref0)
        self.assertEqual(ref0.q_rad, 0.0)
        self.assertEqual(ref0.position_step_velocity_rad_s, 0.0)
        self.assertEqual(ref0.encoded_velocity_rad_s, 0.0)
        self.assertEqual(ref0.sample_time_s, 10.0)
        self.assertFalse(ref0.is_emitted)

    def test_zero_elapsed_on_fresh_stop_anchor_cannot_dispatch_write(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        target = Target(
            1,
            (
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        # clock has NOT advanced (h == 0) from checked anchor
        self.assertIsNone(core.prepare_refresh())
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.FAULT)
        self.assertTrue(snapshot.faults[0].constraint_fault)

    def test_zero_elapsed_repeated_refresh_emits_unchanged_command(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        target1 = Target(
            1,
            (
                JointTarget(0.05, 0.25, 0.15, 1.2, 0.3),
                JointTarget(0.05, 0.25, 0.15, 1.2, 0.3),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target1).accepted)
        station.writes.clear()
        self.clock.value += 0.05
        plan1 = station.refresh()
        self.assertIsNotNone(plan1)
        self.assertEqual(len(station.writes), 2)
        first_packets = list(station.writes)
        first_reference = core.snapshot().confirmed_references[0]
        self.assertNotEqual(first_reference.q_rad, target1.joints[0].q_rad)

        # Without advancing clock (h == 0), refresh again with same target
        station.writes.clear()
        plan2 = station.refresh()
        self.assertIsNotNone(plan2)
        self.assertEqual(station.writes, first_packets)
        self.assertEqual(plan2.packets, plan1.packets)
        self.assertEqual(plan2.joint_targets[0].q_rad, plan1.joint_targets[0].q_rad)
        self.assertEqual(core.snapshot().confirmed_references[0], first_reference)

        # But changing target at h == 0 cannot jump
        target2 = Target(
            2,
            (
                JointTarget(0.1, 0.25, 0.2, 1.2, 0.3),
                JointTarget(0.1, 0.25, 0.2, 1.2, 0.3),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target2).accepted)
        station.writes.clear()
        self.assertIsNone(station.refresh())
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(station.writes, [])
        self.assertEqual(core.snapshot().confirmed_references[0], first_reference)

    def test_fixed_and_varied_h_progression_within_speed_bound(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        # Large goal demanding maximum reference speed
        target = Target(
            1,
            (
                JointTarget(1.5, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.8, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)

        previous_ref = core.snapshot().confirmed_references[0]
        self.clock.value += 0.05
        plan1 = station.refresh()
        self.assertIsNotNone(plan1)
        ref1 = core.snapshot().confirmed_references[0]
        self.assertGreater(ref1.q_rad, 0.04)
        self.assert_reference_speed_certificate(previous_ref, ref1, self.spec1)

        previous_ref = ref1
        self.clock.value += 0.02
        plan2 = station.refresh()
        self.assertIsNotNone(plan2)
        ref2 = core.snapshot().confirmed_references[0]
        step2 = ref2.q_rad - previous_ref.q_rad
        self.assertGreater(step2, 0.015)
        self.assert_reference_speed_certificate(previous_ref, ref2, self.spec1)

        previous_ref = ref2
        self.clock.value += 0.1
        plan3 = station.refresh()
        self.assertIsNotNone(plan3)
        ref3 = core.snapshot().confirmed_references[0]
        step3 = ref3.q_rad - previous_ref.q_rad
        self.assertGreater(step3, 0.09)
        self.assert_reference_speed_certificate(previous_ref, ref3, self.spec1)

    def test_positive_negative_and_reversal_bounded(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        # Move positive toward 0.5
        t_pos = Target(
            1,
            (
                JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(t_pos).accepted)
        previous_ref = core.snapshot().confirmed_references[0]
        self.clock.value += 0.05
        self.assertIsNotNone(station.refresh())
        ref_pos = core.snapshot().confirmed_references[0]
        self.assertGreater(ref_pos.position_step_velocity_rad_s, 0.0)
        self.assert_reference_speed_certificate(previous_ref, ref_pos, self.spec1)

        # Reverse toward negative -0.5
        t_neg = Target(
            2,
            (
                JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(t_neg).accepted)
        previous_ref = ref_pos
        self.clock.value += 0.05
        self.assertIsNotNone(station.refresh())
        ref_rev = core.snapshot().confirmed_references[0]
        self.assertLess(ref_rev.position_step_velocity_rad_s, 0.0)
        self.assert_reference_speed_certificate(previous_ref, ref_rev, self.spec1)

    def test_reference_gap_exact_bound_and_exceeded(self):
        core, station, owner = self._armed_station(max_reference_gap_s=0.125)
        window = station.issue_window(1.0)
        _, canonical_j1, _ = prepare_reference_joint(
            self.spec1, JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        )
        _, canonical_j2, _ = prepare_reference_joint(
            self.spec2, JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        )
        self.assertIsNotNone(canonical_j1)
        self.assertIsNotNone(canonical_j2)
        target = Target(
            1,
            (
                canonical_j1,
                canonical_j2,
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)

        # The binary-exact cadence is exactly max_reference_gap_s.
        self.clock.value = 10.125
        self.assertIsNotNone(station.refresh())
        self.assertIs(core.snapshot().state, State.MOVING)

        # Move the absolute sample time one float step beyond an exact second interval.
        self.clock.value = math.nextafter(10.25, math.inf)
        self.assertIsNone(station.refresh())
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.STOPPED)
        self.assertIsNone(snapshot.admitted_target)

    def test_invalid_and_nonfinite_dt_fails_closed(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        target = Target(
            1,
            (
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)

        # Clock goes backwards (negative dt)
        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertIs(core.snapshot().prepared_plan, plan)
        confirmed_before = core.snapshot().confirmed_references
        station.writes.clear()
        self.clock.value -= 0.1
        with self.assertRaises(ValueError):
            core.prepare_refresh()
        self.clock.value = plan.prepared_at_s
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.STOPPED)
        self.assertIsNone(snapshot.admitted_target)
        self.assertIsNone(snapshot.prepared_plan)
        self.assertIsNone(snapshot.current_owner)
        self.assertEqual(snapshot.confirmed_references, confirmed_before)
        self.assertEqual(snapshot.last_sequence, 1)
        self.assertEqual(station.writes, [])
        self.assertIsNone(core.prepare_refresh())

        # Non-finite clock
        self.clock.value = plan.prepared_at_s
        core2, station2, owner2 = self._armed_station()
        window2 = station2.issue_window(1.0)
        target2 = Target(
            1,
            (
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window2,
            owner=owner2,
        )
        self.assertTrue(station2.submit(target2).accepted)
        confirmed_before2 = core2.snapshot().confirmed_references
        station2.writes.clear()
        self.clock.value = float("nan")
        with self.assertRaises(ValueError):
            core2.prepare_refresh()
        self.clock.value = plan.prepared_at_s
        snapshot2 = core2.snapshot()
        self.assertIs(snapshot2.state, State.STOPPED)
        self.assertIsNone(snapshot2.admitted_target)
        self.assertIsNone(snapshot2.prepared_plan)
        self.assertIsNone(snapshot2.current_owner)
        self.assertEqual(snapshot2.confirmed_references, confirmed_before2)
        self.assertEqual(snapshot2.last_sequence, 1)
        self.assertEqual(station2.writes, [])
        self.assertIsNone(core2.prepare_refresh())

    def test_preparing_and_unwritten_vector_cannot_advance_history(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        target = Target(
            1,
            (
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)

        init_ref = core.snapshot().confirmed_references[0]
        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # Preparing did NOT advance confirmed references
        ref_after_prep = core.snapshot().confirmed_references[0]
        self.assertEqual(init_ref, ref_after_prep)

        # Unwritten vector dropped; advance clock further
        self.clock.value += 0.05
        plan2 = core.prepare_refresh()
        self.assertIsNotNone(plan2)
        # History is still the initial confirmed reference
        self.assertEqual(core.snapshot().confirmed_references[0], init_ref)

    def test_first_joint_full_write_and_second_failure_commits_only_first(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        target = Target(
            1,
            (
                JointTarget(0.05, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.05, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        ref0_init = core.snapshot().confirmed_references[0]
        ref1_init = core.snapshot().confirmed_references[1]

        # Joint 0 completes write
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        self.assertTrue(core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE).accepted)

        # Joint 0 confirmed reference is committed
        ref0_committed = core.snapshot().confirmed_references[0]
        self.assertNotEqual(ref0_committed, ref0_init)
        self.assertEqual(ref0_committed.sample_time_s, plan.prepared_at_s)

        # Joint 1 fails write
        self.assertTrue(core.claim_dispatch(plan.plan_id, 1))
        failure = core.dispatch_result(plan.plan_id, 1, WriteResult.FAILED)
        self.assertFalse(failure.accepted)
        self.assertIs(failure.code, DecisionCode.FAULT_LATCHED)

        # The failed write is recorded and faults J2, without advancing its history.
        snapshot = core.snapshot()
        self.assertEqual(snapshot.confirmed_references[0], ref0_committed)
        self.assertEqual(snapshot.confirmed_references[1], ref1_init)
        self.assertEqual(snapshot.dispatch_attempts[1].result, WriteResult.FAILED)
        self.assertIs(snapshot.state, State.FAULT)
        self.assertIsNone(snapshot.admitted_target)
        self.assertIsNone(snapshot.current_owner)
        self.assertEqual(len(snapshot.faults), 1)
        self.assertEqual(snapshot.faults[0].motor_id, self.specs[1].motor_id)
        self.assertIs(
            snapshot.faults[0].communication_error,
            CommunicationFailure.DRIVER_FAILURE,
        )

    def test_quantization_stall_on_nonzero_objective_vs_deliberate_hold(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)

        # Case A: Deliberate hold (goal matches prev q, zero velocity)
        hold_target = Target(
            1,
            (
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(hold_target).accepted)
        self.clock.value += 0.001
        hold_plan = station.refresh()
        self.assertIsNotNone(hold_plan)
        self.assertIs(core.snapshot().state, State.MOVING)

        # Case B: Nonzero objective with too-small cadence (quantization stall)
        stall_target = Target(
            2,
            (
                JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(stall_target).accepted)
        self.clock.value += 1e-6  # Tiny cadence where v_max * h < 1 raw tick
        stall_plan = station.refresh()
        self.assertIsNone(stall_plan)
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.FAULT)
        self.assertTrue(snapshot.faults[0].constraint_fault)

    def test_dual_stream_velocities_independent_and_both_bounded(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(1.0)
        target = Target(
            1,
            (
                JointTarget(0.08, 0.2, 0.0, 1.0, 0.1),
                JointTarget(0.05, 0.1, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        self.clock.value += 0.05
        plan = station.refresh()
        self.assertIsNotNone(plan)

        ref0 = core.snapshot().confirmed_references[0]
        # Verify both independent streams are exactly within spec V (1.0 rad/s).
        initial_ref = JointTrajectoryReference(0.0, Fraction(0), 0.0, 10.0, False)
        self.assert_reference_speed_certificate(initial_ref, ref0, self.spec1)
        # Verify they are tracked as separate explicit values
        self.assertIsInstance(ref0.position_step_velocity_rad_s, Fraction)
        self.assertIsInstance(ref0.encoded_velocity_rad_s, float)

    def test_zero_objective_cancellation_is_not_quantization_stall(self):
        spec = JointSpec(
            "J1", 0, JointMap(1.0, 1, 0.0), -1.0, 1.0, 1.0,
            "fixture: zero-objective cancellation test",
        )
        anchor = JointTrajectoryReference(0.0, Fraction(0), 0.0, 0.0, False)
        target = JointTarget(0.1, -1.0, 0.0, 1.0, 0.1)

        code, emitted, reference, packet = shape_reference_joint(
            spec, target, anchor, 0.1, 1.0
        )

        self.assertIsNone(code)
        self.assertIsNotNone(emitted)
        self.assertIsNotNone(reference)
        self.assertIsNotNone(packet)
        self.assertEqual(emitted.q_rad, 0.0)
        self.assertEqual(reference.position_step_velocity_rad_s, Fraction(0))

    def test_finite_extreme_objective_uses_exact_projection_without_overflow(self):
        spec = JointSpec(
            "J1", 0, JointMap(1.0, 1, 0.0), -1.0, 1.0, 1.5e308,
            "fixture: finite extreme objective",
        )
        anchor = JointTrajectoryReference(-1.0, Fraction(0), 0.0, 0.0, False)
        target = JointTarget(1.0, 0.0, 0.0, 1.0, 0.1)

        code, emitted, reference, packet = shape_reference_joint(
            spec, target, anchor, 1e-308, 1.0
        )

        self.assertIsNone(code)
        self.assertIsNotNone(emitted)
        self.assertIsNotNone(reference)
        self.assertIsNotNone(packet)
        self.assertGreater(reference.position_step_velocity_rad_s, 0)
        self.assertLessEqual(
            reference.position_step_velocity_rad_s, Fraction(spec.v_max_rad_s)
        )

    def test_oversized_integer_time_inputs_fail_closed_without_raising(self):
        spec = JointSpec(
            "J1", 0, JointMap(1.0, 1, 0.0), -1.0, 1.0, 1.0,
            "fixture: oversized integer time",
        )
        anchor = JointTrajectoryReference(0.0, Fraction(0), 0.0, 0.0, False)
        target = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)

        for now, gap in ((10**400, 1.0), (1.0, 10**400)):
            code, emitted, reference, packet = shape_reference_joint(
                spec, target, anchor, now, gap
            )
            self.assertIs(code, DecisionCode.INVALID_ARGUMENT)
            self.assertIsNone(emitted)
            self.assertIsNone(reference)
            self.assertIsNone(packet)

    def test_quantization_stall_fault_is_attributed_only_to_failing_joint(self):
        clock = DeterministicClock(0.0)
        specs = (
            JointSpec(
                "J1", 0, JointMap(1.0, 1, 0.0), -1.0, 1.0, 0.001,
                "fixture: tiny-cadence stall J1",
            ),
            JointSpec(
                "J2", 1, JointMap(1.0, 1, 0.0), -1.0, 1.0, 0.001,
                "fixture: tiny-cadence hold J2",
            ),
        )
        core = JointCore(specs, clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("quantization-stall-test")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        window = station.issue_window(1.0)
        target = Target(
            1,
            (
                JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        clock.value = 1e-10

        self.assertIsNone(core.prepare_refresh())
        snapshot = core.snapshot()
        self.assertEqual(snapshot.affected_joints, ("J1",))
        self.assertTrue(snapshot.faults[0].constraint_fault)

    def test_final_write_gate_rechecks_gap_and_limits(self):
        core, station, owner = self._armed_station(max_reference_gap_s=0.2)
        window = station.issue_window(0.5)
        target = Target(
            1,
            (
                JointTarget(0.05, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.05, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)
        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # Before claim_dispatch, time advances past max_reference_gap_s
        self.clock.value += 0.25
        # Gate must deny write
        self.assertFalse(core.claim_dispatch(plan.plan_id, 0))


if __name__ == "__main__":
    unittest.main()
