"""Targeted unit and acceptance tests for C09 inclusive joint position bounds."""

import math
import unittest

from im6014_codec import (
    RAW_POSITION_MAX,
    RAW_POSITION_MIN,
    command,
    decode_command,
    position_to_raw,
    raw_to_position,
)
from joint_contract import (
    DecisionCode,
    JointFault,
    JointSpec,
    MotorHealth,
    OwnerToken,
    SourceIdentity,
    State,
    Target,
    WriteResult,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget, OutputSample
from joint_reference import find_inward_position_raw, prepare_reference_joint
from test_controller import DeterministicClock, FakeStation, HEALTHY_STOP, SAMPLE

SOURCE = SourceIdentity("fixture", "limits-test", 1)


class PositionLimitTests(unittest.TestCase):
    def setUp(self):
        self.clock = DeterministicClock(0.0)
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), -1.0, 2.0, "fixture: software position limits J1"),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), -0.5, 0.5, "fixture: software position limits J2"),
        )
        self.valid_sample_j1 = JointSample(0.1, 0.0, 0.0)
        self.valid_sample_j2 = JointSample(0.0, 0.0, 0.0)

    def _armed_station(self, connection_id="tester"):
        core = JointCore(self.specs, clock=self.clock)
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

        plan = station.refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(len(station.writes), 2)
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
                "fixture: software limits negative direction",
            ),
            JointSpec(
                "J2",
                1,
                JointMap(reduction=2.0, direction=1, zero_output_rad=-0.1),
                -0.5,
                0.5,
                "fixture: software limits positive direction",
            ),
        )
        core = JointCore(specs_mapped, clock=self.clock)
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
        plan = station.refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(len(station.writes), 2)

        # Verify decoded command positions stay within bounds
        dec_pkt_0 = decode_command(station.writes[0])
        decoded_sample_0 = specs_mapped[0].mapping.from_output(
            OutputSample(dec_pkt_0["position"], 0.0, 0.0)
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
            "fixture: exact grid point interval",
        )
        found_code = find_inward_position_raw(spec_zero_width, grid_pos)
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
            "fixture: empty grid interval",
        )
        found_none = find_inward_position_raw(spec_empty_grid, mid)
        self.assertIsNone(found_none)

        # Submission on spec with no representable command is rejected
        core = JointCore((spec_empty_grid,), clock=self.clock)
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
            JointSpec("J1", 0, mapping, 1.0, 0.0, "fixture: invalid min max")

        # non-numeric / bool types
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, True, 1.0, "fixture: bool min")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, 0.0, False, "fixture: bool max")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, "0.0", 1.0, "fixture: str min")

        # infinite / NaN
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, float("-inf"), 1.0, "fixture: neg inf min")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, 0.0, float("inf"), "fixture: inf max")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, float("nan"), 1.0, "fixture: nan min")

        # invalid provenance
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, "")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, "   ")
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, 123)
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, mapping, -1.0, 1.0, "x" * 257)

        # JointFault position_limit type validation
        with self.assertRaises(ValueError):
            JointFault(position_limit="bad")
        jf = JointFault(position_limit=True)
        self.assertTrue(jf.position_limit)
        self.assertTrue(jf.position_out_of_bounds)

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
        core = JointCore(self.specs, clock=self.clock)
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
        plan = core2.prepare_refresh()
        self.assertIsNotNone(plan)

        # Before claim_dispatch, feedback reports J2 position exceeds upper limit 0.5
        core2.receive(1, JointSample(0.6, 0.0, 0.0), HEALTHY_STOP)
        # claim_dispatch must fail, latch position_limit fault, and deny write
        claimed = core2.claim_dispatch(plan.plan_id, 0)
        self.assertFalse(claimed)
        self.assertTrue(core2.snapshot().faults[1].position_limit)
        self.assertIs(core2.snapshot().state, State.FAULT)

    def test_attributed_fault_and_recovery(self):
        core = JointCore(self.specs, clock=self.clock)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("recovery-tester")
        self.assertTrue(core.begin_check().accepted)
        # Induce position_limit fault on J1
        core.receive(0, JointSample(3.0, 0.0, 0.0), HEALTHY_STOP)
        self.assertTrue(core.snapshot().faults[0].position_limit)
        self.assertTrue(core.snapshot().faults[0].position_out_of_bounds)

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
        self.assertTrue(core.receive(1, self.valid_sample_j2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner3).accepted)
        self.assertFalse(core.snapshot().faults[0].position_limit)

    def test_codec_rounding_overshoot_produces_inward_actual_safe_endpoint(self):
        # Upper bound q_max_rad = 1.0 rad on 1:1 mapping.
        # Direct float conversion of 1.0: 1.0 / (2*pi) * 1310720 = 208607.54 -> rounds to 208608.
        # raw_to_position(208608) = 1.00000220... > 1.0!
        spec = JointSpec("J1", 0, JointMap(1.0, 1, 0.0), -1.0, 1.0, "fixture: rounding overshoot test")
        target = JointTarget(1.0, 0.0, 0.0, 1.0, 0.1)

        code, actual_target, packet = prepare_reference_joint(spec, target)
        self.assertIsNone(code)
        self.assertIsNotNone(actual_target)
        self.assertIsNotNone(packet)

        # Actual reference must be strictly <= spec.q_max_rad
        self.assertLessEqual(actual_target.q_rad, spec.q_max_rad)
        self.assertGreaterEqual(actual_target.q_rad, spec.q_min_rad)

        # Packet decoded raw position corresponds to chosen inward raw code (208607)
        dec = decode_command(packet)
        self.assertEqual(dec["raw_position"], 208607)
        self.assertLessEqual(dec["position"], 1.0)

    def test_repeated_safe_refresh_preserves_canonical_packets_and_inward_reference(self):
        core, station, owner = self._armed_station()
        window = station.issue_window(2.0)
        target = Target(
            1,
            (
                JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.4, 0.0, 0.0, 1.0, 0.1),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(station.submit(target).accepted)

        # Multiple refreshes within window emit canonical packets consistently
        for step in range(5):
            self.clock.step(0.01)
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
        self.assertAlmostEqual(dec["position"], 1.5, delta=1e-4)
        self.assertAlmostEqual(dec["rpm"], 10.0, delta=1e-3)
        self.assertAlmostEqual(dec["torque"], 0.5, delta=1e-3)

        # Corrupt CRC
        corrupt = bytearray(pkt)
        corrupt[-1] ^= 0xFF
        with self.assertRaises(ValueError):
            decode_command(bytes(corrupt))

        # Invalid length
        with self.assertRaises(ValueError):
            decode_command(pkt[:19])

        # Invalid mode (mode 0 stop command)
        stop_pkt = command(2, mode=0)
        with self.assertRaises(ValueError):
            decode_command(stop_pkt)

        # Raw conversions
        self.assertEqual(position_to_raw(raw_to_position(0)), 0)
        self.assertEqual(position_to_raw(raw_to_position(100)), 100)
        self.assertEqual(position_to_raw(raw_to_position(-100)), -100)
        self.assertEqual(position_to_raw(raw_to_position(RAW_POSITION_MAX)), RAW_POSITION_MAX)
        self.assertEqual(position_to_raw(raw_to_position(RAW_POSITION_MIN)), RAW_POSITION_MIN)


if __name__ == "__main__":
    unittest.main()
