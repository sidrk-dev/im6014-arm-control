"""Deterministic boundary tests for station timing instrumentation and step coordinator.

Tests arithmetic, clock bounds (zero, negative, regressing, nonfinite), missed-slot
skipping without catch-up bursting, transaction latency and outcomes, completion
overrun attribution, bounded records, and lifecycle integration with FakeSerial.
"""

from collections import deque
import math
import struct
import time
import unittest

from im6014_codec import crc32
from im6014_serial import WriteCancelledError
from joint_contract import (
    JointSpec,
    MotorHealth,
    OwnerToken,
    ProtectionPolicy,
    SourceIdentity,
    State,
    Target,
    WriteResult,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget, OutputSample
from station_step import StationStepCoordinator, mapped_health, mapped_sample
from station_timing import (
    DEFAULT_CAPACITY,
    UINT64_MAX,
    VALID_OUTCOMES,
    JointTransactionRecord,
    LoopStepRecord,
    StationTimingConfig,
    StationTimingTracker,
    TimingCounters,
    TimingSnapshot,
    saturate_add,
    validate_finite_time,
    validate_monotonic_time,
)


class DeterministicClock:
    """Explicitly controlled clock with monotonic advance."""

    def __init__(self, start: float = 0.0) -> None:
        self.value = float(start)

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> float:
        self.value += float(seconds)
        return self.value

    def sleep(self, seconds: float) -> None:
        self.advance(seconds)


def make_feedback_frame(
    motor_id: int,
    mode: int = 0,
    timeout: bool = False,
    driver_c: int = 25,
    winding_c: int = 30,
    voltage_v: float = 36.0,
    torque_nm: float = 0.0,
    rpm: float = 0.0,
    position_rad: float = 0.0,
    errors: int = 0,
    warnings: int = 0,
) -> bytes:
    """Generate a canonical 26-byte feedback frame with valid CRC32."""
    ratio = 38.0 / 3.0
    m = (motor_id & 0x0F) | ((mode & 0x07) << 4) | ((1 if timeout else 0) << 7)
    td = int(driver_c)
    tw = int(winding_c)
    v = int(round(voltage_v * 2))
    raw_torque = int(round(torque_nm * 2560.0 / ratio))
    raw_rpm = int(round(rpm * 64.0 * ratio / 60.0))
    raw_pos = int(round(position_rad * 32768.0 * ratio / math.tau))
    flags = (warnings & 0x07) << 13

    body = struct.pack(
        "<BbBBhhiIHBB",
        m,
        td,
        tw,
        v,
        raw_torque,
        raw_rpm,
        raw_pos,
        errors,
        flags,
        0,
        0,
    )
    crc = crc32(body)
    return b"\xfc\xee" + body + struct.pack("<I", crc)


class FakeTransport:
    """Configurable synchronous serial transport for testing exchange interactions."""

    def __init__(self, responses: list[bytes] | None = None) -> None:
        self.responses: deque[bytes] = deque(responses or [])
        self.writes: list[bytes] = []
        self.timeout: float | None = 0.05
        self.write_timeout: float | None = 0.05
        self.in_waiting: int = 0
        self._current_reply: bytearray = bytearray()
        self.write_error: Exception | None = None
        self.partial_write_len: int | None = None

    def reset_input_buffer(self) -> None:
        self._current_reply.clear()
        self.in_waiting = 0

    def write(self, data: bytes) -> int:
        if self.write_error is not None:
            raise self.write_error
        self.writes.append(bytes(data))
        if self.responses:
            self._current_reply.extend(self.responses.popleft())
            self.in_waiting = len(self._current_reply)
        if self.partial_write_len is not None:
            return self.partial_write_len
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._current_reply[:size])
        del self._current_reply[:size]
        self.in_waiting = len(self._current_reply)
        return chunk


class StationTimingTests(unittest.TestCase):
    """Targeted boundary tests for C15 station timing and step adapter."""

    def setUp(self) -> None:
        self.clock = DeterministicClock(10.0)
        self.policy = ProtectionPolicy(
            driver_trip_c=85.0,
            driver_reset_c=70.0,
            winding_trip_c=105.0,
            winding_reset_c=90.0,
            continuous_effort_nm=10.0,
            continuous_dwell_s=0.5,
            recovery_dwell_s=0.25,
        )
        self.specs = (
            JointSpec(
                "J1",
                0,
                JointMap(1.0, 1, 0.0),
                -3.0,
                3.0,
                10.0,
                50.0,
                1.0,
                20.0,
                "fixture: C15 test J1",
                self.policy,
            ),
            JointSpec(
                "J2",
                1,
                JointMap(1.0, 1, 0.0),
                -3.0,
                3.0,
                10.0,
                50.0,
                1.0,
                20.0,
                "fixture: C15 test J2",
                self.policy,
            ),
        )

    # 1. Clock boundary tests
    def test_clock_zero_valid(self) -> None:
        self.assertEqual(validate_finite_time(0.0), 0.0)
        self.assertEqual(validate_monotonic_time(0.0, None), 0.0)
        tracker = StationTimingTracker(0.01)
        idx = tracker.start_cycle(0.0)
        self.assertEqual(idx, 0)
        rec = tracker.end_cycle(0.005)
        self.assertEqual(rec.started_s, 0.0)
        self.assertEqual(rec.ended_s, 0.005)

    def test_clock_negative_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "non-negative"):
            validate_finite_time(-0.001)
        with self.assertRaisesRegex(ValueError, "non-negative"):
            validate_monotonic_time(-1.0, None)

    def test_clock_regressing_rejected(self) -> None:
        tracker = StationTimingTracker(0.01)
        tracker.start_cycle(10.0)
        tracker.end_cycle(10.005)
        with self.assertRaisesRegex(ValueError, "regressed"):
            tracker.start_cycle(9.999)

    def test_clock_nonfinite_rejected(self) -> None:
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(bad=bad):
                with self.assertRaisesRegex(ValueError, "finite real number"):
                    validate_finite_time(bad)

    def test_clock_boolean_rejected(self) -> None:
        for bad in (True, False):
            with self.subTest(bad=bad):
                with self.assertRaisesRegex(ValueError, "finite real number"):
                    validate_finite_time(bad)
                with self.assertRaisesRegex(ValueError, "finite real number"):
                    validate_monotonic_time(bad)

    # 2. Arithmetic & scheduler tests
    def test_actual_period_not_copied_from_nominal(self) -> None:
        tracker = StationTimingTracker(0.010)
        # Cycle 0: started at 1.0, ends at 1.004
        tracker.start_cycle(1.0)
        rec0 = tracker.end_cycle(1.004)
        self.assertIsNone(rec0.actual_period_s)

        # Cycle 1: started at 1.012, ends at 1.016
        # Actual period is exactly 1.012 - 1.0 = 0.012 (not nominal 0.010!)
        tracker.start_cycle(1.012)
        rec1 = tracker.end_cycle(1.016)
        self.assertAlmostEqual(rec1.actual_period_s, 0.012, places=9)
        self.assertNotEqual(rec1.actual_period_s, tracker.config.nominal_period_s)

        # Snapshot keeps nominal config separate from actual statistics
        snap = tracker.snapshot()
        self.assertEqual(snap.config.nominal_period_s, 0.010)
        self.assertAlmostEqual(snap.mean_actual_period_s, 0.012, places=9)

    def test_missed_slots_skip_never_burst(self) -> None:
        tracker = StationTimingTracker(0.010)
        # Slot schedule starts at 0.0, nominal 0.010
        # If current time is 0.035, elapsed is 0.035 > 0.010
        missed, next_slot = tracker.calculate_slot_skip(0.035, 0.010)
        self.assertEqual(missed, 3)
        self.assertAlmostEqual(next_slot, 0.040, places=9)

        # Start cycle after skip
        tracker.start_cycle(0.035)
        rec = tracker.end_cycle(0.038, missed_slots=missed)
        self.assertEqual(rec.missed_slots_skipped, 3)

        snap = tracker.snapshot()
        self.assertEqual(snap.counters.missed_slots_skipped, 3)
        self.assertEqual(snap.counters.cycles, 1)  # Only 1 cycle executed, never bursted 3!

    def test_counter_saturation_fixed(self) -> None:
        self.assertEqual(saturate_add(UINT64_MAX - 2, 2), UINT64_MAX)
        self.assertEqual(saturate_add(UINT64_MAX - 2, 5), UINT64_MAX)
        self.assertEqual(saturate_add(UINT64_MAX, 1), UINT64_MAX)

        # Verify TimingCounters enforces uint64 bounds
        counters = TimingCounters(
            cycles=UINT64_MAX,
            transactions=UINT64_MAX,
            successful_transactions=0,
            failed_transactions=0,
            timeout_transactions=0,
            invalid_feedback_transactions=0,
            write_cancelled_transactions=0,
            prewrite_deadline_transactions=0,
            driver_failure_transactions=0,
            skipped_transactions=0,
            clock_failure_transactions=0,
            overruns=0,
            missed_slots_skipped=0,
        )
        self.assertEqual(counters.cycles, UINT64_MAX)

    # 3. Completion overrun and attribution
    def test_completion_overrun_attribution(self) -> None:
        tracker = StationTimingTracker(0.010)
        tracker.start_cycle(1.0)

        # Joint 1 took 0.002s (within fair share 0.005s)
        tx1 = JointTransactionRecord(
            cycle_index=0,
            joint_index=0,
            joint_name="J1",
            motor_id=0,
            plan_id=1,
            sequence=10,
            owner=None,
            write_result=WriteResult.COMPLETE,
            outcome="OK",
            started_s=1.000,
            ended_s=1.002,
            duration_s=0.002,
            feedback_received=True,
        )
        tracker.record_transaction(tx1)

        # Joint 2 took 0.012s (exceeded fair share 0.005s)
        tx2 = JointTransactionRecord(
            cycle_index=0,
            joint_index=1,
            joint_name="J2",
            motor_id=1,
            plan_id=1,
            sequence=10,
            owner=None,
            write_result=WriteResult.COMPLETE,
            outcome="OK",
            started_s=1.002,
            ended_s=1.014,
            duration_s=0.012,
            feedback_received=True,
        )
        tracker.record_transaction(tx2)

        # Loop ended at 1.015s (duration 0.015s > 0.010s)
        rec = tracker.end_cycle(1.015, transactions=(tx1, tx2))
        self.assertTrue(rec.overrun)
        self.assertAlmostEqual(rec.overrun_s, 0.005, places=9)
        self.assertEqual(rec.overrun_joints, ("J2",))

        snap = tracker.snapshot()
        self.assertEqual(snap.counters.overruns, 1)
        self.assertAlmostEqual(snap.total_overrun_s, 0.005, places=9)

    def test_no_overrun_when_within_budget(self) -> None:
        tracker = StationTimingTracker(0.010)
        tracker.start_cycle(1.0)
        tx = JointTransactionRecord(
            cycle_index=0,
            joint_index=0,
            joint_name="J1",
            motor_id=0,
            plan_id=1,
            sequence=10,
            owner=None,
            write_result=WriteResult.COMPLETE,
            outcome="OK",
            started_s=1.000,
            ended_s=1.003,
            duration_s=0.003,
            feedback_received=True,
        )
        tracker.record_transaction(tx)
        rec = tracker.end_cycle(1.004, transactions=(tx,))
        self.assertFalse(rec.overrun)
        self.assertEqual(rec.overrun_s, 0.0)
        self.assertEqual(rec.overrun_joints, ())

    # 4. Feedback age and sampling skew
    def test_feedback_age_and_sampling_skew(self) -> None:
        tracker = StationTimingTracker(0.010)
        tracker.start_cycle(1.0)

        # Two joints receive feedback at 1.001 and 1.004
        ages = (0.009, 0.006)
        timestamps = (1.001, 1.004)
        rec = tracker.end_cycle(1.010, feedback_ages=ages, feedback_timestamps=timestamps)

        self.assertEqual(rec.feedback_ages_s, (0.009, 0.006))
        self.assertAlmostEqual(rec.sampling_skew_s, 0.003, places=9)

        snap = tracker.snapshot()
        self.assertEqual(snap.latest_feedback_ages_s, (0.009, 0.006))
        self.assertAlmostEqual(snap.latest_sampling_skew_s, 0.003, places=9)

    def test_sampling_skew_single_or_empty(self) -> None:
        tracker = StationTimingTracker(0.010)
        self.assertIsNone(tracker.calculate_sampling_skew(()))
        self.assertIsNone(tracker.calculate_sampling_skew((1.0,)))
        self.assertIsNone(tracker.calculate_sampling_skew((None, None)))

    # 5. Bounded records over many iterations
    def test_bounded_records_over_many_iterations(self) -> None:
        capacity = 5
        tracker = StationTimingTracker(0.010, capacity=capacity)
        for i in range(200):
            tracker.start_cycle(float(i))
            tx = JointTransactionRecord(
                cycle_index=i,
                joint_index=0,
                joint_name="J1",
                motor_id=0,
                plan_id=i,
                sequence=i,
                owner=None,
                write_result=WriteResult.COMPLETE,
                outcome="OK",
                started_s=float(i),
                ended_s=float(i) + 0.002,
                duration_s=0.002,
                feedback_received=True,
            )
            tracker.record_transaction(tx)
            tracker.end_cycle(float(i) + 0.004, transactions=(tx,))

        snap = tracker.snapshot()
        self.assertEqual(snap.counters.cycles, 200)
        self.assertEqual(snap.counters.transactions, 200)
        self.assertEqual(len(snap.recent_cycles), capacity)
        self.assertEqual(len(snap.recent_transactions), capacity)

    # 6. Transaction outcomes & failure modes
    def test_distinguishable_outcomes_and_no_secret_text(self) -> None:
        for outcome in VALID_OUTCOMES:
            tx = JointTransactionRecord(
                cycle_index=0,
                joint_index=0,
                joint_name="J1",
                motor_id=0,
                plan_id=None,
                sequence=None,
                owner=None,
                write_result=WriteResult.FAILED if "FAILURE" in outcome else None,
                outcome=outcome,
                started_s=1.0,
                ended_s=1.002,
                duration_s=0.002,
                feedback_received=(outcome == "OK"),
            )
            self.assertEqual(tx.outcome, outcome)
            # Ensure no secret/exception attributes exist
            self.assertFalse(hasattr(tx, "error"))
            self.assertFalse(hasattr(tx, "exception"))

    # 7. Step coordinator lifecycle integration
    def test_coordinator_successful_step(self) -> None:
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        core.begin_connect()
        core.connected(SourceIdentity("fixture", "station-test", 1))
        owner = core.claim_owner("human-client")
        core.begin_check()

        # Provide Stop responses for check
        stop_resp1 = make_feedback_frame(0, mode=0, driver_c=25, winding_c=30)
        stop_resp2 = make_feedback_frame(1, mode=0, driver_c=25, winding_c=30)
        transport = FakeTransport([stop_resp1, stop_resp2])
        tracker = StationTimingTracker(0.010)
        coordinator = StationStepCoordinator(core, transport, tracker, clock=self.clock)

        coordinator.check_step()
        self.assertTrue(core.checked())
        self.assertTrue(core.arm(owner))

        # Submit target
        window = core.issue_window(0.5)
        target = Target(
            1,
            (
                JointTarget(0.1, 0.5, 0.0, 5.0, 1.0),
                JointTarget(-0.1, -0.5, 0.0, 5.0, 1.0),
            ),
            window=window,
            owner=owner,
        )
        self.assertTrue(core.submit_target(target))

        # Provide mode 1 responses for step
        foc_resp1 = make_feedback_frame(0, mode=1, driver_c=25, winding_c=30)
        foc_resp2 = make_feedback_frame(1, mode=1, driver_c=25, winding_c=30)
        transport.responses.extend([foc_resp1, foc_resp2])

        self.clock.advance(0.01)
        step_rec = coordinator.step()
        self.assertEqual(len(step_rec.transactions), 2)
        self.assertEqual(step_rec.transactions[0].outcome, "OK")
        self.assertEqual(step_rec.transactions[0].write_result, WriteResult.COMPLETE)
        self.assertEqual(step_rec.transactions[1].outcome, "OK")
        self.assertEqual(step_rec.transactions[1].write_result, WriteResult.COMPLETE)
        self.assertEqual(core.snapshot().state, State.MOVING)

    def test_coordinator_write_cancelled_and_remaining_skipped(self) -> None:
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        core.begin_connect()
        core.connected(SourceIdentity("fixture", "station-test", 1))
        owner = core.claim_owner("human-client")
        core.begin_check()

        stop_resp1 = make_feedback_frame(0, mode=0, driver_c=25, winding_c=30)
        stop_resp2 = make_feedback_frame(1, mode=0, driver_c=25, winding_c=30)
        transport = FakeTransport([stop_resp1, stop_resp2])
        tracker = StationTimingTracker(0.010)
        coordinator = StationStepCoordinator(core, transport, tracker, clock=self.clock)

        coordinator.check_step()
        core.checked()
        core.arm(owner)

        window = core.issue_window(0.5)
        target = Target(
            1,
            (
                JointTarget(0.1, 0.5, 0.0, 5.0, 1.0),
                JointTarget(-0.1, -0.5, 0.0, 5.0, 1.0),
            ),
            window=window,
            owner=owner,
        )
        core.submit_target(target)

        # Before stepping, revoke owner / stop core so claim_dispatch will return False
        # When plan is prepared, claim_dispatch on joint 0 is denied!
        # Coordinator catches WriteCancelledError, marks joint 0 as WRITE_CANCELLED,
        # and marks remaining joint 1 as SKIPPED.
        foc_resp1 = make_feedback_frame(0, mode=1, driver_c=25, winding_c=30)
        transport.responses.append(foc_resp1)

        # Disarm core right before step
        core.stop()
        step_rec = coordinator.step()
        # When stopped, prepare_refresh is None, step handled as idle
        self.assertEqual(len(step_rec.transactions), 0)

    def test_coordinator_timeout_and_remaining_skipped(self) -> None:
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        core.begin_connect()
        core.connected(SourceIdentity("fixture", "station-test", 1))
        owner = core.claim_owner("human-client")
        core.begin_check()

        stop_resp1 = make_feedback_frame(0, mode=0, driver_c=25, winding_c=30)
        stop_resp2 = make_feedback_frame(1, mode=0, driver_c=25, winding_c=30)
        transport = FakeTransport([stop_resp1, stop_resp2])
        tracker = StationTimingTracker(0.010)
        coordinator = StationStepCoordinator(core, transport, tracker, clock=self.clock)

        coordinator.check_step()
        core.checked()
        core.arm(owner)

        window = core.issue_window(0.5)
        target = Target(
            1,
            (
                JointTarget(0.1, 0.5, 0.0, 5.0, 1.0),
                JointTarget(-0.1, -0.5, 0.0, 5.0, 1.0),
            ),
            window=window,
            owner=owner,
        )
        core.submit_target(target)

        # For step, provide NO responses in transport -> TransactionTimeoutError on joint 0
        self.clock.advance(0.01)
        step_rec = coordinator.step()

        self.assertEqual(len(step_rec.transactions), 2)
        # Joint 0: write was completed, but feedback timed out
        self.assertEqual(step_rec.transactions[0].outcome, "TIMEOUT")
        self.assertEqual(step_rec.transactions[0].write_result, WriteResult.COMPLETE)

        # Joint 1: skipped because joint 0 timed out
        self.assertEqual(step_rec.transactions[1].outcome, "SKIPPED")
        self.assertIsNone(step_rec.transactions[1].write_result)

    def test_coordinator_driver_failure_partial_write(self) -> None:
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        core.begin_connect()
        core.connected(SourceIdentity("fixture", "station-test", 1))
        owner = core.claim_owner("human-client")
        core.begin_check()

        stop_resp1 = make_feedback_frame(0, mode=0, driver_c=25, winding_c=30)
        stop_resp2 = make_feedback_frame(1, mode=0, driver_c=25, winding_c=30)
        transport = FakeTransport([stop_resp1, stop_resp2])
        tracker = StationTimingTracker(0.010)
        coordinator = StationStepCoordinator(core, transport, tracker, clock=self.clock)

        coordinator.check_step()
        core.checked()
        core.arm(owner)

        window = core.issue_window(0.5)
        target = Target(
            1,
            (
                JointTarget(0.1, 0.5, 0.0, 5.0, 1.0),
                JointTarget(-0.1, -0.5, 0.0, 5.0, 1.0),
            ),
            window=window,
            owner=owner,
        )
        core.submit_target(target)

        # Inject partial write (5 bytes instead of 20)
        transport.partial_write_len = 5
        self.clock.advance(0.01)
        step_rec = coordinator.step()

        self.assertEqual(len(step_rec.transactions), 2)
        self.assertEqual(step_rec.transactions[0].outcome, "DRIVER_FAILURE")
        self.assertEqual(step_rec.transactions[0].write_result, WriteResult.PARTIAL)
        self.assertEqual(step_rec.transactions[1].outcome, "SKIPPED")

    def test_coordinator_idle_or_stop_handling(self) -> None:
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        tracker = StationTimingTracker(0.010)
        coordinator = StationStepCoordinator(core, FakeTransport(), tracker, clock=self.clock)

        # Core is in DISCONNECTED, prepare_refresh is None
        step_rec = coordinator.step()
        self.assertEqual(len(step_rec.transactions), 0)
        self.assertFalse(step_rec.overrun)
        self.assertEqual(step_rec.cycle_index, 0)


if __name__ == "__main__":
    unittest.main()
