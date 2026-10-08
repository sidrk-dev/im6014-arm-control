"""Acceptance tests for C07 command expiry, window authority, and refresh dispatch."""

import math
import unittest
from unittest.mock import patch

from im6014_codec import command
from joint_contract import (
    CommunicationFailure,
    DecisionCode,
    IntentWindow,
    JointSpec,
    MotorHealth,
    OwnerToken,
    SourceIdentity,
    State,
    Target,
    WriteResult,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget
from im6014_serial import TransactionTimeoutError, WriteCancelledError, exchange
from test_controller import DeterministicClock, FakeStation, HEALTHY_STOP


SOURCE = SourceIdentity("fixture", "expiry-test", 1)


class ControllerExpiryTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), -2.0, 2.0, 10.0, "fixture: software position limits"),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), -2.0, 2.0, 10.0, "fixture: software position limits"),
        )
        self.valid_joints = (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        )

    def _armed_core(self, clock=None, *, feedback_max_age_s=1.0, max_window_s=1.0):
        if clock is None:
            clock = DeterministicClock(0.0)
        core = JointCore(
            self.specs,
            clock=clock,
            feedback_max_age_s=feedback_max_age_s,
            max_window_s=max_window_s,
            max_reference_gap_s=1.0,
        )
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, clock

    def test_expired_incoming_and_equality_boundary_reject_without_watermark_corruption(self):
        core, station, clock = self._armed_core()
        self.assertEqual(core.snapshot().last_sequence, -1)

        # Submit valid initial target to establish watermark at 5
        w0 = station.issue_window(0.5)
        t0 = Target(5, self.valid_joints, window=w0, owner=station.owner)
        self.assertTrue(station.submit(t0).accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t0)

        # Issue window valid for 0.4s (expires at 0.4s)
        w1 = station.issue_window(0.4)
        self.assertEqual(w1.created_at_s, 0.0)
        self.assertEqual(w1.expires_at_s, 0.4)

        # Exact equality boundary: now == expires_at_s is expired
        clock.value = 0.4
        t_boundary = Target(6, self.valid_joints, window=w1, owner=station.owner)
        d_boundary = station.submit(t_boundary)
        self.assertFalse(d_boundary.accepted)
        self.assertEqual(d_boundary.code, DecisionCode.EXPIRED)
        # Watermark and valid goal preserved
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t0)

        # Strictly past expiry: now > expires_at_s is expired
        clock.value = 0.40001
        t_past = Target(7, self.valid_joints, window=w1, owner=station.owner)
        d_past = station.submit(t_past)
        self.assertFalse(d_past.accepted)
        self.assertEqual(d_past.code, DecisionCode.EXPIRED)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t0)

    def test_delayed_computation_cannot_get_new_arrival_ttl(self):
        core, station, clock = self._armed_core()
        # Client requests window at time 0.0 valid for 0.2s
        w1 = station.issue_window(0.2)
        self.assertEqual(w1.expires_at_s, 0.2)

        # Client computation takes 0.3s (now is 0.3)
        clock.value = 0.3
        target1 = Target(1, self.valid_joints, window=w1, owner=station.owner)
        # Arrival at core does NOT grant new TTL; rejects as EXPIRED
        decision = station.submit(target1)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.code, DecisionCode.EXPIRED)
        self.assertEqual(core.snapshot().last_sequence, -1)
        self.assertIsNone(core.snapshot().admitted_target)

        # Client cannot forge a window with extended expiry
        forged_window = IntentWindow(w1.id, w1.created_at_s, 0.9)
        forged_target = Target(1, self.valid_joints, window=forged_window, owner=station.owner)
        d_forged = station.submit(forged_target)
        self.assertFalse(d_forged.accepted)
        self.assertEqual(d_forged.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().last_sequence, -1)

        # If station issues a new window w2, client cannot submit old computation with w1
        w2 = station.issue_window(0.2)
        d_old = station.submit(target1)
        self.assertFalse(d_old.accepted)
        self.assertEqual(d_old.code, DecisionCode.EXPIRED)

    def test_new_submission_cannot_revive_expired_moving_goal(self):
        core, station, clock = self._armed_core()
        first_window = station.issue_window(0.2)
        first = Target(5, self.valid_joints, window=first_window, owner=station.owner)
        self.assertTrue(station.submit(first).accepted)
        clock.value = 0.05
        self.assertIsNotNone(station.refresh())
        self.assertIs(core.snapshot().state, State.MOVING)

        clock.value = first_window.expires_at_s
        fresh_window = station.issue_window(0.2)
        replacement = Target(6, self.valid_joints, window=fresh_window, owner=station.owner)
        decision = station.submit(replacement)

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.code, DecisionCode.EXPIRED)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().admitted_target)
        self.assertEqual(core.snapshot().last_sequence, 5)

    def test_invalid_replacement_preserves_only_a_still_valid_goal(self):
        core, station, clock = self._armed_core()
        window = station.issue_window(0.2)
        current = Target(5, self.valid_joints, window=window, owner=station.owner)
        self.assertTrue(station.submit(current).accepted)

        invalid = Target(9, (self.valid_joints[0],), window=window, owner=station.owner)
        decision = station.submit(invalid)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().admitted_target, current)
        self.assertEqual(core.snapshot().last_sequence, 5)

        clock.value = window.expires_at_s
        fresh_window = station.issue_window(0.2)
        invalid_after_expiry = Target(10, (self.valid_joints[0],), window=fresh_window, owner=station.owner)
        expired_decision = station.submit(invalid_after_expiry)
        self.assertFalse(expired_decision.accepted)
        self.assertEqual(expired_decision.code, DecisionCode.EXPIRED)
        self.assertIsNone(core.snapshot().admitted_target)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertEqual(core.snapshot().last_sequence, 5)

    def test_goal_expires_after_admission_and_before_prepare_or_actual_write(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(0.5)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)
        self.assertEqual(core.snapshot().admitted_target, target)

        # Goal expires after admission: clock moves to 0.55 before prepare_refresh
        clock.value = 0.55
        plan = core.prepare_refresh()
        self.assertIsNone(plan)
        # Goal is withdrawn and core disarms to STOPPED
        snap = core.snapshot()
        self.assertIsNone(snap.admitted_target)
        self.assertIsNone(snap.prepared_plan)
        self.assertIs(snap.state, State.STOPPED)

    def test_two_joint_first_write_then_expiry_or_revoke_blocks_second_and_records_only_first(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(0.5)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)

        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # Joint 0 claim and complete write
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        d0 = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)
        self.assertTrue(d0.accepted)
        self.assertIs(core.snapshot().state, State.MOVING)

        # Check J0 record is stored, J1 is still None
        records_after_j0 = core.snapshot().dispatched_records
        self.assertIsNotNone(records_after_j0[0])
        self.assertEqual(records_after_j0[0].sequence, 1)
        self.assertEqual(records_after_j0[0].target, plan.joint_targets[0])
        self.assertIsNone(records_after_j0[1])

        # Expiry occurs before Joint 1 write
        clock.value = 0.55
        # Joint 1 claim fails due to expiry
        self.assertFalse(core.claim_dispatch(plan.plan_id, 1))

        # Core disarms to STOPPED, goal withdrawn, J0 record preserved, J1 remained unwritten
        snap = core.snapshot()
        self.assertIs(snap.state, State.STOPPED)
        self.assertIsNone(snap.admitted_target)
        self.assertIsNone(snap.prepared_plan)
        self.assertIsNotNone(snap.dispatched_records[0])
        self.assertIsNone(snap.dispatched_records[1])

    def test_reset_input_buffer_advancing_clock_causes_final_gate_cancellation_with_zero_mode1_write(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(0.5)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)

        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # Fake bus where reset_input_buffer takes enough time to advance the clock past expiry
        written_packets = []

        class TimeAdvancingBus:
            def __init__(self):
                self.timeout = 0.1
                self.in_waiting = 0

            def reset_input_buffer(self):
                clock.value = 0.6  # Past window expiry of 0.5s!

            def write(self, packet):
                written_packets.append(packet)
                return len(packet)

            def read(self, size):
                return b""

        bus = TimeAdvancingBus()
        pkt = command(0, mode=1)

        with self.assertRaises(WriteCancelledError):
            exchange(
                bus,
                pkt,
                motor_id=0,
                deadline_s=0.2,
                before_write=lambda: core.claim_dispatch(plan.plan_id, 0),
                on_write=lambda res: core.dispatch_result(plan.plan_id, 0, res),
            )

        # Zero mode-1 packets were written to the bus!
        self.assertEqual(written_packets, [])
        # Final gate cancellation disarmed core to STOPPED and withdrew goal
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().admitted_target)

    def test_expiry_during_feedback_wait_causes_no_later_write(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(0.5)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)

        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # First joint writes successfully
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        self.assertTrue(core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE).accepted)

        # Feedback wait takes long time, crossing expiry
        clock.value = 0.55

        # Attempting second joint write: claim fails because goal has expired
        self.assertFalse(core.claim_dispatch(plan.plan_id, 1))
        # Core is disarmed to STOPPED
        self.assertIs(core.snapshot().state, State.STOPPED)

    def test_repeated_refresh_bounded_until_expiry(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(0.5)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)

        # At t=0.1, refresh succeeds
        clock.value = 0.1
        plan1 = station.refresh()
        self.assertIsNotNone(plan1)
        self.assertEqual(plan1.target, target)
        self.assertIs(core.snapshot().state, State.MOVING)

        # At t=0.2, second refresh succeeds with the same admitted goal
        clock.value = 0.2
        plan2 = station.refresh()
        self.assertIsNotNone(plan2)
        self.assertEqual(plan2.target, target)

        # At t=0.4, third refresh succeeds
        clock.value = 0.4
        plan3 = station.refresh()
        self.assertIsNotNone(plan3)

        # At t=0.5, window expires: refresh returns None and disarms to STOPPED
        clock.value = 0.5
        plan_expired = station.refresh()
        self.assertIsNone(plan_expired)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().admitted_target)

    def test_invalid_higher_seq_preserves_valid_goal(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(1.0)
        t5 = Target(5, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(t5).accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

        # High sequence with invalid joint target count
        bad_high = Target(100, (self.valid_joints[0],), window=w, owner=station.owner)
        d_bad = station.submit(bad_high)
        self.assertFalse(d_bad.accepted)
        self.assertEqual(d_bad.code, DecisionCode.INVALID_ARGUMENT)

        # Watermark is still 5, admitted goal is still t5
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

        # High sequence with out-of-range torque
        bad_torque = Target(100, (JointTarget(0, 0, 10.0, 1, .1), self.valid_joints[1]), window=w, owner=station.owner)
        d_torque = station.submit(bad_torque)
        self.assertFalse(d_torque.accepted)
        self.assertEqual(d_torque.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

    def test_stop_fault_disconnect_invalidate_plans_but_retain_sequence(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(1.0)
        target = Target(10, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)
        self.assertEqual(core.snapshot().last_sequence, 10)

        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(core.snapshot().prepared_plan, plan)

        # Stop invalidates plan and goal, retains sequence watermark
        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().admitted_target)
        self.assertIsNone(core.snapshot().prepared_plan)
        self.assertEqual(core.snapshot().last_sequence, 10)

        # Re-arm and set up new target
        owner2 = station.claim("fixture-2")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner2).accepted)
        w2 = station.issue_window(1.0)
        target2 = Target(12, self.valid_joints, window=w2, owner=owner2)
        self.assertTrue(station.submit(target2).accepted)
        clock.value = 0.1
        plan2 = core.prepare_refresh()
        self.assertIsNotNone(plan2)

        # Fault invalidates plan and goal, retains sequence
        self.assertTrue(core.communication_failed(0, CommunicationFailure.TIMEOUT).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertIsNone(core.snapshot().admitted_target)
        self.assertIsNone(core.snapshot().prepared_plan)
        self.assertEqual(core.snapshot().last_sequence, 12)

        # Disconnect invalidates plan and goal, retains sequence
        self.assertTrue(station.disconnect().accepted)
        self.assertIs(core.snapshot().state, State.DISCONNECTED)
        self.assertIsNone(core.snapshot().admitted_target)
        self.assertIsNone(core.snapshot().prepared_plan)
        self.assertEqual(core.snapshot().last_sequence, 12)

    def test_wrong_duplicate_out_of_order_claims_reject(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(1.0)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)

        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # Wrong plan ID rejects
        self.assertFalse(core.claim_dispatch(plan.plan_id + 99, 0))

        # Out-of-order joint index rejects (index 1 before index 0)
        self.assertFalse(core.claim_dispatch(plan.plan_id, 1))

        # Correct claim of index 0
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))

        # Duplicate claim of index 0 rejects
        self.assertFalse(core.claim_dispatch(plan.plan_id, 0))

        # Out-of-bounds index rejects
        self.assertFalse(core.claim_dispatch(plan.plan_id, 5))
        self.assertFalse(core.claim_dispatch(plan.plan_id, -1))

        # One unresolved write blocks both reprepare and the next joint.
        self.assertIsNone(core.prepare_refresh())
        self.assertFalse(core.claim_dispatch(plan.plan_id, 1))
        self.assertFalse(core.dispatch_result(plan.plan_id + 99, 0, WriteResult.COMPLETE).accepted)
        self.assertFalse(core.dispatch_result(plan.plan_id, 1, WriteResult.COMPLETE).accepted)
        self.assertFalse(core.dispatch_result(plan.plan_id, 0, None).accepted)
        self.assertTrue(core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE).accepted)

        # Joint 1 is claimable only after joint 0's outcome arrives.
        self.assertTrue(core.claim_dispatch(plan.plan_id, 1))
        self.assertTrue(core.dispatch_result(plan.plan_id, 1, WriteResult.COMPLETE).accepted)

    def test_stop_after_claim_preserves_completed_receipt_without_rearming(self):
        core, station, clock = self._armed_core()
        target = Target(1, self.valid_joints, window=station.issue_window(1.0), owner=station.owner)
        self.assertTrue(station.submit(target).accepted)
        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        claim_time = clock.value

        self.assertTrue(core.stop().accepted)
        receipt = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)

        self.assertTrue(receipt.accepted)
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.STOPPED)
        self.assertIsNone(snapshot.admitted_target)
        self.assertIsNone(snapshot.prepared_plan)
        self.assertEqual(snapshot.dispatched_records[0].sequence, 1)
        self.assertEqual(snapshot.dispatched_records[0].dispatched_at_s, claim_time)
        self.assertIsNone(core.prepare_refresh())
        self.assertFalse(core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE).accepted)

    def test_expiry_after_claim_preserves_completed_receipt_without_moving(self):
        core, station, clock = self._armed_core()
        window = station.issue_window(0.2)
        target = Target(1, self.valid_joints, window=window, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)
        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        claim_time = clock.value

        clock.value = window.expires_at_s
        receipt = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)

        self.assertTrue(receipt.accepted)
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.STOPPED)
        self.assertIsNone(snapshot.admitted_target)
        self.assertEqual(snapshot.dispatched_records[0].sequence, 1)
        self.assertEqual(snapshot.dispatched_records[0].dispatched_at_s, claim_time)
        self.assertFalse(core.claim_dispatch(plan.plan_id, 1))

    def test_invalidated_plan_receipt_records_write_without_reviving_old_goal(self):
        core, station, clock = self._armed_core()
        first = Target(1, self.valid_joints, window=station.issue_window(1.0), owner=station.owner)
        self.assertTrue(station.submit(first).accepted)
        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))

        replacement = Target(2, self.valid_joints, window=station.issue_window(1.0), owner=station.owner)
        self.assertTrue(station.submit(replacement).accepted)
        self.assertIsNone(core.snapshot().prepared_plan)
        self.assertIsNone(core.prepare_refresh())

        receipt = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)
        self.assertTrue(receipt.accepted)
        snapshot = core.snapshot()
        self.assertIs(snapshot.admitted_target, replacement)
        self.assertIs(snapshot.state, State.ARMED)
        self.assertEqual(snapshot.dispatched_records[0].sequence, 1)
        self.assertIsNone(snapshot.prepared_plan)

        clock.value = 0.1
        replacement_plan = core.prepare_refresh()
        self.assertIsNotNone(replacement_plan)
        self.assertIs(replacement_plan.target, replacement)

    def test_complete_write_then_bad_timeout_feedback_still_counted_actually_dispatched(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(1.0)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)

        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # Joint 0 claims and writes successfully
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        d_res = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)
        self.assertTrue(d_res.accepted)

        # Dispatched record is committed
        snap_after_write = core.snapshot()
        self.assertIsNotNone(snap_after_write.dispatched_records[0])
        self.assertEqual(snap_after_write.dispatched_records[0].sequence, 1)

        # Feedback from serial communication times out
        self.assertTrue(core.communication_failed(0, CommunicationFailure.TIMEOUT).accepted)
        snap_after_failure = core.snapshot()
        self.assertIs(snap_after_failure.state, State.FAULT)
        # Dispatched record is STILL preserved despite communication failure!
        self.assertIsNotNone(snap_after_failure.dispatched_records[0])
        self.assertEqual(snap_after_failure.dispatched_records[0].sequence, 1)

    def test_partial_failed_write_faults(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(1.0)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)

        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # J0 claims, but write fails
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        d_fail = core.dispatch_result(plan.plan_id, 0, WriteResult.FAILED)
        self.assertFalse(d_fail.accepted)
        self.assertEqual(d_fail.code, DecisionCode.FAULT_LATCHED)

        # Latches fault on joint 0
        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertEqual(snap.affected_joints, ("J1",))
        self.assertEqual(snap.faults[0].communication_error, CommunicationFailure.DRIVER_FAILURE)
        # Motion withdrawn, plan invalidated
        self.assertIsNone(snap.admitted_target)
        self.assertIsNone(snap.prepared_plan)
        # J0 was not completely written so dispatched record is None
        self.assertIsNone(snap.dispatched_records[0])
        self.assertEqual(snap.dispatch_attempts[0].result, WriteResult.FAILED)

    def test_partial_attempt_keeps_previous_complete_dispatched_record(self):
        core, station, clock = self._armed_core()
        first = Target(1, self.valid_joints, window=station.issue_window(1.0), owner=station.owner)
        self.assertTrue(station.submit(first).accepted)
        clock.value = 0.05
        plan = core.prepare_refresh()
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        self.assertTrue(core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE).accepted)

        next_target = Target(2, self.valid_joints, window=station.issue_window(1.0), owner=station.owner)
        self.assertTrue(core.submit_target(next_target).accepted)
        clock.value = 0.1
        next_plan = core.prepare_refresh()
        self.assertTrue(core.claim_dispatch(next_plan.plan_id, 0))
        failed = core.dispatch_result(next_plan.plan_id, 0, WriteResult.PARTIAL)

        self.assertFalse(failed.accepted)
        snapshot = core.snapshot()
        self.assertEqual(snapshot.dispatched_records[0].sequence, 1)
        self.assertEqual(snapshot.dispatch_attempts[0].result, WriteResult.PARTIAL)
        self.assertEqual(snapshot.dispatch_attempts[0].sequence, 2)
        self.assertEqual(snapshot.faults[0].communication_error, CommunicationFailure.DRIVER_FAILURE)

    def test_complete_write_is_recorded_then_invalid_end_clock_faults_joint(self):
        for invalid_end_time in (math.nan, 0.5):
            with self.subTest(invalid_end_time=invalid_end_time):
                core, station, clock = self._armed_core(DeterministicClock(1.0))
                target = Target(1, self.valid_joints, window=station.issue_window(1.0), owner=station.owner)
                self.assertTrue(station.submit(target).accepted)
                clock.value = 1.05
                plan = core.prepare_refresh()
                claim_time = clock.value

                transaction_clock = DeterministicClock(0.0)
                writes = []

                class ClockFailingBus:
                    timeout = 0.1

                    def reset_input_buffer(self):
                        pass

                    def write(self, packet):
                        writes.append(packet)
                        clock.value = invalid_end_time
                        transaction_clock.value = 0.1
                        return len(packet)

                with patch("im6014_serial._monotonic", transaction_clock):
                    with self.assertRaises(TransactionTimeoutError):
                        exchange(
                            ClockFailingBus(), b"x" * 20, 0, deadline_s=0.05,
                            before_write=lambda: core.claim_dispatch(plan.plan_id, 0),
                            on_write=lambda result: core.dispatch_result(plan.plan_id, 0, result),
                        )
                self.assertEqual(len(writes), 1)
                with self.assertRaises(ValueError):
                    core.snapshot()

                clock.value = claim_time
                snapshot = core.snapshot()
                self.assertIs(snapshot.state, State.FAULT)
                self.assertEqual(snapshot.faults[0].communication_error, CommunicationFailure.CLOCK_FAILURE)
                self.assertEqual(snapshot.dispatched_records[0].sequence, 1)
                self.assertEqual(snapshot.dispatched_records[0].dispatched_at_s, claim_time)
                self.assertEqual(snapshot.dispatch_attempts[0].result, WriteResult.COMPLETE)
                self.assertIsNone(snapshot.admitted_target)
                self.assertIsNone(snapshot.prepared_plan)
                self.assertFalse(core.begin_check().accepted)
                self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

    def test_exchange_reports_actual_write_before_enforcing_deadline(self):
        transaction_clock = DeterministicClock(0.0)
        results = []

        class DeadlineBus:
            timeout = 0.1

            def __init__(self, returned):
                self.returned = returned
                self.write_count = 0

            def reset_input_buffer(self):
                pass

            def write(self, packet):
                self.write_count += 1
                transaction_clock.value = 0.1
                return self.returned

        complete_bus = DeadlineBus(20)
        with patch("im6014_serial._monotonic", transaction_clock):
            with self.assertRaises(TransactionTimeoutError):
                exchange(complete_bus, b"x" * 20, 0, deadline_s=0.05, on_write=results.append)
        self.assertEqual(complete_bus.write_count, 1)
        self.assertEqual(results, [WriteResult.COMPLETE])

        transaction_clock.value = 0.0
        results.clear()
        partial_bus = DeadlineBus(1)
        with patch("im6014_serial._monotonic", transaction_clock):
            with self.assertRaises(TransactionTimeoutError):
                exchange(partial_bus, b"x" * 20, 0, deadline_s=0.05, on_write=results.append)
        self.assertEqual(partial_bus.write_count, 1)
        self.assertEqual(results, [WriteResult.PARTIAL])

    def test_expiry_after_successful_claim_reports_prewrite_deadline(self):
        core, station, clock = self._armed_core()
        target = Target(1, self.valid_joints, window=station.issue_window(1.0), owner=station.owner)
        self.assertTrue(station.submit(target).accepted)
        clock.value = 0.05
        plan = core.prepare_refresh()

        transaction_clock = DeterministicClock(0.0)
        results = []

        class DeadlineBus:
            timeout = 0.1

            def __init__(self):
                self.write_count = 0

            def reset_input_buffer(self):
                pass

            def write(self, packet):
                self.write_count += 1
                return len(packet)

        def claim_then_expire():
            self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
            transaction_clock.value = 0.05
            return True

        def record_result(result):
            results.append(result)
            return core.dispatch_result(plan.plan_id, 0, result)

        bus = DeadlineBus()
        with patch("im6014_serial._monotonic", transaction_clock):
            with self.assertRaises(TransactionTimeoutError):
                exchange(
                    bus, b"x" * 20, 0, deadline_s=0.05,
                    before_write=claim_then_expire,
                    on_write=record_result,
                )

        self.assertEqual(bus.write_count, 0)
        self.assertEqual(results, [WriteResult.PREWRITE_DEADLINE])
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.FAULT)
        self.assertEqual(snapshot.faults[0].motor_id, 0)
        self.assertEqual(snapshot.faults[0].communication_error, CommunicationFailure.PREWRITE_DEADLINE)
        self.assertEqual(snapshot.dispatch_attempts[0].result, WriteResult.PREWRITE_DEADLINE)
        self.assertIsNone(snapshot.dispatched_records[0])
        self.assertIsNone(snapshot.admitted_target)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

    def test_exchange_reports_raised_write_once_and_callback_failure_stops_feedback(self):
        results = []
        core, station, clock = self._armed_core()
        target = Target(1, self.valid_joints, window=station.issue_window(1.0), owner=station.owner)
        self.assertTrue(station.submit(target).accepted)
        clock.value = 0.05
        plan = core.prepare_refresh()

        class RaisingBus:
            timeout = 0.1

            def __init__(self):
                self.write_count = 0

            def reset_input_buffer(self):
                pass

            def write(self, packet):
                self.write_count += 1
                raise OSError("driver failure")

        def record_result(result):
            results.append(result)
            return core.dispatch_result(plan.plan_id, 0, result)

        raising_bus = RaisingBus()
        with self.assertRaisesRegex(OSError, "driver failure"):
            exchange(
                raising_bus, b"x" * 20, 0,
                before_write=lambda: core.claim_dispatch(plan.plan_id, 0),
                on_write=record_result,
            )
        self.assertEqual(raising_bus.write_count, 1)
        self.assertEqual(results, [WriteResult.FAILED])
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().dispatch_attempts[0].result, WriteResult.FAILED)
        self.assertEqual(core.snapshot().faults[0].communication_error, CommunicationFailure.DRIVER_FAILURE)

        class CompleteBus:
            timeout = 0.1

            def __init__(self):
                self.write_count = 0
                self.read_called = False

            def reset_input_buffer(self):
                pass

            def write(self, packet):
                self.write_count += 1
                return len(packet)

            def read(self, size):
                self.read_called = True
                return b""

        bus = CompleteBus()
        callback_calls = []

        def broken_callback(result):
            callback_calls.append(result)
            raise RuntimeError("receipt failed")

        with self.assertRaisesRegex(RuntimeError, "receipt failed"):
            exchange(bus, b"x" * 20, 0, on_write=broken_callback)
        self.assertEqual(bus.write_count, 1)
        self.assertEqual(callback_calls, [WriteResult.COMPLETE])
        self.assertFalse(bus.read_called)

    def test_clock_regression_before_receive_cannot_replace_sample_or_timestamp(self):
        clock = DeterministicClock(10.0)
        core = JointCore(self.specs, clock=clock, max_reference_gap_s=1.0)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertTrue(core.begin_check().accepted)

        # Valid receive at t=10.0
        sample_initial = JointSample(0.5, 0.0, 0.0)
        self.assertTrue(core.receive(0, sample_initial, HEALTHY_STOP).accepted)
        snap_initial = core.snapshot()
        self.assertEqual(snap_initial.joints[0].received_at, 10.0)
        self.assertEqual(snap_initial.joints[0].sample.q_rad, 0.5)

        # Clock regresses to 5.0
        clock.value = 5.0
        corrupt_sample = JointSample(9.9, 0.0, 0.0)
        with self.assertRaises(ValueError):
            core.receive(0, corrupt_sample, HEALTHY_STOP)

        # High-water mark check threw BEFORE overwriting observation
        # Previous observation remains uncorrupted
        clock.value = 10.0  # restore to valid non-regressing time for snapshot
        snap_after = core.snapshot()
        self.assertEqual(snap_after.joints[0].received_at, 10.0)
        self.assertEqual(snap_after.joints[0].sample.q_rad, 0.5)

    def test_invalid_clock_cannot_release_writes(self):
        core, station, clock = self._armed_core()
        w = station.issue_window(1.0)
        target = Target(1, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(target).accepted)

        # Non-finite clock raises before prepare_refresh releases any plan
        clock.value = math.nan
        with self.assertRaises(ValueError):
            core.prepare_refresh()

        clock.value = 0.1
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        # Regressing clock raises before claim_dispatch releases write
        clock.value = 0.05  # Regressed!
        with self.assertRaises(ValueError):
            core.claim_dispatch(plan.plan_id, 0)

        # Stop still works even if clock fails or raises
        clock.value = math.nan
        stop_decision = station.stop()
        self.assertTrue(stop_decision.accepted)

    def test_max_window_s_configuration_and_monotonically_increasing_window_ids(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_window_s=0.5, max_reference_gap_s=1.0)

        # Window exceeding max_window_s rejects
        with self.assertRaises(ValueError):
            core.issue_window(0.6)

        # Establish station authority before offering windows; claiming invalidates prior offers.
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)

        # Monotonically increasing window IDs without wrap
        w0 = core.issue_window(0.2)
        w1 = core.issue_window(0.2)
        w2 = core.issue_window(0.5)
        self.assertEqual(w0.id, 0)
        self.assertEqual(w1.id, 1)
        self.assertEqual(w2.id, 2)
        self.assertTrue(w0.id < w1.id < w2.id)

        # Older window w0 is rejected
        t_old = Target(1, self.valid_joints, window=w0, owner=owner)
        d_old = station.submit(t_old)
        self.assertFalse(d_old.accepted)
        self.assertEqual(d_old.code, DecisionCode.EXPIRED)

        # Latest window w2 is accepted
        t_latest = Target(1, self.valid_joints, window=w2, owner=owner)
        d_latest = station.submit(t_latest)
        self.assertTrue(d_latest.accepted)


if __name__ == "__main__":
    unittest.main()
