from dataclasses import FrozenInstanceError, replace
import math
import threading
import time
import unittest

from arm_fixture import (
    ArmPacketFixture,
    J1_MAX_RAD,
    J1_MIN_RAD,
    SYNTHETIC_J2_MAX_RAD,
    SYNTHETIC_J2_MIN_RAD,
    SYNTHETIC_J3_MAX_RAD,
    SYNTHETIC_J3_MIN_RAD,
    create_synthetic_arm_specs,
)
from arm_station import (
    ArmAction,
    ArmStation,
    ArmStationSnapshot,
    CheckAction,
    ConnectAction,
    DisconnectAction,
    JogIntent,
    PhysicalDraft,
    PhysicalJointDraft,
    StopAction,
    compute_zero_output,
    validate_physical_draft,
)
from im6014_codec import decode_command
from im6014_port import PortConfig
from joint_contract import State, WriteResult
from joint_protection import ProtectionPolicy


class DeterministicClock:
    def __init__(self, start: float = 0.0) -> None:
        self.value = float(start)

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> float:
        self.value += float(seconds)
        return self.value


class ArmStationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = DeterministicClock(10.0)
        self.fixture = ArmPacketFixture(clock=self.clock)
        self.station = ArmStation(clock=self.clock, fixture=self.fixture)

    def connect_and_check(self, station: ArmStation | None = None):
        station = self.station if station is None else station
        session = station.register_session("owner")
        self.assertEqual(station.request(ConnectAction(), session).status, "queued")
        station.step()
        self.assertEqual(station.snapshot().lifecycle_state, State.CONNECTED.value)
        self.assertEqual(station.request(CheckAction(), session).status, "queued")
        station.step()
        self.assertEqual(station.snapshot().lifecycle_state, State.READY.value)
        return session

    def arm(self, session, *, elapsed: float = 0.020) -> None:
        if elapsed:
            self.clock.advance(elapsed)
        receipt = self.station.request(ArmAction(), session)
        self.assertEqual(receipt.status, "queued")
        self.station.step()
        self.assertIn(
            self.station.snapshot().lifecycle_state,
            (State.ARMED.value, State.MOVING.value),
        )
        self.assertTrue(self.station.snapshot().hold_energized)

    def input(self, session, sequence, axes, deadman):
        ticket = self.station.issue_ticket(session, sequence)
        return self.station.request(
            JogIntent(
                sequence=sequence,
                axes=axes,
                deadman=deadman,
                focus=True,
                device_epoch="fixture-device-1",
                ticket=ticket,
            ),
            session,
        )

    @staticmethod
    def mode_writes(fixture: ArmPacketFixture, mode: int) -> list[dict]:
        return [decode_command(packet) for packet in fixture.writes
                if decode_command(packet)["mode"] == mode]

    def valid_physical_draft(self) -> PhysicalDraft:
        protection = ProtectionPolicy(85, 70, 105, 90, 1.0, 1.0, 0.5)
        joints = tuple(
            PhysicalJointDraft(
                motor_id=motor_id,
                name=f"J{motor_id + 1}",
                direction=1,
                reference_joint_rad=0.5 if motor_id == 0 else 0.0,
                zero_output_rad=0.0,
                q_min_rad=-1.4 if motor_id == 0 else -10.0,
                q_max_rad=1.4 if motor_id == 0 else 10.0,
                v_max_rad_s=1.0,
                a_max_rad_s2=2.0,
                max_lead_rad=0.5,
                max_effort_nm=2.0,
                kp=3.0,
                kd=0.2,
                protection=protection,
                limits_provenance="operator-confirmed travel measurement",
            )
            for motor_id in range(3)
        )
        return PhysicalDraft(
            port_config=PortConfig("/dev/draft-only", 6_000_000, "DRAFT", 1),
            joints=joints,
            support_acknowledged=True,
            timeout_facts_acknowledged=True,
            allow_hardware_opt_in=True,
        )

    def test_startup_and_supported_timing_constructor(self) -> None:
        initial = self.station.snapshot()
        self.assertEqual(initial.lifecycle_state, State.DISCONNECTED.value)
        self.assertIsNone(initial.source)
        self.assertEqual(initial.measured_sample, (None, None, None))
        self.assertEqual(initial.measured_health, (None, None, None))
        self.assertEqual(initial.confirmed_reference, (None, None, None))
        self.assertFalse(initial.hold_energized)
        self.assertFalse(hasattr(self.station, "core"))
        self.assertEqual(self.fixture.writes, ())

        self.station.step()
        self.assertEqual(
            self.station._timing.config.nominal_period_s,
            self.station.slot_period_s,
        )

    def test_sessions_and_lifecycle_ingress_are_bounded(self) -> None:
        sessions = [self.station.register_session(f"s-{i}") for i in range(16)]
        with self.assertRaises(RuntimeError):
            self.station.register_session("overflow")

        owner, other = sessions[:2]
        queued = [self.station.request(ConnectAction(), owner) for _ in range(16)]
        self.assertTrue(all(receipt.status == "queued" for receipt in queued))
        overflow = self.station.request(ConnectAction(), owner)
        self.assertEqual(overflow.code, "QUEUE_FULL")

        stop = self.station.request(StopAction(), other)
        self.assertEqual(stop.code, "STOP_QUEUED")
        self.station.step()
        self.assertEqual(self.station.snapshot().lifecycle_state, State.DISCONNECTED.value)
        self.assertEqual(self.fixture.writes, ())
        self.assertFalse(self.station.snapshot().stop_pending)
        for receipt in queued:
            self.assertEqual(self.station._receipts[receipt.request_id].code, "STOP_PREEMPTED")

    def test_lifecycle_receipt_survives_denied_request_flood(self) -> None:
        owner = self.station.register_session("owner")
        lifecycle = self.station.request(ConnectAction(), owner)
        denied = [self.station.request(object(), owner) for _ in range(100)]

        queued = self.station.receipt(lifecycle.request_id)
        self.assertEqual(queued.status, "present")
        self.assertEqual(queued.receipt.status, "queued")
        self.assertEqual(self.station.receipt(denied[0].request_id).status, "expired")
        self.assertEqual(self.station.receipt(self.station._next_request_id + 3).status, "unknown")

        self.station.step()
        terminal = self.station.receipt(lifecycle.request_id)
        self.assertEqual(terminal.status, "present")
        self.assertEqual(terminal.receipt.code, "CONNECTED")
        self.assertEqual(terminal.receipt.status, "accepted")
        self.assertEqual(
            self.station.receipt(denied[-1].request_id).receipt,
            self.station.snapshot().latest_receipt,
        )
        self.assertLessEqual(len(self.station._receipts), 64)
        self.assertLessEqual(len(self.station._active_lifecycle_receipts), 16)
        self.assertLessEqual(len(self.station._terminal_lifecycle_receipts), 64)

    def test_complete_fixture_lifecycle_and_three_axis_signs(self) -> None:
        session = self.connect_and_check()
        ready = self.station.snapshot()
        self.assertTrue(ready.stop_confirmed)
        self.assertEqual(ready.per_joint_stop_confirmed, (True, True, True))
        self.assertEqual(self.mode_writes(self.fixture, 1), [])

        self.arm(session)
        armed = self.station.snapshot()
        self.assertTrue(armed.hold_energized)
        self.assertEqual(armed.input_phase, "awaiting_neutral")

        self.input(session, 1, (0.0, 0.0, 0.0), False)
        self.station.step()
        self.clock.advance(0.020)
        self.input(session, 2, (1.0, 0.0, 0.0), True)
        self.station.step()
        writes = self.mode_writes(self.fixture, 1)[-3:]
        self.assertEqual([write["motor_id"] for write in writes], [0, 1, 2])
        self.assertGreater(writes[0]["position_rad"], 0.0)

        self.input(session, 3, (0.0, 0.0, 0.0), False)
        self.station.step()
        self.clock.advance(0.020)
        self.input(session, 4, (0.0, -1.0, 0.0), True)
        self.station.step()
        writes = self.mode_writes(self.fixture, 1)[-3:]
        self.assertLess(writes[1]["position_rad"], 0.0)

        self.input(session, 5, (0.0, 0.0, 0.0), False)
        self.station.step()
        self.clock.advance(0.020)
        self.input(session, 6, (0.0, 0.0, 1.0), True)
        self.station.step()
        writes = self.mode_writes(self.fixture, 1)[-3:]
        self.assertGreater(writes[2]["position_rad"], 0.0)
        self.assertEqual(self.station.snapshot().input_phase, "jogging")

        self.station.request(StopAction(), session)
        self.station.step()
        self.assertTrue(self.station.snapshot().stop_logical)
        self.assertTrue(self.station.snapshot().stop_confirmed)
        self.assertEqual(self.station.snapshot().per_joint_stop_confirmed, (True, True, True))

    def test_arm_same_timestamp_defers_mode1_until_real_elapsed_time(self) -> None:
        session = self.connect_and_check()
        mode1_before = len(self.mode_writes(self.fixture, 1))
        self.station.request(ArmAction(), session)
        self.station.step()

        snap = self.station.snapshot()
        self.assertEqual(snap.lifecycle_state, State.ARMED.value)
        self.assertTrue(snap.hold_energized)
        self.assertEqual(len(self.mode_writes(self.fixture, 1)), mode1_before)
        self.assertEqual(snap.faults, ())

        self.clock.advance(0.020)
        self.station.step()
        self.assertGreater(len(self.mode_writes(self.fixture, 1)), mode1_before)
        self.assertIn(
            self.station.snapshot().lifecycle_state,
            (State.ARMED.value, State.MOVING.value),
        )

    def test_arm_requires_observed_neutral_then_new_deadman_edge(self) -> None:
        session = self.connect_and_check()
        self.arm(session)

        first = self.input(session, 1, (0.5, 0.0, 0.0), True)
        self.assertEqual(first.status, "accepted")
        self.station.step()
        self.assertEqual(self.station._receipts[first.request_id].code, "NEUTRAL_REQUIRED")
        self.assertEqual(self.station.snapshot().input_phase, "awaiting_neutral")

        self.input(session, 2, (0.0, 0.0, 0.0), False)
        self.station.step()
        self.clock.advance(0.020)
        self.input(session, 3, (0.5, 0.0, 0.0), True)
        self.station.step()
        self.assertEqual(self.station.snapshot().input_phase, "jogging")

    def test_release_captures_confirmed_hold_once(self) -> None:
        session = self.connect_and_check()
        self.arm(session)
        capture_calls: list[bool] = []
        original_capture = self.station._capture_release_hold

        def capture_once() -> None:
            capture_calls.append(True)
            original_capture()

        self.station._capture_release_hold = capture_once
        self.input(session, 1, (0.0, 0.0, 0.0), False)
        self.station.step()
        self.clock.advance(0.020)
        self.input(session, 2, (1.0, 0.0, 0.0), True)
        self.station.step()
        self.input(session, 3, (0.0, 0.0, 0.0), False)
        self.station.step()
        hold_after_release = self.station._hold_targets
        self.input(session, 4, (0.0, 0.0, 0.0), False)
        self.station.step()

        self.assertEqual(len(capture_calls), 1)
        self.assertEqual(self.station._hold_targets, hold_after_release)
        self.assertTrue(self.station.snapshot().hold_energized)
        self.assertEqual(self.station.snapshot().input_phase, "holding")

    def test_release_without_hold_target_is_not_reported_as_holding(self) -> None:
        session = self.connect_and_check()
        self.arm(session)
        self.station._hold_targets = None

        admitted = self.input(session, 1, (0.0, 0.0, 0.0), False)
        self.assertEqual(admitted.status, "accepted")
        self.station.step()

        terminal = self.station.receipt(admitted.request_id)
        self.assertEqual(terminal.receipt.status, "rejected")
        self.assertEqual(terminal.receipt.code, "NO_HOLD_TARGET")
        self.assertEqual(self.station.snapshot().input_phase, "hold_unavailable")

    def test_pending_input_gets_terminal_not_armed_receipt(self) -> None:
        session = self.connect_and_check()
        self.arm(session)
        admitted = self.input(session, 1, (0.5, 0.0, 0.0), True)
        self.assertEqual(admitted.status, "accepted")
        self.station._core.stop()

        self.station.step()

        terminal = self.station.receipt(admitted.request_id)
        self.assertEqual(terminal.status, "present")
        self.assertEqual(terminal.receipt.status, "rejected")
        self.assertEqual(terminal.receipt.code, "NOT_ARMED")

    def test_ticket_is_canonical_single_use_and_bounded(self) -> None:
        session = self.connect_and_check()
        self.arm(session)
        tickets = [self.station.issue_ticket(session, sequence) for sequence in range(1000)]
        self.assertEqual(self.station._ticket, tickets[-1])

        old = self.station.request(
            JogIntent(0, (0.0, 0.0, 0.0), False, True, "fixture-device-1", tickets[0]),
            session,
        )
        self.assertEqual(old.code, "INVALID_TICKET")
        self.assertEqual(self.station._ticket, tickets[-1])
        latest = self.station.request(
            JogIntent(999, (0.0, 0.0, 0.0), False, True, "fixture-device-1", tickets[-1]),
            session,
        )
        self.assertEqual(latest.status, "accepted")
        self.assertIsNone(self.station._ticket)
        replay = self.station.request(
            JogIntent(999, (0.0, 0.0, 0.0), False, True, "fixture-device-1", tickets[-1]),
            session,
        )
        self.assertEqual(replay.code, "INVALID_TICKET")
        self.assertLessEqual(len(self.station._receipts), 64)

    def test_forged_expiry_rejected_and_expired_owner_revoked(self) -> None:
        session = self.connect_and_check()
        self.arm(session)
        ticket = self.station.issue_ticket(session, 1)
        forged = replace(ticket, expires_at_s=ticket.expires_at_s + 1.0)
        receipt = self.station.request(
            JogIntent(1, (1.0, 0.0, 0.0), True, True, "fixture-device-1", forged),
            session,
        )
        self.assertEqual(receipt.code, "INVALID_TICKET")
        self.assertIsNone(self.station._ticket)

        expired = self.station.issue_ticket(session, 2)
        self.clock.advance(self.station.input_lease_s + 0.010)
        forged_expired = replace(expired, expires_at_s=self.clock() + 1.0)
        receipt = self.station.request(
            JogIntent(2, (1.0, 0.0, 0.0), True, True, "fixture-device-1", forged_expired),
            session,
        )
        self.assertEqual(receipt.code, "OWNER_EXPIRED")
        self.assertTrue(self.station.snapshot().stop_pending)
        self.station.step()
        self.assertTrue(self.station.snapshot().stop_logical)
        self.assertFalse(self.station.snapshot().hold_energized)

    def test_nonowner_forged_input_does_not_consume_owners_ticket(self) -> None:
        owner = self.connect_and_check()
        other = self.station.register_session("other")
        self.arm(owner)
        ticket = self.station.issue_ticket(owner, 1)

        rejected = self.station.request(
            JogIntent(1, (0.0, 0.0, 0.0), False, True, "fixture-device-1", ticket),
            other,
        )
        self.assertEqual(rejected.code, "NOT_OWNER")
        admitted = self.station.request(
            JogIntent(1, (0.0, 0.0, 0.0), False, True, "fixture-device-1", ticket),
            owner,
        )
        self.assertEqual(admitted.status, "accepted")

    def test_authority_seam_and_focus_loss_revoke_ingress_immediately(self) -> None:
        session = self.connect_and_check()
        self.arm(session)
        authority = self.station.dispatch_authority()
        self.assertFalse(authority.revoked)
        self.assertIsNotNone(authority.expires_at_s)

        ticket = self.station.issue_ticket(session, 1)
        rejected = self.station.request(
            JogIntent(1, (1.0, 0.0, 0.0), True, False, "fixture-device-1", ticket),
            session,
        )
        self.assertEqual(rejected.code, "FOCUS_LOST")
        self.assertTrue(self.station.dispatch_authority().revoked)
        self.assertTrue(self.station.snapshot().stop_pending)
        self.station.step()
        self.assertTrue(self.station.snapshot().stop_logical)

    def test_snapshot_is_immutable_and_never_reads_core_on_caller_thread(self) -> None:
        self.station.step()
        original_view = self.station.snapshot()
        self.assertIsInstance(original_view, ArmStationSnapshot)
        with self.assertRaises(FrozenInstanceError):
            original_view.lifecycle_state = "armed"

        core = self.station._core
        original_snapshot = core.snapshot
        core_threads: list[int] = []

        def tracked_snapshot():
            core_threads.append(threading.get_ident())
            return original_snapshot()

        core.snapshot = tracked_snapshot
        result: list[object] = []
        reader = threading.Thread(target=lambda: result.append(self.station.snapshot()))
        reader.start()
        reader.join(timeout=1.0)
        self.assertFalse(reader.is_alive())
        self.assertEqual(result, [original_view])
        self.assertEqual(core_threads, [])

        errors: list[Exception] = []
        wrong_executor = threading.Thread(
            target=lambda: self._capture_error(errors, self.station.step)
        )
        wrong_executor.start()
        wrong_executor.join(timeout=1.0)
        self.assertFalse(wrong_executor.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], RuntimeError)
        self.assertEqual(core_threads, [])

    @staticmethod
    def _capture_error(errors: list[Exception], action) -> None:
        try:
            action()
        except Exception as exc:
            errors.append(exc)

    def test_stop_requires_fresh_mode0_feedback_for_each_joint(self) -> None:
        session = self.connect_and_check()
        self.arm(session)
        self.fixture.inject_reply_modes = {1: 1}
        self.station.request(StopAction(), session)
        self.station.step()

        snap = self.station.snapshot()
        self.assertTrue(snap.stop_logical)
        self.assertFalse(snap.stop_confirmed)
        self.assertEqual(snap.per_joint_stop_confirmed, (True, False, True))
        self.assertEqual([item["mode"] for item in self.mode_writes(self.fixture, 0)[-3:]], [0, 0, 0])

    def test_partial_write_keeps_attempt_separate_from_dispatch_history(self) -> None:
        session = self.connect_and_check()
        self.arm(session)
        self.input(session, 1, (0.0, 0.0, 0.0), False)
        self.station.step()
        self.clock.advance(0.020)
        self.fixture.inject_partial_write_ids = {0}
        self.input(session, 2, (1.0, 0.0, 0.0), True)
        self.station.step()

        snap = self.station.snapshot()
        self.assertEqual(snap.lifecycle_state, State.FAULT.value)
        self.assertTrue(snap.fault_latched)
        self.assertIsNotNone(snap.dispatch_attempts[0])
        self.assertEqual(snap.dispatch_attempts[0].result, WriteResult.PARTIAL)
        self.assertIsNotNone(snap.dispatch_records[0])
        self.assertGreater(
            snap.dispatch_attempts[0].sequence,
            snap.dispatch_records[0].sequence,
        )
        self.assertEqual(snap.per_joint_stop_confirmed, (False, True, True))

    def test_failed_close_keeps_owner_and_retry_confirms_close(self) -> None:
        session = self.connect_and_check()
        self.fixture.fail_close = True
        self.assertFalse(self.station.close())
        failed = self.station.snapshot()
        self.assertTrue(failed.close_attempted)
        self.assertTrue(failed.close_unconfirmed)
        self.assertFalse(failed.close_confirmed)
        self.assertEqual(failed.owner_session_id, session.session_id)

        other = self.station.register_session("other")
        blocked = self.station.request(ConnectAction(), other)
        self.assertEqual(blocked.code, "CLOSE_IN_PROGRESS")
        self.fixture.fail_close = False
        self.assertTrue(self.station.close())
        closed = self.station.snapshot()
        self.assertTrue(closed.close_confirmed)
        self.assertIsNone(closed.owner_session_id)

    def test_confirmed_disconnect_releases_owner_for_new_session(self) -> None:
        owner = self.connect_and_check()
        self.arm(owner)
        previous_input = self.input(owner, 41, (0.0, 0.0, 0.0), False)
        self.assertEqual(previous_input.status, "accepted")
        self.station.step()
        self.assertEqual(
            self.station.receipt(previous_input.request_id).receipt.code,
            "HOLDING",
        )

        disconnect = self.station.request(DisconnectAction(), owner)
        self.station.step()

        self.assertEqual(self.station.receipt(disconnect.request_id).receipt.code, "DISCONNECTED")
        self.assertEqual(self.station.snapshot().lifecycle_state, State.DISCONNECTED.value)
        self.assertIsNone(self.station.snapshot().owner_session_id)

        next_owner = self.station.register_session("next owner")
        connect = self.station.request(ConnectAction(), next_owner)
        self.assertEqual(connect.status, "queued")
        self.station.step()
        self.assertEqual(self.station.snapshot().lifecycle_state, State.CONNECTED.value)
        self.assertEqual(self.station.snapshot().owner_session_id, next_owner.session_id)
        self.assertEqual(self.station.request(CheckAction(), next_owner).status, "queued")
        self.station.step()
        self.clock.advance(0.020)
        self.assertEqual(self.station.request(ArmAction(), next_owner).status, "queued")
        self.station.step()
        next_ticket = self.station.issue_ticket(next_owner, 1)
        next_input = self.station.request(
            JogIntent(
                sequence=1,
                axes=(0.0, 0.0, 0.0),
                deadman=False,
                focus=True,
                device_epoch="next-device",
                ticket=next_ticket,
            ),
            next_owner,
        )
        self.assertEqual(next_input.status, "accepted")
        self.station.step()
        self.assertEqual(
            self.station.receipt(next_input.request_id).receipt.code,
            "HOLDING",
        )

    def test_ready_telemetry_stays_fresh_and_explicit_arm_reanchors_after_idle(self) -> None:
        session = self.connect_and_check()
        for _ in range(100):
            self.clock.advance(0.020)
            self.station.step()
        self.assertEqual(self.station.snapshot().lifecycle_state, State.READY.value)
        self.assertEqual(self.station.snapshot().faults, ())

        old_references = self.station.snapshot().confirmed_reference
        self.clock.advance(2.0)
        self.station.request(ArmAction(), session)
        self.station.step()
        snap = self.station.snapshot()
        self.assertEqual(snap.lifecycle_state, State.ARMED.value)
        self.assertTrue(snap.hold_energized)
        self.assertEqual(snap.faults, ())
        self.assertTrue(snap.stop_confirmed)
        self.assertTrue(all(ref is not None for ref in snap.confirmed_reference))
        self.assertTrue(
            all(
                current.sample_time_s > previous.sample_time_s
                for current, previous in zip(snap.confirmed_reference, old_references)
                if current is not None and previous is not None
            )
        )

    def test_stale_ready_transport_still_faults_and_attempts_every_stop(self) -> None:
        self.connect_and_check()
        self.fixture.inject_no_reply_ids = {0}
        self.clock.advance(0.600)
        self.station.step()

        snap = self.station.snapshot()
        self.assertEqual(snap.lifecycle_state, State.FAULT.value)
        self.assertTrue(snap.fault_latched)
        self.assertEqual(snap.per_joint_stop_confirmed, (False, True, True))

    def test_fault_and_protection_history_survive_disconnect_and_reconnect(self) -> None:
        session = self.connect_and_check()
        self.fixture.inject_temperatures = {0: (90, 30)}
        self.clock.advance(0.020)
        self.station.step()
        fault = self.station.snapshot()
        self.assertEqual(fault.lifecycle_state, State.FAULT.value)
        self.assertTrue(fault.fault_latched)
        self.assertTrue(fault.stop_logical)
        self.assertTrue(fault.stop_confirmed)

        self.fixture.inject_temperatures = {}
        self.station.request(DisconnectAction(), session)
        self.station.step()
        self.assertEqual(self.station.snapshot().lifecycle_state, State.DISCONNECTED.value)
        self.assertTrue(self.station.snapshot().fault_latched)

        self.station.request(ConnectAction(), session)
        self.station.step()
        reconnected = self.station.snapshot()
        self.assertEqual(reconnected.connection_phase, "fixture")
        self.assertEqual(reconnected.lifecycle_state, State.FAULT.value)
        self.assertTrue(reconnected.fault_latched)
        self.assertTrue(reconnected.stop_confirmed)

        rejected = self.station.request(CheckAction(acknowledge_fault=False), session)
        self.assertEqual(rejected.status, "queued")
        self.station.step()
        self.assertEqual(self.station.snapshot().latest_receipt.code, "fault_ack_required")
        self.assertTrue(self.station.snapshot().fault_latched)

    def test_physical_draft_finite_boundaries_and_commissioning_phase(self) -> None:
        good = self.valid_physical_draft()
        self.assertTrue(validate_physical_draft(good)[0])
        self.assertEqual(
            (create_synthetic_arm_specs()[0].q_min_rad,
             create_synthetic_arm_specs()[0].q_max_rad),
            (J1_MIN_RAD, J1_MAX_RAD),
        )
        specs = create_synthetic_arm_specs()
        self.assertEqual((specs[1].q_min_rad, specs[1].q_max_rad),
                         (SYNTHETIC_J2_MIN_RAD, SYNTHETIC_J2_MAX_RAD))
        self.assertEqual((specs[2].q_min_rad, specs[2].q_max_rad),
                         (SYNTHETIC_J3_MIN_RAD, SYNTHETIC_J3_MAX_RAD))
        self.assertAlmostEqual(compute_zero_output(0.6, -1, 0.5), 1.1)
        with self.assertRaises(ValueError):
            compute_zero_output(math.inf, 1, 0.0)

        bad_joints = list(good.joints)
        bad_joints[0] = replace(bad_joints[0], q_min_rad=math.nan)
        self.assertFalse(validate_physical_draft(replace(good, joints=tuple(bad_joints)))[0])
        bad_joints = list(good.joints)
        bad_joints[1] = replace(bad_joints[1], v_max_rad_s=math.inf)
        self.assertFalse(validate_physical_draft(replace(good, joints=tuple(bad_joints)))[0])
        bad_joints = list(good.joints)
        bad_joints[2] = replace(bad_joints[2], kp=math.inf)
        self.assertFalse(validate_physical_draft(replace(good, joints=tuple(bad_joints)))[0])
        bad_joints = list(good.joints)
        bad_joints[0] = replace(bad_joints[0], direction=True)
        self.assertFalse(validate_physical_draft(replace(good, joints=tuple(bad_joints)))[0])
        bad_joints = list(good.joints)
        bad_joints[0] = replace(
            bad_joints[0], q_min_rad=J1_MIN_RAD - 0.01, q_max_rad=J1_MAX_RAD
        )
        self.assertFalse(validate_physical_draft(replace(good, joints=tuple(bad_joints)))[0])

        self.assertGreater(good.joints[1].q_max_rad - good.joints[1].q_min_rad, math.tau)
        self.assertTrue(validate_physical_draft(good)[0])

        fixture = ArmPacketFixture(clock=self.clock)
        fixture.is_open = False
        station = ArmStation(clock=self.clock, fixture=fixture)
        session = station.register_session("owner")
        incomplete = replace(good, joints=good.joints[:1])
        self.assertFalse(validate_physical_draft(incomplete)[0])
        station.request(ConnectAction(source_kind="physical", draft=incomplete), session)
        station.step()
        self.assertTrue(station.snapshot().commissioning_blocked)
        self.assertEqual(station.snapshot().lifecycle_state, State.DISCONNECTED.value)
        self.assertFalse(fixture.is_open)
        self.assertEqual(fixture.writes, ())

        station.request(ConnectAction(source_kind="physical", draft=good), session)
        station.step()
        self.assertEqual(station.snapshot().latest_receipt.code, "COMMISSIONING_REQUIRED")
        self.assertEqual(station.snapshot().lifecycle_state, State.DISCONNECTED.value)
        self.assertIsNone(station.snapshot().source)
        self.assertFalse(fixture.is_open)
        self.assertEqual(fixture.writes, ())
        self.assertIsNone(station.snapshot().owner_session_id)

        next_session = station.register_session("next client")
        connect = station.request(
            ConnectAction(source_kind="physical", draft=good), next_session
        )
        self.assertEqual(connect.status, "queued")
        station.step()
        self.assertEqual(station.snapshot().latest_receipt.code, "COMMISSIONING_REQUIRED")
        self.assertEqual(station.snapshot().lifecycle_state, State.DISCONNECTED.value)
        self.assertIsNone(station.snapshot().owner_session_id)

    def test_background_executor_exception_records_logical_stop(self) -> None:
        fixture = ArmPacketFixture()
        fixture.inject_reply_modes = {1: 1}
        station = ArmStation(fixture=fixture)
        session = station.register_session("owner")
        station.start_loop()
        station.request(ConnectAction(), session)
        self.assertTrue(self.wait_until(
            lambda: station.snapshot().lifecycle_state == State.CONNECTED.value
        ))

        original_step = station._step_once
        should_raise = [True]

        def fail_one_step(*, missed_slots: int = 0):
            if should_raise[0]:
                should_raise[0] = False
                raise RuntimeError("private test detail")
            return original_step(missed_slots=missed_slots)

        station._step_once = fail_one_step
        self.assertTrue(self.wait_until(lambda: station.snapshot().executor_failed))
        thread = station._loop_thread
        self.assertIsNotNone(thread)
        thread.join(timeout=1.0)
        snap = station.snapshot()
        self.assertFalse(thread.is_alive())
        self.assertEqual(snap.lifecycle_state, State.DISCONNECTED.value)
        self.assertTrue(snap.stop_logical)
        self.assertFalse(snap.stop_confirmed)
        self.assertFalse(snap.stop_pending)
        self.assertEqual(snap.input_phase, "executor_failed")
        self.assertTrue(snap.close_confirmed)
        self.assertEqual(snap.per_joint_stop_confirmed, (True, False, True))
        stop_writes = self.mode_writes(fixture, 0)
        self.assertEqual([write["motor_id"] for write in stop_writes], [0, 1, 2])
        self.assertEqual(self.mode_writes(fixture, 1), [])
        self.assertEqual(snap.stop_attempts[1].outcome, "MODE_MISMATCH")
        self.assertEqual(snap.stop_attempts[1].feedback_mode, 1)
        self.assertTrue(snap.stop_attempts[1].feedback_received)
        self.assertFalse(snap.stop_attempts[1].confirmed)

    def test_executor_failure_attempts_stop_when_core_clock_is_invalid(self) -> None:
        clock_invalid = [False]

        def invalidatable_clock() -> float:
            return math.nan if clock_invalid[0] else 10.0

        fixture = ArmPacketFixture(clock=invalidatable_clock)
        station = ArmStation(clock=invalidatable_clock, fixture=fixture)
        session = station.register_session("owner")
        station.start_loop()
        station.request(ConnectAction(), session)
        self.assertTrue(self.wait_until(
            lambda: station.snapshot().lifecycle_state == State.CONNECTED.value
        ))

        clock_invalid[0] = True
        self.assertTrue(self.wait_until(lambda: station.snapshot().executor_failed))
        thread = station._loop_thread
        self.assertIsNotNone(thread)
        thread.join(timeout=1.0)

        snap = station.snapshot()
        self.assertFalse(thread.is_alive())
        self.assertTrue(snap.close_confirmed)
        self.assertTrue(snap.stop_logical)
        self.assertFalse(snap.stop_confirmed)
        self.assertEqual(snap.per_joint_stop_confirmed, (False, False, False))
        self.assertEqual(
            [write["motor_id"] for write in self.mode_writes(fixture, 0)],
            [0, 1, 2],
        )
        self.assertTrue(all(attempt.attempted for attempt in snap.stop_attempts))
        self.assertTrue(all(attempt.feedback_received for attempt in snap.stop_attempts))
        self.assertTrue(all(not attempt.confirmed for attempt in snap.stop_attempts))

    def test_executor_failure_keeps_executor_for_close_retry(self) -> None:
        fixture = ArmPacketFixture()
        station = ArmStation(fixture=fixture)
        session = station.register_session("owner")
        station.start_loop()
        station.request(ConnectAction(), session)
        self.assertTrue(self.wait_until(
            lambda: station.snapshot().lifecycle_state == State.CONNECTED.value
        ))

        fixture.fail_close = True
        original_step = station._step_once
        should_raise = [True]

        def fail_one_step(*, missed_slots: int = 0):
            if should_raise[0]:
                should_raise[0] = False
                raise RuntimeError("private close retry test detail")
            return original_step(missed_slots=missed_slots)

        station._step_once = fail_one_step
        self.assertTrue(self.wait_until(lambda: station.snapshot().close_unconfirmed))
        thread = station._loop_thread
        self.assertIsNotNone(thread)
        self.assertTrue(thread.is_alive())
        self.assertTrue(fixture.is_open)

        fixture.fail_close = False
        self.assertTrue(station.close())
        thread.join(timeout=1.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(station.snapshot().close_confirmed)
        self.assertFalse(fixture.is_open)

    def test_arm_exception_clears_transition_and_finishes_receipt(self) -> None:
        session = self.connect_and_check()
        original_arm = self.station._core.arm

        def fail_arm(owner):
            raise RuntimeError("arm transition fixture failure")

        self.station._core.arm = fail_arm
        receipt = self.station.request(ArmAction(), session)
        with self.assertRaisesRegex(RuntimeError, "arm transition fixture failure"):
            self.station.step()

        self.station._core.arm = original_arm
        terminal = self.station.receipt(receipt.request_id)
        self.assertEqual(terminal.status, "present")
        self.assertEqual(terminal.receipt.status, "rejected")
        self.assertEqual(terminal.receipt.code, "EXECUTOR_FAILED")
        self.assertFalse(self.station._arm_transition)
        self.assertTrue(self.station.snapshot().executor_failed)
        self.assertTrue(self.station.snapshot().close_confirmed)
        self.assertEqual(
            [write["motor_id"] for write in self.mode_writes(self.fixture, 0)[-3:]],
            [0, 1, 2],
        )

    def test_stop_loop_closes_before_executor_exits(self) -> None:
        fixture = ArmPacketFixture()
        station = ArmStation(fixture=fixture)
        session = station.register_session("owner")
        station.start_loop()
        station.request(ConnectAction(), session)
        self.assertTrue(self.wait_until(
            lambda: station.snapshot().lifecycle_state == State.CONNECTED.value
        ))

        station.stop_loop()
        thread = station._loop_thread
        self.assertIsNotNone(thread)
        self.assertFalse(thread.is_alive())
        self.assertTrue(station.snapshot().close_confirmed)
        station.stop_loop()
        self.assertFalse(station.snapshot().stop_pending)
        self.assertFalse(fixture.is_open)
        self.assertTrue(station.close())

    def test_stop_ingress_does_not_wait_for_executor_exchange(self) -> None:
        fixture = ArmPacketFixture()
        station = ArmStation(fixture=fixture)
        owner = station.register_session("owner")
        other = station.register_session("stopper")
        station.start_loop()
        station.request(ConnectAction(), owner)
        self.assertTrue(self.wait_until(
            lambda: station.snapshot().lifecycle_state == State.CONNECTED.value
        ))

        write_entered = threading.Event()
        release_write = threading.Event()
        block_lock = threading.Lock()
        block_next = [True]
        original_write = fixture.write

        def blocked_write(data):
            with block_lock:
                should_block = block_next[0]
                block_next[0] = False
            if should_block:
                write_entered.set()
                release_write.wait(timeout=2.0)
            return original_write(data)

        fixture.write = blocked_write
        station.request(CheckAction(), owner)
        self.assertTrue(write_entered.wait(timeout=1.0))

        result: list[object] = []
        stop_done = threading.Event()

        def submit_stop():
            result.append(station.request(StopAction(), other))
            stop_done.set()

        stop_thread = threading.Thread(target=submit_stop, daemon=True)
        stop_thread.start()
        completed_before_release = stop_done.wait(timeout=0.100)
        self.assertTrue(station.snapshot().stop_pending)
        release_write.set()
        stop_thread.join(timeout=1.0)
        self.assertFalse(stop_thread.is_alive())
        self.assertTrue(completed_before_release)
        self.assertEqual(result[0].code, "STOP_QUEUED")
        self.assertTrue(self.wait_until(
            lambda: station.snapshot().stop_logical
            and not station.snapshot().stop_pending
        ))
        self.assertTrue(station.snapshot().stop_confirmed)
        self.assertTrue(station.close())

    @staticmethod
    def wait_until(predicate, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.005)
        return bool(predicate())


if __name__ == "__main__":
    unittest.main()
