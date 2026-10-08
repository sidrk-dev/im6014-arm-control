
from dataclasses import FrozenInstanceError
import unittest

from im6014_codec import command
from joint_contract import (
    CommunicationFailure,
    DecisionCode,
    JointSpec,
    MotorHealth,
    OwnerToken,
    SourceIdentity,
    State,
    Target,
    WriteResult,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointTarget
from test_controller import DEFAULT_PROTECTION, DeterministicClock, FakeStation, HEALTHY_STOP, SAMPLE


SOURCE = SourceIdentity("fixture", "owner-test", 1)


class ControllerOwnerTests(unittest.TestCase):
    def setUp(self):
        self.clock = DeterministicClock(0.0)
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), -2.0, 2.0, 10.0, 100.0, 1.0, 50.0, "fixture: software position limits", DEFAULT_PROTECTION),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), -2.0, 2.0, 10.0, 100.0, 1.0, 50.0, "fixture: software position limits", DEFAULT_PROTECTION),
        )
        self.valid_joints = (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        )

    def _armed_station(self, connection_id="human"):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim(connection_id)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, owner

    def test_owner_token_validation_and_immutability(self):
        tok = OwnerToken("epoch-1", 1, "client-1")
        self.assertEqual(tok.epoch, "epoch-1")
        self.assertEqual(tok.generation, 1)
        self.assertEqual(tok.connection_id, "client-1")

        with self.assertRaises(FrozenInstanceError):
            tok.generation = 2
        with self.assertRaises(FrozenInstanceError):
            tok.epoch = "other"
        with self.assertRaises(FrozenInstanceError):
            tok.connection_id = "other"

        for bad_epoch in ("", "a" * 65, 123, None, True, b"epoch"):
            with self.subTest(bad_epoch=bad_epoch), self.assertRaises(ValueError):
                OwnerToken(bad_epoch, 1, "client")

        for bad_gen in (-1, 0, 1 << 64, True, False, 1.5, "1", None):
            with self.subTest(bad_gen=bad_gen), self.assertRaises(ValueError):
                OwnerToken("epoch", bad_gen, "client")

        for bad_conn in ("", "c" * 65, 123, None, False, b"conn"):
            with self.subTest(bad_conn=bad_conn), self.assertRaises(ValueError):
                OwnerToken("epoch", 1, bad_conn)

    def test_delayed_old_generation_rejected_after_stop_and_rearm(self):
        core, station, old_owner = self._armed_station("conn-1")
        w1 = station.issue_window(1.0)
        t1 = Target(1, self.valid_joints, window=w1, owner=old_owner)
        self.assertTrue(station.submit(t1).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().current_owner)

        new_owner = station.claim("conn-1")
        self.assertNotEqual(old_owner.generation, new_owner.generation)
        self.assertTrue(new_owner.generation > old_owner.generation)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(new_owner).accepted)

        w2 = station.issue_window(1.0)
        stale_target = Target(2, self.valid_joints, window=w2, owner=old_owner)
        rej = station.submit(stale_target)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_OWNER)
        self.assertEqual(core.snapshot().last_sequence, 1)

        valid_target = Target(2, self.valid_joints, window=w2, owner=new_owner)
        acc = station.submit(valid_target)
        self.assertTrue(acc.accepted)
        self.assertEqual(core.snapshot().last_sequence, 2)
        self.assertEqual(core.snapshot().admitted_target, valid_target)

    def test_wrong_epoch_and_wrong_connection_rejected(self):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("client-A")
        self.assertTrue(station.check().accepted)
        w = station.issue_window(1.0)

        wrong_epoch = OwnerToken("wrong-epoch", owner.generation, owner.connection_id)
        wrong_conn = OwnerToken(owner.epoch, owner.generation, "client-B")

        rej_arm_epoch = core.arm(wrong_epoch)
        self.assertFalse(rej_arm_epoch.accepted)
        self.assertEqual(rej_arm_epoch.code, DecisionCode.INVALID_OWNER)

        rej_arm_conn = core.arm(wrong_conn)
        self.assertFalse(rej_arm_conn.accepted)
        self.assertEqual(rej_arm_conn.code, DecisionCode.INVALID_OWNER)

        self.assertTrue(station.arm(owner).accepted)
        rej_epoch = station.submit(Target(1, self.valid_joints, window=w, owner=wrong_epoch))
        self.assertFalse(rej_epoch.accepted)
        self.assertEqual(rej_epoch.code, DecisionCode.INVALID_OWNER)

        rej_conn = station.submit(Target(1, self.valid_joints, window=w, owner=wrong_conn))
        self.assertFalse(rej_conn.accepted)
        self.assertEqual(rej_conn.code, DecisionCode.INVALID_OWNER)

    def test_same_sequence_old_token_cannot_affect_ordering(self):
        core, station, owner1 = self._armed_station("conn-1")
        w1 = station.issue_window(1.0)
        t5 = Target(5, self.valid_joints, window=w1, owner=owner1)
        self.assertTrue(station.submit(t5).accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)

        self.assertTrue(station.stop().accepted)
        owner2 = station.claim("conn-2")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner2).accepted)

        w2 = station.issue_window(1.0)
        old_t5 = Target(5, self.valid_joints, window=w2, owner=owner1)
        rej = station.submit(old_t5)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_OWNER)
        self.assertEqual(core.snapshot().last_sequence, 5)

        new_t6 = Target(6, self.valid_joints, window=w2, owner=owner2)
        self.assertTrue(station.submit(new_t6).accepted)
        self.assertEqual(core.snapshot().last_sequence, 6)

    def test_different_core_epoch_isolation(self):
        with self.assertRaises(TypeError):
            JointCore(self.specs, clock=self.clock, epoch="injected", max_reference_gap_s=1.0)

        core1 = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        core2 = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station1 = FakeStation(core1)
        self.assertTrue(station1.connect().accepted)
        owner1 = station1.claim("conn-1")
        self.assertTrue(station1.check().accepted)
        self.assertTrue(station1.arm(owner1).accepted)

        station2 = FakeStation(core2)
        self.assertTrue(station2.connect().accepted)
        owner2 = station2.claim("conn-2")
        self.assertNotEqual(owner1.epoch, owner2.epoch)
        self.assertTrue(station2.check().accepted)

        cross_arm = core2.arm(owner1)
        self.assertFalse(cross_arm.accepted)
        self.assertEqual(cross_arm.code, DecisionCode.INVALID_OWNER)

        self.assertTrue(station2.arm(owner2).accepted)

        w2 = station2.issue_window(1.0)
        cross_target = Target(1, self.valid_joints, window=w2, owner=owner1)
        rej = station2.submit(cross_target)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_OWNER)

    def test_no_owner_arm_rejected(self):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertTrue(core.begin_check().accepted)
        for spec in self.specs:
            self.assertTrue(core.receive(spec.motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertIs(core.snapshot().state, State.READY)

        bogus_token = OwnerToken("fake-epoch", 1, "fake-client")
        rej = core.arm(bogus_token)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_OWNER)
        self.assertIs(core.snapshot().state, State.READY)

        missing_owner = core.arm(None)
        self.assertFalse(missing_owner.accepted)
        self.assertEqual(missing_owner.code, DecisionCode.INVALID_OWNER)

        real_owner = core.claim_owner("valid-client")
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertTrue(core.begin_check().accepted)
        for spec in self.specs:
            self.assertTrue(core.receive(spec.motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(core.arm(real_owner).accepted)
        self.assertIs(core.snapshot().state, State.ARMED)

    def test_takeover_during_vector_blocks_next_joint(self):
        core, station, owner_a = self._armed_station("owner-A")
        w = station.issue_window(1.0)
        target = Target(1, self.valid_joints, window=w, owner=owner_a)
        self.assertTrue(station.submit(target).accepted)

        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)

        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        d0 = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)
        self.assertTrue(d0.accepted)
        self.assertIs(core.snapshot().state, State.MOVING)

        owner_b = core.claim_owner("owner-B")
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().admitted_target)
        self.assertIsNone(core.snapshot().prepared_plan)
        self.assertEqual(core.snapshot().current_owner, owner_b)

        self.assertFalse(core.claim_dispatch(plan.plan_id, 1))

    def test_receipt_after_revoke_records_actual_write_but_no_motion(self):
        core, station, owner = self._armed_station("owner-1")
        w = station.issue_window(1.0)
        target = Target(1, self.valid_joints, window=w, owner=owner)
        self.assertTrue(station.submit(target).accepted)

        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        claim_time = self.clock.value

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.STOPPED)

        receipt = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)
        self.assertTrue(receipt.accepted)

        snap = core.snapshot()
        self.assertIs(snap.state, State.STOPPED)
        self.assertIsNone(snap.admitted_target)
        record = snap.dispatched_records[0]
        self.assertIsNotNone(record)
        self.assertEqual(record.sequence, 1)
        self.assertEqual(record.owner, owner)
        self.assertEqual(record.dispatched_at_s, claim_time)
        self.assertEqual(snap.dispatch_attempts[0].owner, owner)

    def test_old_prepared_plan_not_revived_by_new_owner(self):
        core, station, owner_a = self._armed_station("owner-A")
        w_a = station.issue_window(1.0)
        target_a = Target(1, self.valid_joints, window=w_a, owner=owner_a)
        self.assertTrue(station.submit(target_a).accepted)

        self.clock.value += 0.05
        plan_a = core.prepare_refresh()
        self.assertIsNotNone(plan_a)

        owner_b = station.claim("owner-B")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner_b).accepted)

        self.assertFalse(core.claim_dispatch(plan_a.plan_id, 0))
        self.assertFalse(core.dispatch_result(plan_a.plan_id, 0, WriteResult.COMPLETE).accepted)

        w_b = station.issue_window(1.0)
        target_b = Target(2, self.valid_joints, window=w_b, owner=owner_b)
        self.assertTrue(station.submit(target_b).accepted)
        self.clock.value += 0.05
        plan_b = core.prepare_refresh()
        self.assertIsNotNone(plan_b)
        self.assertEqual(plan_b.target, target_b)
        self.assertEqual(plan_b.owner, owner_b)

    def test_repeated_stop_reconnect_and_fault_invalidation(self):
        core, station, owner1 = self._armed_station("owner-1")
        self.assertEqual(owner1.generation, 1)

        self.assertTrue(station.stop().accepted)
        self.assertIsNone(core.snapshot().current_owner)

        owner2 = station.claim("owner-2")
        self.assertEqual(owner2.generation, 2)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner2).accepted)

        self.assertTrue(core.communication_failed(0, CommunicationFailure.TIMEOUT).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertIsNone(core.snapshot().current_owner)

        self.assertTrue(station.disconnect().accepted)
        self.assertIs(core.snapshot().state, State.DISCONNECTED)
        self.assertIsNone(core.snapshot().current_owner)

        self.assertTrue(station.connect().accepted)
        self.assertIsNone(core.snapshot().current_owner)

        owner3 = station.claim("owner-3")
        self.assertEqual(owner3.generation, 3)
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        self.assertTrue(station.arm(owner3).accepted)
        self.assertIs(core.snapshot().state, State.ARMED)
        self.assertEqual(core.snapshot().current_owner, owner3)

    def test_exhausted_generation_fails_closed(self):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        core._next_generation = (1 << 64) - 1
        tok_max = station.claim("client")
        self.assertEqual(tok_max.generation, (1 << 64) - 1)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(tok_max).accepted)

        window = station.issue_window(1.0)
        target = Target(1, self.valid_joints, window=window, owner=tok_max)
        self.assertTrue(station.submit(target).accepted)
        self.clock.value += 0.05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        self.assertTrue(core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE).accepted)
        self.assertIs(core.snapshot().state, State.MOVING)

        with self.assertRaises(OverflowError):
            core.claim_owner("client-overflow")

        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.STOPPED)
        self.assertIsNone(snapshot.current_owner)
        self.assertIsNone(snapshot.admitted_target)
        self.assertIsNone(snapshot.prepared_plan)
        self.assertEqual(snapshot.latest_owner, tok_max)
        self.assertFalse(core.claim_dispatch(plan.plan_id, 1))

    def test_invalid_high_sequence_stale_owner_preserves_healthy_current_goal(self):
        core, station, owner = self._armed_station("owner-1")
        w = station.issue_window(1.0)
        t5 = Target(5, self.valid_joints, window=w, owner=owner)
        self.assertTrue(station.submit(t5).accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

        stale_owner = OwnerToken("other-epoch", 999, "stale-client")
        high_target = Target(9999, self.valid_joints, window=w, owner=stale_owner)
        rej = station.submit(high_target)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_OWNER)

        snap = core.snapshot()
        self.assertEqual(snap.last_sequence, 5)
        self.assertEqual(snap.admitted_target, t5)
        self.assertIs(snap.state, State.ARMED)
        self.assertEqual(snap.current_owner, owner)

    def test_bounded_storage_after_many_claims(self):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=1.0)
        for i in range(10_000):
            token = core.claim_owner(f"client-{i % 10}")

        snap = core.snapshot()
        self.assertEqual(snap.latest_owner.generation, 10_000)
        self.assertEqual(snap.current_owner.generation, 10_000)

        self.assertFalse(hasattr(core, "_owner_history"))
        self.assertFalse(hasattr(core, "_owners"))
        self.assertFalse(hasattr(core, "claim"))
        self.assertIs(core._latest_owner, snap.latest_owner)
        self.assertFalse(hasattr(snap, "generation"))
        self.assertFalse(hasattr(snap, "latest_generation"))
        self.assertIs(core._current_owner, snap.current_owner)


if __name__ == "__main__":
    unittest.main()
