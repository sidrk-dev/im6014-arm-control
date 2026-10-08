import unittest
from dataclasses import FrozenInstanceError
import math
from im6014_codec import command
from joint_contract import (
    CommunicationFailure,
    DecisionCode,
    IntentWindow,
    JointSpec,
    MotorHealth,
    OwnerToken,
    State,
    Target,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointTarget

from test_controller import DEFAULT_PROTECTION, DeterministicClock, FakeStation, SAMPLE, HEALTHY_STOP, core_at


class LatestTargetTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), -1e8, 1e8, 1e8, 1e8, 1e8, 1e8, "fixture: wide test limits", DEFAULT_PROTECTION),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), -1e8, 1e8, 1e8, 1e8, 1e8, 1e8, "fixture: wide test limits", DEFAULT_PROTECTION),
        )

    def test_ten_thousand_accepted_updates_leave_one_newest_target_and_repeated_refresh(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=1.2, max_reference_gap_s=1.2)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)
        self.assertIsNone(core.snapshot().admitted_target)

        window = station.issue_window(1.0)
        last_target = None
        for i in range(10_000):
            target = Target(
                i,
                (
                    JointTarget(q_rad=i * 0.0001, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
                    JointTarget(q_rad=-i * 0.0001, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
                ),
                window=window,
                owner=owner,
            )
            decision = station.submit(target)
            self.assertTrue(decision.accepted)
            last_target = target

        snap = core.snapshot()
        self.assertEqual(snap.admitted_target, last_target)
        self.assertEqual(snap.last_sequence, 9999)
        self.assertIs(snap.state, State.ARMED)

        clock.value = 0.05
        refreshed = station.refresh()
        self.assertIsNotNone(refreshed)
        self.assertEqual(refreshed.target, last_target)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertEqual(core.snapshot().admitted_target, last_target)

        second_refreshed = station.refresh()
        self.assertIsNotNone(second_refreshed)
        self.assertEqual(second_refreshed.target, last_target)
        self.assertIs(core.snapshot().state, State.MOVING)

        clock.value = 1.1
        expired_refreshed = station.refresh()
        self.assertIsNone(expired_refreshed)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().admitted_target)

    def test_immutable_owned_vector(self):
        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        window = IntentWindow(0, 0.0, 1.0)
        owner = OwnerToken("epoch-1", 1, "fixture")

        for bad_joints in (
            [jt1, jt2],
            "not a tuple",
            123,
            None,
            {jt1, jt2},
        ):
            with self.subTest(bad_joints=repr(bad_joints)), self.assertRaises(ValueError):
                Target(0, bad_joints, window=window, owner=owner)

        with self.assertRaises(ValueError):
            Target(0, (), window=window, owner=owner)

        for bad_elements in (
            (jt1, "not a joint target"),
            (1, 2),
            (jt1, None),
        ):
            with self.subTest(bad_elements=repr(bad_elements)), self.assertRaises(ValueError):
                Target(0, bad_elements, window=window, owner=owner)

        for bad_window in (None, "window", 123, object()):
            with self.subTest(bad_window=repr(bad_window)), self.assertRaises(ValueError):
                Target(0, (jt1, jt2), window=bad_window, owner=owner)

        for bad_owner in (None, "owner", 123, object()):
            with self.subTest(bad_owner=repr(bad_owner)), self.assertRaises(ValueError):
                Target(0, (jt1, jt2), window=window, owner=bad_owner)

        target = Target(0, (jt1, jt2), window=window, owner=owner)
        with self.assertRaises(FrozenInstanceError):
            target.sequence = 1
        with self.assertRaises(FrozenInstanceError):
            target.joints = (jt2, jt1)
        with self.assertRaises(FrozenInstanceError):
            target.window = window
        with self.assertRaises(FrozenInstanceError):
            target.owner = owner
        with self.assertRaises(TypeError):
            target.joints[0] = jt2
        with self.assertRaises(FrozenInstanceError):
            target.joints[0].q_rad = 99.0

        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        station_owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(station_owner).accepted)
        core_window = station.issue_window(1.0)
        valid_target = Target(0, (jt1, jt2), window=core_window, owner=station_owner)
        self.assertTrue(station.submit(valid_target).accepted)

        snap = core.snapshot()
        self.assertEqual(snap.admitted_target, valid_target)
        self.assertEqual(snap.last_sequence, 0)
        with self.assertRaises(FrozenInstanceError):
            snap.admitted_target = None
        with self.assertRaises(FrozenInstanceError):
            snap.last_sequence = 1
        with self.assertRaises(TypeError):
            snap.admitted_target.joints[0] = jt1
        with self.assertRaises(FrozenInstanceError):
            snap.admitted_target.joints[0].tau_nm = 0.5

    def test_malformed_incomplete_and_invalid_mapped_outputs_preserve_prior_target(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        w1 = station.issue_window(1.0)
        prior_target = Target(1, (jt1, jt2), window=w1, owner=owner)
        self.assertTrue(station.submit(prior_target).accepted)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        for malformed in (None, "target", 42, object(), [jt1, jt2]):
            with self.subTest(malformed=repr(malformed)):
                decision = core.submit_target(malformed)
                self.assertFalse(decision.accepted)
                self.assertEqual(decision.code, DecisionCode.INVALID_ARGUMENT)
                self.assertEqual(core.snapshot().admitted_target, prior_target)
                self.assertEqual(core.snapshot().last_sequence, 1)

        for wrong_vector in (
            Target(2, (jt1,), window=w1, owner=owner),
            Target(2, (jt1, jt2, jt1), window=w1, owner=owner),
        ):
            with self.subTest(wrong_vector=wrong_vector):
                decision = core.submit_target(wrong_vector)
                self.assertFalse(decision.accepted)
                self.assertEqual(decision.code, DecisionCode.INVALID_ARGUMENT)
                self.assertEqual(core.snapshot().admitted_target, prior_target)
                self.assertEqual(core.snapshot().last_sequence, 1)

        huge_speed = Target(2, (JointTarget(0.0, 1e7, 0.0, 1.0, 0.1), jt2), window=w1, owner=owner)
        rej = core.submit_target(huge_speed)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        huge_kp = Target(2, (JointTarget(0.0, 0.0, 0.0, 31.0, 0.1), jt2), window=w1, owner=owner)
        rej = core.submit_target(huge_kp)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        huge_kd = Target(2, (JointTarget(0.0, 0.0, 0.0, 1.0, 6.1), jt2), window=w1, owner=owner)
        rej = core.submit_target(huge_kd)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        huge_pos = Target(2, (JointTarget(1e7, 0.0, 0.0, 1.0, 0.1), jt2), window=w1, owner=owner)
        rej = core.submit_target(huge_pos)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

    def test_each_fault_path_clears_pending_without_fake_foc_writes(self):
        for action in ('packet', 'communication', 'tick'):
            with self.subTest(action=action):
                clock = DeterministicClock(0.0)
                core = JointCore(self.specs, clock=clock, max_reference_gap_s=1.0)
                station = FakeStation(core)
                self.assertTrue(station.connect())
                owner = station.claim("fixture")
                self.assertTrue(station.check())
                self.assertTrue(station.arm(owner))
                w = station.issue_window(1.0)
                target = Target(0, (JointTarget(0, 0, 0, 1, .1),) * 2, window=w, owner=owner)
                self.assertTrue(station.submit(target))
                self.assertEqual(core.snapshot().last_sequence, 0)
                station.writes.clear()
                if action == 'packet':
                    self.assertTrue(core.receive(1, SAMPLE, MotorHealth(0, 4, 0, False, 25.0, 30.0)))
                elif action == 'communication':
                    self.assertTrue(core.communication_failed(1, CommunicationFailure.TIMEOUT))
                else:
                    clock.value = .6
                    self.assertTrue(core.tick())
                self.assertIs(core.snapshot().state, State.FAULT)
                self.assertIsNone(core.snapshot().admitted_target)
                self.assertEqual(core.snapshot().last_sequence, 0)
                self.assertIsNone(station.refresh())
                self.assertEqual(station.writes, [])

    def test_mapped_torque_outside_codec_bounds_rejects(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        w = station.issue_window(1.0)
        prior_target = Target(1, (jt1, jt2), window=w, owner=owner)
        self.assertTrue(station.submit(prior_target).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)

        too_high = Target(2, (JointTarget(0.0, 0.0, 3.501, 1.0, 0.1), jt2), window=w, owner=owner)
        rej = core.submit_target(too_high)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        too_low = Target(2, (JointTarget(0.0, 0.0, -3.501, 1.0, 0.1), jt2), window=w, owner=owner)
        rej = core.submit_target(too_low)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().admitted_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        specs_ratio = (
            JointSpec("J1", 0, JointMap(reduction=3.0, direction=-1, zero_output_rad=0.0), -1e8, 1e8, 1e8, 1e8, 1e8, 1e8, "fixture: wide test limits", DEFAULT_PROTECTION),
            JointSpec("J2", 1, JointMap(reduction=1.0, direction=1, zero_output_rad=0.0), -1e8, 1e8, 1e8, 1e8, 1e8, 1e8, "fixture: wide test limits", DEFAULT_PROTECTION),
        )
        core_ratio = JointCore(specs_ratio, clock=clock, max_reference_gap_s=1.0)
        station_ratio = FakeStation(core_ratio)
        self.assertTrue(station_ratio.connect().accepted)
        owner_ratio = station_ratio.claim("fixture-ratio")
        self.assertTrue(station_ratio.check().accepted)
        self.assertTrue(station_ratio.arm(owner_ratio).accepted)
        w_ratio = station_ratio.issue_window(1.0)

        rej_ratio = core_ratio.submit_target(
            Target(1, (JointTarget(0.0, 0.0, 12.0, 9.0, 0.9), jt2), window=w_ratio, owner=owner_ratio)
        )
        self.assertFalse(rej_ratio.accepted)
        self.assertEqual(rej_ratio.code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(core_ratio.snapshot().admitted_target)
        self.assertEqual(core_ratio.snapshot().last_sequence, -1)

    def test_successful_fake_adapter_uses_actual_codec_foc_packets_with_independent_expected_mappings(self):
        clock = DeterministicClock(0.0)
        specs = (
            JointSpec("J1", 0, JointMap(reduction=3.0, direction=-1, zero_output_rad=0.25), -1e8, 1e8, 1e8, 1e8, 1e8, 1e8, "fixture: wide test limits", DEFAULT_PROTECTION),
            JointSpec("J2", 1, JointMap(reduction=2.0, direction=1, zero_output_rad=0.1), -1e8, 1e8, 1e8, 1e8, 1e8, 1e8, "fixture: wide test limits", DEFAULT_PROTECTION),
        )
        core = JointCore(specs, clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)
        station.writes.clear()

        jt1 = JointTarget(q_rad=0.1, dq_rad_s=1.0, tau_nm=-6.0, kp=18.0, kd=4.5)
        jt2 = JointTarget(q_rad=0.2, dq_rad_s=-1.0, tau_nm=3.0, kp=8.0, kd=2.0)
        w = station.issue_window(1.0)
        target = Target(1, (jt1, jt2), window=w, owner=owner)

        decision = station.submit(target)
        self.assertTrue(decision.accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        self.assertEqual(station.writes, [])

        expected_pkt_1 = command(
            0,
            mode=1,
            rpm=-180.0 / math.tau,
            kd=0.5,
            torque=2.0,
            watchdog=True,
            position=-0.2,
            kp=2.0,
        )

        expected_pkt_2 = command(
            1,
            mode=1,
            rpm=-120.0 / math.tau,
            kd=0.5,
            torque=1.5,
            watchdog=True,
            position=0.4,
            kp=2.0,
        )

        clock.value += 0.05
        refreshed = station.refresh()
        self.assertIsNotNone(refreshed)
        self.assertEqual(refreshed.target, target)
        self.assertEqual(station.writes, [expected_pkt_1, expected_pkt_2])
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertEqual(core.snapshot().last_sequence, 1)

    def test_no_writes_on_rejection_stale_fault_stop_disconnect(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)
        station.writes.clear()

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        w = station.issue_window(1.0)
        valid_target = Target(1, (jt1, jt2), window=w, owner=owner)

        bad_target = Target(2, (JointTarget(0.0, 0.0, 10.0, 1.0, 0.1), jt2), window=w, owner=owner)
        self.assertFalse(station.submit(bad_target).accepted)
        self.assertEqual(station.writes, [])

        self.assertIsNone(station.refresh())
        self.assertEqual(station.writes, [])

        self.assertTrue(station.submit(valid_target).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        clock.value = 1.000001
        w2 = station.issue_window(1.0)
        valid_target_newer = Target(2, (jt1, jt2), window=w2, owner=owner)
        self.assertEqual(station.submit(valid_target_newer).code, DecisionCode.STALE_FEEDBACK)
        self.assertEqual(station.writes, [])

        self.assertIsNone(station.refresh())
        self.assertEqual(station.writes, [])
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertIsNone(core.snapshot().admitted_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        w3 = station.issue_window(1.0)
        valid_target_3 = Target(3, (jt1, jt2), window=w3, owner=owner)
        self.assertEqual(station.submit(valid_target_3).code, DecisionCode.INVALID_STATE)
        self.assertIsNone(station.refresh())
        self.assertEqual(station.writes, [])

        station.writes.clear()
        owner_rec = station.claim("fixture-rec")
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        station.writes.clear()
        self.assertTrue(station.arm(owner_rec).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        w4 = station.issue_window(1.0)
        valid_target_4 = Target(4, (jt1, jt2), window=w4, owner=owner_rec)
        self.assertTrue(station.submit(valid_target_4).accepted)
        self.assertEqual(core.snapshot().last_sequence, 4)

        station.writes.clear()
        self.assertTrue(station.stop().accepted)
        self.assertEqual(station.writes, [command(spec.motor_id, mode=0) for spec in self.specs])
        self.assertIsNone(core.snapshot().admitted_target)
        self.assertEqual(core.snapshot().last_sequence, 4)
        station.writes.clear()
        self.assertIsNone(station.refresh())
        self.assertEqual(station.writes, [])

        core2 = JointCore(self.specs, clock=clock, max_reference_gap_s=1.0)
        station2 = FakeStation(core2)
        self.assertTrue(station2.connect().accepted)
        owner2 = station2.claim("fixture-2")
        self.assertTrue(station2.check().accepted)
        self.assertTrue(station2.arm(owner2).accepted)
        self.assertEqual(core2.snapshot().last_sequence, -1)
        w5 = station2.issue_window(1.0)
        valid_target_5 = Target(1, (jt1, jt2), window=w5, owner=owner2)
        self.assertTrue(station2.submit(valid_target_5).accepted)
        self.assertEqual(core2.snapshot().last_sequence, 1)
        station2.writes.clear()

        self.assertTrue(station2.disconnect().accepted)
        self.assertEqual(station2.writes, [])
        self.assertIsNone(core2.snapshot().admitted_target)
        self.assertEqual(core2.snapshot().last_sequence, 1)
        self.assertIsNone(station2.refresh())
        self.assertEqual(station2.writes, [])

    def test_stale_feedback_discovered_on_refresh_clears_pending_and_latches_fault(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=0.5, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)

        w = station.issue_window(1.0)
        target = Target(1, (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        ), window=w, owner=owner)
        self.assertTrue(station.submit(target).accepted)
        self.assertEqual(core.snapshot().admitted_target, target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        clock.value = 0.4
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)

        clock.value = 0.6
        refreshed = station.refresh()
        self.assertIsNone(refreshed)

        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertIsNone(snap.admitted_target)
        self.assertEqual(snap.last_sequence, 1)
        self.assertEqual(snap.affected_joints, ("J2",))
        self.assertEqual(len(snap.faults), 1)
        self.assertTrue(snap.faults[0].stale)
        self.assertEqual(snap.faults[0].name, "J2")

        self.assertEqual(station.check().code, DecisionCode.FAULT_ACK_REQUIRED)

    def test_only_armed_and_moving_may_submit_and_refresh(self):
        clock = DeterministicClock(0.0)
        w = IntentWindow(0, 0.0, 1.0)
        dummy_owner = OwnerToken("epoch-dummy", 1, "human")
        target = Target(1, (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        ), window=w, owner=dummy_owner)

        rejected_states = (
            State.DISCONNECTED,
            State.CONNECTING,
            State.CONNECTED,
            State.CHECKING,
            State.READY,
            State.STOPPED,
            State.FAULT,
        )
        for state in rejected_states:
            with self.subTest(state=state):
                core = core_at(self, state)
                before = core.snapshot()
                self.assertEqual(core.submit_target(target).code, DecisionCode.INVALID_STATE)
                self.assertIsNone(core.prepare_refresh())
                self.assertEqual(core.snapshot(), before)

        clock = DeterministicClock(0.0)
        core = core_at(self, State.ARMED, clock=clock)
        armed_station = FakeStation(core)
        armed_owner = core.snapshot().latest_owner
        w_arm = armed_station.issue_window(1.0)
        arm_target = Target(1, (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        ), window=w_arm, owner=armed_owner)
        self.assertTrue(core.submit_target(arm_target).accepted)
        self.assertEqual(core.snapshot().admitted_target, arm_target)
        self.assertEqual(core.snapshot().last_sequence, 1)
        clock.value += 0.05
        plan = armed_station.refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(plan.target, arm_target)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertEqual(core.snapshot().last_sequence, 1)

        w_move = armed_station.issue_window(1.0)
        target2 = Target(2, (
            JointTarget(0.3, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.4, 0.0, 0.0, 1.0, 0.1),
        ), window=w_move, owner=armed_owner)
        self.assertTrue(core.submit_target(target2).accepted)
        self.assertEqual(core.snapshot().admitted_target, target2)
        self.assertEqual(core.snapshot().last_sequence, 2)
        clock.value += 0.05
        plan2 = armed_station.refresh()
        self.assertIsNotNone(plan2)
        self.assertEqual(plan2.target, target2)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertEqual(core.snapshot().last_sequence, 2)


class CommandSequenceTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), -1e8, 1e8, 1e8, 1e8, 1e8, 1e8, "fixture: wide test limits", DEFAULT_PROTECTION),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), -1e8, 1e8, 1e8, 1e8, 1e8, 1e8, "fixture: wide test limits", DEFAULT_PROTECTION),
        )
        self.valid_joints = (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        )

    def _armed_station(self, clock=None):
        if clock is None:
            clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("fixture")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, clock

    def test_actual_order_1_5_5_2_6_ends_at_6(self):
        core, station, clock = self._armed_station()
        self.assertEqual(core.snapshot().last_sequence, -1)
        self.assertIsNone(core.snapshot().admitted_target)

        w = station.issue_window(1.0)
        t1 = Target(1, self.valid_joints, window=w, owner=station.owner)
        d1 = station.submit(t1)
        self.assertTrue(d1.accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        self.assertEqual(core.snapshot().admitted_target, t1)

        t5 = Target(5, self.valid_joints, window=w, owner=station.owner)
        d5 = station.submit(t5)
        self.assertTrue(d5.accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

        station.writes.clear()
        t5_dup = Target(5, self.valid_joints, window=w, owner=station.owner)
        d5_dup = station.submit(t5_dup)
        self.assertFalse(d5_dup.accepted)
        self.assertEqual(d5_dup.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)
        self.assertEqual(station.writes, [])

        t2 = Target(2, self.valid_joints, window=w, owner=station.owner)
        d2 = station.submit(t2)
        self.assertFalse(d2.accepted)
        self.assertEqual(d2.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)
        self.assertEqual(station.writes, [])

        t6 = Target(6, self.valid_joints, window=w, owner=station.owner)
        d6 = station.submit(t6)
        self.assertTrue(d6.accepted)
        self.assertEqual(core.snapshot().last_sequence, 6)
        self.assertEqual(core.snapshot().admitted_target, t6)

        station.writes.clear()
        clock.value += 0.05
        plan = station.refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(plan.target, t6)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertEqual(core.snapshot().admitted_target, t6)
        self.assertEqual(core.snapshot().last_sequence, 6)
        self.assertEqual(len(station.writes), 2)

    def test_boundaries_0_and_uint64max(self):
        core, station, _ = self._armed_station()
        self.assertEqual(core.snapshot().last_sequence, -1)

        w = station.issue_window(1.0)
        t0 = Target(0, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(t0).accepted)
        self.assertEqual(core.snapshot().last_sequence, 0)
        self.assertEqual(core.snapshot().admitted_target, t0)

        d0_dup = station.submit(t0)
        self.assertFalse(d0_dup.accepted)
        self.assertEqual(d0_dup.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 0)
        self.assertEqual(core.snapshot().admitted_target, t0)

        uint64_max = (1 << 64) - 1
        t_max = Target(uint64_max, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(t_max).accepted)
        self.assertEqual(core.snapshot().last_sequence, uint64_max)
        self.assertEqual(core.snapshot().admitted_target, t_max)

        d_max_dup = station.submit(t_max)
        self.assertFalse(d_max_dup.accepted)
        self.assertEqual(d_max_dup.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, uint64_max)

        t_smaller = Target(uint64_max - 1, self.valid_joints, window=w, owner=station.owner)
        d_smaller = station.submit(t_smaller)
        self.assertFalse(d_smaller.accepted)
        self.assertEqual(d_smaller.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, uint64_max)

    def test_sequence_type_and_range_rejections(self):
        w = IntentWindow(0, 0.0, 1.0)
        owner = OwnerToken("epoch-1", 1, "fixture")
        invalid_sequences = (
            True,
            False,
            -1,
            -100,
            0.0,
            1.5,
            math.nan,
            math.inf,
            -math.inf,
            "0",
            "1",
            "100",
            1 << 64,
            (1 << 64) + 1,
            None,
            object(),
        )
        for bad_seq in invalid_sequences:
            with self.subTest(bad_seq=repr(bad_seq)), self.assertRaises(ValueError):
                Target(bad_seq, self.valid_joints, window=w, owner=owner)

        target = Target(1, self.valid_joints, window=w, owner=owner)
        with self.assertRaises(FrozenInstanceError):
            target.sequence = 2

    def test_invalid_higher_vector_or_mapping_preserves_pending_and_watermark_and_never_blocks_smaller(self):
        core, station, _ = self._armed_station()
        w = station.issue_window(1.0)
        t5 = Target(5, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(t5).accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

        bad_len = Target(10, (self.valid_joints[0],), window=w, owner=station.owner)
        d_bad_len = station.submit(bad_len)
        self.assertFalse(d_bad_len.accepted)
        self.assertEqual(d_bad_len.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

        bad_map = Target(10, (JointTarget(0.0, 0.0, 10.0, 1.0, 0.1), self.valid_joints[1]), window=w, owner=station.owner)
        d_bad_map = station.submit(bad_map)
        self.assertFalse(d_bad_map.accepted)
        self.assertEqual(d_bad_map.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

        bad_second = Target(10, (self.valid_joints[0], JointTarget(0, 0, 10, 1, .1)), window=w, owner=station.owner)
        before = core.snapshot()
        self.assertEqual(station.submit(bad_second).code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot(), before)

        t7 = Target(7, self.valid_joints, window=w, owner=station.owner)
        d7 = station.submit(t7)
        self.assertTrue(d7.accepted)
        self.assertEqual(core.snapshot().last_sequence, 7)
        self.assertEqual(core.snapshot().admitted_target, t7)

    def test_watermark_survives_consumption_stop_fault_disconnect_reconnect_and_recovery(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=1.0, max_reference_gap_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner1 = station.claim("fixture-1")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner1).accepted)

        w1 = station.issue_window(1.0)
        t10 = Target(10, self.valid_joints, window=w1, owner=owner1)
        self.assertTrue(station.submit(t10).accepted)
        self.assertEqual(core.snapshot().last_sequence, 10)

        clock.value += 0.05
        plan = station.refresh()
        self.assertIsNotNone(plan)
        self.assertEqual(plan.target, t10)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertEqual(core.snapshot().last_sequence, 10)

        owner2 = station.claim("fixture-2")
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner2).accepted)
        self.assertEqual(core.snapshot().last_sequence, 10)

        bad_health = MotorHealth(mode=0, errors=1, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, bad_health).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.disconnect().accepted)
        self.assertIs(core.snapshot().state, State.DISCONNECTED)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.connect().accepted)
        self.assertIs(core.snapshot().state, State.CONNECTED)
        self.assertEqual(core.snapshot().last_sequence, 10)

        owner3 = station.claim("fixture-3")
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        self.assertIs(core.snapshot().state, State.READY)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.arm(owner3).accepted)
        self.assertIs(core.snapshot().state, State.ARMED)
        self.assertEqual(core.snapshot().last_sequence, 10)

        w2 = station.issue_window(1.0)
        t10_new = Target(10, self.valid_joints, window=w2, owner=owner3)
        d10_dup = station.submit(t10_new)
        self.assertFalse(d10_dup.accepted)
        self.assertEqual(d10_dup.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 10)

        t9 = Target(9, self.valid_joints, window=w2, owner=owner3)
        d9 = station.submit(t9)
        self.assertFalse(d9.accepted)
        self.assertEqual(d9.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 10)

        t11 = Target(11, self.valid_joints, window=w2, owner=owner3)
        d11 = station.submit(t11)
        self.assertTrue(d11.accepted)
        self.assertEqual(core.snapshot().last_sequence, 11)
        self.assertEqual(core.snapshot().admitted_target, t11)

    def test_stale_submission_latches_fault_clears_goal_and_preserves_watermark(self):
        core, station, clock = self._armed_station()
        w = station.issue_window(1.0)
        t5 = Target(5, self.valid_joints, window=w, owner=station.owner)
        self.assertTrue(station.submit(t5).accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().admitted_target, t5)

        clock.value = 1.000001
        w2 = station.issue_window(1.0)
        t10 = Target(10, self.valid_joints, window=w2, owner=station.owner)
        d_stale = station.submit(t10)
        self.assertFalse(d_stale.accepted)
        self.assertEqual(d_stale.code, DecisionCode.STALE_FEEDBACK)
        self.assertEqual(core.snapshot().last_sequence, 5)
        stale_snapshot = core.snapshot()
        self.assertIsNone(stale_snapshot.admitted_target)
        self.assertIs(stale_snapshot.state, State.FAULT)

        rejected_states = (
            State.DISCONNECTED,
            State.CONNECTING,
            State.CONNECTED,
            State.CHECKING,
            State.READY,
            State.STOPPED,
            State.FAULT,
        )
        dummy_owner = OwnerToken("dummy", 1, "human")
        for state in rejected_states:
            with self.subTest(state=state):
                state_core = core_at(self, state)
                self.assertIs(state_core.snapshot().state, state)
                self.assertEqual(state_core.snapshot().last_sequence, -1)
                w_state = state_core.issue_window(1.0)
                d = state_core.submit_target(Target(20, self.valid_joints, window=w_state, owner=dummy_owner))
                self.assertFalse(d.accepted)
                self.assertEqual(d.code, DecisionCode.INVALID_STATE)
                self.assertEqual(state_core.snapshot().last_sequence, -1)
                self.assertIsNone(state_core.snapshot().admitted_target)

    def test_new_core_starts_negative_one_and_independent_watermarks(self):
        core1 = JointCore(self.specs, clock=DeterministicClock(0.0), max_reference_gap_s=1.0)
        core2 = JointCore(self.specs, clock=DeterministicClock(0.0), max_reference_gap_s=1.0)
        self.assertEqual(core1.snapshot().last_sequence, -1)
        self.assertEqual(core2.snapshot().last_sequence, -1)

        station1 = FakeStation(core1)
        self.assertTrue(station1.connect().accepted)
        owner1 = station1.claim("fixture-1")
        self.assertTrue(station1.check().accepted)
        self.assertTrue(station1.arm(owner1).accepted)

        w = station1.issue_window(1.0)
        t10 = Target(10, self.valid_joints, window=w, owner=owner1)
        self.assertTrue(station1.submit(t10).accepted)
        self.assertEqual(core1.snapshot().last_sequence, 10)
        self.assertEqual(core2.snapshot().last_sequence, -1)


if __name__ == "__main__":
    unittest.main()
