import unittest
from dataclasses import FrozenInstanceError

from im6014_codec import command
from joint_controller import (
    DecisionCode,
    JointCore,
    JointSpec,
    MotorHealth,
    SourceIdentity,
    State,
)
from joint_mapping import JointMap, JointSample


SAMPLE = JointSample(0.1, 0.0, 0.0)
HEALTHY_STOP = MotorHealth(mode=0, errors=0, warnings=0, timeout=False)
SOURCE = SourceIdentity("fixture", "controller-test", 1)


class FakeStation:
    """Test-only owner that emits fake Stop frames after accepted decisions."""

    def __init__(self, core):
        self.core = core
        self.writes = []

    def _write_stops(self):
        self.writes.extend(command(spec.motor_id, mode=0) for spec in self.core.joint_specs)

    def connect(self):
        decision = self.core.begin_connect()
        if not decision.accepted:
            return decision
        decision = self.core.connected(SOURCE)
        if decision.accepted:
            self._write_stops()
        return decision

    def check(self, *, acknowledge_fault=False):
        decision = self.core.begin_check(acknowledge_fault=acknowledge_fault)
        if not decision.accepted:
            return decision
        self._write_stops()
        for spec in self.core.joint_specs:
            decision = self.core.receive(spec.motor_id, SAMPLE, HEALTHY_STOP)
            if not decision.accepted:
                return decision
        return self.core.checked()

    def arm(self):
        return self.core.arm()

    def stop(self):
        decision = self.core.stop()
        if decision.accepted:
            self._write_stops()
        return decision

    def disconnect(self):
        return self.core.disconnect()


class ControllerStateTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0)),
            JointSpec("J2", 1, JointMap(1, 1, 0.0)),
        )

    def _core_at(self, state):
        core = JointCore(self.specs)
        station = FakeStation(core)
        if state is State.DISCONNECTED:
            return core
        if state is State.CONNECTING:
            self.assertTrue(core.begin_connect().accepted)
            return core

        self.assertTrue(station.connect().accepted)
        if state is State.CONNECTED:
            return core
        if state is State.CHECKING:
            self.assertTrue(core.begin_check().accepted)
            return core

        self.assertTrue(station.check().accepted)
        if state is State.READY:
            return core
        if state is State.ARMED:
            self.assertTrue(station.arm().accepted)
            return core
        if state is State.STOPPED:
            self.assertTrue(station.stop().accepted)
            return core
        if state is State.FAULT:
            self.assertTrue(station.arm().accepted)
            self.assertTrue(core.receive(
                self.specs[0].motor_id,
                SAMPLE,
                MotorHealth(mode=0, errors=1, warnings=0, timeout=False),
            ).accepted)
            return core
        raise AssertionError(f"{state} is not reachable in C02")

    def test_joint_set_is_nonempty_unique_and_owned(self):
        mapping = JointMap(1, 1, 0.0)
        with self.assertRaises(ValueError):
            JointCore(())
        with self.assertRaises(ValueError):
            JointCore((JointSpec("J1", 0, mapping), JointSpec("J1", 1, mapping)))
        with self.assertRaises(ValueError):
            JointCore((JointSpec("J1", 0, mapping), JointSpec("J2", 0, mapping)))
        with self.assertRaises(ValueError):
            JointSpec("J1", True, mapping)

        supplied = [JointSpec("J1", 0, mapping)]
        core = JointCore(supplied)
        supplied.clear()
        self.assertEqual(tuple(spec.name for spec in core.joint_specs), ("J1",))

    def test_health_and_source_records_reject_invalid_values(self):
        for kwargs in (
            dict(mode=True, errors=0, warnings=0, timeout=False),
            dict(mode=0, errors=True, warnings=0, timeout=False),
            dict(mode=0, errors=0, warnings=-1, timeout=False),
            dict(mode=0, errors=0, warnings=0, timeout=1),
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                MotorHealth(**kwargs)
        with self.assertRaises(ValueError):
            SourceIdentity("other", "x", 1)

    def test_fake_lifecycle_writes_only_after_accepted_actions(self):
        core = JointCore(self.specs)
        station = FakeStation(core)
        expected = [command(spec.motor_id, mode=0) for spec in self.specs]

        self.assertFalse(station.check().accepted)
        self.assertFalse(station.stop().accepted)
        self.assertEqual(station.writes, [])

        self.assertTrue(station.connect().accepted)
        self.assertEqual(station.writes, expected)
        station.writes.clear()
        self.assertFalse(station.connect().accepted)
        self.assertEqual(station.writes, [])

        self.assertTrue(station.check().accepted)
        self.assertEqual(station.writes, expected)
        self.assertIs(core.snapshot().state, State.READY)
        station.writes.clear()
        self.assertTrue(station.arm().accepted)
        self.assertIs(core.snapshot().state, State.ARMED)
        self.assertTrue(station.stop().accepted)
        self.assertEqual(station.writes, expected)
        self.assertIs(core.snapshot().state, State.STOPPED)

    def test_check_requires_every_healthy_joint_in_stop_mode(self):
        core = JointCore(self.specs)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertTrue(core.begin_check().accepted)

        before = core.snapshot()
        rejected = core.checked()
        self.assertFalse(rejected)
        self.assertEqual(rejected.code, DecisionCode.OBSERVATIONS_INCOMPLETE)
        self.assertEqual(core.snapshot(), before)

        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        foc = MotorHealth(mode=1, errors=0, warnings=0, timeout=False)
        self.assertTrue(core.receive(self.specs[1].motor_id, SAMPLE, foc).accepted)
        before = core.snapshot()
        self.assertEqual(core.checked().code, DecisionCode.NOT_STOPPED)
        self.assertEqual(core.snapshot(), before)

        unhealthy = MotorHealth(mode=0, errors=0, warnings=1, timeout=False)
        self.assertTrue(core.receive(self.specs[1].motor_id, SAMPLE, unhealthy).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().fault_latched)
        before = core.snapshot()
        rejected = core.checked()
        self.assertFalse(rejected)
        self.assertEqual(rejected.code, DecisionCode.INVALID_STATE)
        self.assertEqual(core.snapshot(), before)

        self.assertTrue(core.receive(self.specs[1].motor_id, SAMPLE, HEALTHY_STOP))
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP))
        self.assertTrue(core.receive(self.specs[1].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertIs(core.snapshot().state, State.READY)

    def test_fault_survives_stop_disconnect_reconnect_and_healthy_packets(self):
        core = JointCore(self.specs)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)

        bad_health = MotorHealth(mode=0, errors=4, warnings=2, timeout=True)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, bad_health).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().fault_latched)

        station.writes.clear()
        self.assertTrue(station.stop().accepted)
        self.assertEqual(station.writes, [command(spec.motor_id, mode=0) for spec in self.specs])
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().fault_latched)

        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().fault_latched)

        self.assertTrue(station.disconnect().accepted)
        disconnected = core.snapshot()
        self.assertIs(disconnected.state, State.DISCONNECTED)
        self.assertTrue(disconnected.fault_latched)
        self.assertTrue(all(not joint.valid for joint in disconnected.joints))
        self.assertIsNone(disconnected.source)

        self.assertTrue(station.connect().accepted)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertTrue(core.snapshot().fault_latched)
        before = core.snapshot()
        self.assertEqual(core.begin_check().code, DecisionCode.FAULT_ACK_REQUIRED)
        self.assertEqual(core.snapshot(), before)

        station.writes.clear()
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        self.assertEqual(station.writes, [command(spec.motor_id, mode=0) for spec in self.specs])
        self.assertIs(core.snapshot().state, State.READY)
        self.assertFalse(core.snapshot().fault_latched)

    def test_disconnect_and_new_connection_invalidate_observations(self):
        core = JointCore(self.specs)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        prior = core.snapshot()
        self.assertTrue(prior.joints[0].valid)

        self.assertTrue(core.disconnect().accepted)
        self.assertTrue(all(not joint.valid for joint in core.snapshot().joints))
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SourceIdentity("fixture", "controller-test", 2)).accepted)
        self.assertTrue(all(not joint.valid for joint in core.snapshot().joints))
        self.assertTrue(prior.joints[0].valid)

    def test_snapshots_are_immutable_owned_values(self):
        core = JointCore(self.specs)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        before = core.snapshot()
        self.assertIsInstance(before.joints, tuple)
        self.assertIsInstance(before.joint_names, tuple)

        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        after = core.snapshot()
        self.assertFalse(before.joints[0].valid)
        self.assertTrue(after.joints[0].valid)
        with self.assertRaises(FrozenInstanceError):
            after.state = State.FAULT
        with self.assertRaises(FrozenInstanceError):
            after.joints[0].health = HEALTHY_STOP
        with self.assertRaises(TypeError):
            after.joints[0] = before.joints[0]

    def test_invalid_transition_matrix_preserves_snapshot(self):
        rejected = {
            State.DISCONNECTED: ("connected", "begin_check", "checked", "arm", "stop"),
            State.CONNECTING: ("begin_connect", "begin_check", "checked", "arm", "stop"),
            State.CONNECTED: ("begin_connect", "connected", "checked", "arm"),
            State.CHECKING: ("begin_connect", "connected", "begin_check", "arm"),
            State.READY: ("begin_connect", "connected", "begin_check", "checked"),
            State.ARMED: ("begin_connect", "connected", "begin_check", "checked", "arm"),
            State.STOPPED: ("begin_connect", "connected", "checked", "arm"),
            State.FAULT: ("begin_connect", "connected", "begin_check", "checked", "arm"),
        }

        for state, events in rejected.items():
            for event in events + ("wrong_joint",):
                with self.subTest(state=state, event=event):
                    core = self._core_at(state)
                    station = FakeStation(core)
                    before = core.snapshot()
                    action = {
                        "begin_connect": station.connect,
                        "connected": lambda: core.connected(SOURCE),
                        "begin_check": station.check,
                        "checked": core.checked,
                        "arm": core.arm,
                        "stop": station.stop,
                        "wrong_joint": lambda: core.receive(14, SAMPLE, HEALTHY_STOP),
                    }[event]
                    decision = action()
                    self.assertFalse(decision.accepted)
                    self.assertFalse(decision)
                    self.assertEqual(station.writes, [])
                    self.assertEqual(core.snapshot(), before)


if __name__ == "__main__":
    unittest.main()
