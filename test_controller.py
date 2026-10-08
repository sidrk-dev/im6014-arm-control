import unittest
from dataclasses import FrozenInstanceError
import math

from im6014_codec import command
from joint_controller import (
    CommunicationFailure,
    DecisionCode,
    JointCore,
    JointFault,
    JointSpec,
    MotorHealth,
    SourceIdentity,
    State,
    Target,
)
from joint_mapping import JointMap, JointSample, JointTarget


SAMPLE = JointSample(0.1, 0.0, 0.0)
HEALTHY_STOP = MotorHealth(mode=0, errors=0, warnings=0, timeout=False)
SOURCE = SourceIdentity("fixture", "controller-test", 1)


class FakeStation:

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

    def submit(self, target):
        return self.core.submit_target(target)

    def consume(self):
        target = self.core.consume_target()
        if target is not None:
            for spec, joint_target in zip(self.core.joint_specs, target.joints):
                output = spec.mapping.to_output(joint_target)
                self.writes.append(
                    command(
                        spec.motor_id,
                        mode=1,
                        rpm=output.rpm,
                        kd=output.kd,
                        torque=output.tau_nm,
                        watchdog=True,
                        position=output.q_rad,
                        kp=output.kp,
                    )
                )
        return target

    def stop(self):
        decision = self.core.stop()
        if decision.accepted:
            self._write_stops()
        return decision

    def disconnect(self):
        return self.core.disconnect()


class DeterministicClock:
    def __init__(self, value=0.0):
        self.value = value
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.value


class FeedbackAgeTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0)),
            JointSpec("J2", 1, JointMap(1, 1, 0.0)),
        )

    def _ready_station(self, clock, *, feedback_max_age_s=1.0):
        core = JointCore(
            self.specs,
            clock=clock,
            feedback_max_age_s=feedback_max_age_s,
        )
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        return core, station

    def test_feedback_age_configuration_is_finite_positive_and_typed(self):
        for value in (True, False, 0, -1.0, math.nan, math.inf, -math.inf, 10**400, "1"):
            with self.subTest(value=repr(value)), self.assertRaises(ValueError):
                JointCore(self.specs, feedback_max_age_s=value)
        with self.assertRaises(ValueError):
            JointCore(self.specs, clock=0)

    def test_zero_timestamp_and_exact_freshness_boundary(self):
        clock = DeterministicClock(0.0)
        core, _station = self._ready_station(clock)

        received = core.snapshot()
        self.assertEqual(tuple(joint.received_at for joint in received.joints), (0.0, 0.0))
        self.assertEqual(received.feedback_ages_s, (0.0, 0.0))

        clock.value = 1.0
        at_boundary = core.snapshot()
        self.assertEqual(at_boundary.feedback_ages_s, (1.0, 1.0))
        self.assertTrue(core.arm().accepted)

        with self.assertRaises(FrozenInstanceError):
            at_boundary.joints[0].received_at = 1.0
        with self.assertRaises(TypeError):
            at_boundary.feedback_ages_s[0] = 0.0

    def test_checked_rejects_feedback_older_than_limit(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=1.0)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertTrue(core.begin_check().accepted)
        for spec in self.specs:
            self.assertTrue(core.receive(spec.motor_id, SAMPLE, HEALTHY_STOP).accepted)

        clock.value = 1.000001
        rejected = core.checked()
        self.assertEqual(rejected.code, DecisionCode.STALE_FEEDBACK)
        self.assertIs(core.snapshot().state, State.CHECKING)
        self.assertFalse(core.snapshot().fault_latched)
        self.assertTrue(core.tick().accepted)
        self.assertIs(core.snapshot().state, State.CHECKING)

    def test_bad_id_and_observation_do_not_refresh_receipt_time(self):
        clock = DeterministicClock(0.0)
        core, _station = self._ready_station(clock)
        initial = core.snapshot()
        calls_before = clock.calls

        clock.value = 0.5
        self.assertEqual(
            core.receive(14, SAMPLE, HEALTHY_STOP).code,
            DecisionCode.UNKNOWN_JOINT,
        )
        self.assertEqual(
            core.receive(self.specs[0].motor_id, object(), HEALTHY_STOP).code,
            DecisionCode.INVALID_OBSERVATION,
        )
        self.assertEqual(clock.calls, calls_before)
        self.assertEqual(
            tuple(joint.received_at for joint in core.snapshot().joints),
            tuple(joint.received_at for joint in initial.joints),
        )

        clock.value = 1.000001
        self.assertEqual(core.arm().code, DecisionCode.STALE_FEEDBACK)

    def test_fresh_j1_cannot_hide_stale_j2_from_arm_or_tick(self):
        clock = DeterministicClock(0.0)
        core, station = self._ready_station(clock)
        station.writes.clear()

        clock.value = 0.8
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        clock.value = 1.000001
        before_writes = list(station.writes)
        rejected = station.arm()
        self.assertEqual(rejected.code, DecisionCode.STALE_FEEDBACK)
        self.assertIs(core.snapshot().state, State.READY)
        self.assertEqual(station.writes, before_writes)

        clock = DeterministicClock(0.0)
        core, station = self._ready_station(clock)
        station.writes.clear()
        clock.value = 0.8
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertTrue(station.arm().accepted)
        clock.value = 1.000001
        before_writes = list(station.writes)
        self.assertTrue(core.tick().accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertEqual(station.writes, before_writes)

    def test_disconnect_invalidates_age_and_stale_fault_needs_acknowledged_check(self):
        clock = DeterministicClock(0.0)
        core, station = self._ready_station(clock)
        self.assertTrue(station.arm().accepted)

        clock.value = 1.1
        self.assertTrue(core.tick().accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().fault_latched)
        before_disconnect = core.snapshot()

        self.assertTrue(station.disconnect().accepted)
        disconnected = core.snapshot()
        self.assertTrue(all(joint.received_at is None for joint in disconnected.joints))
        self.assertEqual(disconnected.feedback_ages_s, (None, None))
        self.assertEqual(before_disconnect.feedback_ages_s, (1.1, 1.1))
        self.assertTrue(disconnected.fault_latched)

        self.assertTrue(station.connect().accepted)
        writes_after_connect = list(station.writes)
        self.assertEqual(
            station.check().code,
            DecisionCode.FAULT_ACK_REQUIRED,
        )
        self.assertEqual(station.writes, writes_after_connect)

        station.writes.clear()
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        self.assertEqual(
            station.writes,
            [command(spec.motor_id, mode=0) for spec in self.specs],
        )
        self.assertIs(core.snapshot().state, State.READY)
        self.assertFalse(core.snapshot().fault_latched)


class ControllerStateTests(unittest.TestCase):
    def setUp(self):
        self.clock = lambda: 0.0
        self.specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0)),
            JointSpec("J2", 1, JointMap(1, 1, 0.0)),
        )

    def _core_at(self, state):
        core = JointCore(self.specs, clock=self.clock)
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
        if state is State.MOVING:
            self.assertTrue(station.arm().accepted)
            target = Target((
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ))
            self.assertTrue(core.submit_target(target).accepted)
            self.assertIsNotNone(core.consume_target())
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
        raise AssertionError(f"{state} is not reachable")

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
        core = JointCore(supplied, clock=self.clock)
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
        core = JointCore(self.specs, clock=self.clock)
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
        core = JointCore(self.specs, clock=self.clock)
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
        core = JointCore(self.specs, clock=self.clock)
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
        core = JointCore(self.specs, clock=self.clock)
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
        core = JointCore(self.specs, clock=self.clock)
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
            State.MOVING: ("begin_connect", "connected", "begin_check", "checked", "arm"),
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


class JointFaultTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0)),
            JointSpec("J2", 1, JointMap(1, 1, 0.0)),
        )

    def _ready_core(self, clock=None, *, feedback_max_age_s=1.0):
        if clock is None:
            clock = DeterministicClock(0.0)
        core = JointCore(
            self.specs,
            clock=clock,
            feedback_max_age_s=feedback_max_age_s,
        )
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SOURCE).accepted)
        self.assertTrue(core.begin_check().accepted)
        for spec in self.specs:
            self.assertTrue(core.receive(spec.motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        return core

    def test_stale_j2_attributes_only_stale_joint_with_refreshed_j1(self):
        clock = DeterministicClock(0.0)
        core = self._ready_core(clock, feedback_max_age_s=1.0)
        self.assertTrue(core.arm().accepted)
        self.assertIs(core.snapshot().state, State.ARMED)

        clock.value = 0.8
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)

        clock.value = 1.1
        self.assertTrue(core.tick().accepted)
        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertEqual(snap.affected_joints, ("J2",))
        self.assertEqual(len(snap.faults), 1)

        j2_fault = snap.faults[0]
        self.assertEqual(j2_fault.name, "J2")
        self.assertEqual(j2_fault.motor_id, 1)
        self.assertTrue(j2_fault.stale)
        self.assertEqual(j2_fault.errors, 0)
        self.assertEqual(j2_fault.warnings, 0)
        self.assertFalse(j2_fault.timeout)
        self.assertIsNone(j2_fault.communication_error)

        self.assertNotIn("J1", snap.affected_joints)
        self.assertEqual(snap.joints[0].received_at, 0.8)
        self.assertEqual(snap.joints[1].received_at, 0.0)

    def test_unknown_bits_and_32bit_error_mask_retained(self):
        clock = DeterministicClock(0.0)
        core = self._ready_core(clock)
        self.assertTrue(core.arm().accepted)

        unknown_errors = 0x80000004
        status = MotorHealth(mode=0, errors=unknown_errors, warnings=5, timeout=True)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, status).accepted)

        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertEqual(snap.affected_joints, ("J1",))
        j1_fault = snap.faults[0]
        self.assertEqual(j1_fault.errors, unknown_errors)
        self.assertEqual(j1_fault.errors & 0x80000000, 0x80000000)
        self.assertEqual(j1_fault.warnings, 5)
        self.assertTrue(j1_fault.timeout)
        self.assertFalse(j1_fault.stale)

        for kwargs in (
            dict(mode=0, errors=-1, warnings=0, timeout=False),
            dict(mode=0, errors=0x1_0000_0000, warnings=0, timeout=False),
            dict(mode=0, errors=0, warnings=-1, timeout=False),
            dict(mode=0, errors=0, warnings=8, timeout=False),
            dict(mode=0, errors=0, warnings=0, timeout="True"),
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                MotorHealth(**kwargs)

        for kwargs in (
            dict(name="J1", motor_id=0, errors=-1),
            dict(name="J1", motor_id=0, errors=0x1_0000_0000),
            dict(name="J1", motor_id=0, warnings=-1),
            dict(name="J1", motor_id=0, warnings=8),
            dict(name="J1", motor_id=0, timeout=1),
            dict(name="J1", motor_id=0, stale=1),
            dict(name="", motor_id=0),
            dict(name="J1", motor_id=15),
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                JointFault(**kwargs)

    def test_healthy_after_fault_preserves_latched_attribution(self):
        core = self._ready_core()
        self.assertTrue(core.arm().accepted)

        unhealthy = MotorHealth(mode=0, errors=0x02, warnings=0, timeout=False)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, unhealthy).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))
        self.assertEqual(core.snapshot().faults[0].errors, 0x02)

        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        snap = core.snapshot()

        self.assertTrue(snap.joints[0].healthy)
        self.assertEqual(snap.joints[0].errors, 0)
        self.assertEqual(snap.joints[0].health.errors, 0)

        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertEqual(snap.affected_joints, ("J1",))
        self.assertEqual(len(snap.faults), 1)
        self.assertEqual(snap.faults[0].errors, 0x02)
        self.assertNotEqual(snap.faults[0].errors, 0)

    def test_multiple_faults_on_joint_preserves_prior_flags_with_bounded_storage(self):
        core = self._ready_core()

        self.assertTrue(core.receive(
            self.specs[0].motor_id,
            SAMPLE,
            MotorHealth(mode=0, errors=0x01, warnings=1, timeout=False),
        ).accepted)

        self.assertTrue(core.receive(
            self.specs[0].motor_id,
            SAMPLE,
            MotorHealth(mode=0, errors=0x04, warnings=2, timeout=True),
        ).accepted)

        self.assertTrue(core.communication_failed(
            self.specs[0].motor_id,
            CommunicationFailure.DRIVER_FAILURE,
        ).accepted)

        snap = core.snapshot()
        self.assertEqual(len(snap.faults), 1)
        j1_fault = snap.faults[0]
        self.assertEqual(j1_fault.name, "J1")
        self.assertEqual(j1_fault.errors, 0x05)
        self.assertEqual(j1_fault.warnings, 3)
        self.assertTrue(j1_fault.timeout)
        self.assertEqual(j1_fault.communication_error, CommunicationFailure.DRIVER_FAILURE)
        self.assertIn(CommunicationFailure.DRIVER_FAILURE, j1_fault.communication_errors)

        for _ in range(10):
            self.assertTrue(core.receive(
                self.specs[0].motor_id,
                SAMPLE,
                MotorHealth(mode=0, errors=0x02, warnings=0, timeout=False),
            ).accepted)
        self.assertEqual(len(core.snapshot().faults), 1)
        self.assertEqual(core.snapshot().faults[0].errors, 0x07)

    def test_communication_failures_with_fixed_enum_codes(self):
        for code in (
            CommunicationFailure.TIMEOUT,
            CommunicationFailure.INVALID_FEEDBACK,
            CommunicationFailure.DRIVER_FAILURE,
        ):
            with self.subTest(code=code):
                core = self._ready_core()
                decision = core.communication_failed(self.specs[0].motor_id, code)
                self.assertTrue(decision.accepted)
                snap = core.snapshot()
                self.assertIs(snap.state, State.FAULT)
                self.assertEqual(snap.affected_joints, ("J1",))
                fault = snap.faults[0]
                self.assertEqual(fault.communication_error, code)
                self.assertFalse(fault.timeout)
                self.assertEqual(snap.joints[0].communication_error, code)
                self.assertFalse(snap.joints[0].healthy)

        core = self._ready_core()
        self.assertTrue(core.communication_failed(self.specs[1].motor_id, CommunicationFailure.INVALID_FEEDBACK).accepted)
        self.assertEqual(core.snapshot().affected_joints, ("J2",))
        self.assertEqual(core.snapshot().faults[0].communication_error, CommunicationFailure.INVALID_FEEDBACK)

    def test_current_communication_status_clears_without_erasing_cause(self):
        core = self._ready_core()
        self.assertTrue(core.communication_failed(0, CommunicationFailure.TIMEOUT))
        before = core.snapshot()
        self.assertTrue(core.receive(0, SAMPLE, HEALTHY_STOP))
        after = core.snapshot()
        self.assertTrue(after.joints[0].healthy)
        self.assertIsNone(after.joints[0].communication_error)
        self.assertEqual(after.faults, before.faults)
        self.assertIs(after.state, State.FAULT)

    def test_unknown_ids_and_invalid_codes_reject_unchanged(self):
        core = self._ready_core()
        before = core.snapshot()

        self.assertEqual(core.communication_failed(0, "timeout").code,
                         DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot(), before)

        rejected_id = core.communication_failed(14, CommunicationFailure.TIMEOUT)
        self.assertFalse(rejected_id)
        self.assertEqual(rejected_id.code, DecisionCode.UNKNOWN_JOINT)
        self.assertEqual(core.snapshot(), before)

        self.assertEqual(
            core.communication_failed(True, CommunicationFailure.TIMEOUT).code,
            DecisionCode.INVALID_ARGUMENT,
        )
        self.assertEqual(
            core.communication_failed("0", CommunicationFailure.TIMEOUT).code,
            DecisionCode.INVALID_ARGUMENT,
        )
        self.assertEqual(core.snapshot(), before)

        self.assertEqual(
            core.communication_failed(0, "unrecognized_failure").code,
            DecisionCode.INVALID_ARGUMENT,
        )
        self.assertEqual(
            core.communication_failed(0, RuntimeError("comm timeout")).code,
            DecisionCode.INVALID_ARGUMENT,
        )
        self.assertEqual(
            core.communication_failed(0, 999).code,
            DecisionCode.INVALID_ARGUMENT,
        )
        self.assertEqual(
            core.communication_failed(0, None).code,
            DecisionCode.INVALID_ARGUMENT,
        )
        self.assertEqual(core.snapshot(), before)

    def test_failed_recovery_retains_causes(self):
        clock = DeterministicClock(0.0)
        core = self._ready_core(clock, feedback_max_age_s=1.0)
        self.assertTrue(core.arm().accepted)

        self.assertTrue(core.receive(
            self.specs[0].motor_id,
            SAMPLE,
            MotorHealth(mode=0, errors=0x04, warnings=0, timeout=False),
        ).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))

        rejected = core.begin_check(acknowledge_fault=False)
        self.assertFalse(rejected)
        self.assertEqual(rejected.code, DecisionCode.FAULT_ACK_REQUIRED)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))

        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.assertIs(core.snapshot().state, State.CHECKING)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))

        self.assertEqual(core.checked().code, DecisionCode.OBSERVATIONS_INCOMPLETE)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))

        non_stop = MotorHealth(mode=1, errors=0, warnings=0, timeout=False)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, non_stop).accepted)
        self.assertTrue(core.receive(self.specs[1].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertEqual(core.checked().code, DecisionCode.NOT_STOPPED)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))

        clock.value = 2.0
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)
        self.assertEqual(core.checked().code, DecisionCode.STALE_FEEDBACK)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))

        self.assertEqual(core.arm().code, DecisionCode.INVALID_STATE)

    def test_successful_recovery_clears_fault_causes(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=1.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)

        self.assertTrue(core.communication_failed(
            self.specs[1].motor_id,
            CommunicationFailure.TIMEOUT,
        ).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().affected_joints, ("J2",))

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().affected_joints, ("J2",))

        self.assertTrue(station.disconnect().accepted)
        self.assertIs(core.snapshot().state, State.DISCONNECTED)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertEqual(core.snapshot().affected_joints, ("J2",))

        self.assertTrue(station.connect().accepted)
        self.assertIs(core.snapshot().state, State.CONNECTED)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertEqual(core.snapshot().affected_joints, ("J2",))

        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        snap = core.snapshot()
        self.assertIs(snap.state, State.READY)
        self.assertFalse(snap.fault_latched)
        self.assertEqual(snap.faults, ())
        self.assertEqual(snap.affected_joints, ())

    def test_independent_second_core_unchanged_by_fault(self):
        specs2 = (
            JointSpec("J3", 2, JointMap(1, 1, 0.0)),
            JointSpec("J4", 3, JointMap(1, 1, 0.0)),
        )
        clock = DeterministicClock(0.0)
        core1 = JointCore(self.specs, clock=clock)
        core2 = JointCore(specs2, clock=clock)

        station1 = FakeStation(core1)
        station2 = FakeStation(core2)
        self.assertTrue(station1.connect().accepted)
        self.assertTrue(station1.check().accepted)
        self.assertTrue(station1.arm().accepted)

        self.assertTrue(station2.connect().accepted)
        self.assertTrue(station2.check().accepted)
        self.assertTrue(station2.arm().accepted)

        self.assertTrue(core1.communication_failed(
            self.specs[0].motor_id,
            CommunicationFailure.DRIVER_FAILURE,
        ).accepted)
        snap1 = core1.snapshot()
        self.assertIs(snap1.state, State.FAULT)
        self.assertTrue(snap1.fault_latched)
        self.assertEqual(snap1.affected_joints, ("J1",))

        snap2 = core2.snapshot()
        self.assertIs(snap2.state, State.ARMED)
        self.assertFalse(snap2.fault_latched)
        self.assertEqual(snap2.faults, ())
        self.assertEqual(snap2.affected_joints, ())
        self.assertTrue(station2.stop().accepted)
        self.assertIs(core2.snapshot().state, State.STOPPED)

    def test_immutability_of_fault_records_and_snapshot_collections(self):
        fault = JointFault("J1", 0, errors=1, warnings=2, timeout=True)
        with self.assertRaises(FrozenInstanceError):
            fault.errors = 0
        with self.assertRaises(FrozenInstanceError):
            fault.name = "other"

        core = self._ready_core()
        self.assertTrue(core.communication_failed(0, CommunicationFailure.TIMEOUT).accepted)
        snap = core.snapshot()

        self.assertIsInstance(snap.faults, tuple)
        self.assertIsInstance(snap.affected_joints, tuple)
        with self.assertRaises(FrozenInstanceError):
            snap.faults[0].timeout = False
        with self.assertRaises(TypeError):
            snap.faults[0] = fault
        with self.assertRaises(TypeError):
            snap.affected_joints[0] = "J2"


class LatestTargetTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0)),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0)),
        )

    def _core_at(self, state, clock=None):
        if clock is None:
            clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock)
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
        if state is State.MOVING:
            self.assertTrue(station.arm().accepted)
            target = Target((
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
                JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            ))
            self.assertTrue(core.submit_target(target).accepted)
            self.assertIsNotNone(core.consume_target())
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
        raise AssertionError(f"{state} is not reachable")

    def test_ten_thousand_accepted_updates_leave_one_newest_target_and_one_consume(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)
        self.assertIsNone(core.snapshot().pending_target)

        last_target = None
        for i in range(10_000):
            target = Target((
                JointTarget(q_rad=i * 0.0001, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
                JointTarget(q_rad=-i * 0.0001, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
            ))
            decision = station.submit(target)
            self.assertTrue(decision.accepted)
            last_target = target

        snap = core.snapshot()
        self.assertEqual(snap.pending_target, last_target)
        self.assertIs(snap.state, State.ARMED)

        consumed = station.consume()
        self.assertEqual(consumed, last_target)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertIsNone(core.snapshot().pending_target)

        second_consumed = station.consume()
        self.assertIsNone(second_consumed)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertIsNone(core.snapshot().pending_target)

    def test_immutable_owned_vector(self):
        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)

        for bad_joints in (
            [jt1, jt2],
            "not a tuple",
            123,
            None,
            {jt1, jt2},
        ):
            with self.subTest(bad_joints=repr(bad_joints)), self.assertRaises(ValueError):
                Target(bad_joints)

        with self.assertRaises(ValueError):
            Target(())

        for bad_elements in (
            (jt1, "not a joint target"),
            (1, 2),
            (jt1, None),
        ):
            with self.subTest(bad_elements=repr(bad_elements)), self.assertRaises(ValueError):
                Target(bad_elements)

        target = Target((jt1, jt2))
        with self.assertRaises(FrozenInstanceError):
            target.joints = (jt2, jt1)
        with self.assertRaises(TypeError):
            target.joints[0] = jt2
        with self.assertRaises(FrozenInstanceError):
            target.joints[0].q_rad = 99.0

        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)
        self.assertTrue(station.submit(target).accepted)

        snap = core.snapshot()
        self.assertEqual(snap.pending_target, target)
        with self.assertRaises(FrozenInstanceError):
            snap.pending_target = None
        with self.assertRaises(TypeError):
            snap.pending_target.joints[0] = jt1
        with self.assertRaises(FrozenInstanceError):
            snap.pending_target.joints[0].tau_nm = 0.5

    def test_malformed_incomplete_and_invalid_mapped_outputs_preserve_prior_target(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        prior_target = Target((jt1, jt2))
        self.assertTrue(station.submit(prior_target).accepted)
        self.assertEqual(core.snapshot().pending_target, prior_target)

        for malformed in (None, "target", 42, object(), [jt1, jt2]):
            with self.subTest(malformed=repr(malformed)):
                decision = core.submit_target(malformed)
                self.assertFalse(decision.accepted)
                self.assertEqual(decision.code, DecisionCode.INVALID_ARGUMENT)
                self.assertEqual(core.snapshot().pending_target, prior_target)

        for wrong_vector in (
            Target((jt1,)),
            Target((jt1, jt2, jt1)),
        ):
            with self.subTest(wrong_vector=wrong_vector):
                decision = core.submit_target(wrong_vector)
                self.assertFalse(decision.accepted)
                self.assertEqual(decision.code, DecisionCode.INVALID_ARGUMENT)
                self.assertEqual(core.snapshot().pending_target, prior_target)

        huge_speed = Target((JointTarget(0.0, 1e7, 0.0, 1.0, 0.1), jt2))
        rej = core.submit_target(huge_speed)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)

        huge_kp = Target((JointTarget(0.0, 0.0, 0.0, 31.0, 0.1), jt2))
        rej = core.submit_target(huge_kp)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)

        huge_kd = Target((JointTarget(0.0, 0.0, 0.0, 1.0, 6.1), jt2))
        rej = core.submit_target(huge_kd)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)

        huge_pos = Target((JointTarget(1e7, 0.0, 0.0, 1.0, 0.1), jt2))
        rej = core.submit_target(huge_pos)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)

    def test_each_fault_path_clears_pending_without_fake_foc_writes(self):
        for action in ('packet', 'communication', 'tick'):
            with self.subTest(action=action):
                clock = DeterministicClock(0.0)
                core = JointCore(self.specs, clock=clock)
                station = FakeStation(core)
                self.assertTrue(station.connect())
                self.assertTrue(station.check())
                self.assertTrue(station.arm())
                target = Target((JointTarget(0, 0, 0, 1, .1),) * 2)
                self.assertTrue(station.submit(target))
                station.writes.clear()
                if action == 'packet':
                    self.assertTrue(core.receive(1, SAMPLE, MotorHealth(0, 4, 0, False)))
                elif action == 'communication':
                    self.assertTrue(core.communication_failed(1, CommunicationFailure.TIMEOUT))
                else:
                    clock.value = .6
                    self.assertTrue(core.tick())
                self.assertIs(core.snapshot().state, State.FAULT)
                self.assertIsNone(core.snapshot().pending_target)
                self.assertIsNone(station.consume())
                self.assertEqual(station.writes, [])

    def test_mapped_torque_outside_codec_bounds_rejects(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        prior_target = Target((jt1, jt2))
        self.assertTrue(station.submit(prior_target).accepted)

        too_high = Target((JointTarget(0.0, 0.0, 3.501, 1.0, 0.1), jt2))
        rej = core.submit_target(too_high)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)

        too_low = Target((JointTarget(0.0, 0.0, -3.501, 1.0, 0.1), jt2))
        rej = core.submit_target(too_low)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)

        specs_ratio = (
            JointSpec("J1", 0, JointMap(reduction=3.0, direction=-1, zero_output_rad=0.0)),
            JointSpec("J2", 1, JointMap(reduction=1.0, direction=1, zero_output_rad=0.0)),
        )
        core_ratio = JointCore(specs_ratio, clock=clock)
        station_ratio = FakeStation(core_ratio)
        self.assertTrue(station_ratio.connect().accepted)
        self.assertTrue(station_ratio.check().accepted)
        self.assertTrue(station_ratio.arm().accepted)

        rej_ratio = core_ratio.submit_target(
            Target((JointTarget(0.0, 0.0, 12.0, 9.0, 0.9), jt2))
        )
        self.assertFalse(rej_ratio.accepted)
        self.assertEqual(rej_ratio.code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(core_ratio.snapshot().pending_target)

    def test_successful_fake_adapter_uses_actual_codec_foc_packets_with_independent_expected_mappings(self):
        clock = DeterministicClock(0.0)
        specs = (
            JointSpec("J1", 0, JointMap(reduction=3.0, direction=-1, zero_output_rad=0.25)),
            JointSpec("J2", 1, JointMap(reduction=2.0, direction=1, zero_output_rad=0.1)),
        )
        core = JointCore(specs, clock=clock)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)
        station.writes.clear()

        jt1 = JointTarget(q_rad=0.1, dq_rad_s=1.0, tau_nm=-6.0, kp=18.0, kd=4.5)
        jt2 = JointTarget(q_rad=0.2, dq_rad_s=-1.0, tau_nm=3.0, kp=8.0, kd=2.0)
        target = Target((jt1, jt2))

        decision = station.submit(target)
        self.assertTrue(decision.accepted)
        self.assertEqual(station.writes, [])

        expected_pkt_1 = command(
            0,
            mode=1,
            rpm=-180.0 / math.tau,
            kd=0.5,
            torque=2.0,
            watchdog=True,
            position=-0.05,
            kp=2.0,
        )

        expected_pkt_2 = command(
            1,
            mode=1,
            rpm=-120.0 / math.tau,
            kd=0.5,
            torque=1.5,
            watchdog=True,
            position=0.5,
            kp=2.0,
        )

        consumed = station.consume()
        self.assertEqual(consumed, target)
        self.assertEqual(station.writes, [expected_pkt_1, expected_pkt_2])
        self.assertIs(core.snapshot().state, State.MOVING)

    def test_no_writes_on_rejection_stale_fault_stop_disconnect(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)
        station.writes.clear()

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        valid_target = Target((jt1, jt2))

        bad_target = Target((JointTarget(0.0, 0.0, 10.0, 1.0, 0.1), jt2))
        self.assertFalse(station.submit(bad_target).accepted)
        self.assertEqual(station.writes, [])

        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])

        self.assertTrue(station.submit(valid_target).accepted)
        clock.value = 1.000001
        self.assertEqual(station.submit(valid_target).code, DecisionCode.STALE_FEEDBACK)
        self.assertEqual(station.writes, [])

        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertIsNone(core.snapshot().pending_target)

        self.assertEqual(station.submit(valid_target).code, DecisionCode.INVALID_STATE)
        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])

        station.writes.clear()
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        station.writes.clear()
        self.assertTrue(station.arm().accepted)
        self.assertTrue(station.submit(valid_target).accepted)

        station.writes.clear()
        self.assertTrue(station.stop().accepted)
        self.assertEqual(station.writes, [command(spec.motor_id, mode=0) for spec in self.specs])
        self.assertIsNone(core.snapshot().pending_target)
        station.writes.clear()
        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])

        core2 = JointCore(self.specs, clock=clock)
        station2 = FakeStation(core2)
        self.assertTrue(station2.connect().accepted)
        self.assertTrue(station2.check().accepted)
        self.assertTrue(station2.arm().accepted)
        self.assertTrue(station2.submit(valid_target).accepted)
        station2.writes.clear()

        self.assertTrue(station2.disconnect().accepted)
        self.assertEqual(station2.writes, [])
        self.assertIsNone(core2.snapshot().pending_target)
        self.assertIsNone(station2.consume())
        self.assertEqual(station2.writes, [])

    def test_stale_feedback_discovered_on_consume_clears_pending_and_latches_fault(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=0.5)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm().accepted)

        target = Target((
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(target).accepted)
        self.assertEqual(core.snapshot().pending_target, target)

        clock.value = 0.4
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)

        clock.value = 0.6
        consumed = station.consume()
        self.assertIsNone(consumed)

        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertIsNone(snap.pending_target)
        self.assertEqual(snap.affected_joints, ("J2",))
        self.assertEqual(len(snap.faults), 1)
        self.assertTrue(snap.faults[0].stale)
        self.assertEqual(snap.faults[0].name, "J2")

        self.assertEqual(station.check().code, DecisionCode.FAULT_ACK_REQUIRED)

    def test_only_armed_and_moving_may_submit_and_consume(self):
        target = Target((
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        ))

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
                core = self._core_at(state)
                before = core.snapshot()
                self.assertEqual(core.submit_target(target).code, DecisionCode.INVALID_STATE)
                self.assertIsNone(core.consume_target())
                self.assertEqual(core.snapshot(), before)

        core = self._core_at(State.ARMED)
        self.assertTrue(core.submit_target(target).accepted)
        self.assertEqual(core.snapshot().pending_target, target)
        self.assertEqual(core.consume_target(), target)
        self.assertIs(core.snapshot().state, State.MOVING)

        target2 = Target((
            JointTarget(0.3, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.4, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(core.submit_target(target2).accepted)
        self.assertEqual(core.snapshot().pending_target, target2)
        self.assertEqual(core.consume_target(), target2)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertIsNone(core.snapshot().pending_target)


if __name__ == "__main__":
    unittest.main()
