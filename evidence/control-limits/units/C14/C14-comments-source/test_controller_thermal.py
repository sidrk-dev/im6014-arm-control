
from fractions import Fraction
import math
import unittest

from joint_contract import (
    CommunicationFailure,
    DecisionCode,
    JointFault,
    JointSpec,
    JointTrajectoryReference,
    MotorHealth,
    SourceIdentity,
    State,
    Target,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget
from joint_protection import JointProtectionState, ProtectionPolicy, check_joint_limits
from test_controller import DeterministicClock, FakeStation, HEALTHY_STOP, SOURCE


class ThermalPersistentEffortTests(unittest.TestCase):
    def setUp(self):
        self.clock = DeterministicClock(10.0)
        self.policy1 = ProtectionPolicy(
            driver_trip_c=85.0,
            driver_reset_c=70.0,
            winding_trip_c=105.0,
            winding_reset_c=90.0,
            continuous_effort_nm=10.0,
            continuous_dwell_s=0.5,
            recovery_dwell_s=0.2,
        )
        self.policy2 = ProtectionPolicy(
            driver_trip_c=80.0,
            driver_reset_c=65.0,
            winding_trip_c=100.0,
            winding_reset_c=85.0,
            continuous_effort_nm=8.0,
            continuous_dwell_s=0.4,
            recovery_dwell_s=0.2,
        )
        self.spec1 = JointSpec(
            "J1",
            0,
            JointMap(1.0, 1, 0.0),
            -2.0,
            2.0,
            5.0,
            50.0,
            1.0,
            20.0,
            "fixture: C14 thermal test J1",
            self.policy1,
        )
        self.spec2 = JointSpec(
            "J2",
            1,
            JointMap(1.0, 1, 0.0),
            -2.0,
            2.0,
            5.0,
            50.0,
            1.0,
            15.0,
            "fixture: C14 thermal test J2",
            self.policy2,
        )
        self.specs = (self.spec1, self.spec2)
        self.sample1 = JointSample(0.0, 0.0, 0.0)
        self.sample2 = JointSample(0.0, 0.0, 0.0)

    def _armed_station(self, max_reference_gap_s=1.0):
        core = JointCore(self.specs, clock=self.clock, max_reference_gap_s=max_reference_gap_s)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("thermal-tester")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, owner


    def test_protection_policy_field_validation(self):
        p = ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, 0.5, 0.2)
        self.assertEqual(p.driver_trip_c, 85.0)
        self.assertEqual(p.driver_reset_c, 70.0)
        self.assertEqual(p.winding_trip_c, 105.0)
        self.assertEqual(p.winding_reset_c, 90.0)
        self.assertEqual(p.continuous_effort_nm, 10.0)
        self.assertEqual(p.continuous_dwell_s, 0.5)
        self.assertEqual(p.recovery_dwell_s, 0.2)

        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 85.0, 105.0, 90.0, 10.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 90.0, 105.0, 90.0, 10.0, 0.5, 0.2)

        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 105.0, 10.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 110.0, 10.0, 0.5, 0.2)

        with self.assertRaises(ValueError):
            ProtectionPolicy(True, 70.0, 105.0, 90.0, 10.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, False, 105.0, 90.0, 10.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(float("nan"), 70.0, 105.0, 90.0, 10.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, float("inf"), 90.0, 10.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, "10.0", 0.5, 0.2)

        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 0.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, -1.0, 0.5, 0.2)

        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, 0.0, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, -0.5, 0.2)

        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, 0.5, 0.0)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, 0.5, -0.2)

    def test_joint_spec_rejects_continuous_effort_exceeding_max(self):
        bad_policy = ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 25.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            JointSpec(
                "J1", 0, JointMap(1.0, 1, 0.0), -2.0, 2.0, 5.0, 50.0, 1.0, 20.0,
                "fixture: bad continuous effort", bad_policy,
            )

    def test_motor_health_temperature_validation(self):
        h = MotorHealth(0, 0, 0, False, driver_c=35.5, winding_c=40.0)
        self.assertEqual(h.driver_c, 35.5)
        self.assertEqual(h.winding_c, 40.0)

        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c=True, winding_c=40.0)
        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c=35.5, winding_c=False)

        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c=float("nan"), winding_c=40.0)
        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c=35.5, winding_c=float("inf"))

        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c="35.5", winding_c=40.0)


    def test_driver_temperature_exact_trip_boundary(self):
        core, station, owner = self._armed_station()

        self.clock.value += 0.01
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=84.999999, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value += 0.01
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertFalse(dec.accepted)
        self.assertIs(dec.code, DecisionCode.TEMPERATURE_OUT_OF_BOUNDS)
        snap = core.snapshot()
        self.assertTrue(snap.fault_latched)
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.faults[0].driver_temperature_limit)
        self.assertFalse(snap.faults[0].winding_temperature_limit)
        self.assertEqual(snap.affected_joints, ("J1",))

    def test_winding_temperature_exact_trip_boundary(self):
        core, station, owner = self._armed_station()

        self.clock.value += 0.01
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=104.999999),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value += 0.01
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=105.0),
        )
        self.assertFalse(dec.accepted)
        self.assertIs(dec.code, DecisionCode.TEMPERATURE_OUT_OF_BOUNDS)
        snap = core.snapshot()
        self.assertTrue(snap.fault_latched)
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.faults[0].winding_temperature_limit)
        self.assertFalse(snap.faults[0].driver_temperature_limit)
        self.assertEqual(snap.affected_joints, ("J1",))


    def test_peak_effort_immediate_trip(self):
        core, station, owner = self._armed_station()
        self.clock.value += 0.01

        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 20.001),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertFalse(dec.accepted)
        self.assertIs(dec.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)
        snap = core.snapshot()
        self.assertTrue(snap.fault_latched)
        self.assertTrue(snap.faults[0].effort_limit)
        self.assertFalse(snap.faults[0].continuous_effort_limit)

    def test_continuous_effort_exact_dwell_equality(self):
        core, station, owner = self._armed_station()

        self.clock.value += 0.01
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 10.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 10.02
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 10.02 + 0.499
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 10.02 + 0.5
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertFalse(dec.accepted)
        self.assertIs(dec.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)
        snap = core.snapshot()
        self.assertTrue(snap.fault_latched)
        self.assertTrue(snap.faults[0].continuous_effort_limit)
        self.assertFalse(snap.faults[0].effort_limit)


    def test_fluctuating_effort_resets_overload_timer(self):
        core, station, owner = self._armed_station()

        self.clock.value = 10.1
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        self.clock.value = 10.4
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        self.clock.value = 10.5
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 8.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        self.clock.value = 10.6
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        self.clock.value = 10.9
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 11.1
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertFalse(dec.accepted)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)


    def test_interrupted_observations_do_not_credit_dwell(self):
        core, station, owner = self._armed_station(max_reference_gap_s=0.5)

        self.clock.value = 10.1
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        self.clock.value = 11.1
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)
        self.assertFalse(core.snapshot().fault_latched)


    def test_simultaneous_causes_union_on_single_and_multiple_joints(self):
        core, station, owner = self._armed_station()

        self.clock.value = 10.1
        core.receive(
            0,
            JointSample(0.0, 0.0, 5.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].driver_temperature_limit)

        core.receive(
            0,
            JointSample(0.0, 0.0, 25.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        fault_j1 = next(f for f in core.snapshot().faults if f.name == "J1")
        self.assertTrue(fault_j1.driver_temperature_limit)
        self.assertTrue(fault_j1.effort_limit)

        core.receive(
            1,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=100.0),
        )
        snap = core.snapshot()
        self.assertEqual(snap.affected_joints, ("J1", "J2"))
        fault_j2 = next(f for f in snap.faults if f.name == "J2")
        self.assertTrue(fault_j2.winding_temperature_limit)


    def test_fault_persists_through_stop_reconnect_healthy_and_new_targets(self):
        core, station, owner = self._armed_station()

        self.clock.value += 0.01
        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=90.0, winding_c=30.0),
        )
        self.assertTrue(core.snapshot().fault_latched)

        self.clock.value += 0.01
        core.receive(0, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP)
        self.assertTrue(core.snapshot().fault_latched)

        w = station.issue_window(1.0)
        t = Target(1, (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)), window=w, owner=owner)
        dec = station.submit(t)
        self.assertFalse(dec.accepted)
        self.assertIs(dec.code, DecisionCode.FAULT_LATCHED)
        self.assertTrue(core.snapshot().fault_latched)

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].driver_temperature_limit)

        self.assertTrue(core.disconnect().accepted)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].driver_temperature_limit)

        owner2 = station.claim("new-tester")
        self.assertFalse(station.arm(owner2).accepted)


    def test_recovery_requires_explicit_acknowledgment_and_full_dwell(self):
        core, station, owner = self._armed_station()

        self.clock.value = 20.0
        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(station.stop().accepted)

        self.assertFalse(core.begin_check(acknowledge_fault=False).accepted)

        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.assertIs(core.snapshot().state, State.CHECKING)

        self.clock.value = 20.1
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        self.clock.value = 20.1 + 0.199
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        self.clock.value = 20.1 + 0.2
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.assertTrue(core.checked().accepted)
        snap = core.snapshot()
        self.assertIs(snap.state, State.CHECKED)
        self.assertFalse(snap.fault_latched)
        self.assertEqual(snap.faults, ())

    def test_recovery_exact_reset_temperature_equality(self):
        core, station, owner = self._armed_station()

        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(station.stop().accepted)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

        self.clock.value = 30.0
        h_exact_reset = MotorHealth(0, 0, 0, False, driver_c=70.0, winding_c=30.0)
        self.assertTrue(core.receive(0, self.sample1, h_exact_reset).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.clock.value = 30.2
        self.assertTrue(core.receive(0, self.sample1, h_exact_reset).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertFalse(core.snapshot().fault_latched)

    def test_recovery_rejected_if_above_reset_temperature(self):
        core, station, owner = self._armed_station()

        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(station.stop().accepted)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

        self.clock.value = 40.0
        h_above_reset = MotorHealth(0, 0, 0, False, driver_c=70.0001, winding_c=30.0)
        self.assertTrue(core.receive(0, self.sample1, h_above_reset).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.clock.value = 40.5
        self.assertTrue(core.receive(0, self.sample1, h_above_reset).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

    def test_recovery_requires_both_joints_fresh_simultaneously(self):
        core, station, owner = self._armed_station(max_reference_gap_s=0.5)

        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(station.stop().accepted)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

        self.clock.value = 50.0
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.clock.value = 50.6
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

    def test_bad_or_regressed_clock_cannot_complete_recovery(self):
        core, station, owner = self._armed_station()

        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(station.stop().accepted)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

        self.clock.value = 60.0
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.clock.value = 59.9
        dec = core.receive(0, self.sample1, HEALTHY_STOP)
        self.assertFalse(dec.accepted)

        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)


if __name__ == "__main__":
    unittest.main()
