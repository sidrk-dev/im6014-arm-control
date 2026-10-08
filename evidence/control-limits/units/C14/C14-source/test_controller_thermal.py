"""Acceptance and unit tests for C14 persistent temperature and continuous effort latches."""

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

    # -------------------------------------------------------------------------
    # 1. ProtectionPolicy and MotorHealth validation tests
    # -------------------------------------------------------------------------

    def test_protection_policy_field_validation(self):
        # Valid policy works
        p = ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, 0.5, 0.2)
        self.assertEqual(p.driver_trip_c, 85.0)
        self.assertEqual(p.driver_reset_c, 70.0)
        self.assertEqual(p.winding_trip_c, 105.0)
        self.assertEqual(p.winding_reset_c, 90.0)
        self.assertEqual(p.continuous_effort_nm, 10.0)
        self.assertEqual(p.continuous_dwell_s, 0.5)
        self.assertEqual(p.recovery_dwell_s, 0.2)

        # driver_reset_c >= driver_trip_c rejected
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 85.0, 105.0, 90.0, 10.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 90.0, 105.0, 90.0, 10.0, 0.5, 0.2)

        # winding_reset_c >= winding_trip_c rejected
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 105.0, 10.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 110.0, 10.0, 0.5, 0.2)

        # Non-numeric / bool / NaN / Inf rejected
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

        # continuous_effort_nm <= 0 rejected
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 0.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, -1.0, 0.5, 0.2)

        # continuous_dwell_s <= 0 rejected
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, 0.0, 0.2)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, -0.5, 0.2)

        # recovery_dwell_s <= 0 rejected
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, 0.5, 0.0)
        with self.assertRaises(ValueError):
            ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 10.0, 0.5, -0.2)

    def test_joint_spec_rejects_continuous_effort_exceeding_max(self):
        # continuous_effort_nm > max_effort_nm rejected
        bad_policy = ProtectionPolicy(85.0, 70.0, 105.0, 90.0, 25.0, 0.5, 0.2)
        with self.assertRaises(ValueError):
            JointSpec(
                "J1", 0, JointMap(1.0, 1, 0.0), -2.0, 2.0, 5.0, 50.0, 1.0, 20.0,
                "fixture: bad continuous effort", bad_policy,
            )

    def test_motor_health_temperature_validation(self):
        # Valid MotorHealth with driver_c and winding_c
        h = MotorHealth(0, 0, 0, False, driver_c=35.5, winding_c=40.0)
        self.assertEqual(h.driver_c, 35.5)
        self.assertEqual(h.winding_c, 40.0)

        # bool rejected
        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c=True, winding_c=40.0)
        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c=35.5, winding_c=False)

        # NaN / inf rejected
        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c=float("nan"), winding_c=40.0)
        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c=35.5, winding_c=float("inf"))

        # String rejected
        with self.assertRaises(ValueError):
            MotorHealth(0, 0, 0, False, driver_c="35.5", winding_c=40.0)

    # -------------------------------------------------------------------------
    # 2. Exact trip equality for driver and winding temperature
    # -------------------------------------------------------------------------

    def test_driver_temperature_exact_trip_boundary(self):
        core, station, owner = self._armed_station()

        # Just below trip (85.0 - 1e-6): accepted, no fault
        self.clock.value += 0.01
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=84.999999, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        # Exact trip threshold (85.0): trips immediately
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

        # Just below winding trip (105.0 - 1e-6): accepted, no fault
        self.clock.value += 0.01
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=104.999999),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        # Exact trip threshold (105.0): trips immediately
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

    # -------------------------------------------------------------------------
    # 3. Peak effort immediate trip vs continuous effort dwell
    # -------------------------------------------------------------------------

    def test_peak_effort_immediate_trip(self):
        # max_effort_nm is 20.0, continuous_effort_nm is 10.0
        core, station, owner = self._armed_station()
        self.clock.value += 0.01

        # Measured effort at 20.001 Nm (> peak): trips immediately without waiting for dwell
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
        # continuous_effort_nm is 10.0, continuous_dwell_s is 0.5s
        core, station, owner = self._armed_station()

        # Exactly at continuous threshold (10.0 Nm): does not accumulate overload
        self.clock.value += 0.01
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 10.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        # Above continuous threshold (12.0 Nm): overload starts at t = 10.02
        self.clock.value = 10.02
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        # At t = 10.02 + 0.499: duration is 0.499s (< 0.5s): accepted, no fault
        self.clock.value = 10.02 + 0.499
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        # At t = 10.02 + 0.5: duration is 0.5s (exact equality continuous_dwell_s): trips!
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

    # -------------------------------------------------------------------------
    # 4. Fluctuating effort resets overload accumulation
    # -------------------------------------------------------------------------

    def test_fluctuating_effort_resets_overload_timer(self):
        core, station, owner = self._armed_station()

        # Overload at 12.0 Nm begins at t = 10.1
        self.clock.value = 10.1
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        # Persists for 0.3s (duration = 0.3 < 0.5)
        self.clock.value = 10.4
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        # Effort drops below continuous threshold to 8.0 Nm at t = 10.5
        self.clock.value = 10.5
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 8.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        # New overload burst at 12.0 Nm starts at t = 10.6
        self.clock.value = 10.6
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        # At t = 10.9 (duration of current burst is 0.3s < 0.5s): accepted, no fault
        # (even though total overload time across bursts is 0.3 + 0.3 = 0.6s)
        self.clock.value = 10.9
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)
        self.assertFalse(core.snapshot().fault_latched)

        # At t = 11.1 (duration of current burst reaches 0.5s): trips!
        self.clock.value = 11.1
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertFalse(dec.accepted)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)

    # -------------------------------------------------------------------------
    # 5. Interrupted / stale observations do not credit dwell
    # -------------------------------------------------------------------------

    def test_interrupted_observations_do_not_credit_dwell(self):
        # max_reference_gap_s = 0.5
        core, station, owner = self._armed_station(max_reference_gap_s=0.5)

        # Overload starts at t = 10.1
        self.clock.value = 10.1
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        # Telemetry gap of 1.0s (exceeds max_reference_gap_s 0.5s)
        self.clock.value = 11.1
        # Next sample arrives after gap: gap cannot count as confirmed overload duration!
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)
        # Not faulted because gap reset timing continuity
        self.assertFalse(core.snapshot().fault_latched)

    # -------------------------------------------------------------------------
    # 6. Two simultaneous causes and joints
    # -------------------------------------------------------------------------

    def test_simultaneous_causes_union_on_single_and_multiple_joints(self):
        core, station, owner = self._armed_station()

        # Induce driver temperature fault on J1 at t = 10.1
        self.clock.value = 10.1
        core.receive(
            0,
            JointSample(0.0, 0.0, 5.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].driver_temperature_limit)

        # Now also exceed peak effort on J1
        core.receive(
            0,
            JointSample(0.0, 0.0, 25.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        # Union on J1 has BOTH causes
        fault_j1 = next(f for f in core.snapshot().faults if f.name == "J1")
        self.assertTrue(fault_j1.driver_temperature_limit)
        self.assertTrue(fault_j1.effort_limit)

        # Induce winding temperature fault on J2
        core.receive(
            1,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=100.0),
        )
        snap = core.snapshot()
        self.assertEqual(snap.affected_joints, ("J1", "J2"))
        fault_j2 = next(f for f in snap.faults if f.name == "J2")
        self.assertTrue(fault_j2.winding_temperature_limit)

    # -------------------------------------------------------------------------
    # 7. Fault persistence through Stop / Reconnect / Healthy / New Targets
    # -------------------------------------------------------------------------

    def test_fault_persists_through_stop_reconnect_healthy_and_new_targets(self):
        core, station, owner = self._armed_station()

        # Fault J1 on driver temperature
        self.clock.value += 0.01
        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=90.0, winding_c=30.0),
        )
        self.assertTrue(core.snapshot().fault_latched)

        # 1. Healthy packet arrival cannot clear fault
        self.clock.value += 0.01
        core.receive(0, JointSample(0.0, 0.0, 0.0), HEALTHY_STOP)
        self.assertTrue(core.snapshot().fault_latched)

        # 2. Submitting new target rejected with FAULT_LATCHED
        w = station.issue_window(1.0)
        t = Target(1, (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)), window=w, owner=owner)
        dec = station.submit(t)
        self.assertFalse(dec.accepted)
        self.assertIs(dec.code, DecisionCode.FAULT_LATCHED)
        self.assertTrue(core.snapshot().fault_latched)

        # 3. Stop transitions to STOPPED but preserves fault
        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].driver_temperature_limit)

        # 4. Disconnect and reconnect preserves fault
        self.assertTrue(core.disconnect().accepted)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].driver_temperature_limit)

        # 5. Arming while faulted is rejected
        owner2 = station.claim("new-tester")
        self.assertFalse(station.arm(owner2).accepted)

    # -------------------------------------------------------------------------
    # 8. Recovery requirements: explicit acknowledgment, dwell equality, reset temp
    # -------------------------------------------------------------------------

    def test_recovery_requires_explicit_acknowledgment_and_full_dwell(self):
        core, station, owner = self._armed_station()

        # Fault J1 on driver temperature
        self.clock.value = 20.0
        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(station.stop().accepted)

        # Attempt recovery without acknowledge_fault: rejected
        self.assertFalse(core.begin_check(acknowledge_fault=False).accepted)

        # Begin check with acknowledge_fault=True: accepted
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.assertIs(core.snapshot().state, State.CHECKING)

        # Sample 1 at t = 20.1: healthy Stop mode below reset temp
        self.clock.value = 20.1
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        # Checked immediately: duration is 0 < recovery_dwell_s (0.2s): cannot complete recovery
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        # Sample 2 at t = 20.1 + 0.199s (< 0.2s recovery dwell)
        self.clock.value = 20.1 + 0.199
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        # Sample 3 at t = 20.1 + 0.2s (exact equality: recovery_dwell_s reached!)
        self.clock.value = 20.1 + 0.2
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        # Now checked succeeds!
        self.assertTrue(core.checked().accepted)
        snap = core.snapshot()
        self.assertIs(snap.state, State.CHECKED)
        self.assertFalse(snap.fault_latched)
        self.assertEqual(snap.faults, ())

    def test_recovery_exact_reset_temperature_equality(self):
        core, station, owner = self._armed_station()

        # Fault J1 on driver temperature
        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(station.stop().accepted)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

        # driver_reset_c is 70.0. Exactly at reset (70.0 C): valid for recovery
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

        # Fault J1 on driver temperature
        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(station.stop().accepted)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

        # Just above reset (70.0001 C): cannot count toward recovery dwell
        self.clock.value = 40.0
        h_above_reset = MotorHealth(0, 0, 0, False, driver_c=70.0001, winding_c=30.0)
        self.assertTrue(core.receive(0, self.sample1, h_above_reset).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.clock.value = 40.5
        self.assertTrue(core.receive(0, self.sample1, h_above_reset).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        # Recovery rejected because temperature above reset
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

    def test_recovery_requires_both_joints_fresh_simultaneously(self):
        core, station, owner = self._armed_station(max_reference_gap_s=0.5)

        # Fault J1
        core.receive(
            0,
            JointSample(0.0, 0.0, 0.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=85.0, winding_c=30.0),
        )
        self.assertTrue(station.stop().accepted)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

        # J1 and J2 receive initial sample at t = 50.0
        self.clock.value = 50.0
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        # At t = 50.6 (exceeds max_reference_gap_s 0.5s), only J1 receives fresh sample
        self.clock.value = 50.6
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        # J2 is now stale!
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

    def test_bad_or_regressed_clock_cannot_complete_recovery(self):
        core, station, owner = self._armed_station()

        # Fault J1
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

        # Clock steps backwards
        self.clock.value = 59.9
        dec = core.receive(0, self.sample1, HEALTHY_STOP)
        self.assertFalse(dec.accepted)

        # Recovery cannot complete
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)


if __name__ == "__main__":
    unittest.main()
