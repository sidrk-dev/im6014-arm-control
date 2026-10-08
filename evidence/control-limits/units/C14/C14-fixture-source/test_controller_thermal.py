
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
    WriteResult,
)
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget
from joint_protection import (
    JointProtectionState,
    ProtectionPolicy,
    check_joint_limits,
    transition_protection,
)
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
            recovery_dwell_s=0.25,
        )
        self.policy2 = ProtectionPolicy(
            driver_trip_c=80.0,
            driver_reset_c=65.0,
            winding_trip_c=100.0,
            winding_reset_c=85.0,
            continuous_effort_nm=8.0,
            continuous_dwell_s=0.4,
            recovery_dwell_s=0.25,
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

    def _armed_station(self, max_reference_gap_s=1.0, feedback_max_age_s=0.5):
        core = JointCore(
            self.specs,
            clock=self.clock,
            max_reference_gap_s=max_reference_gap_s,
            feedback_max_age_s=feedback_max_age_s,
        )
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        owner = station.claim("thermal-tester")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(station.arm(owner).accepted)
        return core, station, owner

    def _single_joint_spec(
        self,
        *,
        continuous_dwell_s=0.25,
        recovery_dwell_s=0.25,
    ):
        policy = ProtectionPolicy(
            driver_trip_c=80.0,
            driver_reset_c=60.0,
            winding_trip_c=100.0,
            winding_reset_c=70.0,
            continuous_effort_nm=1.0,
            continuous_dwell_s=continuous_dwell_s,
            recovery_dwell_s=recovery_dwell_s,
        )
        spec = JointSpec(
            name="J",
            motor_id=0,
            mapping=JointMap(1.0, 1, 0.0),
            q_min_rad=-2.0,
            q_max_rad=2.0,
            v_max_rad_s=5.0,
            a_max_rad_s2=50.0,
            max_lead_rad=1.0,
            max_effort_nm=4.0,
            limits_provenance="fixture: C14 timing regression",
            protection=policy,
        )
        return spec

    def _single_joint_core(
        self,
        *,
        continuous_dwell_s=0.25,
        recovery_dwell_s=0.25,
        feedback_max_age_s=0.5,
    ):
        clock = DeterministicClock(0.0)
        spec = self._single_joint_spec(
            continuous_dwell_s=continuous_dwell_s,
            recovery_dwell_s=recovery_dwell_s,
        )
        core = JointCore(
            (spec,),
            clock=clock,
            feedback_max_age_s=feedback_max_age_s,
            max_reference_gap_s=0.5,
        )
        sample = JointSample(0.0, 0.0, 0.0)
        healthy_stop = MotorHealth(0, 0, 0, False, 20.0, 30.0)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(
            core.connected(SourceIdentity("fixture", "thermal-regression", 1)).accepted
        )
        owner = core.claim_owner("thermal-regression")
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0, sample, healthy_stop).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(core.arm(owner).accepted)
        return core, clock, spec, sample, healthy_stop

    def _receive_overload(self, core, clock, timestamp, *, mode=0):
        clock.value = timestamp
        return core.receive(
            0,
            JointSample(0.0, 0.0, 2.0),
            MotorHealth(mode, 0, 0, False, 20.0, 30.0),
        )


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

        self.clock.value = 10.0
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 10.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 10.0
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 10.25
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 10.499
        dec = core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        )
        self.assertTrue(dec.accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 10.5
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
        self.assertTrue(snap.faults[0].effort_limit)


    def test_fluctuating_effort_resets_overload_timer(self):
        core, station, owner = self._armed_station()

        self.clock.value = 10.0
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        self.clock.value = 10.25
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

        self.clock.value = 10.625
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)

        self.clock.value = 10.875
        self.assertTrue(core.receive(
            0,
            JointSample(0.0, 0.0, 12.0),
            MotorHealth(mode=1, errors=0, warnings=0, timeout=False, driver_c=25.0, winding_c=30.0),
        ).accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.clock.value = 11.125
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

    def test_communication_failure_preserves_overload_history_and_starts_a_new_interval(self):
        core, clock, spec, sample, healthy_stop = self._single_joint_core()
        self.assertTrue(self._receive_overload(core, clock, 0.0).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.0625).accepted)

        clock.value = 0.125
        self.assertTrue(
            core.communication_failed(0, CommunicationFailure.TIMEOUT).accepted
        )
        self.assertTrue(self._receive_overload(core, clock, 0.1875).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.25).accepted)
        self.assertFalse(core.snapshot().faults[0].continuous_effort_limit)

        self.assertTrue(self._receive_overload(core, clock, 0.3125).accepted)
        decision = self._receive_overload(core, clock, 0.375)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.EFFORT_OUT_OF_BOUNDS)
        fault = core.snapshot().faults[0]
        self.assertTrue(fault.continuous_effort_limit)
        self.assertTrue(fault.effort_limit)

    def test_failed_dispatch_preserves_overload_history_and_starts_a_new_interval(self):
        core, clock, spec, sample, healthy_stop = self._single_joint_core()
        owner = core.snapshot().current_owner
        self.assertIsNotNone(owner)
        target = Target(
            0,
            (JointTarget(0.0, 0.0, 0.0, 0.0, 0.0),),
            window=core.issue_window(0.5),
            owner=owner,
        )
        self.assertTrue(core.submit_target(target).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.0).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.0625).accepted)

        clock.value = 0.125
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        self.assertTrue(
            core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE).accepted
        )
        prior_record = core.snapshot().dispatched_records[0]
        self.assertIsNotNone(prior_record)

        self.assertTrue(self._receive_overload(core, clock, 0.125).accepted)
        clock.value = 0.1875
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        self.assertFalse(
            core.dispatch_result(plan.plan_id, 0, WriteResult.FAILED).accepted
        )
        snapshot = core.snapshot()
        self.assertEqual(snapshot.dispatched_records[0], prior_record)
        self.assertIs(snapshot.dispatch_attempts[0].result, WriteResult.FAILED)

        self.assertTrue(self._receive_overload(core, clock, 0.25).accepted)
        self.assertFalse(core.snapshot().faults[0].continuous_effort_limit)
        self.assertTrue(self._receive_overload(core, clock, 0.3125).accepted)
        decision = self._receive_overload(core, clock, 0.375)
        self.assertFalse(decision.accepted)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)

    def test_begin_check_breaks_overload_interval_without_erasing_history(self):
        clock = DeterministicClock(0.0)
        spec = self._single_joint_spec()
        core = JointCore(
            (spec,), clock=clock, feedback_max_age_s=0.5, max_reference_gap_s=0.5
        )
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(
            core.connected(SourceIdentity("fixture", "initial-check-gap", 1)).accepted
        )
        self.assertTrue(self._receive_overload(core, clock, 0.0).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.0625).accepted)

        clock.value = 0.125
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.1875).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.25).accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.assertTrue(self._receive_overload(core, clock, 0.3125).accepted)
        decision = self._receive_overload(core, clock, 0.375)
        self.assertFalse(decision.accepted)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)

    def test_takeover_breaks_overload_interval_without_erasing_history(self):
        core, clock, spec, sample, healthy_stop = self._single_joint_core()
        self.assertTrue(self._receive_overload(core, clock, 0.0).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.0625).accepted)

        clock.value = 0.125
        core.claim_owner("thermal-takeover")
        self.assertTrue(self._receive_overload(core, clock, 0.1875).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.25).accepted)
        self.assertFalse(core.snapshot().fault_latched)

        self.assertTrue(self._receive_overload(core, clock, 0.3125).accepted)
        decision = self._receive_overload(core, clock, 0.375)
        self.assertFalse(decision.accepted)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)

    def test_stop_and_recheck_break_overload_interval_without_erasing_history(self):
        core, clock, spec, sample, healthy_stop = self._single_joint_core()
        self.assertTrue(self._receive_overload(core, clock, 0.0).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.0625).accepted)
        self.assertTrue(core.stop().accepted)
        self.assertTrue(core.begin_check().accepted)

        self.assertTrue(self._receive_overload(core, clock, 0.125).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.1875).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.25).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertFalse(core.snapshot().fault_latched)

        decision = self._receive_overload(core, clock, 0.3125)
        self.assertFalse(decision.accepted)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)

    def test_binary_exact_continuous_dwell_before_equal_and_after(self):
        cases = (
            (math.nextafter(0.25, 0.0), False),
            (0.25, True),
            (math.nextafter(0.25, math.inf), True),
        )
        for timestamp, expected_fault in cases:
            with self.subTest(timestamp=timestamp):
                core, clock, spec, sample, healthy_stop = self._single_joint_core()
                self.assertTrue(self._receive_overload(core, clock, 0.0).accepted)
                self._receive_overload(core, clock, timestamp)
                self.assertEqual(core.snapshot().fault_latched, expected_fault)

    def test_exact_received_times_do_not_round_overload_up_to_dwell(self):
        dwell_s = 0.30000000000000004
        self.assertLess(Fraction(0.4) - Fraction(0.1), Fraction(dwell_s))
        core, clock, spec, sample, healthy_stop = self._single_joint_core(
            continuous_dwell_s=dwell_s
        )
        self.assertTrue(self._receive_overload(core, clock, 0.1).accepted)
        self.assertTrue(self._receive_overload(core, clock, 0.4).accepted)
        self.assertFalse(core.snapshot().fault_latched)

        decision = self._receive_overload(core, clock, 0.5)
        self.assertFalse(decision.accepted)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)

    def test_binary_exact_recovery_dwell_before_equal_and_after(self):
        cases = (
            (math.nextafter(0.25, 0.0), False),
            (0.25, True),
            (math.nextafter(0.25, math.inf), True),
        )
        for timestamp, expected_recovery in cases:
            with self.subTest(timestamp=timestamp):
                core, clock, spec, sample, healthy_stop = self._single_joint_core()
                hot = MotorHealth(1, 0, 0, False, 80.0, 30.0)
                self.assertFalse(core.receive(0, sample, hot).accepted)
                self.assertTrue(core.stop().accepted)
                core.claim_owner("thermal-recovery")
                self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
                self.assertTrue(core.receive(0, sample, healthy_stop).accepted)
                clock.value = timestamp
                self.assertTrue(core.receive(0, sample, healthy_stop).accepted)

                decision = core.checked()
                self.assertEqual(decision.accepted, expected_recovery)
                self.assertEqual(core.snapshot().fault_latched, not expected_recovery)

    def test_exact_received_times_do_not_round_recovery_up_to_dwell(self):
        dwell_s = 0.30000000000000004
        self.assertLess(Fraction(0.4) - Fraction(0.1), Fraction(dwell_s))
        core, clock, spec, sample, healthy_stop = self._single_joint_core(
            recovery_dwell_s=dwell_s
        )
        hot = MotorHealth(1, 0, 0, False, 80.0, 30.0)
        self.assertFalse(core.receive(0, sample, hot).accepted)
        self.assertTrue(core.stop().accepted)
        core.claim_owner("thermal-recovery")
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        clock.value = 0.1
        self.assertTrue(core.receive(0, sample, healthy_stop).accepted)
        clock.value = 0.4
        self.assertTrue(core.receive(0, sample, healthy_stop).accepted)
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        clock.value = 0.5
        self.assertTrue(core.receive(0, sample, healthy_stop).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertFalse(core.snapshot().fault_latched)

    def test_recovery_gap_resets_each_joint_independently(self):
        core, station, owner = self._armed_station(feedback_max_age_s=0.125)
        hot = MotorHealth(1, 0, 0, False, 85.0, 30.0)
        self.assertFalse(core.receive(0, self.sample1, hot).accepted)
        self.assertTrue(station.stop().accepted)
        core.claim_owner("two-joint-recovery")
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)

        self.clock.value = 10.0
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertFalse(core.checked().accepted)

        self.clock.value = 10.125
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.clock.value = 10.25
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        self.clock.value = 10.375
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        self.clock.value = 10.5
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertFalse(core.snapshot().fault_latched)

    def test_finite_clock_extremes_keep_dwell_records_bounded(self):
        spec = self._single_joint_spec()
        overloaded_sample = JointSample(0.0, 0.0, 2.0)
        healthy_sample = JointSample(0.0, 0.0, 0.0)
        healthy_stop = MotorHealth(0, 0, 0, False, 20.0, 30.0)

        overload_state = JointProtectionState()
        recovery_state = JointProtectionState()
        for timestamp in (-1e308, 0.0, 1e308):
            _, overload_state, _ = transition_protection(
                spec,
                overloaded_sample,
                healthy_stop,
                overload_state,
                timestamp,
                1e308,
                False,
            )
            _, recovery_state, _ = transition_protection(
                spec,
                healthy_sample,
                healthy_stop,
                recovery_state,
                timestamp,
                1e308,
                True,
            )

        self.assertTrue(math.isfinite(overload_state.overload_duration_s))
        self.assertLessEqual(
            overload_state.overload_duration_s,
            Fraction(spec.protection.continuous_dwell_s),
        )
        self.assertTrue(math.isfinite(recovery_state.recovery_duration_s))
        self.assertLessEqual(
            recovery_state.recovery_duration_s,
            Fraction(spec.protection.recovery_dwell_s),
        )


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
        self.assertIs(dec.code, DecisionCode.INVALID_STATE)
        self.assertTrue(core.snapshot().fault_latched)

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].driver_temperature_limit)

        self.assertTrue(core.disconnect().accepted)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertTrue(core.snapshot().faults[0].driver_temperature_limit)

        owner2 = station.claim("new-tester")
        self.assertFalse(station.arm(owner2).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)


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

        self.clock.value = 20.0
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        self.clock.value = 20.125
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)

        self.clock.value = 20.25
        self.assertTrue(core.receive(0, self.sample1, HEALTHY_STOP).accepted)
        self.assertTrue(core.receive(1, self.sample2, HEALTHY_STOP).accepted)

        self.assertTrue(core.checked().accepted)
        snap = core.snapshot()
        self.assertIs(snap.state, State.READY)
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

        self.clock.value = 30.25
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
        with self.assertRaises(ValueError):
            core.receive(0, self.sample1, HEALTHY_STOP)

        self.clock.value = 60.0
        self.assertFalse(core.checked().accepted)
        self.assertTrue(core.snapshot().fault_latched)


if __name__ == "__main__":
    unittest.main()
