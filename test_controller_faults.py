import math
import unittest
from dataclasses import FrozenInstanceError, replace
from unittest.mock import patch

from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget
from joint_records import (
    CommunicationFailure, DecisionCode, JointFault, JointSpec, MotorHealth,
    PositionLimits, ProtectionPolicy, SourceIdentity, State, Target,
)


SYNTHETIC_PROTECTION = ProtectionPolicy(70, 90, 5, 1, 0.5, 0.25)
SOURCE = SourceIdentity("fixture", "c14-protection", 1)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def spec(name="J1", motor_id=0, policy=SYNTHETIC_PROTECTION):
    return JointSpec(name, motor_id, JointMap(1, 1, 0), PositionLimits(-1, 1),
                     2, 10, 0.5, 20, protection=policy)


class ProtectionTests(unittest.TestCase):
    def make_core(self, specs=None, *, arm=True):
        clock = Clock()
        core = JointCore(specs or (spec(),), clock=clock, feedback_max_age_s=1,
                         max_reference_step_s=0.1)
        self.assertTrue(core.begin_connect())
        self.assertTrue(core.connected(SOURCE))
        self.assertTrue(core.begin_check())
        for joint in core.joint_specs:
            self.assertTrue(core.receive(joint.motor_id, JointSample(0, 0, 0),
                                        MotorHealth(0, 0, 0, False, 25, 25)))
        self.assertTrue(core.checked())
        if arm:
            self.assertTrue(core.arm(owner="fixture"))
        return core, clock

    def observe(self, core, clock, at, *, effort=6, temperature=25, mode=1, motor_id=0):
        clock.now = at
        return core.receive(motor_id, JointSample(0, 0, effort),
                            MotorHealth(mode, 0, 0, False, temperature, temperature))

    def overload(self, core, clock, *, start=0, motor_id=0):
        for i in range(5):
            self.assertTrue(self.observe(core, clock, start + i * 0.25, motor_id=motor_id))

    def fault(self, core, name="J1"):
        return next(f for f in core.snapshot().faults if f.name == name)

    def recover(self, core, clock, start, *, temperature=25):
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        for i in range(3):
            self.assertTrue(self.observe(core, clock, start + i * 0.25,
                                         effort=0, temperature=temperature, mode=0))
        return core.checked()

    def test_policy_values_are_explicit_finite_positive_and_frozen(self):
        for field in SYNTHETIC_PROTECTION.__dataclass_fields__:
            for value in (True, False, 0, -1, math.nan, math.inf, -math.inf, 10**400, "1", None):
                with self.subTest(field=field, value=repr(value)), self.assertRaises(ValueError):
                    replace(SYNTHETIC_PROTECTION, **{field: value})
        with self.assertRaises(FrozenInstanceError):
            SYNTHETIC_PROTECTION.driver_limit_c = 100
        with self.assertRaises(TypeError):
            JointSpec("J1", 0, JointMap(1, 1, 0), PositionLimits(-1, 1), 2, 10, .5, 20)
        with self.assertRaises(ValueError):
            replace(spec(), protection=None)

    def test_temperature_is_required_and_unknown_is_not_healthy(self):
        with self.assertRaises(TypeError):
            MotorHealth(0, 0, 0, False)
        self.assertFalse(MotorHealth(0, 0, 0, False, None, None).healthy)
        for bad in (True, False, math.nan, math.inf, -math.inf, 10**400, "25"):
            with self.subTest(bad=repr(bad)), self.assertRaises(ValueError):
                MotorHealth(0, 0, 0, False, bad, bad)
        self.assertTrue(MotorHealth(0, 0, 0, False, -20, -20).healthy)

    def test_protection_cause_flags_are_strict_bools(self):
        for field in ("over_temperature", "persistent_effort", "protection_unavailable"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                JointFault("J1", 0, **{field: 1})

    def test_temperature_threshold_latches_revokes_and_discards_pending(self):
        core, clock = self.make_core()
        owner = core.snapshot().ownership
        target = Target(1, (JointTarget(.2, 0, 0, 1, .1),), 0, 5, owner)
        self.assertTrue(core.submit_target(target))
        self.observe(core, clock, 0, effort=0, temperature=70)
        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(self.fault(core).over_temperature)
        self.assertIsNone(snap.ownership)
        self.assertIsNone(snap.pending_target)
        self.assertEqual(snap.last_sequence, 1)
        self.assertIsNone(core.consume_target())
        self.assertFalse(core.submit_target(target))

    def test_missing_temperature_latches_unavailable_and_cannot_arm(self):
        core, clock = self.make_core(arm=False)
        self.observe(core, clock, 0, effort=0, temperature=None, mode=0)
        self.assertTrue(self.fault(core).protection_unavailable)
        self.assertFalse(core.arm(owner="fixture"))
        self.assertEqual(core.begin_check().code, DecisionCode.FAULT_ACK_REQUIRED)

    def test_exact_sustained_duration_latches_without_commands(self):
        core, clock = self.make_core()
        for t in (0, .25, .5, .75):
            self.observe(core, clock, t)
            self.assertFalse(core.snapshot().fault_latched)
        self.observe(core, clock, 1)
        self.assertTrue(self.fault(core).persistent_effort)
        self.assertIs(core.snapshot().state, State.FAULT)

    def test_effort_sign_does_not_evade_threshold(self):
        core, clock = self.make_core()
        for t, effort in zip((0, .25, .5, .75, 1), (6, -6, 6, -6, 6)):
            self.observe(core, clock, t, effort=effort)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_effort_at_threshold_is_not_overload(self):
        core, clock = self.make_core()
        for t in (0, .25, .5, .75, 1, 1.25, 1.5):
            self.observe(core, clock, t, effort=5)
        self.assertFalse(core.snapshot().fault_latched)

    def test_new_targets_cannot_reset_confirmed_exposure(self):
        core, clock = self.make_core()
        owner = core.snapshot().ownership
        for i in range(5):
            self.observe(core, clock, i * .25)
            if i < 4:
                target = Target(i, (JointTarget(.2, 0, 0, 1, .1),), clock.now,
                                clock.now + 2, owner)
                self.assertTrue(core.submit_target(target))
        self.assertTrue(self.fault(core).persistent_effort)
        self.assertEqual(core.snapshot().last_sequence, 3)

    def test_suppressed_reference_does_not_reset_exposure(self):
        core, clock = self.make_core()
        owner = core.snapshot().ownership
        self.assertTrue(core.submit_target(Target(1, (JointTarget(.3, 0, 0, 1, .1),), 0, 5, owner)))
        with patch("joint_controller.plan_reference", return_value=None):
            for i in range(5):
                self.observe(core, clock, i * .25)
                self.assertIsNone(core.consume_target())
        self.assertTrue(self.fault(core).persistent_effort)

    def test_rejected_targets_do_not_reset_exposure(self):
        core, clock = self.make_core()
        for i in range(5):
            self.observe(core, clock, i * .25)
            self.assertFalse(core.submit_target(None))
        self.assertTrue(self.fault(core).persistent_effort)

    def test_long_gap_does_not_count_or_erase_prior_overload(self):
        core, clock = self.make_core()
        for t in (0, .25, .5):
            self.observe(core, clock, t)
        self.observe(core, clock, 100)
        self.assertFalse(core.snapshot().fault_latched)
        self.observe(core, clock, 100.25)
        self.assertFalse(core.snapshot().fault_latched)
        self.observe(core, clock, 100.5)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_repeated_same_timestamp_does_not_add_duration(self):
        core, clock = self.make_core()
        for _ in range(100):
            self.observe(core, clock, 0)
        self.assertFalse(core.snapshot().fault_latched)
        self.observe(core, clock, .25)
        self.assertFalse(core.snapshot().fault_latched)

    def test_brief_healthy_interval_does_not_erase_exposure(self):
        core, clock = self.make_core()
        for t in (0, .25, .5):
            self.observe(core, clock, t)
        for t in (.75, 1):
            self.observe(core, clock, t, effort=0)
        for t in (1.25, 1.5, 1.75):
            self.observe(core, clock, t)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_contiguous_healthy_interval_resets_unlatched_exposure(self):
        core, clock = self.make_core()
        for t in (0, .25, .5):
            self.observe(core, clock, t)
        for t in (.75, 1, 1.25):
            self.observe(core, clock, t, effort=0)
        for t in (1.5, 1.75, 2, 2.25):
            self.observe(core, clock, t)
            self.assertFalse(core.snapshot().fault_latched)
        self.observe(core, clock, 2.5)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_stop_and_recheck_do_not_erase_partial_exposure(self):
        core, clock = self.make_core()
        for t in (0, .25, .5):
            self.observe(core, clock, t)
        self.assertTrue(core.stop())
        self.assertTrue(core.begin_check())
        self.observe(core, clock, .5, effort=0, mode=0)
        self.assertTrue(core.checked())
        self.assertTrue(core.arm(owner="second"))
        for t in (.5, .75, 1):
            self.observe(core, clock, t)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_disconnect_reconnect_does_not_erase_partial_exposure(self):
        core, clock = self.make_core()
        for t in (0, .25, .5):
            self.observe(core, clock, t)
        self.assertTrue(core.disconnect())
        self.assertTrue(core.begin_connect())
        self.assertTrue(core.connected(replace(SOURCE, connection=2)))
        self.assertTrue(core.begin_check())
        self.observe(core, clock, .5, effort=0, mode=0)
        self.assertTrue(core.checked())
        self.assertTrue(core.arm(owner="second"))
        for t in (.5, .75, 1):
            self.observe(core, clock, t)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_fault_survives_stop_reconnect_and_unacknowledged_recovery(self):
        core, clock = self.make_core()
        self.overload(core, clock)
        self.assertTrue(core.stop())
        self.assertTrue(core.disconnect())
        self.assertTrue(core.begin_connect())
        self.assertTrue(core.connected(replace(SOURCE, connection=2)))
        self.observe(core, clock, 1.25, effort=0, mode=0)
        self.assertTrue(core.snapshot().fault_latched)
        self.assertEqual(core.begin_check().code, DecisionCode.FAULT_ACK_REQUIRED)
        self.assertFalse(core.arm(owner="second"))
        self.assertTrue(self.recover(core, clock, 1.5))
        self.assertFalse(core.snapshot().fault_latched)
        self.assertIsNone(core.snapshot().ownership)

    def test_acknowledgment_requires_contiguous_healthy_stopped_recovery(self):
        core, clock = self.make_core()
        self.overload(core, clock)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        self.observe(core, clock, 1.25, effort=0, mode=0)
        self.assertEqual(core.checked().code, DecisionCode.RECOVERY_PENDING)
        self.observe(core, clock, 1.5, effort=0, mode=0)
        self.assertEqual(core.checked().code, DecisionCode.RECOVERY_PENDING)
        self.observe(core, clock, 1.75, effort=0, mode=0)
        self.assertTrue(core.checked())
        self.assertIsNone(core.snapshot().ownership)

    def test_recovery_gap_restarts_credit_instead_of_concatenating_it(self):
        core, clock = self.make_core()
        self.overload(core, clock)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        for t in (1.25, 1.5, 10, 10.25):
            self.observe(core, clock, t, effort=0, mode=0)
            self.assertEqual(core.checked().code, DecisionCode.RECOVERY_PENDING)
        self.observe(core, clock, 10.5, effort=0, mode=0)
        self.assertTrue(core.checked())

    def test_nonstop_feedback_cannot_earn_recovery_credit(self):
        core, clock = self.make_core()
        self.overload(core, clock)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        for t in (1.25, 1.5, 1.75):
            self.observe(core, clock, t, effort=0, mode=1)
        self.assertEqual(core.checked().code, DecisionCode.NOT_STOPPED)
        self.observe(core, clock, 2, effort=0, mode=0)
        self.assertEqual(core.checked().code, DecisionCode.RECOVERY_PENDING)

    def test_recovery_unknown_temperature_adds_cause_and_requires_new_ack(self):
        core, clock = self.make_core()
        self.overload(core, clock)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        self.observe(core, clock, 1.25, effort=0, temperature=None, mode=0)
        self.assertTrue(self.fault(core).persistent_effort)
        self.assertTrue(self.fault(core).protection_unavailable)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(self.recover(core, clock, 1.5))

    def test_hot_and_hardware_fault_causes_are_additive(self):
        core, clock = self.make_core()
        clock.now = .1
        self.assertTrue(core.receive(0, JointSample(0, 0, 0), MotorHealth(0, 4, 2, True, 80, 80)))
        f = self.fault(core)
        self.assertTrue(f.over_temperature)
        self.assertEqual(f.errors, 4)
        self.assertEqual(f.warnings, 2)
        self.assertTrue(f.timeout)
        self.assertTrue(core.communication_failed(0, CommunicationFailure.TIMEOUT))
        self.assertTrue(self.fault(core).over_temperature)
        self.assertIn(CommunicationFailure.TIMEOUT, self.fault(core).communication_errors)

    def test_multiple_joints_keep_independent_exposure_and_fault_causes(self):
        core, clock = self.make_core((spec(), spec("J2", 1)))
        for t in (0, .25, .5, .75, 1):
            self.observe(core, clock, t, motor_id=0)
            self.observe(core, clock, t, effort=0, motor_id=1)
        self.assertEqual(core.snapshot().affected_joints, ("J1",))
        self.observe(core, clock, 1, effort=0, temperature=80, motor_id=1)
        self.assertEqual(core.snapshot().affected_joints, ("J1", "J2"))
        self.assertTrue(self.fault(core).persistent_effort)
        self.assertTrue(self.fault(core, "J2").over_temperature)

    def test_two_controller_instances_do_not_share_protection_history(self):
        a, ac = self.make_core()
        b, bc = self.make_core()
        self.overload(a, ac)
        self.observe(b, bc, 0)
        self.assertTrue(a.snapshot().fault_latched)
        self.assertFalse(b.snapshot().fault_latched)

    def test_unknown_id_and_invalid_samples_do_not_refresh_or_reset_history(self):
        core, clock = self.make_core()
        for t in (0, .25, .5):
            self.observe(core, clock, t)
        before = core.snapshot()
        self.assertEqual(self.observe(core, clock, .5, motor_id=14).code, DecisionCode.UNKNOWN_JOINT)
        self.assertEqual(core.receive(0, None, MotorHealth(0, 0, 0, False, 25, 25)).code,
                         DecisionCode.INVALID_OBSERVATION)
        self.assertEqual(core.snapshot(), before)
        for t in (.75, 1):
            self.observe(core, clock, t)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_feedback_limit_caps_the_eligible_sample_gap(self):
        policy = replace(SYNTHETIC_PROTECTION, max_sample_gap_s=10)
        core, clock = self.make_core((spec(policy=policy),))
        for t in (0, 2, 4, 6):
            self.observe(core, clock, t)
        self.assertFalse(core.snapshot().fault_latched)
        self.observe(core, clock, 7)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_initial_stopped_overload_rejects_check_without_granting_owner(self):
        core, clock = self.make_core(arm=False)
        self.assertTrue(core.stop())
        self.assertTrue(core.begin_check())
        self.observe(core, clock, 0, mode=0)
        self.assertEqual(core.checked().code, DecisionCode.UNHEALTHY)
        self.assertIsNone(core.snapshot().ownership)

    def test_stale_recovery_cannot_clear_protection_fault(self):
        core, clock = self.make_core()
        self.overload(core, clock)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        for t in (1.25, 1.5, 1.75):
            self.observe(core, clock, t, effort=0, mode=0)
        clock.now = 3
        self.assertEqual(core.checked().code, DecisionCode.STALE_FEEDBACK)
        self.assertTrue(core.snapshot().fault_latched)
        self.observe(core, clock, 3, effort=0, mode=0)
        self.assertEqual(core.checked().code, DecisionCode.RECOVERY_PENDING)

    def test_communication_gap_does_not_erase_confirmed_overload(self):
        core, clock = self.make_core()
        for t in (0, .25, .5):
            self.observe(core, clock, t)
        self.assertTrue(core.communication_failed(0, CommunicationFailure.TIMEOUT))
        for t in (.5, .75, 1):
            self.observe(core, clock, t)
        f = self.fault(core)
        self.assertTrue(f.persistent_effort)
        self.assertIn(CommunicationFailure.TIMEOUT, f.communication_errors)

    def test_tick_and_snapshot_cannot_invent_measured_overload_duration(self):
        core, clock = self.make_core()
        self.observe(core, clock, 0)
        for t in (.1, .2):
            clock.now = t
            self.assertTrue(core.tick())
            self.assertFalse(core.snapshot().fault_latched)
        self.observe(core, clock, .25)
        self.assertFalse(core.snapshot().fault_latched)

    def test_hot_recovery_requires_new_acknowledgment_after_cooling(self):
        core, clock = self.make_core()
        self.observe(core, clock, 0, effort=0, temperature=80)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        self.observe(core, clock, .25, effort=0, temperature=75, mode=0)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.checked().code, DecisionCode.INVALID_STATE)
        self.assertTrue(self.recover(core, clock, .5))

    def test_complete_recovery_starts_a_new_exposure_budget(self):
        core, clock = self.make_core()
        self.overload(core, clock)
        self.assertTrue(self.recover(core, clock, 1.25))
        self.assertTrue(core.arm(owner="second"))
        for t in (1.75, 2, 2.25, 2.5):
            self.observe(core, clock, t)
            self.assertFalse(core.snapshot().fault_latched)
        self.observe(core, clock, 2.75)
        self.assertTrue(self.fault(core).persistent_effort)

    def test_repeated_stop_does_not_erase_measured_time(self):
        core, clock = self.make_core()
        for i in range(9):
            self.observe(core, clock, i * .25)
            core.stop()
        self.assertTrue(any(f.persistent_effort for f in core.snapshot().faults))

    def test_repeated_check_does_not_erase_measured_time(self):
        core, clock = self.make_core(arm=False)
        for i in range(9):
            core.stop()
            core.begin_check(acknowledge_fault=True)
            self.observe(core, clock, i * .25, mode=0)
        self.assertTrue(any(f.persistent_effort for f in core.snapshot().faults))

    def test_either_temperature_channel_can_latch_with_other_channel_cool(self):
        for driver, winding in ((70, 25), (25, 90), (71, 91)):
            with self.subTest(driver=driver, winding=winding):
                core, clock = self.make_core()
                self.assertTrue(core.receive(0, JointSample(0, 0, 0),
                                             MotorHealth(1, 0, 0, False, driver, winding)))
                self.assertTrue(self.fault(core).over_temperature)

    def test_unknown_and_hot_channels_are_additive_in_both_orders(self):
        for driver, winding in ((None, 95), (75, None)):
            with self.subTest(driver=driver, winding=winding):
                core, clock = self.make_core()
                core.receive(0, JointSample(0, 0, 0),
                             MotorHealth(1, 0, 0, False, driver, winding))
                self.assertTrue(self.fault(core).over_temperature)
                self.assertTrue(self.fault(core).protection_unavailable)

    def test_either_unknown_temperature_channel_blocks_recovery(self):
        for driver, winding in ((None, 25), (25, None)):
            with self.subTest(driver=driver, winding=winding):
                core, clock = self.make_core()
                self.overload(core, clock)
                self.assertTrue(core.begin_check(acknowledge_fault=True))
                core.receive(0, JointSample(0, 0, 0),
                             MotorHealth(0, 0, 0, False, driver, winding))
                self.assertTrue(self.fault(core).protection_unavailable)
                self.assertFalse(core.checked())

    def test_temperature_validation_checks_each_channel(self):
        for field in ("driver_c", "winding_c"):
            for value in (True, False, math.nan, math.inf, -math.inf, 10**400, "25"):
                with self.subTest(field=field, value=repr(value)), self.assertRaises(ValueError):
                    replace(MotorHealth(0, 0, 0, False, 25, 25), **{field: value})

    def test_clock_regression_does_not_commit_partial_protection_state(self):
        core, clock = self.make_core()
        self.observe(core, clock, .25)
        before = core.snapshot()
        clock.now = 0
        with self.assertRaises(ValueError):
            core.receive(0, JointSample(0, 0, 6), MotorHealth(1, 0, 0, False, 25, 25))
        clock.now = .25
        self.assertEqual(core.snapshot(), before)


if __name__ == "__main__":
    unittest.main()
