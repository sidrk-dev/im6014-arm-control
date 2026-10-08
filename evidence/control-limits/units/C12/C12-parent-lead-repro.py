"""Parent exact lead-bound checks over finite float domain values; no I/O."""
from pathlib import Path
from fractions import Fraction
import math
import sys
import unittest

sys.path.insert(0, str(Path(sys.argv[1]).resolve()))
from im6014_codec import raw_to_position
from joint_contract import DecisionCode, JointSpec, JointTrajectoryReference, Target
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample, JointTarget
from joint_reference import shape_reference_joint
from test_controller import DeterministicClock, FakeStation, HEALTHY_STOP


class ExactLeadBoundary(unittest.TestCase):
    def test_rounded_admission_boundary_cannot_accept_outside_requested_goal(self):
        clock = DeterministicClock(0.0)
        spec = JointSpec('J', 0, JointMap(1, 1, 0), -1, 1, 1, 10, .2,
                         'parent exact lead software fixture')
        core = JointCore((spec,), clock=clock)
        station = FakeStation(core)
        self.assertTrue(station.connect())
        owner = station.claim('parent')
        self.assertTrue(core.begin_check())
        self.assertTrue(core.receive(0, JointSample(.1, 0, 0), HEALTHY_STOP))
        self.assertTrue(core.checked())
        self.assertTrue(station.arm(owner))
        window = station.issue_window(1)
        valid = Target(1, (JointTarget(.1, 0, 0, 1, .1),), window=window, owner=owner)
        self.assertTrue(station.submit(valid))
        before = core.snapshot()
        q = .1 + .2
        self.assertGreater(Fraction(q) - Fraction(.1), Fraction(.2))
        outside = Target(2, (JointTarget(q, 0, 0, 1, .1),), window=window, owner=owner)
        decision = station.submit(outside)
        self.assertFalse(decision, 'Rounded lead boundary admitted an exactly outside goal')
        self.assertEqual(decision.code, DecisionCode.LEAD_OUT_OF_BOUNDS)
        after = core.snapshot()
        self.assertEqual(after.admitted_target, before.admitted_target)
        self.assertEqual(after.last_sequence, before.last_sequence)

    def test_zero_cadence_replay_cannot_use_rounded_lead_interval(self):
        mapping = JointMap(math.ldexp(raw_to_position(1), 2), 1, 0)
        spec = JointSpec('J', 0, mapping, -1, 1, 1, 4, .15,
                         'parent exact cached lead software fixture')
        goal = JointTarget(.25, 0, 0, 0, 0)
        anchor = JointTrajectoryReference(.25, 0, 0, 0, False)
        code, actual, emitted, packet = shape_reference_joint(spec, goal, anchor, .25, .25, .5)
        self.assertIsNone(code)
        self.assertEqual(actual.q_rad, .25)
        self.assertGreater(Fraction(actual.q_rad) - Fraction(.1), Fraction(spec.max_lead_rad))
        self.assertEqual(.1 + spec.max_lead_rad, actual.q_rad)
        repeated = shape_reference_joint(spec, goal, emitted, .1, .25, .5)
        self.assertEqual(repeated[0], DecisionCode.CONSTRAINT_VIOLATION,
                         'Zero-cadence cached packet exceeded the exact fresh lead bound')
        self.assertIsNone(repeated[3])


unittest.main(argv=[sys.argv[0]], verbosity=2)
