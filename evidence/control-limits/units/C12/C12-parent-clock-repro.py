"""Parent write-gate clock failure and completed-write history checks; fake I/O."""
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(sys.argv[1]).resolve()))
import test_controller_limits as fixtures
from joint_contract import CommunicationFailure, State, Target, WriteResult
from joint_mapping import JointTarget


class WriteGateClock(unittest.TestCase):
    def _prepared(self):
        fixture = fixtures.ReferenceAccelerationTests()
        fixture.setUp()
        core, station, owner = fixture._armed_station()
        goal = Target(1, (JointTarget(.1, 0, 0, 1, .1), JointTarget(0, 0, 0, 1, .1)),
                      window=station.issue_window(1), owner=owner)
        self.assertTrue(station.submit(goal))
        fixture.clock.value += .05
        plan = core.prepare_refresh()
        self.assertIsNotNone(plan)
        return fixture, core, plan

    def test_invalid_gate_clock_permanently_withdraws_prepared_authority(self):
        for value in (math.nan, 0.0):
            with self.subTest(value=value):
                fixture, core, plan = self._prepared()
                good = fixture.clock.value
                fixture.clock.value = value
                try:
                    accepted = core.claim_dispatch(plan.plan_id, 0)
                except ValueError:
                    accepted = False
                self.assertFalse(accepted)
                fixture.clock.value = good
                snapshot = core.snapshot()
                self.assertIsNone(snapshot.prepared_plan, 'Bad gate clock left a reusable mode-1 plan')
                self.assertIsNone(snapshot.admitted_target)
                self.assertIsNone(snapshot.current_owner)
                self.assertIsNone(core._dispatch_claim)
                self.assertFalse(core.claim_dispatch(plan.plan_id, 0))

    def test_throwing_end_clock_keeps_complete_write_and_latches_cause(self):
        fixture, core, plan = self._prepared()
        self.assertTrue(core.claim_dispatch(plan.plan_id, 0))
        def throwing_clock():
            raise RuntimeError('fixture clock callback failed')
        original = core._clock
        core._clock = throwing_clock
        result = core.dispatch_result(plan.plan_id, 0, WriteResult.COMPLETE)
        self.assertFalse(result)
        core._clock = original
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.FAULT)
        self.assertEqual(snapshot.faults[0].communication_error, CommunicationFailure.CLOCK_FAILURE)
        self.assertEqual(snapshot.dispatched_records[0].sequence, 1)
        self.assertEqual(snapshot.dispatch_attempts[0].result, WriteResult.COMPLETE)
        self.assertIsNone(snapshot.prepared_plan)


unittest.main(argv=[sys.argv[0]], verbosity=2)
