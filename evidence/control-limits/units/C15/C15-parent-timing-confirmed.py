"""Parent public-behavior checks for timing and real fake-driver boundaries."""
from pathlib import Path
import math
import sys
import time
import unittest

sys.path.insert(0, str(Path(sys.argv.pop(1)).resolve()))
from joint_contract import CommunicationFailure, MotorHealth, SourceIdentity, Target, WriteResult
from joint_controller import JointCore
from joint_mapping import JointSample, JointTarget
from station_step import StationStepCoordinator
from station_timing import JointTransactionRecord, StationTimingTracker
from test_station_timing import DeterministicClock, FakeTransport
import test_station_timing as fixtures


def transaction(start, end, index=0):
    return JointTransactionRecord(0, index, 'J', 0, None, None, None,
                                  WriteResult.COMPLETE, 'OK', start, end,
                                  end-start, True)


class TimingBoundaryChecks(unittest.TestCase):
    def test_cycle_end_cannot_rewind_actual_transaction_time(self):
        tracker = StationTimingTracker(.01)
        tracker.start_cycle(1.0)
        tx = transaction(1.0, 2.0)
        tracker.record_transaction(tx)
        with self.assertRaises(ValueError):
            tracker.end_cycle(1.5, transactions=(tx,))

    def test_new_transaction_cannot_precede_prior_actual_end(self):
        tracker = StationTimingTracker(.01)
        tracker.start_cycle(1.0)
        tracker.record_transaction(transaction(1.0, 2.0))
        with self.assertRaises(ValueError):
            tracker.record_transaction(transaction(1.5, 1.75, 1))

    def test_start_cannot_replace_an_unfinished_cycle(self):
        tracker = StationTimingTracker(.01)
        tracker.start_cycle(1.0)
        with self.assertRaises(RuntimeError):
            tracker.start_cycle(1.1)

    def test_finite_slot_inputs_have_finite_result_or_explicit_rejection(self):
        tracker = StationTimingTracker(math.nextafter(0.0, 1.0))
        try:
            missed, slot = tracker.calculate_slot_skip(1.0, 0.0)
        except ValueError:
            return
        self.assertTrue(math.isfinite(slot))
        self.assertGreater(slot, 1.0)
        self.assertGreater(missed, 0)


class CoordinatorFailureChecks(unittest.TestCase):
    def fixture(self, armed=True):
        values = fixtures.StationTimingTests(); values.setUp()
        clock = DeterministicClock(10.0)
        core = JointCore(values.specs, clock=clock, max_reference_gap_s=1.0)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SourceIdentity('fixture', 'C15-parent', 1)).accepted)
        owner = core.claim_owner('C15-parent')
        self.assertTrue(core.begin_check().accepted)
        for spec in values.specs:
            self.assertTrue(core.receive(spec.motor_id, JointSample(0, 0, 0),
                                        MotorHealth(0, 0, 0, False, 25, 30)).accepted)
        self.assertTrue(core.checked().accepted)
        bus = FakeTransport()
        coordinator = StationStepCoordinator(core, bus, StationTimingTracker(.01),
                                             clock=clock, deadline_s=.002)
        if armed:
            self.assertTrue(core.arm(owner).accepted)
            goal = Target(1, tuple(JointTarget(0, 0, 0, 0, 0) for _ in values.specs),
                          core.issue_window(.5), owner)
            self.assertTrue(core.submit_target(goal).accepted)
            clock.advance(.01)
        return core, coordinator

    def test_actual_read_timeout_invalidates_joint_and_latches_cause(self):
        core, coordinator = self.fixture()
        record = coordinator.step()
        self.assertEqual(record.transactions[0].outcome, 'TIMEOUT')
        self.assertIs(record.transactions[0].write_result, WriteResult.COMPLETE)
        snapshot = core.snapshot()
        self.assertFalse(snapshot.joints[0].valid)
        self.assertTrue(snapshot.fault_latched)
        self.assertIn(CommunicationFailure.TIMEOUT, snapshot.faults[0].communication_errors)
        self.assertEqual(snapshot.dispatched_records[0].sequence, 1)

    def test_stop_query_timeout_invalidates_prior_healthy_observation(self):
        core, coordinator = self.fixture(armed=False)
        records = coordinator.check_step()
        self.assertEqual(records[0].outcome, 'TIMEOUT')
        self.assertFalse(core.snapshot().joints[0].valid)
        self.assertTrue(core.snapshot().fault_latched)

    def test_default_real_clock_runner_actually_paces(self):
        values = fixtures.StationTimingTests(); values.setUp()
        core = JointCore(values.specs, clock=time.monotonic, max_reference_gap_s=1.0)
        coordinator = StationStepCoordinator(core, FakeTransport(), StationTimingTracker(.01),
                                             clock=time.monotonic)
        records = coordinator.run_steps(3)
        self.assertEqual(len(records), 3)
        for record in records[1:]:
            self.assertGreaterEqual(record.actual_period_s, .005,
                                    'Default runner burst idle cycles without nominal pacing')


if __name__ == '__main__':
    unittest.main(verbosity=2)
