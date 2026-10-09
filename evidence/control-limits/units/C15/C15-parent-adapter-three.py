from pathlib import Path
import sys, unittest
sys.path.insert(0,str(Path(sys.argv.pop(1)).resolve()))
import test_station_timing as fixtures
from joint_contract import CommunicationFailure, MotorHealth, SourceIdentity, Target, WriteResult
from joint_controller import JointCore
from joint_mapping import JointSample, JointTarget
from station_step import StationStepCoordinator
from station_timing import StationTimingTracker

class AdapterReviewChecks(unittest.TestCase):
    def fixture(self,bad_write_clock=False):
        f=fixtures.StationTimingTests(); f.setUp()
        class Clock(fixtures.DeterministicClock):
            fail_once=False
            def __call__(self):
                if self.fail_once:
                    self.fail_once=False
                    return float('nan')
                return super().__call__()
        clock=Clock(10)
        core=JointCore(f.specs,clock=clock,max_reference_gap_s=1)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SourceIdentity('fixture','C15-adapter',1)).accepted)
        owner=core.claim_owner('C15-adapter')
        self.assertTrue(core.begin_check().accepted)
        for s in f.specs:
            self.assertTrue(core.receive(s.motor_id,JointSample(0,0,0),MotorHealth(0,0,0,False,25,30)).accepted)
        self.assertTrue(core.checked().accepted)
        self.assertTrue(core.arm(owner).accepted)
        self.assertTrue(core.submit_target(Target(1,tuple(JointTarget(0,0,0,0,0) for _ in f.specs),core.issue_window(.5),owner)).accepted)
        clock.advance(.01)
        class Bus(fixtures.FakeTransport):
            def write(self,data):
                result=super().write(data)
                if bad_write_clock: clock.fail_once=True
                return result
        bus=Bus([fixtures.make_feedback_frame(s.motor_id,mode=1) for s in f.specs])
        coordinator=StationStepCoordinator(core,bus,StationTimingTracker(.01),clock=lambda:20.,deadline_s=.002)
        return core,coordinator

    def test_age_uses_core_receive_clock_not_tracker_offset(self):
        core,coordinator=self.fixture()
        record=coordinator.step()
        for tx in record.transactions:
            self.assertEqual(tx.outcome,'OK')
            self.assertEqual(tx.feedback_age_s,0.,'core receive clock is unchanged; offset tracker clock is not age authority')

    def test_completed_write_clock_failure_keeps_actual_receipt_and_class(self):
        core,coordinator=self.fixture(True)
        record=coordinator.step()
        tx=record.transactions[0]
        self.assertIs(tx.write_result,WriteResult.COMPLETE)
        self.assertEqual(tx.outcome,'CLOCK_FAILURE')
        self.assertFalse(tx.feedback_received)
        self.assertIn(CommunicationFailure.CLOCK_FAILURE,core.snapshot().faults[0].communication_errors)
        self.assertEqual(core.snapshot().dispatched_records[0].sequence,1)

    def test_current_cycle_skew_excludes_old_skipped_or_failed_receipts(self):
        core,coordinator=self.fixture()
        coordinator.bus.responses.clear()
        coordinator.bus.responses.append(fixtures.make_feedback_frame(0,mode=1))
        record=coordinator.step()
        self.assertEqual([tx.outcome for tx in record.transactions],['OK','TIMEOUT'])
        self.assertEqual(sum(tx.feedback_received for tx in record.transactions),1)
        self.assertIsNone(record.sampling_skew_s,'one current receive sample has no within-cycle sampling spread')

if __name__=='__main__':unittest.main(verbosity=2)
