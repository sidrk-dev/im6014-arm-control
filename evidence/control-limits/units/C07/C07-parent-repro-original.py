import math,unittest
from unittest.mock import patch
from joint_controller import JointCore
from joint_contract import JointSpec,Target,WriteResult,State
from joint_mapping import JointMap,JointTarget
from test_controller import DeterministicClock,FakeStation
from im6014_serial import exchange,TransactionTimeoutError
class Reproductions(unittest.TestCase):
 def armed(self):
  clock=DeterministicClock(0.0)
  core=JointCore((JointSpec('J1',0,JointMap(1,1,0)),JointSpec('J2',1,JointMap(1,1,0))),clock=clock,feedback_max_age_s=2)
  station=FakeStation(core)
  assert station.connect() and station.check() and station.arm()
  goal=Target(1,(JointTarget(0.1,0,0,1,.1),JointTarget(.2,0,0,1,.1)),core.issue_window(.2))
  assert core.submit_target(goal)
  return core,station,clock,goal
 def test_fresh_target_cannot_revive_expired_moving_goal(self):
  core,station,clock,goal=self.armed();assert station.refresh()
  clock.value=.2
  newer=Target(2,goal.joints,core.issue_window(.2))
  decision=core.submit_target(newer)
  self.assertFalse(decision.accepted,'expired motion revived without Stop/check/arm')
  self.assertEqual(core.snapshot().state,State.STOPPED)
  self.assertEqual(core.snapshot().last_sequence,1)
 def test_write_at_deadline_still_reports_actual_completion(self):
  transaction_clock=DeterministicClock(0)
  results=[]
  class Bus:
   timeout=.1
   def reset_input_buffer(self):pass
   def write(self,packet):transaction_clock.value=.1;return len(packet)
  with patch('im6014_serial._monotonic',transaction_clock),self.assertRaises(TransactionTimeoutError):
   exchange(Bus(),b'x'*20,0,deadline_s=.05,on_write=results.append)
  self.assertEqual(results,[WriteResult.COMPLETE])
 def test_complete_write_survives_invalid_core_end_clock(self):
  core,station,clock,goal=self.armed();plan=core.prepare_refresh();writes=[]
  class Bus:
   timeout=.1
   def reset_input_buffer(self):pass
   def write(self,packet):writes.append(packet);clock.value=math.nan;return len(packet)
  try:
   exchange(Bus(),b'x'*20,0,before_write=lambda:core.claim_dispatch(plan.plan_id,0),on_write=lambda res:core.dispatch_result(plan.plan_id,0,res))
  except (RuntimeError,ValueError):pass
  self.assertEqual(len(writes),1)
  clock.value=0
  self.assertIsNotNone(core.snapshot().dispatched_records[0],'actual write evidence was lost')
  self.assertNotEqual(core.snapshot().state,State.ARMED)
 def test_one_inflight_write_blocks_next_claim_and_reprepare(self):
  core,station,clock,goal=self.armed();plan=core.prepare_refresh()
  assert core.claim_dispatch(plan.plan_id,0)
  self.assertFalse(core.claim_dispatch(plan.plan_id,1),'next claim allowed with prior outcome unresolved')
  self.assertIsNone(core.prepare_refresh(),'inflight plan was overwritten')
if __name__=='__main__':unittest.main(verbosity=2)
