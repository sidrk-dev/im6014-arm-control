import sys,unittest
from pathlib import Path
sys.path.insert(0,sys.argv.pop(1))
from joint_contract import JointSpec,State,Target,WriteResult
from joint_controller import JointCore
from joint_mapping import JointMap,JointTarget
from test_controller import FakeStation,DeterministicClock
class ParentOwnerBoundary(unittest.TestCase):
 def test_overflow_revokes_active_last_generation_motion(self):
  c=JointCore((JointSpec('J1',0,JointMap(1,1,0)),),clock=DeterministicClock(0))
  s=FakeStation(c);self.assertTrue(s.connect());c._next_generation=(1<<64)-1
  o=s.claim('a');self.assertTrue(s.check());self.assertTrue(s.arm(o))
  self.assertTrue(s.submit(Target(1,(JointTarget(.1,0,0,1,.1),),s.issue_window(1),o)))
  p=c.prepare_refresh();self.assertTrue(c.claim_dispatch(p.plan_id,0));self.assertTrue(c.dispatch_result(p.plan_id,0,WriteResult.COMPLETE))
  p=c.prepare_refresh();self.assertEqual(c.snapshot().state,State.MOVING)
  with self.assertRaises(OverflowError):c.claim_owner('b')
  self.assertFalse(c.claim_dispatch(p.plan_id,0),'Exhausted takeover must cancel old moving authority')
  self.assertIsNone(c.snapshot().current_owner)
  self.assertEqual(c.snapshot().state,State.STOPPED)
if __name__=='__main__':unittest.main()
