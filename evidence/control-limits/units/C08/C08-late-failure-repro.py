import sys,unittest
sys.path.insert(0,sys.argv.pop(1))
from joint_contract import JointSpec,State,Target,WriteResult
from joint_mapping import JointMap,JointTarget
from joint_controller import JointCore
from test_controller import FakeStation,DeterministicClock
class LateFailureLifecycle(unittest.TestCase):
 def test_late_failure_retains_disconnected_and_fault_attribution(self):
  for result in (WriteResult.FAILED,WriteResult.PARTIAL,WriteResult.PREWRITE_DEADLINE):
   with self.subTest(result=result):
    c=JointCore((JointSpec('J1',0,JointMap(1,1,0)),),clock=DeterministicClock(0));s=FakeStation(c);self.assertTrue(s.connect());owner=s.claim('real');self.assertTrue(s.check());self.assertTrue(s.arm(owner))
    self.assertTrue(s.submit(Target(1,(JointTarget(.1,0,0,1,.1),),s.issue_window(1),owner)))
    p=c.prepare_refresh();self.assertTrue(c.claim_dispatch(p.plan_id,0));self.assertTrue(c.disconnect())
    self.assertFalse(c.begin_connect(),'Outstanding outcome must block reconnect')
    self.assertFalse(c.dispatch_result(p.plan_id,0,result))
    snap=c.snapshot();self.assertEqual(snap.state,State.DISCONNECTED);self.assertTrue(snap.fault_latched);self.assertEqual(snap.dispatch_attempts[0].result,result);self.assertIsNone(snap.current_owner)
    self.assertTrue(c.begin_connect(),'Settled receipt must allow explicit reconnect with latched fault retained')
if __name__=='__main__':unittest.main()
