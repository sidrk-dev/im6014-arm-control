"""Parent checks protection history at actual core dispatch-failure boundaries."""
from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(sys.argv.pop(1)).resolve()))
from joint_contract import JointSpec,MotorHealth,SourceIdentity,Target,WriteResult
from joint_controller import JointCore
from joint_mapping import JointMap,JointSample,JointTarget
from joint_protection import ProtectionPolicy
class DispatchGapChecks(unittest.TestCase):
    def test_failed_write_cannot_credit_unknown_overload_interval(self):
        clock=[0.0];spec=JointSpec('J',0,JointMap(1,1,0),-2,2,5,50,1,4,'fixture: dispatch gap',ProtectionPolicy(80,60,100,70,1,.25,.25))
        core=JointCore((spec,),clock=lambda:clock[0],max_reference_gap_s=.5,feedback_max_age_s=.5)
        core.begin_connect();core.connected(SourceIdentity('fixture','write-gap-C14',1));owner=core.claim_owner('writer');core.begin_check()
        core.receive(0,JointSample(0,0,0),MotorHealth(0,0,0,False,20,30));self.assertTrue(core.checked().accepted);self.assertTrue(core.arm(owner).accepted)
        self.assertTrue(core.submit_target(Target(0,(JointTarget(0,0,0,0,0),),window=core.issue_window(.5),owner=owner)).accepted)
        def sample(t):
            clock[0]=t;core.receive(0,JointSample(0,0,2),MotorHealth(0,0,0,False,20,30))
        sample(0);sample(.0625);clock[0]=.125
        plan=core.prepare_refresh();self.assertIsNotNone(plan)
        self.assertTrue(core.claim_dispatch(plan.plan_id,0))
        self.assertFalse(core.dispatch_result(plan.plan_id,0,WriteResult.FAILED).accepted)
        sample(.1875);sample(.25)
        self.assertFalse(core.snapshot().faults[0].continuous_effort_limit,'Failed driver transaction interval credited')
        sample(.3125);sample(.375)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)
if __name__=='__main__':unittest.main(verbosity=2)
