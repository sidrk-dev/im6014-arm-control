"""Parent verifies authority/check transitions cannot bridge unknown telemetry."""
from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(sys.argv.pop(1)).resolve()))
from joint_contract import JointSpec,MotorHealth,SourceIdentity
from joint_controller import JointCore
from joint_mapping import JointMap,JointSample
from joint_protection import ProtectionPolicy
class AuthorityGapChecks(unittest.TestCase):
    def run_gap(self,kind):
        clock=[0.0];spec=JointSpec('J',0,JointMap(1,1,0),-2,2,5,50,1,4,'fixture: authority gap',ProtectionPolicy(80,60,100,70,1,.25,.25))
        core=JointCore((spec,),clock=lambda:clock[0],max_reference_gap_s=.5,feedback_max_age_s=.5)
        core.begin_connect();core.connected(SourceIdentity('fixture','gap-C14',1))
        if kind=='takeover':
            core.claim_owner('first');core.begin_check();core.receive(0,JointSample(0,0,0),MotorHealth(0,0,0,False,20,30));self.assertTrue(core.checked().accepted)
        def sample(t):
            clock[0]=t;core.receive(0,JointSample(0,0,2),MotorHealth(0,0,0,False,20,30))
        sample(0);sample(.0625);clock[0]=.125
        if kind=='takeover':core.claim_owner('second')
        else:self.assertTrue(core.begin_check().accepted)
        sample(.1875);sample(.25)
        self.assertFalse(core.snapshot().fault_latched,kind+' bridged its explicit observation/authority reset')
        sample(.3125);sample(.375)
        self.assertTrue(core.snapshot().fault_latched,kind+' discarded established unsafe history')
    def test_takeover_preserves_time_without_gap_credit(self):self.run_gap('takeover')
    def test_initial_check_preserves_time_without_gap_credit(self):self.run_gap('check')
if __name__=='__main__':unittest.main(verbosity=2)
