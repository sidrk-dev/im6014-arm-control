"""Parent reproduces dwell authority rounding using exact accepted clock floats."""
from pathlib import Path
from fractions import Fraction
import sys,unittest
sys.path.insert(0,str(Path(sys.argv.pop(1)).resolve()))
from joint_contract import JointSpec,MotorHealth,SourceIdentity
from joint_controller import JointCore
from joint_mapping import JointMap,JointSample
from joint_protection import ProtectionPolicy

class ExactDwellChecks(unittest.TestCase):
    def fixture(self):
        clock=[0.0];dwell=.30000000000000004
        self.assertLess(Fraction(.4)-Fraction(.1),Fraction(dwell))
        p=ProtectionPolicy(80,60,100,70,1,dwell,dwell)
        spec=JointSpec('J',0,JointMap(1,1,0),-2,2,5,50,1,4,'fixture: exact parent C14 dwell',p)
        core=JointCore((spec,),clock=lambda:clock[0],max_reference_gap_s=.5,feedback_max_age_s=.5)
        core.begin_connect();core.connected(SourceIdentity('fixture','exact-C14',1));core.claim_owner('exact-C14');core.begin_check()
        core.receive(0,JointSample(0,0,0),MotorHealth(0,0,0,False,20,30));self.assertTrue(core.checked().accepted)
        return core,clock
    def test_overload_before_exact_dwell_does_not_trip(self):
        core,clock=self.fixture()
        for t in (.1,.4):
            clock[0]=t;core.receive(0,JointSample(0,0,2),MotorHealth(0,0,0,False,20,30))
        self.assertFalse(core.snapshot().fault_latched,'Rounded elapsed time tripped before exact configured dwell')
    def test_recovery_before_exact_dwell_cannot_clear(self):
        core,clock=self.fixture()
        core.receive(0,JointSample(0,0,0),MotorHealth(0,0,0,False,80,30));self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        for t in (.1,.4):
            clock[0]=t;core.receive(0,JointSample(0,0,0),MotorHealth(0,0,0,False,20,30))
        self.assertFalse(core.checked().accepted,'Rounded elapsed time cleared fault before exact configured dwell')
        self.assertTrue(core.snapshot().fault_latched)
if __name__=='__main__':unittest.main(verbosity=2)
