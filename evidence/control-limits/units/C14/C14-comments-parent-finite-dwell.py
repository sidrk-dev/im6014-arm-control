"""Parent checks finite, bounded dwell records with finite accepted clock values."""
from pathlib import Path
import math,sys,unittest
sys.path.insert(0,str(Path(sys.argv.pop(1)).resolve()))
from joint_contract import JointSpec,MotorHealth
from joint_mapping import JointMap,JointSample
from joint_protection import JointProtectionState,ProtectionPolicy,transition_protection

class FiniteDwellChecks(unittest.TestCase):
    def spec(self):
        return JointSpec('J',0,JointMap(1,1,0),-2,2,5,50,1,4,'fixture: finite parent C14 dwell',ProtectionPolicy(80,60,100,70,1,.25,.25))
    def test_overload_duration_stays_finite_after_fault(self):
        state=JointProtectionState()
        for timestamp in (-1e308,0.0,1e308):
            _,state,_=transition_protection(self.spec(),JointSample(0,0,2),MotorHealth(0,0,0,False,20,30),state,timestamp,1e308,False)
        self.assertTrue(math.isfinite(state.overload_duration_s),'Finite validated clock endpoints overflowed overload history')
    def test_recovery_duration_stays_finite_before_final_check(self):
        state=JointProtectionState()
        for timestamp in (-1e308,0.0,1e308):
            _,state,_=transition_protection(self.spec(),JointSample(0,0,0),MotorHealth(0,0,0,False,20,30),state,timestamp,1e308,True)
        self.assertTrue(math.isfinite(state.recovery_duration_s),'Finite validated clock endpoints overflowed recovery history')
if __name__=='__main__':unittest.main(verbosity=2)
