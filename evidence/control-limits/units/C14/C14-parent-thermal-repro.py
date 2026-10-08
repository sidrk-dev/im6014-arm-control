"""Independent parent checks using binary exact fixture times and public core calls."""
from pathlib import Path
import sys, unittest
sys.path.insert(0,str(Path(sys.argv.pop(1)).resolve()))
from joint_contract import CommunicationFailure, JointSpec, MotorHealth, SourceIdentity
from joint_controller import JointCore
from joint_mapping import JointMap, JointSample
from joint_protection import ProtectionPolicy

class ThermalHistoryChecks(unittest.TestCase):
    def fixture(self):
        clock=[0.0]
        policy=ProtectionPolicy(80,60,100,70,1,.25,.25)
        spec=JointSpec('J',0,JointMap(1,1,0),-2,2,5,50,1,4,'fixture: parent C14 history',policy)
        core=JointCore((spec,),clock=lambda:clock[0],max_reference_gap_s=.5,feedback_max_age_s=.5)
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(core.connected(SourceIdentity('fixture','parent-C14',1)).accepted)
        core.claim_owner('parent-C14')
        self.assertTrue(core.begin_check().accepted)
        self.assertTrue(core.receive(0,JointSample(0,0,0),MotorHealth(0,0,0,False,20,30)).accepted)
        self.assertTrue(core.checked().accepted)
        return core,clock
    def sample(self,core,clock,t,effort=2):
        clock[0]=t
        return core.receive(0,JointSample(0,0,effort),MotorHealth(0,0,0,False,20,30))
    def test_known_communication_gap_cannot_credit_overload_time(self):
        core,clock=self.fixture()
        self.sample(core,clock,0)
        self.sample(core,clock,.0625)
        clock[0]=.125
        self.assertTrue(core.communication_failed(0,CommunicationFailure.TIMEOUT).accepted)
        self.sample(core,clock,.1875)
        self.sample(core,clock,.25)
        fault=core.snapshot().faults[0]
        self.assertFalse(fault.continuous_effort_limit,'Known invalid interval was credited as overload time')
        self.sample(core,clock,.3125)
        self.sample(core,clock,.375)
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit,'Prior confirmed unsafe time must survive the gap')
    def test_stop_check_cannot_erase_confirmed_overload_history(self):
        core,clock=self.fixture()
        self.sample(core,clock,0)
        self.sample(core,clock,.0625)
        self.assertTrue(core.stop().accepted)
        self.assertTrue(core.begin_check().accepted)
        self.sample(core,clock,.125)
        self.assertTrue(core.checked().accepted)
        self.sample(core,clock,.1875)
        self.sample(core,clock,.25)
        self.assertFalse(core.snapshot().fault_latched)
        self.sample(core,clock,.3125)
        self.assertTrue(core.snapshot().fault_latched,'Successful nonfault Check discarded still-overloaded history')
        self.assertTrue(core.snapshot().faults[0].continuous_effort_limit)
    def test_unobserved_long_gap_does_not_count_but_preserves_prior_time(self):
        core,clock=self.fixture()
        self.sample(core,clock,0)
        self.sample(core,clock,.0625)
        self.sample(core,clock,1)
        self.assertFalse(core.snapshot().fault_latched)
        self.sample(core,clock,1.0625)
        self.sample(core,clock,1.125)
        self.assertFalse(core.snapshot().fault_latched)
        self.sample(core,clock,1.1875)
        self.assertTrue(core.snapshot().fault_latched)
    def test_new_ack_recovery_needs_full_binary_exact_interval(self):
        core,clock=self.fixture()
        self.assertFalse(core.receive(0,JointSample(0,0,0),MotorHealth(0,0,0,False,80,30)).accepted)
        self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
        self.sample(core,clock,0,0)
        self.sample(core,clock,.125,0)
        self.assertFalse(core.checked().accepted)
        self.sample(core,clock,.25,0)
        self.assertTrue(core.checked().accepted)
        self.assertFalse(core.snapshot().fault_latched)

if __name__=='__main__':unittest.main(verbosity=2)
