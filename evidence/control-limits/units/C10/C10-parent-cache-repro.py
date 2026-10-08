from pathlib import Path
import sys, unittest
from fractions import Fraction
sys.path.insert(0, str(Path(sys.argv.pop(1)).resolve()))
from joint_contract import JointSpec, JointTrajectoryReference, Target
from joint_mapping import JointMap, JointTarget, JointSample
from joint_reference import shape_reference_joint
from joint_controller import JointCore
from test_controller import DeterministicClock, FakeStation, HEALTHY_STOP

class ParentSpeedReproductions(unittest.TestCase):
    def spec(self, name='J1', motor_id=0, V=1.0):
        return JointSpec(name, motor_id, JointMap(1,1,0), -1,1,V,'parent: software fixture only')
    def first(self):
        ref=JointTrajectoryReference(0,0,0,0,False)
        code,target,next_ref,packet=shape_reference_joint(self.spec(),JointTarget(.1,0,0,1,.1),ref,.1,1)
        self.assertIsNone(code)
        return target,next_ref
    def test_zero_time_refresh_preserves_derivative_history(self):
        target,ref=self.first()
        code,_,new_ref,_=shape_reference_joint(self.spec(),JointTarget(.1,0,0,1,.1),ref,ref.sample_time_s,1)
        self.assertIsNone(code)
        self.assertEqual(new_ref,ref)
    def test_confirmed_step_retains_exact_rational_value(self):
        _,ref=self.first()
        self.assertEqual(ref.position_step_velocity_rad_s,Fraction(ref.q_rad)/Fraction(.1))
    def test_zero_objective_is_not_quantization_stall(self):
        ref=JointTrajectoryReference(0,0,0,0,False)
        code,target,_,_=shape_reference_joint(self.spec(),JointTarget(.1,-1,0,1,.1),ref,.1,1)
        self.assertIsNone(code)
        self.assertEqual(target.q_rad,0)
    def test_quantization_fault_attributes_only_failing_joint(self):
        clock=DeterministicClock(0)
        core=JointCore((self.spec(V=.001),self.spec('J2',1,V=.001)),clock=clock,max_reference_gap_s=1)
        station=FakeStation(core)
        self.assertTrue(station.connect())
        owner=station.claim('parent-fixture')
        self.assertTrue(core.begin_check())
        for motor_id in (0,1):self.assertTrue(core.receive(motor_id,JointSample(0,0,0),HEALTHY_STOP))
        self.assertTrue(core.checked())
        self.assertTrue(station.arm(owner))
        window=station.issue_window(1)
        self.assertTrue(station.submit(Target(1,(JointTarget(.1,0,0,1,.1),JointTarget(0,0,0,1,.1)),window=window,owner=owner)))
        clock.value=1e-10
        self.assertIsNone(core.prepare_refresh())
        self.assertEqual(tuple(f.name for f in core.snapshot().faults),('J1',))

if __name__=='__main__':unittest.main()
