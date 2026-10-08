import sys,unittest,math
sys.path.insert(0,sys.argv.pop(1))
from joint_contract import JointSpec,MotorHealth
from joint_mapping import JointMap,JointTarget,JointSample
from joint_reference import prepare_reference_joint
from im6014_codec import decode_command
from joint_controller import JointCore
from test_controller import FakeStation,DeterministicClock,HEALTHY_STOP
class CanonicalAndFaults(unittest.TestCase):
 def test_actual_reference_matches_all_wire_fields(self):
  m=JointMap(3,-1,.25);s=JointSpec('J1',0,m,-2,2,'fixture: parent software limits')
  code,t,p=prepare_reference_joint(s,JointTarget(.321,.123,.321,1.234,.123))
  self.assertIsNone(code);d=decode_command(p)
  self.assertEqual(t.dq_rad_s,m.direction*d['rpm']*math.tau/60/m.reduction)
  self.assertEqual(t.tau_nm,m.direction*d['torque_nm']*m.reduction)
  self.assertEqual(t.kp,d['kp']*m.reduction**2)
  self.assertEqual(t.kd,d['kd']*m.reduction**2)
 def test_combined_valid_feedback_causes_union(self):
  c=JointCore((JointSpec('J1',0,JointMap(1,1,0),-1,1,'fixture: parent software limits'),),clock=DeterministicClock(0));s=FakeStation(c)
  self.assertTrue(s.connect());s.claim('real');self.assertTrue(s.check())
  c.receive(0,JointSample(2,0,0),MotorHealth(errors=1,warnings=0,timeout=False,mode=0))
  f=c.snapshot().faults[0];self.assertTrue(f.errors);self.assertTrue(f.position_limit)
 def test_valid_outside_observation_during_check_latches_position(self):
  c=JointCore((JointSpec('J1',0,JointMap(1,1,0),-1,1,'fixture: parent software limits'),),clock=DeterministicClock(0));s=FakeStation(c)
  self.assertTrue(s.connect());s.claim('real');self.assertTrue(c.begin_check())
  c.receive(0,JointSample(2,0,0),HEALTHY_STOP)
  self.assertTrue(any(f.position_limit for f in c.snapshot().faults),'fresh outside observation needs attributed position cause')
if __name__=='__main__':unittest.main()
