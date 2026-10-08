"""Parent exact combined-effort boundary tests; canonical bytes and fake authority only."""
from pathlib import Path
from fractions import Fraction
from dataclasses import replace
import sys,unittest,json
sys.path.insert(0,str(Path(sys.argv[1]).resolve()))
from joint_contract import JointSpec,Target,DecisionCode
from joint_mapping import JointMap,JointSample,JointTarget
from joint_controller import JointCore
from joint_reference import prepare_reference_joint,predict_joint_effort
from test_controller import FakeStation,DeterministicClock,HEALTHY_STOP

def exact(t,s):
 return Fraction(t.tau_nm)+Fraction(t.kp)*(Fraction(t.q_rad)-Fraction(s.q_rad))+Fraction(t.kd)*(Fraction(t.dq_rad_s)-Fraction(s.dq_rad_s))

class EffortBoundary(unittest.TestCase):
 def test_canonical_admission_cannot_round_unsafe_combined_sum_into_peak(self):
  broad=JointSpec('J',0,JointMap(1,1,0),-2,2,10,100,1,100,'parent canonical exact-effort fixture')
  goal=JointTarget(.1,.2,.1,1,.1)
  code,t,packet=prepare_reference_joint(broad,goal);self.assertIsNone(code)
  sample=JointSample(-.1,-.2,0)
  e=exact(t,sample)
  limit=0.33977850723047515  # actual original float result; fixed fixture threshold independent of repaired predictor
  self.assertGreater(e,Fraction(limit))
  print(json.dumps({'canonicalPacket':packet.hex(),'exactEffort':str(e),'roundedEffort':limit,'exactExcess':str(e-Fraction(limit))}))
  spec=replace(broad,max_effort_nm=limit)
  clock=DeterministicClock(0);core=JointCore((spec,),clock=clock,max_reference_gap_s=1)
  station=FakeStation(core);self.assertTrue(station.connect());owner=station.claim('parent')
  self.assertTrue(core.begin_check());self.assertTrue(core.receive(0,sample,HEALTHY_STOP));self.assertTrue(core.checked());self.assertTrue(station.arm(owner))
  before=core.snapshot();target=Target(1,(goal,),window=station.issue_window(1),owner=owner)
  result=station.submit(target)
  self.assertFalse(result,'Canonical combined effort exactly exceeds peak even though float predictor rounded to equality')
  self.assertEqual(result.code,DecisionCode.EFFORT_OUT_OF_BOUNDS)
  self.assertEqual(core.snapshot().last_sequence,before.last_sequence)
  self.assertEqual(core.snapshot().admitted_target,before.admitted_target)

 def test_predictor_keeps_finite_cancellation(self):
  t=JointTarget(.25,.125,4,16,8);s=JointSample(.5,.625,0)
  self.assertEqual(exact(t,s),-4)
  self.assertEqual(predict_joint_effort(t,s),-4)

unittest.main(argv=[sys.argv[0]],verbosity=2)
