"""Parent canonical h0 and final-dispatch effort certificates; no transport."""
from pathlib import Path
from fractions import Fraction
from dataclasses import replace
import sys,unittest
sys.path.insert(0,str(Path(sys.argv[1]).resolve()))
from joint_contract import JointSpec,JointTrajectoryReference,DecisionCode
from joint_mapping import JointMap,JointTarget,JointSample
from joint_reference import prepare_reference_joint,shape_reference_joint,verify_dispatch_claim

class EffortGates(unittest.TestCase):
 def setUp(self):
  self.spec=JointSpec('J',0,JointMap(1,1,0),-2,2,10,100,1,0.33977850723047515,'parent exact-effort h0 and claim fixture')
  self.goal=JointTarget(.1,.2,.1,1,.1)
  code,self.actual,self.packet=prepare_reference_joint(self.spec,self.goal);self.assertIsNone(code)
  self.reference=JointTrajectoryReference(self.actual.q_rad,Fraction(0),self.actual.dq_rad_s,0,True,self.goal,self.actual,self.packet)
  self.sample=JointSample(-.1,-.2,0)
  t,s=self.actual,self.sample
  self.exact=Fraction(t.tau_nm)+Fraction(t.kp)*(Fraction(t.q_rad)-Fraction(s.q_rad))+Fraction(t.kd)*(Fraction(t.dq_rad_s)-Fraction(s.dq_rad_s))
  self.assertGreater(self.exact,Fraction(self.spec.max_effort_nm))
 def test_h0_cached_replay_refuses_exactly_over_peak(self):
  code,t,ref,packet=shape_reference_joint(self.spec,self.goal,self.reference,self.sample,0,.5)
  self.assertEqual(code,DecisionCode.EFFORT_OUT_OF_BOUNDS)
  self.assertIsNone(packet)
 def test_finalclaim_refuses_exactly_over_peak(self):
  code=verify_dispatch_claim(spec=self.spec,conf_ref=self.reference,prop_ref=self.reference,plan_joint=self.actual,packet=self.packet,goal=self.goal,sample=self.sample,prepared_at_s=0)
  self.assertEqual(code,DecisionCode.EFFORT_OUT_OF_BOUNDS)
 def test_binary_exact_sum_at_peak_allowed(self):
  # A single decoded nonzero feedforward term gives an exact represented equality.
  goal=JointTarget(0,0,.1,0,0)
  code,t,packet=prepare_reference_joint(self.spec,goal);self.assertIsNone(code)
  spec=replace(self.spec,max_effort_nm=abs(t.tau_nm))
  self.assertGreater(spec.max_effort_nm,0)
  self.assertEqual(Fraction(t.tau_nm),Fraction(spec.max_effort_nm))
  ref=JointTrajectoryReference(t.q_rad,Fraction(0),t.dq_rad_s,0,True,goal,t,packet)
  code=verify_dispatch_claim(spec=spec,conf_ref=ref,prop_ref=ref,plan_joint=t,packet=packet,goal=goal,sample=JointSample(0,0,0),prepared_at_s=0)
  self.assertIsNone(code)

unittest.main(argv=[sys.argv[0]],verbosity=2)
