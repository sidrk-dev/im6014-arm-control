"""Parent complete-reference zero-cadence timestamp check, no I/O."""
from pathlib import Path
from fractions import Fraction
import sys,unittest
sys.path.insert(0,str(Path(sys.argv[1]).resolve()))
from joint_contract import JointSpec,JointTrajectoryReference,DecisionCode,RefreshPlan,Target
from joint_mapping import JointMap,JointTarget,JointSample
from joint_reference import prepare_reference_joint,verify_dispatch_claim
class CacheTime(unittest.TestCase):
 def test_old_cached_sample_cannot_claim_it_was_prepared_at_positive_cadence(self):
  spec=JointSpec('J',0,JointMap(1,1,0),-2,2,10,100,1,10,'parent cachetime fixture')
  goal=JointTarget(0,0,0,0,0)
  code,t,packet=prepare_reference_joint(spec,goal);self.assertIsNone(code)
  ref=JointTrajectoryReference(t.q_rad,Fraction(0),t.dq_rad_s,0,True,goal,t,packet)
  code=verify_dispatch_claim(spec=spec,conf_ref=ref,prop_ref=ref,plan_joint=t,packet=packet,goal=goal,sample=JointSample(0,0,0),prepared_at_s=.1)
  self.assertEqual(code,DecisionCode.CONSTRAINT_VIOLATION,'h0 replay must not silently change its cadence to a positive preparation interval')
unittest.main(argv=[sys.argv[0]],verbosity=2)
