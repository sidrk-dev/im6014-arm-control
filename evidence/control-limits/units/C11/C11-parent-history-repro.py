"""Parent final-gate fault injection; no motor I/O."""
from pathlib import Path
from fractions import Fraction
from dataclasses import replace
import sys,unittest
sys.path.insert(0,str(Path(sys.argv[1]).resolve()))
import test_controller_limits as fixtures
from joint_contract import Target
from joint_mapping import JointTarget
class FinalHistoryCertificate(unittest.TestCase):
 def test_inconsistent_safe_looking_history_cannot_claim_actual_packet(self):
  fixture=fixtures.ReferenceAccelerationTests();fixture.setUp();core,station,owner=fixture._armed_station()
  goal=Target(1,(JointTarget(.1,0,0,1,.1),JointTarget(0,0,0,1,.1)),window=station.issue_window(1),owner=owner)
  self.assertTrue(station.submit(goal).accepted);fixture.clock.value+=.05
  plan=core.prepare_refresh();self.assertIsNotNone(plan);before=core.snapshot().confirmed_references[0]
  proposed=plan.proposed_references[0]
  true_step=(Fraction(proposed.q_rad)-Fraction(before.q_rad))/Fraction(proposed.sample_time_s-before.sample_time_s)
  self.assertGreater(true_step,0)
  forged=replace(proposed,position_step_velocity_rad_s=Fraction(0))
  self.assertLessEqual(abs(forged.position_step_velocity_rad_s),Fraction(fixture.spec1.v_max_rad_s))
  self.assertLessEqual(abs(forged.position_step_velocity_rad_s-before.position_step_velocity_rad_s),Fraction(fixture.spec1.a_max_rad_s2)*Fraction(proposed.sample_time_s-before.sample_time_s))
  core._prepared_plan=replace(plan,proposed_references=(forged,plan.proposed_references[1]))
  self.assertFalse(core.claim_dispatch(plan.plan_id,0),'Gate accepted history inconsistent with actual emitted endpoint')
  self.assertEqual(core.snapshot().confirmed_references[0],before)
  self.assertTrue(core.snapshot().faults[0].constraint_fault)
unittest.main(argv=[sys.argv[0]],verbosity=2)
