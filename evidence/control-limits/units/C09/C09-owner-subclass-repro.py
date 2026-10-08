import sys,unittest
sys.path.insert(0,sys.argv.pop(1))
from joint_contract import JointSpec,OwnerToken,Target
from joint_mapping import JointMap,JointTarget
from joint_controller import JointCore
from test_controller import FakeStation,DeterministicClock
class DifferentToken(OwnerToken):
 def __eq__(self,other):return True
class ExactOwner(unittest.TestCase):
 def test_mismatched_token_subclass_cannot_arm(self):
  c=JointCore((JointSpec('J1',0,JointMap(1,1,0),-2,2,'fixture: parent software limits'),),clock=DeterministicClock(0));s=FakeStation(c);self.assertTrue(s.connect());owner=s.claim('real');self.assertTrue(s.check())
  different=DifferentToken('wrong-epoch',owner.generation,'other-connection')
  self.assertFalse(c.arm(different),'Owner fields differ despite overridden equality')
if __name__=='__main__':unittest.main()
