import json,sys
from pathlib import Path
from unittest.mock import patch
assert len(sys.argv) == 3, 'usage: proof.py REPOSITORY RESULT.json'
sys.path.insert(0,str(Path(sys.argv[1]).resolve()))
import joint_controller
from joint_mapping import JointTarget,JointSample
from joint_records import MotorHealth,Target,State
from test_controller_effort import make_spec,armed_core
spec=make_spec(max_effort=3.0,max_lead=0.5)
core,clock,owner=armed_core((spec,),(0.0,),(0.0,))
target=Target(sequence=1,joints=(JointTarget(q_rad=0.1,dq_rad_s=0.5,tau_nm=0.0,kp=2.0,kd=4.0),),created_at_s=clock(),expires_at_s=clock()+1,ownership=owner)
assert core.submit_target(target).accepted
clock.advance(.05)
original=joint_controller.plan_reference
observed={}
def shift_after_planning(*args,**kwargs):
 candidate=original(*args,**kwargs)
 assert candidate is not None,'fixture must pass planning first'
 observed['plannedCandidate']=True
 core.receive(0,JointSample(q_rad=0.0,dq_rad_s=-1.0,tau_nm=0.0),MotorHealth(mode=1,errors=0,warnings=0,timeout=False))
 return candidate
with patch.object(joint_controller,'plan_reference',side_effect=shift_after_planning):
 issued=core.consume_target()
s=core.snapshot()
assert issued is None,'release must reject newly excessive effort'
assert s.pending_target is target and s.state is State.ARMED and s.ownership is owner and not s.fault_latched
core.receive(0,JointSample(q_rad=0.0,dq_rad_s=0.0,tau_nm=0.0),MotorHealth(mode=1,errors=0,warnings=0,timeout=False))
recovered=core.consume_target();assert recovered is not None
control,control_clock,control_owner=armed_core((spec,),(0.0,),(0.0,))
control_target=Target(sequence=1,joints=target.joints,created_at_s=control_clock(),expires_at_s=control_clock()+1,ownership=control_owner)
assert control.submit_target(control_target).accepted
control_clock.advance(.05);baseline=control.consume_target();assert baseline is not None
assert recovered.joints==baseline.joints and recovered.issued_at_s==baseline.issued_at_s,'veto must preserve issued reference history'
result={'plannedBeforeFeedbackShift':observed['plannedCandidate'],'releaseVeto':True,'pendingOwnerAndArmedStatePreserved':True,'recoveryMatchesUninterruptedControl':True,'hardwareOrNetwork':False}
print(json.dumps(result));Path(sys.argv[2]).write_text(json.dumps(result,indent=2))
