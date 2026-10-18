import sys,json
from pathlib import Path
assert len(sys.argv) == 3, 'usage: proof.py REPOSITORY RESULT.json'
sys.path.insert(0,str(Path(sys.argv[1]).resolve()))
from test_controller_effort import make_spec,armed_core
from joint_mapping import JointTarget,JointSample
from joint_records import Target,MotorHealth
from joint_reference import compute_predicted_effort
spec=make_spec(max_effort=1.,max_lead=.5)
core,clock,owner=armed_core((spec,),(0.,),(0.,))
j=JointTarget(1.,.5,0.,20.,4.)
t=Target(sequence=1,joints=(j,),created_at_s=clock(),expires_at_s=clock()+1,ownership=owner)
assert core.submit_target(t).accepted
clock.advance(.1);core.receive(0,JointSample(0.,0.,0.),MotorHealth(mode=1,errors=0,warnings=0,timeout=False));ref=core.consume_target();assert ref is None
result={"reverseReferenceSuppressed":True,"pendingPreserved":core.snapshot().pending_target is t,"hardwareOrNetwork":False};assert result["pendingPreserved"];Path(sys.argv[2]).write_text(json.dumps(result));print(json.dumps(result));
