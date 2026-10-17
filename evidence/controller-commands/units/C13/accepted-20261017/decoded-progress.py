import sys,json,itertools
from pathlib import Path
assert len(sys.argv) == 3, 'usage: proof.py REPOSITORY RESULT.json'
sys.path.insert(0,str(Path(sys.argv[1]).resolve()))
from test_controller_effort import make_spec,armed_core
from joint_mapping import JointTarget,JointSample,OutputSample
from joint_records import Target,MotorHealth
from im6014_codec import command,decode_command
from joint_reference import compute_predicted_effort
results=[]
for reduction,direction,sgn,zero,feed in itertools.product((1.,3.),(-1,1),(-1,1),(0.,.37),(-.1,.1)):
 spec=make_spec(reduction=reduction,direction=direction,zero_output_rad=zero,max_effort=1.0,max_lead=.5)
 core,clock,owner=armed_core((spec,),(0.0,),(0.0,))
 joint=JointTarget(sgn*1.,0.,feed,20.,0.)
 t=Target(sequence=1,joints=(joint,),created_at_s=clock(),expires_at_s=clock()+1.,ownership=owner)
 assert core.submit_target(t).accepted
 clock.advance(.1);core.receive(0,JointSample(0.,0.,0.),MotorHealth(mode=1,errors=0,warnings=0,timeout=False));ref=core.consume_target()
 assert ref is not None,(reduction,direction,sgn,zero,feed,'stalled')
 c=ref.joints[0];out=spec.mapping.to_output(c);dec=decode_command(command(spec.motor_id,mode=1,rpm=out.rpm,kd=out.kd,torque=out.tau_nm,watchdog=True,position=out.q_rad,kp=out.kp));sample=spec.mapping.from_output(OutputSample(dec['position_rad'],dec['rpm'],dec['torque_nm']));eff=sample.tau_nm+dec['kp']*reduction**2*sample.q_rad+dec['kd']*reduction**2*sample.dq_rad_s
 assert abs(eff)<=1. and 0<c.q_rad*sgn<.5
 assert compute_predicted_effort(spec,c,0.,0.)==eff
 assert c.tau_nm==feed and c.kp==20. and c.kd==0.
 results.append({'reduction':reduction,'direction':direction,'sign':sgn,'zero':zero,'feedforward':feed,'issuedPosition':c.q_rad,'decodedEffort':eff})
record={'passed':len(results),'cases':results,'hardwareOrNetwork':False};Path(sys.argv[2]).write_text(json.dumps(record,indent=2));print(json.dumps({'passed':len(results),'hardwareOrNetwork':False}))
