"""Parent certificate of decoded production points; no motor I/O."""
from pathlib import Path
from fractions import Fraction
import json,sys
sys.path.insert(0,str(Path(sys.argv[1]).resolve()))
from joint_contract import JointSpec,JointTrajectoryReference
from joint_protection import ProtectionPolicy
from joint_mapping import JointMap,JointTarget,JointSample,OutputSample
from joint_reference import shape_reference_joint
from im6014_codec import decode_command
from importlib.util import spec_from_file_location,module_from_spec
spec=spec_from_file_location('parent_oracle',Path(__file__).with_name('reference-oracle.py'))
oracle=module_from_spec(spec);spec.loader.exec_module(oracle)
rows=[]
for reduction in (.5,1,3,100):
 for direction in (-1,1):
  for zero in (-.1,0,.25):
   mapping=JointMap(reduction,direction,zero)
   joint=JointSpec('fixture',0,mapping,-1,1,.2,4.0,.125,1000.0,'parent exact speed acceleration and lead software fixture',ProtectionPolicy(85,70,105,90,500,1,.25))
   ref=JointTrajectoryReference(0,0,0,0,False)
   for sign in (1,-1):
    goal=JointTarget(sign*.075,sign*.05,0,1,.1)
    for step in range(16):
     now=ref.sample_time_s+(1/64 if step%2 else 1/32)
     h=now-ref.sample_time_s
     code,target,next_ref,packet=shape_reference_joint(joint,goal,ref,JointSample(0.0,-.05,0.0),now,.25)
     assert code is None,(reduction,direction,zero,sign,step,code)
     decoded=decode_command(packet)
     actual=mapping.from_output(OutputSample(decoded['position_rad'],decoded['rpm'],decoded['torque_nm']))
     assert target.q_rad==actual.q_rad and target.dq_rad_s==actual.dq_rad_s
     proof=oracle.certify(previous_q=ref.q_rad,emitted_q=actual.q_rad,elapsed=h,
       wire_velocity=actual.dq_rad_s,max_velocity=joint.v_max_rad_s,q_min=joint.q_min_rad,q_max=joint.q_max_rad,
       previous_step_velocity=ref.position_step_velocity_rad_s,previous_wire_velocity=ref.encoded_velocity_rad_s,max_acceleration=joint.a_max_rad_s2,measured_q=0.0,max_lead=joint.max_lead_rad)
     assert proof['passed'],proof
     assert next_ref.position_step_velocity_rad_s==proof['position_step_velocity']
     assert target.kp==decoded['kp']*reduction**2 and target.kd==decoded['kd']*reduction**2
     combined=Fraction(actual.tau_nm)+Fraction(target.kp)*(Fraction(actual.q_rad)-Fraction(0.0))+Fraction(target.kd)*(Fraction(actual.dq_rad_s)-Fraction(-.05))
     assert abs(combined)<=Fraction(joint.max_effort_nm)
     if step >= 8: assert sign*(next_ref.q_rad-ref.q_rad)>0,'feasible cadence must progress after bounded reversal'
     assert next_ref.q_rad!=ref.q_rad,'nonzero objective cannot silently stall'
     rows.append({'reduction':reduction,'direction':direction,'zero':zero,'goalSign':sign,
       'combinedEffortExact':str(combined),'peakEffort':joint.max_effort_nm,'h':h,'previousQ':ref.q_rad,'emittedQ':actual.q_rad,'wireVelocity':actual.dq_rad_s,
       'positionStepExact':str(proof['position_step_velocity']),'previousPositionStepExact':str(ref.position_step_velocity_rad_s),
       'previousWireVelocity':ref.encoded_velocity_rad_s,'maxAcceleration':joint.a_max_rad_s2,'measuredQ':0.0,'maxLead':joint.max_lead_rad,'checks':proof['checks']})
     repeated=shape_reference_joint(joint,goal,next_ref,JointSample(0.0,-.05,0.0),now,.25)
     assert repeated[0] is None and repeated[2]==next_ref and repeated[3]==packet
     ref=next_ref
print(json.dumps({'passed':True,'positiveStepCertificates':len(rows),'combinedEffortCertificates':len(rows),'h0ExactCacheCertificates':len(rows),
 'physicalIO':'none; pure production shaping and independent decoded arithmetic','samples':rows},indent=2))
