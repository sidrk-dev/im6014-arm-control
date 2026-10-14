from pathlib import Path
import json,random
from im6014_codec import command,decode_command,raw_to_position,raw_to_rpm
from joint_mapping import JointMap,JointTarget,OutputSample
rng=random.Random(20261014)
position_codes=[-2147483648,-2147483647,-1000000000,-100000,-1,0,1,100000,1000000000,2147483646,2147483647]+[rng.randint(-2147483648,2147483647) for _ in range(64)]
speed_codes=[-32768,-32767,-2000,-1,0,1,2000,32766,32767]
checks=0
mismatches=[]
rejections=[]
for reduction in (1,3,6,2.7,127):
    for direction in (-1,1):
        for offset in (0.0,0.37,5.0,-5.0,10000.0):
            mapping=JointMap(reduction,direction,offset)
            for p in position_codes:
                for s in speed_codes:
                    sample=mapping.from_output(OutputSample(raw_to_position(p),raw_to_rpm(s),0.0))
                    output=mapping.to_output(JointTarget(sample.q_rad,sample.dq_rad_s,0.0,0.0,0.0))
                    try:
                        frame=command(0,mode=1,position=output.q_rad,rpm=output.rpm)
                    except ValueError as exc:
                        rejections.append({'reduction':reduction,'direction':direction,'offset':offset,'p':p,'s':s,'error':str(exc)})
                        continue
                    decoded=decode_command(frame)
                    checks+=1
                    if decoded['raw_position']!=p or decoded['raw_rpm']!=s:
                        mismatches.append({'reduction':reduction,'direction':direction,'offset':offset,'p':p,'s':s,'decoded_p':decoded['raw_position'],'decoded_s':decoded['raw_rpm']})
domain=[{'reduction':n,'direction':s,'q_joint':0.2,'q_output':JointMap(n,s,0.37).to_output(JointTarget(.2,0,0,1,.1)).q_rad} for n in (1,3,6) for s in (-1,1)]
result={'roundtrip_cases':checks,'mismatches':mismatches,'safe_rejection_count':len(rejections),'safe_rejection_examples':rejections[:3],'domain_test_values':domain,'physicalIO':False,'scope':'finite listed mappings and grid samples; no universal floating-point guarantee'}
Path('/home/sid/Documents/code/im6014Development/.workbench-runs/c07-c10-20261013/numeric-review-probe.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
assert not mismatches
