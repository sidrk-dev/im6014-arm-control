from fractions import Fraction as F
import math,struct,sys
sys.path.insert(0,'quant-design-gemini')
from im6014_codec import command,feedback,crc32
from joint_mapping import JointMap,OutputSample
m=JointMap(3,-1,.001)
def joint(q,v):
 body=struct.pack('<BbBBhhiIHBB',0,20,20,48,0,v,q,0,0,0,0)
 d=feedback(b'\xfc\xee'+body+struct.pack('<I',crc32(body)),0)
 return m.from_output(OutputSample(d['position_rad'],d['rpm'],0))
q0=F(joint(0,0).q_rad); delta=F(joint(-1,0).q_rad)-q0
h=F(1,100); A=delta/(2*h*h)
assert A*h*h<delta
q=q0; u=F(0)
for _ in range(100):
 # A closed interval contains only the unchanged grid point. Actual writes
 # commit this history every sample, so elapsed time never accumulates.
 lo=q+h*(u-A*h); hi=q+h*(u+A*h)
 assert lo < q < hi and hi < q+delta and lo>q-delta
 qnext=q; unext=(qnext-q)/h
 assert unext==0
 q,u=qnext,unext
assert q==q0
# Noninteger 512*h does not imply no integer grid solution.
h2=F(1,200)
assert 512*h2==F(64,25)
assert F(64)==512*h2*25
# The draft Gemini overshoot example qmax=1 was actually inward.
p=command(0,mode=1,position=1)
_,vr,qr,_,_=struct.unpack('<hhihh',p[4:16])
body=struct.pack('<BbBBhhiIHBB',0,20,20,48,0,vr,qr,0,0,0,0)
d=feedback(b'\xfc\xee'+body+struct.pack('<I',crc32(body)),0)
assert d['position_rad']<=1
print({'stall_samples':100,'position_quantum':float(delta),'noninteger_ratio_solution':{'position_code':64,'velocity_code':25},'draft_qmax1_decoded':d['position_rad']})
