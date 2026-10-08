from fractions import Fraction as F
import math, struct
from im6014_codec import command, feedback, crc32
from joint_mapping import JointMap, JointTarget, OutputSample

mapping = JointMap(3.0, -1, 0.001)

# Proof-only codec grid adapter. Production puts this knowledge in codec.
def decoded(qcode, vcode):
    body = struct.pack("<BbBBhhiIHBB",
        0, 20, 20, 48, 0, vcode, qcode, 0, 0, 0, 0)
    return feedback(b"\xfc\xee" + body + struct.pack("<I", crc32(body)), 0)

def joint(qcode, vcode):
    d = decoded(qcode, vcode)
    return mapping.from_output(OutputSample(d["position_rad"], d["rpm"], 0))

def qvalue(i):  # ordinal rises in joint units despite negative mapping
    return F(joint(-i, 0).q_rad)

def vvalue(i):
    return F(joint(0, -i).dq_rad_s)

def inward(value, low, high, wanted):
    # Small fixture code domain. Production uses codec's full raw domains.
    def first(predicate):
        left, right = -8, 9
        while left < right:
            mid = (left + right) // 2
            if predicate(value(mid)):
                right = mid
            else:
                left = mid + 1
        return left
    a, b = first(lambda x: x >= low), first(lambda x: x > high) - 1
    if a > b:
        return None
    p = first(lambda x: x >= wanted)
    choices = {a, b, max(a, min(b, p)), max(a, min(b, p - 1))}
    return min(choices, key=lambda i: (abs(value(i) - wanted), i))

q0, h = qvalue(0), F(1, 10)
delta = qvalue(1) - q0
V, A = 3 * delta / (4 * h), 3 * delta / (4 * h * h)
qmin, qmax = q0 - delta, q0 + 3 * delta / 4
measured, lead, uprev, dprev = q0, 3 * delta / 4, F(0), F(0)

# Naive independently rounded limiting crosses the position/speed boundary.
o = mapping.to_output(JointTarget(float(qmax), float(V), 0, 0, 0))
packet = command(0, mode=1, position=o.q_rad, rpm=o.rpm)
_, vr, qr, _, _ = struct.unpack("<hhihh", packet[4:16])
naive_q = F(joint(qr, vr).q_rad)
assert naive_q > qmax and (naive_q - q0) / h > V

lo = max(-V, uprev-A*h, (qmin-q0)/h, (measured-lead-q0)/h)
hi = min(V, uprev+A*h, (qmax-q0)/h, (measured+lead-q0)/h)
qi = inward(qvalue, q0+h*lo, q0+h*hi, qmax)
assert qi is not None
q, u = qvalue(qi), (qvalue(qi)-q0)/h
vi = inward(vvalue, max(-V, dprev-A*h), min(V, dprev+A*h), u)
assert vi is not None
dq = vvalue(vi)
assert qmin <= q <= qmax and abs(q-measured) <= lead
assert abs(u) <= V and abs(u-uprev) <= A*h
assert abs(dq) <= V and abs(dq-dprev) <= A*h
assert inward(qvalue, q0+delta/4, q0+3*delta/4, qmax) is None
assert inward(vvalue, F(0), 3*vvalue(1)/4, vvalue(1)) == 0

# A nonzero inward position step can coexist with zero encoded velocity.
upper, V2 = q0+7*delta/4, 7*delta/(4*h)
A2 = V2/h
qi = inward(qvalue, q0, upper, upper)
assert qi == 1
u = (qvalue(qi)-q0)/h
vi = inward(vvalue, -V2, V2, u)
assert vi == 0 and F(0) < u <= V2 and abs(u-uprev) <= A2*h
assert abs(vvalue(vi)-dprev) <= A2*h
assert q0 <= qvalue(qi) <= upper and abs(qvalue(qi)-measured) <= 7*delta/4


# Full canonical encoding must preserve the chosen codes.
d = decoded(-qi, -vi)
packet = command(0, mode=1, position=d["position_rad"], rpm=d["rpm"])
_, vr, qr, _, _ = struct.unpack("<hhihh", packet[4:16])
assert (qr, vr) == (-qi, -vi)

def finite_fraction(x):
    if type(x) not in (int, float) or not math.isfinite(x):
        raise ValueError("nonfinite input")
    return F(x)

for bad in (float("inf"), float("nan")):
    try:
        finite_fraction(bad)
    except ValueError:
        pass
    else:
        raise AssertionError("nonfinite input admitted")
