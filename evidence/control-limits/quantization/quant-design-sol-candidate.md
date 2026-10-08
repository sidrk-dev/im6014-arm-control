# Quantized references: use emitted targets with two named velocities

## Decision and counterexample

Choose B. The codec rounds position and velocity independently. After external mapping, position has an offset and signed reduction; velocity has a different grid. If position advances one position quantum in time h, its derived velocity is that quantum divided by h. It generally falls between velocity codes. Exact equality to encoded velocity therefore cannot hold for arbitrary h. Rejection would unnecessarily forbid otherwise bounded references.

A concrete counterexample uses JointMap(3, -1, 0.001). Let q0 decode position code zero, let delta be the positive joint position quantum, and set h=0.1, qmax=q0+0.75*delta, V=0.75*delta/h. The continuous limited endpoint is inside the closed bounds. Nearest codec rounding selects one whole position quantum, exceeding both qmax and V. Its separately rounded motor velocity can be zero. The executable example below obtains the quantum through the existing codec decoder.

## Two minimal representations

A retains a virtual trajectory pair (q_ref,dq_ref), plus an independent decoded emitted JointTarget. Its planner can keep q_ref_next=q_ref_prev+dq_ref_next*h. It also needs emitted-position history and its derived velocity, emitted-velocity history, and three separate certificates. Both emitted velocity streams require speed/acceleration bounds. This can satisfy C09-C12, but the virtual state adds another history that cannot establish safety of emitted samples.

B stores the decoded emitted target only, plus a separately named position_step_velocity. Its two velocities are position_step_velocity=(q_emitted-q_previous)/h and encoded_velocity=decoded motor velocity in joint units. Require speed and acceleration bounds on both. This meets the samplewise milestones without equating different quantities. The previous exact algebra changes to q_next=q_previous+position_step_velocity*h. It must never substitute encoded_velocity on the right.

```python
@dataclass(frozen=True)
class ReferenceHistory:
    emitted: JointTarget
    position_step_velocity: float
    reference_sample_time_s: float

@dataclass(frozen=True)
class PreparedReference:
    emitted: JointTarget
    position_step_velocity: float
    reference_sample_time_s: float
    encoding_fingerprint: bytes
# Actual claim/write time belongs in the distinct dispatch record.
```

## Closed selection and codec API

For h>0, construct the selected rule's velocity intersection using previous position_step_velocity. Translate that intersection into a closed allowable position interval. Select an inward representable position nearest the desired position. Recompute its position_step_velocity from the decoded position; certify its speed, acceleration, position and measured lead.

Independently intersect [-V,V] with [encoded_velocity_previous-A*h, encoded_velocity_previous+A*h]. Select a representable velocity from this interval nearest the selected position_step_velocity. There is no exact coupling requirement. Empty intervals or intervals containing no code withdraw and Stop. A coarse grid can produce unchanged samples; do not invent fractional emitted movement.

Keep scales, raw domains, canonical command decoding and bounded raw-code search inside im6014_codec. Add one narrow projection API and a decoder, retaining command for existing bench callers:

```python
@dataclass(frozen=True)
class ScalarConstraint:
    lower: Fraction        # exact result from finite stored float inputs
    upper: Fraction
    preferred: Fraction
    decode_to_domain: Callable[[float], float]
    order: Literal[1, -1]   # monotonic transform direction

def project_command(motor_id, output_request, *,
                    position: ScalarConstraint,
                    velocity: ScalarConstraint) -> CanonicalCommand | None: ...
def decode_command(packet) -> CommandValues: ...
# CanonicalCommand holds exact validated bytes and decoded output values.
```

JointMap supplies scalar inverse transforms; the codec never imports joint/core types. This is limited to the two command axes, not a strategy framework. The codec performs monotonic binary searches over its position and speed code domains, including negative order, then deterministically chooses the nearest feasible code. Gains/feedforward retain canonical validation. Search checks decoded domain values, so transforming a floating interval cannot silently move a bound outward.

Build interval endpoints and the final certificate with exact rational arithmetic on stored finite float values. Compare each canonical decoded float as its exact rational value. This establishes inequalities on those software values; it does not imply exact physical calibration. Reject nonfinite inputs/results and numerical mapping failures. No arbitrary epsilon and no codec scales in core.

Prepare/validate the entire joint vector before dispatch. Cache exact canonical bytes in the station; bind them to the core certificate with the encoding fingerprint. Send those bytes unchanged. Re-encoding decoded floats would introduce another rounding opportunity. Later effort prediction uses decoded gains/feedforward/encoded velocity.

## History and final gate

Initialize a separate Stop anchor from healthy measured position with both velocities zero. It is not a dispatched record. Until a full write, preparation never advances that anchor/history. At h=0 only an already emitted unchanged command may refresh; with no emitted command, make no motion write. Excessive gap Stops.

A complete write commits that joint's prepared emitted target, position_step_velocity and preparation sample time. Partial/failed writes retain prior known history and latch failure. Other joints commit independently. Replacements, repeat preparations and failed claims never advance history. Preserve actual claim time separately; do not recompute h using it.

Before writing, verify plan identity, committed history identity, canonical encoding fingerprint, current owner, expiry, clock, freshness, maximum gap, and lead/position against current measured state. Recheck both velocity certificates using the immutable preparation interval. If new observations invalidate the packet, withdraw it rather than reshape after claim. A full write completing after revocation is still recorded truthfully and cannot rearm.

## Pure executable example

The proof-only grid adapter below reuses feedback decoding for identical position/speed raw fields. It does not implement production projection. It demonstrates the negative mapping, naive overshoot, inward selection, both acceleration streams, an unrepresentable interval and nonfinite rejection. No checks were executed by this seat.

```python
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
```
