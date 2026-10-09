"""R6: seeded fuzz of every emitted packet against the C09-C12 bounds.

Independent check from the packet BYTES actually written (decode_command ->
JointMap.from_output), not from the stored reference record:
  q_min <= q <= q_max; |dq_wire| <= V; |dq_wire - dq_wire_prev| <= A*h;
  |(q - q_prev)/h| <= V; |(q - q_prev)/h - v_prev| <= A*h; |q - q_meas| <= lead
with exact Fractions over the finite floats, h = prepared_at difference.
Random reductions, direction -1/+1, offsets, limits, V/A/lead, jittered h,
lagging/noisy measurement, goals with velocity feedforward, 3 joints.
"""
from fractions import Fraction
import random
from harness import Rig, spec, JointTarget, JointSample, State
from joint_mapping import JointMap, OutputSample
from im6014_codec import decode_command

rng = random.Random(20261009)
emitted = violations = runs = faults = 0
fault_kinds = {}


def decode_joint(sp, pkt):
    d = decode_command(pkt)
    s = sp.mapping.from_output(OutputSample(d["position_rad"], d["rpm"], d["torque_nm"]))
    return s.q_rad, s.dq_rad_s


for run in range(300):
    specs = []
    q0 = []
    for j in range(3):
        red = rng.choice((1.0, 1.5, 2.0, 3.0, 7.3, 0.6))
        direction = rng.choice((-1, 1))
        zero = rng.uniform(-0.5, 0.5)
        q_min = rng.uniform(-1.5, -0.2)
        q_max = rng.uniform(0.2, 1.5)
        sp = spec(f"J{j}", j, mapping=JointMap(red, direction, zero), q_min=q_min, q_max=q_max,
                  v_max=rng.uniform(0.2, 3.0), a_max=rng.uniform(5.0, 200.0),
                  lead=rng.uniform(0.05, 0.6), effort=1e6)
        specs.append(sp)
        q0.append(rng.uniform(q_min * 0.5, q_max * 0.5))
    rig = Rig(tuple(specs), tuple(q0), gap=0.5, tracking=False)
    runs += 1
    prev = [(r.q_rad, r.position_step_velocity_rad_s, r.encoded_velocity_rad_s, r.sample_time_s)
            for r in rig.snap().confirmed_references]
    meas = list(q0)
    for k in range(200):
        if k % 20 == 0:
            goals = []
            for sp, m in zip(specs, meas):
                lo = max(sp.q_min_rad, m - sp.max_lead_rad)
                hi = min(sp.q_max_rad, m + sp.max_lead_rad)
                goals.append(JointTarget(rng.uniform(lo, hi), rng.uniform(-sp.v_max_rad_s, sp.v_max_rad_s), 0.0, 2.0, 0.1))
            rig.q_meas = meas
            rig.feed()
            if not rig.submit(goals).accepted:
                break
        dt = rng.choice((0.002, 0.005, 0.01, 0.02)) * rng.uniform(0.8, 1.2)
        # measurement: lagging first-order follow of the last emitted reference + noise
        refs = rig.snap().confirmed_references
        meas = [m + 0.5 * (r.q_rad - m) + rng.uniform(-1e-4, 1e-4) for m, r in zip(meas, refs)]
        meas = [min(max(m, sp.q_min_rad), sp.q_max_rad) for m, sp in zip(meas, specs)]
        rig.q_meas = meas
        rig.clock.value += dt
        rig.feed()
        core = rig.core
        plan = core.prepare_refresh()
        if plan is None:
            snap = rig.snap()
            if snap.state is State.FAULT:
                faults += 1
                for f in snap.faults:
                    key = tuple(n for n in ("constraint_fault", "position_limit", "speed_limit", "effort_limit", "stale") if getattr(f, n))
                    fault_kinds[key] = fault_kinds.get(key, 0) + 1
            break
        for i, (sp, pkt) in enumerate(zip(specs, plan.packets)):
            if not core.claim_dispatch(plan.plan_id, i):
                break
            q, dq = decode_joint(sp, pkt)
            pq, pv, pdq, pt = prev[i]
            h = Fraction(plan.prepared_at_s - pt)
            Q, DQ, V, A, L = map(Fraction, (q, dq, sp.v_max_rad_s, sp.a_max_rad_s2, sp.max_lead_rad))
            checks = {"pos": Fraction(sp.q_min_rad) <= Q <= Fraction(sp.q_max_rad),
                      "wireV": abs(DQ) <= V,
                      "lead": abs(Q - Fraction(meas[i])) <= L}
            if h > 0:
                v = (Q - Fraction(pq)) / h
                checks.update({"stepV": abs(v) <= V, "stepA": abs(v - pv) <= A * h,
                               "wireA": abs(DQ - Fraction(pdq)) <= A * h})
            else:
                v = pv
            bad = [n for n, ok in checks.items() if not ok]
            if bad:
                violations += 1
                print(f"VIOLATION run={run} step={k} joint={i} {bad} q={q!r} dq={dq!r} h={float(h)}")
            emitted += 1
            core.dispatch_result(plan.plan_id, i, __import__('joint_contract').WriteResult.COMPLETE)
            prev[i] = (q, v, dq, plan.prepared_at_s)

print(f"runs={runs} emitted_packets_checked={emitted} violations={violations} runs_ending_in_fault={faults}")
print(f"fault kinds (per faulted joint): {fault_kinds}")
