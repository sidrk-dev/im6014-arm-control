"""Randomized model check of the JointCore write gate and fault-ack rules.

Invariants (from docs/joint-api.md and docs/controller-design.md):
 G1 claim_dispatch()==True only in ARMED/MOVING, no latched fault, plan owner is
    the current owner, admitted goal present and now < expires_at, every joint
    observation fresh/healthy/within limits, and an arm() happened since the last
    stop/claim/check/disconnect/fault/expiry.
 F1 latched faults disappear only through checked(), after a begin_check(ack=True)
    that followed the most recent latch, with every joint showing a contiguous
    candidate Stop interval >= recovery_dwell_s.
 S1 READY is entered only by checked(); ARMED only by arm() from READY.
Fake I/O only.
"""
import random
import sys
from fractions import Fraction

from arm3 import (Clock, make_core, health, JointSample, JointTarget, State, Target,
                  WriteResult, POLICY)
from joint_contract import CommunicationFailure, SourceIdentity
from joint_protection import check_joint_limits

MAX_AGE = 0.5
STATS = {'grants': 0, 'clears': 0, 'armed': 0, 'moving': 0}


def fault_flags(snap):
    out = {}
    for f in snap.faults:
        out[f.name] = (f.errors, f.warnings, f.timeout, f.stale, tuple(sorted(c.value for c in f.communication_errors)),
                       f.position_limit, f.speed_limit, f.effort_limit, f.constraint_fault,
                       f.driver_temperature_limit, f.winding_temperature_limit, f.continuous_effort_limit)
    return out


def grew(before, after):
    for name, flags in after.items():
        if name not in before:
            return True
        b = before[name]
        for x, y in zip(b, flags):
            if isinstance(x, tuple):
                if set(y) - set(x):
                    return True
            elif isinstance(x, bool):
                if y and not x:
                    return True
            elif isinstance(x, int):
                if y | x != x:
                    return True
    return False


def run(seed, steps=3000):
    rnd = random.Random(seed)
    clock = Clock(0.0)
    core = make_core(clock, max_reference_gap_s=0.5, feedback_max_age_s=MAX_AGE, max_window_s=1.0)
    specs = core.joint_specs
    owners = []
    windows = []
    seq = 0
    plan = None
    claim = None
    arm_valid = False
    ack_after_latch = False
    cand = {s.motor_id: [] for s in specs}  # shadow recovery candidate timestamps since last check
    log = []
    pending = []

    def sample_choice():
        r = rnd.random()
        q = rnd.choice([0.0, 0.0, 0.05, -0.05])
        if r < 0.80:
            return JointSample(q, 0.0, rnd.choice([0.0, 0.5])), health(mode=rnd.choice([0, 0, 1]))
        if r < 0.85:
            return JointSample(q, 0.0, 0.0), health(mode=0, driver_c=rnd.choice([75.0, 86.0]))
        if r < 0.90:
            return JointSample(q, 0.0, rnd.choice([5.0, 9.0])), health(mode=1)
        if r < 0.94:
            return JointSample(q, 0.0, 0.0), health(mode=0, errors=1)
        if r < 0.97:
            return JointSample(2.5, 0.0, 0.0), health(mode=0)
        return JointSample(q, 6.0, 0.0), health(mode=1)

    for step in range(steps):
        before = core.snapshot()
        bflags = fault_flags(before)
        if not pending and rnd.random() < 0.15:
            if before.state in (State.ARMED, State.MOVING):
                pending.extend(["adv_small", "recv_all_good1", "prepare", "claim_d0", "result_ok",
                                "claim_d1", "result_ok", "claim_d2", "result_ok"])
            else:
                pending.extend(["connect", "claim", "check_ack"] + ["recv_all_good", "adv_small"] * 4
                               + ["recv_all_good", "checked", "arm", "window", "submit"])
        if pending and rnd.random() < 0.85:
            op = pending.pop(0)
        else:
            op = rnd.choices(
                ["adv", "adv_big", "connect", "claim", "check", "check_ack", "recv", "recv_all", "comm",
                 "checked", "arm", "arm_stale", "tick", "window", "submit", "submit_stale", "prepare",
                 "claim_d", "result", "stop", "disconnect"],
                weights=[30, 2, 3, 3, 3, 3, 15, 25, 1, 8, 6, 1, 4, 6, 8, 2, 10, 12, 10, 2, 1])[0]
        if op == "adv_small":
            op = "adv"; forced_adv = 0.1
        else:
            forced_adv = None
        forced_sample = None
        if op == "recv_all_good":
            op = "recv_all"; forced_sample = (JointSample(0.0, 0.0, 0.0), health(mode=0))
        elif op == "recv_all_good1":
            op = "recv_all"; forced_sample = (JointSample(0.0, 0.0, 0.0), health(mode=1))
        forced_idx = None
        if op.startswith("claim_d") and op != "claim_d":
            forced_idx = int(op[-1]); op = "claim_d"
        forced_ok = op == "result_ok"
        if forced_ok:
            op = "result"
        detail = ""
        granted = False
        try:
            if op == "adv":
                clock.value += forced_adv if forced_adv is not None else rnd.choice([0.001, 0.01, 0.05, 0.1, 0.2])
            elif op == "adv_big":
                clock.value += rnd.choice([0.6, 1.5])
            elif op == "connect":
                if core.begin_connect().accepted:
                    core.connected(SourceIdentity("fixture", "fuzz", 1))
            elif op == "claim":
                owners.append(core.claim_owner(f"c{step}"))
            elif op in ("check", "check_ack"):
                ack = op == "check_ack"
                d = core.begin_check(acknowledge_fault=ack)
                if d.accepted:
                    cand = {s.motor_id: [] for s in specs}
                    if ack:
                        ack_after_latch = True
            elif op in ("recv", "recv_all"):
                ids = [rnd.choice(specs).motor_id] if op == "recv" else [s.motor_id for s in specs]
                common = (forced_sample or sample_choice()) if op == "recv_all" else None
                for mid in ids:
                    smp, hl = common or sample_choice()
                    d = core.receive(mid, smp, hl)
                    if d.code.value != "invalid_state":
                        is_c = (hl.mode == 0 and hl.healthy and abs(smp.tau_nm) <= POLICY.continuous_effort_nm
                                and hl.driver_c <= POLICY.driver_reset_c and hl.winding_c <= POLICY.winding_reset_c
                                and not check_joint_limits(specs[mid], smp, hl).has_violation)
                        if is_c:
                            cand[mid].append(clock.value)
                        else:
                            cand[mid] = []
            elif op == "comm":
                mid = rnd.choice(specs).motor_id
                if core.communication_failed(mid, CommunicationFailure.TIMEOUT).accepted:
                    cand[mid] = []
            elif op == "checked":
                core.checked()
            elif op == "arm":
                if owners:
                    if core.arm(owners[-1]).accepted:
                        arm_valid = True
            elif op == "arm_stale":
                if len(owners) > 1:
                    if core.arm(owners[rnd.randrange(len(owners) - 1)]).accepted:
                        raise AssertionError("stale owner armed")
            elif op == "tick":
                core.tick()
            elif op == "window":
                windows.append(core.issue_window(rnd.choice([0.05, 0.2, 0.5, 1.0])))
            elif op in ("submit", "submit_stale"):
                if owners and windows:
                    seq += rnd.choice([1, 1, 1, 0])
                    stale = op == "submit_stale"
                    ow = owners[rnd.randrange(len(owners))] if stale else owners[-1]
                    wi = windows[rnd.randrange(len(windows))] if stale else windows[-1]
                    q = rnd.choice([0.0, 0.02, -0.02, 0.1])
                    t = Target(seq, tuple(JointTarget(q, 0.0, 0.0, 2.0, 0.1) for _ in specs), window=wi, owner=ow)
                    d = core.submit_target(t)
                    if d.accepted and stale and not (ow == before.current_owner and wi == windows[-1]):
                        raise AssertionError("stale owner/window admitted")
            elif op == "prepare":
                p = core.prepare_refresh()
                if p is not None:
                    plan = p
            elif op == "claim_d":
                if plan is not None:
                    idx = forced_idx if forced_idx is not None else rnd.randrange(len(specs))
                    granted = core.claim_dispatch(plan.plan_id, idx)
                    if granted:
                        claim = (plan.plan_id, idx)
            elif op == "result":
                if claim is not None:
                    res = WriteResult.COMPLETE if forced_ok else rnd.choices(list(WriteResult), weights=[85, 5, 5, 5])[0]
                    core.dispatch_result(claim[0], claim[1], res)
                    claim = None
            elif op == "stop":
                core.stop()
            elif op == "disconnect":
                core.disconnect()
        except ValueError as exc:
            detail = f"ValueError {exc}"
        after = core.snapshot()
        aflags = fault_flags(after)
        log.append((step, op, round(clock.value, 4), before.state.name, after.state.name, granted, detail))

        # Track arm validity
        if after.current_owner != before.current_owner:
            arm_valid = False
        if after.fault_latched:
            arm_valid = False
        if before.admitted_target is not None and after.admitted_target is None and op not in ("submit",):
            arm_valid = arm_valid and after.state in (State.ARMED, State.MOVING)
        if after.state not in (State.ARMED, State.MOVING):
            arm_valid = False if op != "arm" else arm_valid
        if grew(bflags, aflags):
            ack_after_latch = False

        if granted:
            STATS['grants'] += 1
        if before.fault_latched and not after.fault_latched:
            STATS['clears'] += 1
        if after.state is State.ARMED and before.state is not State.ARMED:
            STATS['armed'] += 1
        if after.state is State.MOVING and before.state is not State.MOVING:
            STATS['moving'] += 1
        # G1
        if granted:
            now = clock.value
            problems = []
            if before.state not in (State.ARMED, State.MOVING):
                problems.append(f"state {before.state}")
            if before.fault_latched:
                problems.append("fault latched")
            if not arm_valid:
                problems.append("no arm since last revoke")
            if before.current_owner is None or before.current_owner != plan.target.owner:
                problems.append("owner mismatch")
            if before.admitted_target is None or now >= before.admitted_target.window.expires_at_s:
                problems.append("expired/no goal")
            for spec, js in zip(specs, before.joints):
                if not js.valid or now - js.received_at > MAX_AGE or not js.healthy:
                    problems.append(f"{spec.name} stale/unhealthy")
                elif check_joint_limits(spec, js.sample, js.health).has_violation:
                    problems.append(f"{spec.name} limit")
            if problems:
                return seed, step, "G1 " + "; ".join(problems), log
        # F1
        if before.fault_latched and not after.fault_latched:
            problems = []
            if op != "checked":
                problems.append(f"cleared by {op}")
            if not ack_after_latch:
                problems.append("no ack after most recent latch")
            for s in specs:
                ts = cand[s.motor_id]
                ok = False
                if ts:
                    # contiguous chain with gaps <= MAX_AGE ending at last sample
                    start = ts[-1]
                    for a, b in zip(reversed(ts[:-1]), reversed(ts[1:])):
                        if b - a > MAX_AGE:
                            break
                        start = a
                    ok = Fraction(ts[-1]) - Fraction(start) >= Fraction(POLICY.recovery_dwell_s)
                if not ok:
                    problems.append(f"{s.name} dwell not met (shadow {ts[-3:]})")
            if problems:
                return seed, step, "F1 " + "; ".join(problems), log
        # S1
        if after.state is State.READY and before.state is not State.READY and op != "checked":
            return seed, step, f"S1 READY via {op}", log
        if after.state is State.ARMED and before.state is not State.ARMED and op != "arm":
            return seed, step, f"S1 ARMED via {op} from {before.state}", log
    return None


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    grants = 0
    found = 0
    for seed in range(n):
        res = run(seed)
        if res:
            found += 1
            seed, step, msg, log = res
            print(f"seed={seed} step={step}: {msg}")
            for row in log[-12:]:
                print("   ", row)
            if found >= 5:
                break
    print(f"seeds={n} violations={found} stats={STATS}")
