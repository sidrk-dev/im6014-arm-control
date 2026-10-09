"""R1: C14 continuous-effort protection never trips under sustained overload
when the measured effort occasionally dips to <= continuous_effort_nm, or when
the client cycles Stop -> check -> arm.

Fixture: continuous_effort_nm=4.0, continuous_dwell_s=0.5, max_effort_nm=8.0,
feedback_max_age_s=0.5, 100 Hz feedback.
"""
from arm3 import Clock, make_core, connect_check_arm, feed_all, health, JointSample, State


def run_dip_pattern(seconds=30.0, high=7.5, low=3.9, period=10):
    clock = Clock(0.0)
    core = make_core(clock)
    owner, d = connect_check_arm(core, clock)
    assert d.accepted, d
    dt = 0.01
    n = int(seconds / dt)
    above = 0
    sum_tau = 0.0
    sum_sq = 0.0
    max_overload = 0.0
    for k in range(n):
        clock.value = round((k + 1) * dt, 10)
        tau = low if (k % period) == period - 1 else high
        above += tau > 4.0
        sum_tau += tau
        sum_sq += tau * tau
        d0 = core.receive(0, JointSample(0.0, 0.0, tau), health(mode=1))
        core.receive(1, JointSample(0.0, 0.0, 0.0), health(mode=1))
        core.receive(2, JointSample(0.0, 0.0, 0.0), health(mode=1))
        core.tick()
        max_overload = max(max_overload, core._protection_states[0].overload_duration_s)
        if not d0.accepted:
            break
    snap = core.snapshot()
    print(f"[dip] samples={k + 1} sim_time={clock.value:.2f}s time_above_continuous={above * dt:.2f}s "
          f"mean_tau={sum_tau / (k + 1):.3f}Nm rms_tau={(sum_sq / (k + 1)) ** 0.5:.3f}Nm "
          f"(continuous=4.0, dwell=0.5s)")
    print(f"[dip] max accumulated overload_duration_s={max_overload:.3f} state={snap.state.name} "
          f"fault_latched={snap.fault_latched}")
    return snap.fault_latched


def run_noise(seconds=60.0, mean=4.4, spread=1.2):
    clock = Clock(0.0)
    core = make_core(clock)
    owner, d = connect_check_arm(core, clock)
    assert d.accepted, d
    seed = 12345
    dt = 0.01
    n = int(seconds / dt)
    sum_tau = 0.0
    above = 0
    for k in range(n):
        seed = (1103515245 * seed + 12345) % (1 << 31)
        u = seed / (1 << 31)  # deterministic LCG in [0,1)
        tau = mean + (2 * u - 1) * spread
        sum_tau += tau
        above += tau > 4.0
        clock.value = round((k + 1) * dt, 10)
        d0 = core.receive(0, JointSample(0.0, 0.0, tau), health(mode=1))
        core.receive(1, JointSample(0.0, 0.0, 0.0), health(mode=1))
        core.receive(2, JointSample(0.0, 0.0, 0.0), health(mode=1))
        if not d0.accepted:
            break
    snap = core.snapshot()
    print(f"[noise] samples={k + 1} sim_time={clock.value:.2f}s mean_tau={sum_tau / (k + 1):.3f}Nm "
          f"fraction_above_continuous={above / (k + 1):.2f} fault_latched={snap.fault_latched}")
    return snap.fault_latched


def run_stop_recheck_cycle(cycles=100, burst_s=0.45, tau=7.9):
    clock = Clock(0.0)
    core = make_core(clock)
    owner, d = connect_check_arm(core, clock)
    assert d.accepted, d
    dt = 0.01
    overload_time = 0.0
    for c in range(cycles):
        steps = int(round(burst_s / dt))
        for _ in range(steps):
            clock.value = round(clock.value + dt, 10)
            d0 = core.receive(0, JointSample(0.0, 0.0, tau), health(mode=1))
            core.receive(1, JointSample(0.0, 0.0, 0.0), health(mode=1))
            core.receive(2, JointSample(0.0, 0.0, 0.0), health(mode=1))
            if not d0.accepted:
                print(f"[cycle] tripped in cycle {c}")
                return True
            overload_time += dt
        # Stop, one Stop-mode packet per joint, check, arm: overload history is zeroed.
        assert core.stop().accepted
        owner = core.claim_owner("client")
        assert core.begin_check().accepted
        clock.value = round(clock.value + dt, 10)
        feed_all(core, tau=0.0)
        assert core.checked().accepted
        assert core.arm(owner).accepted
    snap = core.snapshot()
    print(f"[cycle] cycles={cycles} total time at {tau}Nm (~{tau / 4.0:.2f}x continuous)={overload_time:.1f}s "
          f"duty={overload_time / clock.value:.2f} state={snap.state.name} fault_latched={snap.fault_latched}")
    return snap.fault_latched


if __name__ == "__main__":
    a = run_dip_pattern()
    b = run_noise()
    c = run_stop_recheck_cycle()
    print("VIOLATION: sustained overload never latched continuous_effort_limit"
          if not (a or b or c) else "no violation")
