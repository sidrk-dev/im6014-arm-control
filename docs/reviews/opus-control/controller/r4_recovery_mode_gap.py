"""R4 (test gap): the documented rule 'any non-Stop observation resets that joint's
recovery interval' (docs/joint-api.md) is enforced by joint_protection.py:206
(health.mode == 0) but no controller test detects its removal (mutant M6 in
mutants_output.txt survives all 112 tests). This script shows the mutant is
behaviourally distinct, i.e. the surviving mutant is a real coverage gap.
"""
from arm3 import Clock, make_core, connect_check_arm, feed_all, health, JointSample  # adds worktree to sys.path
import joint_protection
import joint_controller

ORIG = joint_protection.transition_protection


def mutant(spec, sample, h, state, received_at, max_age_s, recovery_authorized):
    # Same as original except a mode-1 packet counts as a recovery candidate.
    fake = type(h)(mode=0, errors=h.errors, warnings=h.warnings, timeout=h.timeout,
                   driver_c=h.driver_c, winding_c=h.winding_c)
    return ORIG(spec, sample, fake, state, received_at, max_age_s, recovery_authorized)


def scenario(fn):
    joint_controller.transition_protection = fn
    clock = Clock(0.0)
    core = make_core(clock)
    owner, d = connect_check_arm(core, clock)
    clock.value = 0.05
    core.receive(0, JointSample(0.0, 0.0, 0.0), health(mode=1, driver_c=86.0))  # latch driver temp
    core.stop()
    core.claim_owner("client")
    assert core.begin_check(acknowledge_fault=True).accepted
    # Joints keep reporting FOC mode (mode=1) for 0.3 s, then one Stop-mode packet.
    for t in (0.1, 0.2, 0.3, 0.4):
        clock.value = t
        feed_all(core, mode=1)
    clock.value = 0.45
    feed_all(core, mode=0)
    return core.checked().code.value


print("original transition_protection:", scenario(ORIG))
print("mutant (mode-1 counts toward recovery):", scenario(mutant))
joint_controller.transition_protection = ORIG
