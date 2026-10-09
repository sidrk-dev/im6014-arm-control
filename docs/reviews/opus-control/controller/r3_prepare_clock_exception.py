"""R3: prepare_refresh() stops/revokes on a clock ValueError but not on any other
exception raised by the injected clock; after the clock is 'repaired' motion
resumes with the same owner/goal, without Stop or acknowledged recovery.
claim_dispatch() uses `except Exception` for the same situation.
"""
from arm3 import Clock, make_core, connect_check_arm, feed_all, JointTarget, Target, WriteResult


class FlakyClock(Clock):
    def __init__(self, value=0.0):
        super().__init__(value)
        self.fail_with = None

    def __call__(self):
        if self.fail_with is not None:
            raise self.fail_with
        return self.value


def moving_core():
    clock = FlakyClock(0.0)
    core = make_core(clock)
    owner, d = connect_check_arm(core, clock)
    assert d.accepted, d
    w = core.issue_window(1.0)
    t = Target(1, tuple(JointTarget(0.01, 0.0, 0.0, 2.0, 0.1) for _ in range(3)), window=w, owner=owner)
    assert core.submit_target(t).accepted
    clock.value += 0.01
    feed_all(core, mode=1)
    plan = core.prepare_refresh()
    for i in range(3):
        assert core.claim_dispatch(plan.plan_id, i)
        core.dispatch_result(plan.plan_id, i, WriteResult.COMPLETE)
    return core, clock


for exc in (ValueError("clock source failed"), OSError("clock source failed")):
    core, clock = moving_core()
    clock.value += 0.01
    feed_all(core, mode=1)
    clock.fail_with = exc
    try:
        core.prepare_refresh()
    except Exception as e:  # noqa: BLE001
        raised = type(e).__name__
    snap = core.snapshot.__self__  # keep reference
    clock.fail_with = None
    s = core.snapshot()
    print(f"{type(exc).__name__:10s} in prepare_refresh -> raised {raised}; after repair: state={s.state.name} "
          f"current_owner={'kept' if s.current_owner else 'None'} admitted_goal={'kept' if s.admitted_target else 'None'}")
    clock.value += 0.01
    feed_all(core, mode=1)
    plan = core.prepare_refresh()
    granted = plan is not None and core.claim_dispatch(plan.plan_id, 0)
    print(f"{'':10s}   next prepare_refresh plan={'yes' if plan else 'None'} claim_dispatch(0)={granted}")
