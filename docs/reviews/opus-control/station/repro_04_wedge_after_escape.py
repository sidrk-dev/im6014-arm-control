"""An exception escaping StationStepCoordinator.step() between start_cycle and
end_cycle leaves the tracker cycle open forever; an interrupt inside bus.write
after claim_dispatch leaves JointCore._dispatch_claim set, which blocks
begin_check / begin_connect even after disconnect().
"""
import time

import common
from common import ScriptedBus, armed, submit
from joint_contract import SourceIdentity


def attempt(label, fn):
    try:
        r = fn()
        print(f"   {label}: OK -> {getattr(r, 'accepted', r)}")
    except BaseException as e:  # noqa: BLE001
        print(f"   {label}: {type(e).__name__}: {e}")


print("[A] one transient core clock fault inside prepare_refresh (core re-raises by design)")
state = {"bad": False}


def core_clock():
    return float("nan") if state["bad"] else time.monotonic()


bus = ScriptedBus()
core, owner, tracker, coord = armed(bus, core_clock, nominal=0.005)
coord._clock = time.monotonic  # independent healthy coordinator clock
submit(core, owner, 1)
state["bad"] = True
attempt("step() with clock fault", coord.step)
state["bad"] = False
attempt("step() after clock recovered", coord.step)
attempt("run_steps(1) after clock recovered", lambda: coord.run_steps(1))
print(f"   tracker._cycle_in_progress={tracker._cycle_in_progress}; only tracker.reset() (erases history) recovers")

print()
print("[B] KeyboardInterrupt raised by bus.write for J2 (operator Ctrl-C mid-transaction)")
bus = ScriptedBus()
core, owner, tracker, coord = armed(bus, time.monotonic, nominal=0.005)
submit(core, owner, 1)
orig_write = bus.write
calls = {"n": 0}


def write(data):
    calls["n"] += 1
    if calls["n"] == 2:
        raise KeyboardInterrupt
    return orig_write(data)


bus.write = write
attempt("step()", coord.step)
bus.write = orig_write
print(f"   core._dispatch_claim set: {core._dispatch_claim is not None}; core state={core.snapshot().state.value}")
attempt("core.stop()", core.stop)
attempt("core.begin_check(acknowledge_fault=True)", lambda: core.begin_check(acknowledge_fault=True))
attempt("core.disconnect()", core.disconnect)
attempt("core.begin_connect()", core.begin_connect)
attempt("coord.step()", coord.step)
attempt("coord.check_step() (Stop queries still possible)", lambda: [r.outcome for r in coord.check_step()])
