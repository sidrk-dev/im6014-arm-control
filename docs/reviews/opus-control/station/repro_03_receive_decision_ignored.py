"""StationStepCoordinator.step ignores the Decision returned by JointCore.receive.
A reply that latches a fault (motor error flag, out-of-limit position) is recorded
as outcome OK; the failure is attributed to the next joint (WRITE_CANCELLED) or,
for the last joint, does not appear in the transaction history at all.
"""
import time

import common
from common import ScriptedBus, armed, submit, make_feedback_frame


def run(label, replies, nominal=0.005):
    bus = ScriptedBus(replies)
    core, owner, tracker, coord = armed(bus, time.monotonic, nominal=nominal)
    submit(core, owner, 1)
    rec = coord.step()
    snap = core.snapshot()
    c = tracker.snapshot().counters
    print(f"[{label}]")
    for tx in rec.transactions:
        print(f"   {tx.joint_name}: outcome={tx.outcome} write={tx.write_result and tx.write_result.value} "
              f"feedback_received={tx.feedback_received}")
    print(f"   tracker counters: transactions={c.transactions} successful={c.successful_transactions} "
          f"failed={c.failed_transactions} write_cancelled={c.write_cancelled_transactions}")
    print(f"   core state={snap.state.value} faults=" + "; ".join(
        f"{f.name}(errors={f.errors}, position_limit={f.position_limit})" for f in snap.faults))
    submit_ok = None
    try:
        submit(core, owner, 2)
        submit_ok = True
    except AssertionError:
        submit_ok = False
    rec2 = coord.step()
    print(f"   next cycle: transactions={len(rec2.transactions)} (idle) submit_target_accepted={submit_ok}")


chk = lambda m: make_feedback_frame(m, mode=0)  # noqa: E731  reply consumed by arming check_step
run("J3 reply carries motor error flag errors=0x4", {2: [chk(2), make_feedback_frame(2, mode=1, errors=0x4)]})
run("J1 reply carries motor error flag errors=0x4", {0: [chk(0), make_feedback_frame(0, mode=1, errors=0x4)]})
run("J1 reply position 3.5 rad > q_max 3.14", {0: [chk(0), make_feedback_frame(0, mode=1, position_rad=3.5)]})
