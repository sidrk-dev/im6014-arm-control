"""R8 (informational): wall-clock cost of prepare_refresh + 3 claims on this host."""
import time
from harness import Rig, spec, JointTarget
from joint_contract import WriteResult

specs = tuple(spec(f"J{i}", i, q_min=-1.5, q_max=1.5, v_max=1.0, a_max=10.0, lead=0.5) for i in range(3))
rig = Rig(specs, (0.0, 0.0, 0.0))
goals = tuple(JointTarget(0.4, 0.0, 0.0, 5.0, 0.2) for _ in range(3))
rig.submit(goals)
prep, claim = [], []
for k in range(30):
    rig.clock.value += 0.01
    rig.feed()
    t0 = time.perf_counter()
    plan = rig.core.prepare_refresh()
    t1 = time.perf_counter()
    for i in range(3):
        rig.core.claim_dispatch(plan.plan_id, i)
        rig.core.dispatch_result(plan.plan_id, i, WriteResult.COMPLETE)
    t2 = time.perf_counter()
    prep.append(t1 - t0)
    claim.append(t2 - t1)
    rig.q_meas = [r.q_rad for r in rig.snap().confirmed_references]
prep.sort(); claim.sort()
print(f"prepare_refresh (3 joints): median {prep[15]*1e3:.2f} ms, max {prep[-1]*1e3:.2f} ms")
print(f"3x claim+dispatch_result: median {claim[15]*1e3:.2f} ms, max {claim[-1]*1e3:.2f} ms")
