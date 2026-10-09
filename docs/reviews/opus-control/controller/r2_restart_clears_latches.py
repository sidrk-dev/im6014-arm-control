"""R2: C14 'persistent' latches and thermal hysteresis live only in one JointCore
instance. A fresh core (station restart, or the documented remedy for
generation/sequence exhaustion) arms a joint whose winding is still above its
reset threshold, and has no overload history.
"""
from arm3 import Clock, make_core, connect_check_arm, feed_all, health, JointSample, State

clock = Clock(0.0)
core_a = make_core(clock)
owner, d = connect_check_arm(core_a, clock)
assert d.accepted, d

# J2 winding reaches the trip threshold while ARMED -> latched.
clock.value = 0.05
r = core_a.receive(1, JointSample(0.0, 0.0, 0.0), health(mode=1, winding_c=105.0))
snap = core_a.snapshot()
print(f"core A: receive winding=105C -> {r.code.value}; state={snap.state.name} "
      f"winding_latch={snap.faults[0].winding_temperature_limit}")

# Same core: acknowledged recovery at 104C (above reset 90C) is refused, as designed.
core_a.stop()
owner = core_a.claim_owner("client")
assert core_a.begin_check(acknowledge_fault=True).accepted
for _ in range(5):
    clock.value += 0.1
    feed_all(core_a, winding_c=104.0)
print(f"core A: ack'd check at 104C for 0.5s -> checked={core_a.checked().code.value} "
      f"state={core_a.snapshot().state.name}")

# Fresh core for the same three motors (restart / new epoch).
clock2 = Clock(100.0)
core_b = make_core(clock2)
owner_b, d = connect_check_arm(core_b, clock2, winding_c=104.0)
snap_b = core_b.snapshot()
print(f"core B (fresh): check+arm with J1..J3 winding=104C (> reset 90C) -> arm={d.code.value} "
      f"state={snap_b.state.name} fault_latched={snap_b.fault_latched}")
if d.accepted:
    print("VIOLATION: thermal hysteresis/latch lost across core instances; arm accepted above reset threshold")
