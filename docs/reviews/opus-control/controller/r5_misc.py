"""R5 misc low-severity checks.

(a) ProtectionPolicy accepts trip thresholds the wire telemetry can never reach
    (feedback driver_c is int8 <= 127, winding_c is uint8 <= 255 per
    im6014_codec.feedback struct '<BbBB...'), silently disabling that trip.
(b) arm() accepts a READY core whose Stop anchor is already older than
    max_reference_gap_s; the first prepare_refresh() then Stops (no fault),
    so the operator must re-check although feedback stayed fresh throughout.
"""
import struct
from arm3 import Clock, make_core, connect_check_arm, feed_all, JointTarget, Target
from joint_protection import ProtectionPolicy
import im6014_codec

# (a)
p = ProtectionPolicy(driver_trip_c=130.0, driver_reset_c=110.0, winding_trip_c=300.0,
                     winding_reset_c=280.0, continuous_effort_nm=1.0, continuous_dwell_s=1.0,
                     recovery_dwell_s=1.0)
body = bytes([0x00]) + struct.pack('<bBB', 127, 255, 48) + struct.pack('<hhiIHBB', 0, 0, 0, 0, 0, 0, 0)
raw = b'\xfc\xee' + body
raw += struct.pack('<I', im6014_codec.crc32(raw[2:22]))
fb = im6014_codec.feedback(raw, 0)
print(f"(a) policy accepted: driver_trip_c={p.driver_trip_c} winding_trip_c={p.winding_trip_c}; "
      f"max decodable telemetry driver_c={fb['driver_c']} winding_c={fb['winding_c']} -> trips unreachable")

# (b)
clock = Clock(0.0)
core = make_core(clock, max_reference_gap_s=0.5)
owner, d = connect_check_arm(core, clock)
assert d.accepted
for _ in range(6):                       # 0.6 s of fresh Stop-mode feedback while ARMED
    clock.value += 0.1
    feed_all(core, mode=0)
w = core.issue_window(1.0)
t = Target(1, tuple(JointTarget(0.0, 0.0, 0.0, 2.0, 0.1) for _ in range(3)), window=w, owner=owner)
print(f"(b) submit after 0.6 s ARMED with fresh feedback: {core.submit_target(t).code.value}")
plan = core.prepare_refresh()
s = core.snapshot()
print(f"(b) first prepare_refresh -> plan={plan} state={s.state.name} fault_latched={s.fault_latched} "
      f"current_owner={s.current_owner}")
