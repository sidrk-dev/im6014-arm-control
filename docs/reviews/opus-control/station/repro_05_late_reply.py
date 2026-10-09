"""After a TIMEOUT, the next exchange writes immediately after reset_input_buffer.
A reply that misses the deadline but lands after that reset is (i) accepted as the
answer to a newer request to the same motor, (ii) on a shared bus, read during the
next motor's transaction and turns that motor's silence into INVALID_FEEDBACK.
Real-clock in-memory bus; no device.
"""
import time

import common
from common import make_feedback_frame
import im6014_serial as s
from im6014_codec import command
from joint_contract import SourceIdentity
from joint_controller import JointCore
from station_step import StationStepCoordinator
from station_timing import StationTimingTracker


class TimedBus:
    """Bytes arrive at scheduled monotonic times; reset discards only arrived bytes.
    Each motor answers its requests in order (one reply at a time)."""

    def __init__(self, latency_by_motor_and_request):
        self.plan = latency_by_motor_and_request  # {motor: [latency or None, ...]}
        self.count = {}
        self.busy_until = {}
        self.pending = []  # (t, bytes)
        self.buf = bytearray()
        self.timeout = 0.005
        self.write_timeout = 0.05
        self.log = []

    def _pump(self):
        now = time.monotonic()
        keep = []
        for t, b in sorted(self.pending):
            if t <= now:
                self.buf.extend(b)
            else:
                keep.append((t, b))
        self.pending = keep

    @property
    def in_waiting(self):
        self._pump()
        return len(self.buf)

    def reset_input_buffer(self):
        self._pump()
        self.buf.clear()

    def write(self, data):
        now = time.monotonic()
        m = data[2] & 0x0F
        k = self.count.get(m, 0)
        self.count[m] = k + 1
        lat = self.plan.get(m, [])[k] if k < len(self.plan.get(m, [])) else 0.0002
        if lat is not None:
            t = max(now + lat, self.busy_until.get(m, 0.0))
            self.busy_until[m] = t + 0.00005
            # position encodes which request this reply answers: 0.1 * (k + 1) rad
            frame = make_feedback_frame(m, mode=(data[2] >> 4) & 7, position_rad=0.1 * (k + 1))
            self.pending.append((t, frame))
            self.log.append(f"motor {m} request #{k + 1} -> reply scheduled +{lat * 1e3:.1f} ms")
        else:
            self.log.append(f"motor {m} request #{k + 1} -> no reply")
        return len(data)

    def read(self, size):
        end = None if self.timeout is None else time.monotonic() + self.timeout
        while True:
            self._pump()
            if len(self.buf) >= size or (end is not None and time.monotonic() >= end):
                out = bytes(self.buf[:size])
                del self.buf[:size]
                return out
            time.sleep(0.00005)


print("[i] same motor, request #1 answered 3 ms after its 50 ms deadline")
bus = TimedBus({0: [0.053, 0.0002, 0.0002]})
for k in (1, 2):
    t0 = time.monotonic()
    try:
        r = s.exchange(bus, command(0), 0, 0.05)
        print(f"   exchange #{k}: OK in {(time.monotonic() - t0) * 1e3:.1f} ms, reply position_rad={r['position_rad']:.4f} "
              f"(request #1 marker 0.1, request #2 marker 0.2)")
    except Exception as e:  # noqa: BLE001
        print(f"   exchange #{k}: {type(e).__name__} after {(time.monotonic() - t0) * 1e3:.1f} ms")
print("   bus log:", bus.log)

print()
print("[ii] 3 motors in check_step: J1 answers 3 ms late, J2 silent (its request collided on the wire)")
bus = TimedBus({0: [0.053], 1: [None], 2: [0.0002]})
core = JointCore(common.three_joint_specs(), clock=time.monotonic, max_reference_gap_s=1.0)
core.begin_connect()
core.connected(SourceIdentity("fixture", "review", 1))
core.begin_check()
coord = StationStepCoordinator(core, bus, StationTimingTracker(0.005), clock=time.monotonic)
recs = coord.check_step()
for r in recs:
    print(f"   {r.joint_name}: outcome={r.outcome} duration={r.duration_s * 1e3:.1f} ms")
snap = core.snapshot()
print("   core faults:", "; ".join(f"{f.name}:{[c.value for c in f.communication_errors]}" for f in snap.faults))
print(f"   total check_step time {(recs[-1].ended_s - recs[0].started_s) * 1e3:.1f} ms")
