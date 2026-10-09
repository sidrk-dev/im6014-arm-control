"""(A) exchange() computes bus.write_timeout from the remaining budget BEFORE calling
before_write(); time spent in the hook is not deducted, so a blocking write can end
after the transaction deadline. (B) pyserial 3.5 raises SerialTimeoutException
after os.write() already queued every byte when the write timeout expires during
the call; exchange() then reports WriteResult.FAILED for transmitted bytes.
(B) uses an isolated macOS PTY at a standard baud (115200); no device.
"""
import os
import pty
import time

import common  # noqa: F401
import serial
import im6014_serial as s
from im6014_codec import command
from joint_contract import WriteResult


class BlockingWriteBus:
    """Output saturated: write blocks for the configured write_timeout, then times out."""

    def __init__(self):
        self.timeout = 0.005
        self.write_timeout = 0.05
        self.in_waiting = 0
        self.seen_write_timeout = None

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.seen_write_timeout = self.write_timeout
        time.sleep(self.write_timeout)
        raise serial.SerialTimeoutException("Write timeout")

    def read(self, size):
        return b""


print("[A] deadline 50 ms, before_write hook takes 20 ms, saturated output")
bus = BlockingWriteBus()
results = []
t0 = time.monotonic()
try:
    s.exchange(bus, command(0), 0, 0.05, before_write=lambda: (time.sleep(0.020), True)[1],
               on_write=results.append)
except Exception as e:  # noqa: BLE001
    el = time.monotonic() - t0
    print(f"   raised {type(e).__name__} after {el * 1e3:.1f} ms (deadline 50 ms); "
          f"write_timeout used={bus.seen_write_timeout * 1e3:.1f} ms; on_write={[r.value for r in results]}")

print()
print("[B] PTY (115200): exchange with original write_timeout=1e-6 s; 20-byte request")
master, slave = pty.openpty()
bus = serial.Serial(os.ttyname(slave), baudrate=115200, timeout=0.005, write_timeout=1e-6)
results = []
try:
    s.exchange(bus, command(0), 0, 0.02, on_write=results.append)
except Exception as e:  # noqa: BLE001
    print(f"   exchange raised {type(e).__name__}: {e}; on_write={[r.value for r in results]}")
finally:
    bus.close()
got = os.read(master, 64)
print(f"   bytes that reached the PTY master: {len(got)}; equals request: {got == command(0)}")
os.close(master)
os.close(slave)
