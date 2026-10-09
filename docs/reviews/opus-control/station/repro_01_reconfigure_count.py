"""Count pyserial 3.5 _reconfigure_port calls (and emulated termios/ioctl syscalls)
caused by im6014_serial.exchange. No device is opened: a serial.Serial subclass is
marked open with a fake fd and simulated motor I/O; termios/fcntl in
serial.serialposix are replaced by recorders for Part B only.
"""
import array
import sys
import time

import common  # noqa: F401  (sets sys.path)
import serial
import serial.serialposix as sp
import termios as real_termios

import im6014_serial as s
from im6014_codec import command
from benchmark_station_timing import make_feedback_frame


class SimMotorMixin:
    """Real-time motor model. reply_plan: list of (delay_after_write_s, nbytes) or None."""

    def sim_init(self, reply_plan, motor_id=0):
        self.reply_plan = reply_plan
        self.motor_id = motor_id
        self.arrivals = []  # (time, bytes)
        self.consumed = bytearray()
        self.events = []
        self.reads = 0
        self.fd = 1000
        self.is_open = True
        self.trace = None

    def _ev(self, ev):
        self.events.append(ev)
        if self.trace is not None and ev[0] in ("WRITE", "READ", "reset_input_buffer"):
            self.trace.append(ev[0] if ev[0] != "READ" else f"READ({ev[1]})")

    # --- simulated I/O (pyserial semantics for read timeout) ---
    def _arrived(self):
        now = time.monotonic()
        buf = bytearray()
        keep = []
        for t, chunk in self.arrivals:
            if t <= now:
                buf.extend(chunk)
            else:
                keep.append((t, chunk))
        self.arrivals = keep
        self.consumed.extend(buf)

    @property
    def in_waiting(self):
        self._arrived()
        self._ev(("in_waiting", len(self.consumed)))
        return len(self.consumed)

    def reset_input_buffer(self):
        self._arrived()
        self.consumed.clear()
        self._ev(("reset_input_buffer",))

    def write(self, data):
        t = time.monotonic()
        self._ev(("WRITE", len(data)))
        if self.reply_plan is not None:
            frame = make_feedback_frame(self.motor_id, mode=0)
            pos = 0
            for delay, n in self.reply_plan:
                self.arrivals.append((t + delay, frame[pos:pos + n]))
                pos += n
        return len(data)

    def read(self, size=1):
        self.reads += 1
        timeout = self._timeout
        end = None if timeout is None else time.monotonic() + timeout
        while True:
            self._arrived()
            if len(self.consumed) >= size or (end is not None and time.monotonic() >= end):
                out = bytes(self.consumed[:size])
                del self.consumed[:size]
                self._ev(("READ", size, len(out), timeout))
                return out
            time.sleep(0.00002)

    def close(self):
        self.is_open = False


class CountingSerial(SimMotorMixin, serial.Serial):
    def _reconfigure_port(self, force_update=False):
        self.events.append(("RECONFIGURE", self._timeout, self._write_timeout))


def make_counting(reply_plan, timeout, write_timeout):
    bus = CountingSerial()               # port=None -> not opened
    bus.baudrate = 6000000
    bus.exclusive = True
    bus.timeout = timeout
    bus.write_timeout = write_timeout
    bus.sim_init(reply_plan)
    return bus


def run_case(name, reply_plan, timeout, write_timeout, deadline):
    bus = make_counting(reply_plan, timeout, write_timeout)
    t0 = time.monotonic()
    try:
        r = s.exchange(bus, command(0), 0, deadline)
        outcome = "OK"
    except Exception as e:  # noqa: BLE001
        outcome = type(e).__name__
    el = time.monotonic() - t0
    n = sum(1 for e in bus.events if e[0] == "RECONFIGURE")
    seq = [e[0] if e[0] != "READ" else f"READ({e[1]})" for e in bus.events if e[0] != "in_waiting"]
    print(f"[{name}] outcome={outcome} elapsed={el*1000:.2f}ms reconfigure_calls={n} reads={bus.reads}")
    if n <= 12:
        print("    sequence:", " -> ".join(seq))
    bus.close()
    return n


print("python", sys.version.split()[0], "pyserial", serial.__version__, "platform", sys.platform)
print("== Part A: _reconfigure_port calls per exchange (pyserial setters are real) ==")
n_ready = run_case("S1 reply already buffered (benchmark fake model)", [(0.0, 26)], 0.005, 0.05, 0.05)
n_usb = run_case("S2 reply lands 150us after write in one USB packet", [(0.00015, 26)], 0.005, 0.05, 0.05)
n_frag = run_case("S3 reply in 2 USB packets (150us, 400us)", [(0.00015, 10), (0.0004, 16)], 0.005, 0.05, 0.05)
n_miss = run_case("S4 missing motor, port timeout=0.005 (im6014_port default)", None, 0.005, 0.05, 0.05)
n_miss0 = run_case("S5 missing motor, original timeout=0 (tested in test_serial_deadline)", None, 0, 0, 0.02)
n_missN = run_case("S6 missing motor, original timeout=None", None, None, None, 0.05)
print(f"3-joint cycle, all replies like S2: {3 * n_usb} reconfigurations; "
      f"with one missing motor in check_step: {2 * n_usb + n_miss}")

print()
print("== Part B: real serialposix._reconfigure_port with recorded termios/fcntl (darwin branch) ==")


class FakeTermios:
    error = real_termios.error

    def __init__(self, log, report_actual_speed):
        self.log = log
        self.report_actual = report_actual_speed
        cc = [b"\x00"] * real_termios.NCCS
        self.attrs = [0, 0, real_termios.CS8 | real_termios.CREAD | real_termios.CLOCAL, 0,
                      6000000 if report_actual_speed else real_termios.B38400,
                      6000000 if report_actual_speed else real_termios.B38400, cc]

    def __getattr__(self, name):
        return getattr(real_termios, name)

    def tcgetattr(self, fd):
        self.log.append("tcgetattr")
        a = list(self.attrs)
        a[6] = list(self.attrs[6])
        return a

    def tcsetattr(self, fd, when, attrs):
        self.log.append(f"tcsetattr(speed={attrs[4]})")
        self.attrs = [*attrs[:6], list(attrs[6])]

    def tcflush(self, fd, q):
        self.log.append("tcflush")


class FakeFcntl:
    def __init__(self, log, termios_obj):
        self.log = log
        self.t = termios_obj
        self.LOCK_EX = 2
        self.LOCK_NB = 4
        self.LOCK_UN = 8

    def flock(self, fd, op):
        self.log.append("flock")

    def ioctl(self, fd, req, buf=0, mutate=True):
        if req == sp.IOSSIOSPEED:
            speed = buf[0]
            self.log.append(f"ioctl(IOSSIOSPEED,{speed})")
            if self.t.report_actual:
                self.t.attrs[4] = self.t.attrs[5] = speed
            return 0
        self.log.append(f"ioctl({req:#x})")
        return 0


class TracedSerial(SimMotorMixin, sp.Serial):
    pass


for report_actual in (True, False):
    log = []
    ft = FakeTermios(log, report_actual)
    ff = FakeFcntl(log, ft)
    saved = (sp.termios, sp.fcntl)
    sp.termios, sp.fcntl = ft, ff
    bus = TracedSerial()
    bus.baudrate = 6000000
    bus.exclusive = True
    bus.timeout = 0.005
    bus.write_timeout = 0.05
    bus.sim_init([(0.00015, 26)])
    bus.trace = log
    log.clear()
    try:
        s.exchange(bus, command(0), 0, 0.05)
        outcome = "OK"
    except Exception as e:  # noqa: BLE001
        outcome = type(e).__name__
    finally:
        sp.termios, sp.fcntl = saved
        bus.is_open = False
    tcset = [x for x in log if x.startswith("tcsetattr")]
    ioss = [x for x in log if x.startswith("ioctl(IOSSIOSPEED")]
    label = "tcgetattr reports actual 6 Mbaud after IOSSIOSPEED" if report_actual else "tcgetattr keeps reporting B38400"
    print(f"[{label}] outcome={outcome} syscalls={len([x for x in log if x not in ('WRITE','reset_input_buffer') and not x.startswith('READ')])} "
          f"tcsetattr={len(tcset)} {sorted(set(tcset))} IOSSIOSPEED={len(ioss)} flock={log.count('flock')}")
    print("    ", " -> ".join(log))
