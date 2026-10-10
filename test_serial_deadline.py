import os
import pty
import select
import threading
import time
import unittest
from unittest.mock import patch

import serial
import spin_5rpm as s
import im6014_serial as transport
from im6014_transport import PartialWriteTimeout, PosixDeadlineSerial
from fake_serial import FakeClock, FakeChunk, FakeSerial
from test_protocol import CAPTURED


class CountingSerial(serial.Serial):
    def __init__(self, *args, **kwargs):
        self.reconfigure_calls = 0
        super().__init__(*args, **kwargs)

    def _reconfigure_port(self, *args, **kwargs):
        self.reconfigure_calls += 1
        return super()._reconfigure_port(*args, **kwargs)


class B09Deadline(unittest.TestCase):
    def test_read_timeout_cannot_overrun_budget_and_restores(self):
        for original in (None, 0, .5):
            clock = FakeClock()
            bus = FakeSerial(clock=clock, timeout=original, write_timeout=.5)
            with self.subTest(timeout=original), patch.object(transport, "_monotonic", clock.monotonic):
                with self.assertRaisesRegex(RuntimeError, "deadline"):
                    s.exchange(bus, s.command(0), 0, .02)
            self.assertLessEqual(clock.monotonic(), .020001)
            self.assertEqual((bus.timeout, bus.write_timeout), (original, .5))
            self.assertEqual(len(bus.writes), 1)

    def test_reset_and_write_share_entry_budget(self):
        clock = FakeClock()
        bus = FakeSerial(clock=clock, timeout=.5, write_timeout=.5)
        bus.reset_delay = .03
        with patch.object(transport, "_monotonic", clock.monotonic):
            with self.assertRaisesRegex(RuntimeError, "deadline"):
                s.exchange(bus, s.command(0), 0, .02)
        self.assertEqual(len(bus.writes), 0)
        self.assertEqual((bus.timeout, bus.write_timeout), (.5, .5))
        clock = FakeClock()
        bus = FakeSerial(clock=clock, timeout=.5, write_timeout=.5)
        bus.reset_delay = .005
        bus.write_delay = .04
        with patch.object(transport, "_monotonic", clock.monotonic):
            with self.assertRaises((RuntimeError, TimeoutError)):
                s.exchange(bus, s.command(0), 0, .02)
        self.assertLessEqual(clock.monotonic(), .020001)
        self.assertEqual((bus.timeout, bus.write_timeout), (.5, .5))
        self.assertEqual(len(bus.writes), 1)

    def test_late_frame_is_not_success(self):
        clock = FakeClock()
        bus = FakeSerial(responses=[(FakeChunk(CAPTURED, .021),)], clock=clock,
                         timeout=None, write_timeout=None)
        with patch.object(transport, "_monotonic", clock.monotonic):
            with self.assertRaisesRegex(RuntimeError, "deadline"):
                s.exchange(bus, s.command(0), 0, .02)
        self.assertEqual((bus.timeout, bus.write_timeout), (None, None))
        self.assertLessEqual(clock.monotonic(), .020001)

    def test_errors_partial_write_and_success_restore_settings(self):
        for operation in ("read", "write", "reset", "partial", "success"):
            clock = FakeClock()
            bus = FakeSerial(responses=[(CAPTURED,)], clock=clock,
                             timeout=.5, write_timeout=0)
            if operation in ("read", "write", "reset"):
                setattr(bus, operation + "_error", OSError(operation))
            elif operation == "partial":
                bus.write_result = 19
            write_results = []
            with self.subTest(operation=operation), patch.object(transport, "_monotonic", clock.monotonic):
                if operation == "success":
                    self.assertEqual(s.exchange(bus, s.command(0), 0,
                                                on_write=write_results.append)["raw"], CAPTURED.hex())
                else:
                    with self.assertRaises((RuntimeError, OSError)):
                        s.exchange(bus, s.command(0), 0, on_write=write_results.append)
            expected_results = ([] if operation == "reset" else
                                [transport.WriteResult.PARTIAL] if operation == "partial" else
                                [transport.WriteResult.FAILED] if operation == "write" else
                                [transport.WriteResult.COMPLETE])
            self.assertEqual(write_results, expected_results)
            self.assertEqual((bus.timeout, bus.write_timeout), (.5, 0))


    def test_settings_failure_is_transport_fault_and_restores_both(self):
        class SetterBus(FakeSerial):
            def __setattr__(self, name, value):
                if getattr(self, "reject", False) and name in ("timeout", "write_timeout"):
                    self.attempts.append((name, value))
                    if name == "timeout" and (value == .5 if self.restore_failure else value != .5):
                        raise ValueError("timeout setter rejected")
                super().__setattr__(name, value)
        for restore_failure in (False, True):
            clock = FakeClock()
            bus = SetterBus(responses=[(CAPTURED,)], clock=clock, timeout=.5, write_timeout=.5)
            bus.attempts = []
            bus.restore_failure = restore_failure
            bus.reject = True
            with self.subTest(restore_failure=restore_failure), patch.object(transport, "_monotonic", clock.monotonic):
                with self.assertRaises(RuntimeError):
                    s.exchange(bus, s.command(0), 0, .02)
            self.assertIn(("timeout", .5), bus.attempts)
            self.assertIn(("write_timeout", .5), bus.attempts)
            self.assertEqual(bus.write_timeout, .5)

    def test_restore_failure_preserves_primary_fault_and_no_result(self):
        class RestoreBus(FakeSerial):
            def __setattr__(self, name, value):
                if getattr(self, "reject_restore", False) and name == "timeout" and value == .5:
                    raise ValueError("cannot restore")
                super().__setattr__(name, value)
        clock = FakeClock()
        bus = RestoreBus(clock=clock, timeout=.5, write_timeout=.5, read_error=OSError("primary read fault"))
        bus.reject_restore = True
        with patch.object(transport, "_monotonic", clock.monotonic):
            with self.assertRaisesRegex(RuntimeError, "restore.*primary read fault"):
                s.exchange(bus, s.command(0), 0, .02)
        self.assertEqual(bus.write_timeout, .5)

    def test_restoration_and_decode_time_cannot_return_late_success(self):
        class SlowRestoreBus(FakeSerial):
            def __setattr__(self, name, value):
                if getattr(self, "slow_restore", False) and name == "timeout" and value == .5:
                    self.clock.advance(.021)
                super().__setattr__(name, value)
        clock = FakeClock()
        bus = SlowRestoreBus(responses=[(CAPTURED,)], clock=clock, timeout=.5, write_timeout=.5)
        bus.slow_restore = True
        with patch.object(transport, "_monotonic", clock.monotonic):
            with self.assertRaisesRegex(RuntimeError, "deadline"):
                s.exchange(bus, s.command(0), 0, .02)
        self.assertEqual((bus.timeout, bus.write_timeout), (.5, .5))
        clock = FakeClock()
        bus = FakeSerial(responses=[(CAPTURED,)], clock=clock)
        decode = transport.feedback
        def slow_decode(*args):
            clock.advance(.021)
            return decode(*args)
        with patch.object(transport, "_monotonic", clock.monotonic), patch.object(transport, "feedback", slow_decode):
            with self.assertRaisesRegex(RuntimeError, "deadline"):
                s.exchange(bus, s.command(0), 0, .02)

    def test_zero_infinite_write_settings_are_bounded_and_invalid_deadlines_do_no_io(self):
        for original in (None, 0, .5):
            clock = FakeClock()
            bus = FakeSerial(responses=[(CAPTURED,)], clock=clock, write_timeout=original)
            write = bus.write
            def checked_write(packet):
                self.assertGreater(bus.write_timeout, 0)
                self.assertLessEqual(bus.write_timeout, .02)
                return write(packet)
            bus.write = checked_write
            with self.subTest(original=original), patch.object(transport, "_monotonic", clock.monotonic):
                s.exchange(bus, s.command(0), 0, .02)
            self.assertEqual(bus.write_timeout, original)
        for deadline in (0, -.1, float("nan"), float("inf")):
            bus = FakeSerial()
            with self.subTest(deadline=deadline), self.assertRaises(ValueError):
                s.exchange(bus, s.command(0), 0, deadline)
            self.assertEqual(bus.writes, ())

    def test_prewrite_hook_consumes_the_budget_without_transmitting(self):
        clock = FakeClock()
        bus = FakeSerial(clock=clock, timeout=.5, write_timeout=.5)
        write_results = []

        def late_but_allowed():
            clock.advance(.021)
            return True

        with patch.object(transport, "_monotonic", clock.monotonic):
            with self.assertRaisesRegex(RuntimeError, "deadline"):
                s.exchange(bus, s.command(0), 0, .02,
                           before_write=late_but_allowed, on_write=write_results.append)
        self.assertEqual(write_results, [transport.WriteResult.PREWRITE_DEADLINE])
        self.assertEqual(bus.writes, ())
        self.assertEqual((bus.timeout, bus.write_timeout), (.5, .5))

    def test_cancelled_prewrite_gate_does_not_write_or_claim_a_result(self):
        bus = FakeSerial()
        write_results = []
        with self.assertRaises(transport.WriteCancelledError):
            s.exchange(bus, s.command(0), 0, before_write=lambda: False,
                       on_write=write_results.append)
        self.assertEqual(bus.writes, ())
        self.assertEqual(write_results, [])


class B09PseudoTerminal(unittest.TestCase):
    def open_bus(self, timeout, write_timeout):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        physical = CountingSerial(os.ttyname(slave), baudrate=6000000,
                                  timeout=timeout, write_timeout=write_timeout, exclusive=True)
        bus = PosixDeadlineSerial(physical)
        self.addCleanup(bus.close)
        return master, bus

    def test_timeout_setters_require_finite_nonnegative_values(self):
        _, bus = self.open_bus(None, None)
        for name in ('timeout', 'write_timeout'):
            for value in (-.1, float('nan'), float('inf'), float('-inf')):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    setattr(bus, name, value)
        bus.timeout = 0
        bus.write_timeout = .25
        self.assertEqual((bus.timeout, bus.write_timeout), (0, .25))

    def test_mocked_read_and_partial_write_stop_at_post_select_cutoff(self):
        master, bus = self.open_bus(None, None)
        clock = FakeClock()
        fd = bus._serial.fileno()

        def late_read_select(readers, writers, errors, timeout):
            self.assertEqual((readers, writers, errors), ([fd], [], []))
            self.assertEqual(timeout, .01)
            clock.advance(.011)
            return [fd], [], []

        with patch('im6014_transport.select.select', side_effect=late_read_select), \
             patch('im6014_transport.os.read') as read:
            self.assertEqual(bus.read_with_deadline(1, 1.0, clock.monotonic, .01), b'')
        read.assert_not_called()

        clock = FakeClock()
        select_calls = []
        writes = []

        def raced_write_select(readers, writers, errors, timeout):
            self.assertEqual((readers, errors), ([], []))
            select_calls.append(timeout)
            clock.advance(.009 if len(select_calls) == 1 else .012)
            return [], writers, []

        def partial_os_write(descriptor, payload):
            writes.append(bytes(payload))
            return 2

        with patch('im6014_transport.select.select', side_effect=raced_write_select), \
             patch('im6014_transport.os.write', side_effect=partial_os_write):
            with self.assertRaises(PartialWriteTimeout) as caught:
                bus.write_with_deadline(b'abcdef', .02, clock.monotonic)
        self.assertEqual(caught.exception.bytes_written, 2)
        self.assertEqual(len(writes), 1)
        self.assertEqual(len(select_calls), 2)

    def test_repeated_real_exchanges_do_not_reconfigure_open_port(self):
        master, bus = self.open_bus(None, None)
        reconfigures_after_open = bus._serial.reconfigure_calls
        requests = []
        failures = []

        def motor_reply():
            try:
                request = bytearray()
                until = time.monotonic() + .2
                while len(request) < 20 and time.monotonic() < until:
                    ready, _, _ = select.select(
                        [master], [], [], max(0, until - time.monotonic()))
                    if ready:
                        request.extend(os.read(master, 20 - len(request)))
                requests.append(bytes(request))
                if len(request) != 20:
                    return
                os.write(master, CAPTURED)
            except BaseException as error:
                failures.append(error)

        for _ in range(3):
            thread = threading.Thread(target=motor_reply)
            thread.start()
            self.assertEqual(s.exchange(bus, s.command(0), 0)["raw"], CAPTURED.hex())
            thread.join(.5)
            self.assertFalse(thread.is_alive())
            self.assertEqual(bus._serial.reconfigure_calls, reconfigures_after_open)

        self.assertEqual(failures, [])
        self.assertEqual(requests, [s.command(0)] * 3)

    def test_direct_zero_timeout_read_and_write_poll_once(self):
        master, bus = self.open_bus(0, 0)
        incoming = b'zero-read'
        os.write(master, incoming)
        self.assertEqual(bus.read(len(incoming)), incoming)
        self.assertEqual(bus.read(1), b'')

        outgoing = b'zero-write'
        self.assertEqual(bus.write(outgoing), len(outgoing))
        self.assertEqual(os.read(master, len(outgoing)), outgoing)

    def test_real_missing_reply_five_runs_per_timeout(self):
        for original in (None, 0, .5):
            elapsed = []
            for repeat in range(5):
                master, bus = self.open_bus(original, original)
                start = time.monotonic()
                with self.subTest(timeout=original, repeat=repeat):
                    with self.assertRaisesRegex(RuntimeError, "deadline"):
                        s.exchange(bus, s.command(0), 0, .02)
                    actual = time.monotonic() - start
                    elapsed.append(actual)
                    self.assertGreaterEqual(actual, .018)
                    self.assertLess(actual, .07)
                    self.assertEqual((bus.timeout, bus.write_timeout), (original, original))
                    self.assertEqual(os.read(master, 20), s.command(0))
            print(f"PTY missing reply timeout={original}: elapsed seconds {elapsed}", flush=True)

    def test_real_fragmented_reply_after_exact_request(self):
        master, bus = self.open_bus(None, None)
        errors = []
        requests = []
        def reply():
            try:
                request = b""
                until = time.monotonic() + .2
                while len(request) < 20 and time.monotonic() < until:
                    ready, _, _ = select.select([master], [], [], max(0, until - time.monotonic()))
                    if ready:
                        request += os.read(master, 20 - len(request))
                requests.append(request)
                for byte in CAPTURED:
                    os.write(master, bytes([byte]))
                    time.sleep(.0002)
            except BaseException as error:
                errors.append(error)
        thread = threading.Thread(target=reply)
        thread.start()
        try:
            self.assertEqual(s.exchange(bus, s.command(0), 0, .1)["raw"], CAPTURED.hex())
        finally:
            thread.join(.5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(requests, [s.command(0)])
        self.assertEqual((bus.timeout, bus.write_timeout), (None, None))

    def test_real_saturated_write_has_positive_deadline(self):
        for original in (None, 0, .5):
            _, bus = self.open_bus(.5, original)
            start = time.monotonic()
            with self.subTest(write_timeout=original), self.assertRaises((TimeoutError, RuntimeError)):
                s.exchange(bus, b"x" * 1000000, 0, .02)
            elapsed = time.monotonic() - start
            self.assertLess(elapsed, .07)
            self.assertEqual((bus.timeout, bus.write_timeout), (.5, original))
            print(f"PTY saturated write timeout={original}: elapsed seconds {elapsed}", flush=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
