import http.client
import json
import threading
import time
import unittest
from unittest.mock import patch

import arm_manual
from arm_dashboard import ArmDashboardHandler, ArmDashboardServer, TOP_LEVEL_FIELDS, _project_fields
from arm_manual import ArmManualSession


class TimingClock:
    def __init__(self):
        self.now = 0
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            return self.now

    def advance(self, value):
        with self.lock:
            self.now += value


class ArmTimingTests(unittest.TestCase):
    def action(self, arm, sid, action, values=None):
        op = arm.request(sid, action, values or {})
        until = time.monotonic() + 2
        while time.monotonic() < until:
            receipt = arm.receipt(sid, op)
            if receipt['status'] != 'pending':
                return receipt
            time.sleep(.005)
        self.fail('fake-owner operation did not settle')

    def test_actual_owner_measures_successful_demo_exchanges(self):
        clock = TimingClock()
        original = arm_manual.capture_exchange
        def measured(*args, **kwargs):
            clock.advance(2_000_000)
            return original(*args, **kwargs)
        with patch('arm_manual.capture_exchange', measured):
            arm = ArmManualSession(physical_opt_in=False, timing_clock_ns=clock)
            try:
                sid = arm.create_session()
                receipt = self.action(arm, sid, 'connect', {'demo': True})
                self.assertEqual(receipt['status'], 'complete', receipt)
                data = arm.snapshot()
                timing = data['timing']
                self.assertEqual(timing['clock_source'], 'injected_clock')
                self.assertGreater(timing['transaction_samples'], 0)
                self.assertEqual(timing['transaction_failures'], 0)
                self.assertEqual(timing['transaction_latency_s'], .002)
                self.assertEqual(timing['transaction_max_s'], .002)
                self.assertIsNotNone(data['joints'][0]['feedback_age_s'])
                self.assertTrue(data['demo'])
                self.assertFalse(data['armed'])
            finally:
                arm.close()

    def test_actual_owner_counts_failed_exchange_attempts(self):
        clock = TimingClock()
        def failed(*args, **kwargs):
            clock.advance(5_000_000)
            raise OSError('injected fake exchange failure')
        with patch('arm_manual.capture_exchange', failed):
            arm = ArmManualSession(physical_opt_in=False, timing_clock_ns=clock)
            try:
                sid = arm.create_session()
                receipt = self.action(arm, sid, 'connect', {'demo': True})
                self.assertEqual(receipt['status'], 'rejected', receipt)
                timing = arm.snapshot()['timing']
                self.assertGreater(timing['transaction_failures'], 0)
                self.assertEqual(timing['transaction_samples'], timing['transaction_failures'])
                self.assertEqual(timing['transaction_max_s'], .005)
                self.assertFalse(arm.snapshot()['armed'])
            finally:
                arm.close()

    def test_actual_owner_loop_uses_host_clock_without_touching_serial(self):
        arm = ArmManualSession(physical_opt_in=False)
        try:
            until = time.monotonic() + 1
            while arm.snapshot()['timing']['loop_samples'] < 2 and time.monotonic() < until:
                time.sleep(.005)
            data = arm.snapshot()
            self.assertEqual(data['timing']['clock_source'], 'host_monotonic_ns')
            self.assertGreater(data['timing']['loop_period_s'], 0)
            self.assertGreaterEqual(data['timing']['loop_samples'], 2)
            self.assertEqual(data['timing']['transaction_samples'], 0)
            self.assertFalse(data['connected'])
        finally:
            arm.close()

    def test_timing_json_allowlist_drops_unknowns_and_nonfinite_values(self):
        data = _project_fields({'timing': {
            'scope': 'owner_lifetime', 'clock_source': 'injected_clock',
            'loop_samples': 3, 'loop_period_s': float('nan'),
            'transaction_latency_s': .002, 'hidden': 'secret',
        }, 'unexpected': 'secret'}, TOP_LEVEL_FIELDS)
        self.assertNotIn('unexpected', data)
        self.assertNotIn('hidden', data['timing'])
        self.assertNotIn('loop_period_s', data['timing'])
        self.assertEqual(data['timing']['transaction_latency_s'], .002)

    def test_real_http_state_reports_timing_without_serial_enumeration(self):
        arm = ArmManualSession(physical_opt_in=False)
        server = ArmDashboardServer(('127.0.0.1', 0), arm)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        connection = http.client.HTTPConnection('127.0.0.1', server.bound_port, timeout=2)
        try:
            with patch.object(ArmDashboardHandler, '_get_serial_ports', return_value=[]), \
                    patch('arm_dashboard.enumerate_ports', side_effect=AssertionError('forbidden enumeration')) as enumeration:
                connection.request('GET', '/api/state')
                response = connection.getresponse()
                data = json.loads(response.read())
                self.assertEqual(response.status, 200)
                self.assertEqual(data['timing']['scope'], 'owner_lifetime')
                self.assertEqual(data['timing']['clock_source'], 'host_monotonic_ns')
                self.assertEqual(data['timing']['transaction_samples'], 0)
                self.assertFalse(data['connected'])
                self.assertFalse(data['armed'])
                enumeration.assert_not_called()
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)
            arm.close()

    def test_unavailable_measurement_clock_does_not_block_owner_actions(self):
        broken = [True]
        def clock():
            if broken[0]:
                raise OSError('injected observation clock failure')
            return time.monotonic_ns()
        arm = ArmManualSession(physical_opt_in=False, timing_clock_ns=clock)
        try:
            sid = arm.create_session()
            receipt = self.action(arm, sid, 'connect', {'demo': True})
            self.assertEqual(receipt['status'], 'complete', receipt)
            data = arm.snapshot()
            self.assertTrue(data['connected'])
            self.assertFalse(data['armed'])
            self.assertGreater(data['timing']['measurement_errors'], 0)
            self.assertGreater(data['timing']['transaction_samples'], 0)
            self.assertIsNone(data['timing']['transaction_latency_s'])
        finally:
            broken[0] = False
            arm.wake.set()
            arm.close()

    def test_measurement_clock_failure_does_not_mask_exchange_failure(self):
        broken = [True]
        def clock():
            if broken[0]:
                raise OSError('injected timing failure')
            return time.monotonic_ns()
        def failed(*args, **kwargs):
            raise OSError('original fake exchange failure')
        with patch('arm_manual.capture_exchange', failed):
            arm = ArmManualSession(physical_opt_in=False, timing_clock_ns=clock)
            try:
                sid = arm.create_session()
                receipt = self.action(arm, sid, 'connect', {'demo': True})
                self.assertEqual(receipt['status'], 'rejected', receipt)
                self.assertIn('Cannot verify Stop', receipt['reason'])
                self.assertTrue(any('original fake exchange failure' in event
                                    for event in arm.snapshot()['events']))
                data = arm.snapshot()['timing']
                self.assertGreater(data['measurement_errors'], 0)
                self.assertGreater(data['transaction_failures'], 0)
            finally:
                broken[0] = False
                arm.wake.set()
                arm.close()

    def test_exchange_latency_excludes_post_exchange_lock_wait(self):
        for fail in (False, True):
            with self.subTest(failed_exchange=fail):
                self.check_post_exchange_contention(fail)

    def check_post_exchange_contention(self, fail):
        completed = threading.Event()
        held = threading.Event()
        sampled = threading.Event()
        clock = TimingClock()
        def observe():
            value = clock()
            if completed.is_set():
                sampled.set()
            return value
        original = arm_manual.capture_exchange
        first = [True]
        def measured(*args, **kwargs):
            result = original(*args, **kwargs)
            if first[0]:
                first[0] = False
                clock.advance(2_000_000)
                completed.set()
                if not held.wait(1):
                    raise AssertionError('contending reader did not acquire lock')
            if fail:
                raise OSError('original contended exchange failure')
            return result
        arm = ArmManualSession(physical_opt_in=False, timing_clock_ns=observe)
        def reader():
            if completed.wait(1):
                with arm.lock:
                    held.set()
                    sampled.wait(.05)
                    clock.advance(50_000_000)
        worker = threading.Thread(target=reader)
        worker.start()
        try:
            with patch('arm_manual.capture_exchange', measured):
                sid = arm.create_session()
                receipt = self.action(arm, sid, 'connect', {'demo': True})
                self.assertEqual(receipt['status'], 'rejected' if fail else 'complete', receipt)
                timing = arm.snapshot()['timing']
                self.assertEqual(timing['transaction_max_s'], .002)
                if fail:
                    self.assertGreater(timing['transaction_failures'], 0)
                    self.assertTrue(any('original contended exchange failure' in event
                                        for event in arm.snapshot()['events']))
        finally:
            completed.set()
            worker.join(1)
            arm.close()
        self.assertFalse(worker.is_alive())

    def test_close_keeps_working_with_unavailable_observation_clock(self):
        for invalid in (False, True):
            with self.subTest(invalid_value=invalid):
                broken = [False]
                def observe():
                    if broken[0]:
                        if invalid:
                            return 'invalid clock'
                        raise OSError('observation clock unavailable')
                    return time.monotonic_ns()
                arm = ArmManualSession(physical_opt_in=False, timing_clock_ns=observe)
                try:
                    sid = arm.create_session()
                    self.assertEqual(self.action(arm, sid, 'connect', {'demo': True})['status'], 'complete')
                    bus = arm.bus
                    broken[0] = True
                    self.assertTrue(arm.close())
                    self.assertFalse(arm.thread.is_alive())
                    self.assertFalse(bus.is_open)
                    self.assertIsNone(arm.bus)
                finally:
                    arm.close()

    def test_real_prewrite_cancellation_counts_failed_attempt_without_write(self):
        original = arm_manual.capture_exchange
        writes = []
        def cancelled(*args, **kwargs):
            bus = args[2]
            with patch.object(bus, 'write', wraps=bus.write) as write:
                kwargs['before_write'] = lambda: False
                try:
                    return original(*args, **kwargs)
                finally:
                    writes.append(write.call_count)
        with patch('arm_manual.capture_exchange', cancelled):
            arm = ArmManualSession(physical_opt_in=False)
            try:
                sid = arm.create_session()
                receipt = self.action(arm, sid, 'connect', {'demo': True})
                self.assertEqual(receipt['status'], 'rejected', receipt)
                timing = arm.snapshot()['timing']
                self.assertEqual(timing['transaction_samples'], len(writes))
                self.assertEqual(timing['transaction_failures'], len(writes))
                self.assertGreater(len(writes), 0)
                self.assertEqual(sum(writes), 0)
                self.assertTrue(any('cancelled by pre-write hook' in event
                                    for event in arm.snapshot()['events']))
            finally:
                arm.close()

    def test_timing_counts_survive_demo_reconnect(self):
        arm = ArmManualSession(physical_opt_in=False)
        try:
            sid = arm.create_session()
            for action, values in [('connect', {'demo': True}), ('disconnect', {}), ('connect', {'demo': True})]:
                before = arm.snapshot()['timing']
                self.assertEqual(self.action(arm, sid, action, values)['status'], 'complete')
                after = arm.snapshot()['timing']
                self.assertGreaterEqual(after['transaction_samples'], before['transaction_samples'])
                self.assertGreaterEqual(after['transaction_failures'], before['transaction_failures'])
                self.assertEqual(after['scope'], 'owner_lifetime')
        finally:
            arm.close()

    def test_timing_clock_must_be_callable(self):
        with self.assertRaises(ValueError):
            ArmManualSession(physical_opt_in=False, timing_clock_ns=3)


if __name__ == '__main__':
    unittest.main()
