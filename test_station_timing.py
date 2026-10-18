import unittest

from station_timing import StationTiming


class StationTimingTests(unittest.TestCase):
    def test_no_samples_are_unknown_instead_of_fake_zero_durations(self):
        data = StationTiming(80_000_000).snapshot()
        self.assertEqual(data['scope'], 'owner_lifetime')
        self.assertEqual(data['loop_samples'], 0)
        self.assertIsNone(data['loop_period_s'])
        self.assertIsNone(data['loop_max_s'])
        self.assertIsNone(data['transaction_latency_s'])
        self.assertIsNone(data['transaction_max_s'])

    def test_actual_consecutive_start_intervals_and_budget_boundary(self):
        timing = StationTiming(80_000_000)
        for now in (0, 10_000_000, 90_000_000, 170_000_001):
            timing.loop_started(now)
        data = timing.snapshot()
        self.assertEqual(data['loop_samples'], 3)
        self.assertEqual(data['loop_period_s'], .080000001)
        self.assertEqual(data['loop_max_s'], .080000001)
        self.assertEqual(data['loop_overruns'], 1)
        self.assertEqual(data['loop_budget_s'], .08)

    def test_zero_elapsed_is_a_real_sample(self):
        timing = StationTiming(1)
        timing.loop_started(0)
        timing.loop_started(0)
        data = timing.snapshot()
        self.assertEqual(data['loop_samples'], 1)
        self.assertEqual(data['loop_period_s'], 0)
        self.assertEqual(data['loop_overruns'], 0)

    def test_failed_and_successful_exchange_attempts_are_both_measured(self):
        timing = StationTiming(80_000_000)
        timing.transaction_finished(0, 2_000_000, succeeded=True)
        timing.transaction_finished(2_000_000, 7_000_000, succeeded=False)
        timing.transaction_finished(7_000_000, 8_000_000, succeeded=True)
        data = timing.snapshot()
        self.assertEqual(data['transaction_samples'], 3)
        self.assertEqual(data['transaction_failures'], 1)
        self.assertEqual(data['transaction_latency_s'], .001)
        self.assertEqual(data['transaction_max_s'], .005)

    def test_bad_constructor_and_clock_values_reject_without_partial_updates(self):
        for bad in (True, False, -1, .1, float('nan'), float('inf'), '1', None, 1 << 64):
            with self.subTest(bad=repr(bad)), self.assertRaises(ValueError):
                StationTiming(bad)
        with self.assertRaises(ValueError):
            StationTiming(0)
        timing = StationTiming(1)
        timing.loop_started(5)
        before = timing.snapshot()
        for bad in (True, False, -1, .1, '1', None, 1 << 64, 4):
            with self.subTest(bad=repr(bad)), self.assertRaises(ValueError):
                timing.loop_started(bad)
            self.assertEqual(timing.snapshot(), before)
        timing.loop_started(6)
        self.assertEqual(timing.snapshot()['loop_period_s'], 1e-9)

    def test_bad_transaction_does_not_consume_a_sample(self):
        timing = StationTiming(1)
        before = timing.snapshot()
        for start, end, success in ((5, 4, True), (True, 5, True), (0, .1, True),
                                    (0, 1, 1), (0, 1, None), (0, 1 << 64, True)):
            with self.subTest(start=start, end=end, success=success), self.assertRaises(ValueError):
                timing.transaction_finished(start, end, succeeded=success)
            self.assertEqual(timing.snapshot(), before)

    def test_long_run_keeps_fixed_storage_and_independent_instances(self):
        timing = StationTiming(10)
        other = StationTiming(10)
        for i in range(10000):
            timing.loop_started(i * 11)
            timing.transaction_finished(i * 11, i * 11 + 5, succeeded=i % 2 == 0)
        data = timing.snapshot()
        self.assertEqual(data['loop_samples'], 9999)
        self.assertEqual(data['loop_overruns'], 9999)
        self.assertEqual(data['transaction_samples'], 10000)
        self.assertEqual(data['transaction_failures'], 5000)
        self.assertFalse(hasattr(timing, '__dict__'))
        self.assertEqual(other.snapshot()['transaction_samples'], 0)

    def test_missing_loop_clock_breaks_only_measurement_continuity(self):
        timing = StationTiming(10)
        timing.loop_started(0)
        timing.loop_started(5)
        timing.loop_unavailable()
        self.assertIsNone(timing.snapshot()['loop_period_s'])
        timing.loop_started(100)
        self.assertIsNone(timing.snapshot()['loop_period_s'])
        timing.loop_started(105)
        data = timing.snapshot()
        self.assertEqual(data['loop_samples'], 2)
        self.assertEqual(data['loop_overruns'], 0)
        self.assertEqual(data['loop_max_s'], 5e-9)
        self.assertEqual(data['measurement_errors'], 1)

    def test_missing_transaction_time_is_unknown_but_retains_actual_outcome(self):
        timing = StationTiming(10)
        timing.transaction_unavailable(succeeded=True)
        self.assertIsNone(timing.snapshot()['transaction_latency_s'])
        self.assertIsNone(timing.snapshot()['transaction_max_s'])
        timing.transaction_finished(0, 5, succeeded=True)
        timing.transaction_unavailable(succeeded=False)
        data = timing.snapshot()
        self.assertEqual(data['transaction_samples'], 3)
        self.assertEqual(data['transaction_failures'], 1)
        self.assertEqual(data['transaction_max_s'], 5e-9)
        self.assertIsNone(data['transaction_latency_s'])
        self.assertEqual(data['measurement_errors'], 2)
        before = timing.snapshot()
        with self.assertRaises(ValueError):
            timing.transaction_unavailable(succeeded=1)
        self.assertEqual(timing.snapshot(), before)

    def test_snapshot_is_detached_from_internal_counters(self):
        timing = StationTiming(10)
        data = timing.snapshot()
        data['loop_overruns'] = 999
        self.assertEqual(timing.snapshot()['loop_overruns'], 0)


if __name__ == '__main__':
    unittest.main()
