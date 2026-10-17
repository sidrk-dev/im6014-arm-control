"""Bounded host timing observations for a single calling owner thread."""


class StationTiming:
    __slots__ = (
        '_budget_ns', '_previous_loop_ns', '_loop_samples', '_loop_last_ns',
        '_loop_max_ns', '_loop_overruns', '_transaction_samples',
        '_transaction_failures', '_transaction_last_ns', '_transaction_max_ns',
    )

    def __init__(self, loop_budget_ns: int):
        self._valid_ns(loop_budget_ns)
        if loop_budget_ns == 0:
            raise ValueError('loop_budget_ns must be positive')
        self._budget_ns = loop_budget_ns
        self._previous_loop_ns = None
        self._loop_samples = 0
        self._loop_last_ns = 0
        self._loop_max_ns = 0
        self._loop_overruns = 0
        self._transaction_samples = 0
        self._transaction_failures = 0
        self._transaction_last_ns = 0
        self._transaction_max_ns = 0

    @staticmethod
    def _valid_ns(value: int) -> None:
        if type(value) is not int or not 0 <= value <= (1 << 64) - 1:
            raise ValueError('time must be a uint64 nanosecond integer')

    def loop_started(self, now_ns: int) -> None:
        self._valid_ns(now_ns)
        previous = self._previous_loop_ns
        if previous is not None:
            if now_ns < previous:
                raise ValueError('loop clock must be monotonic')
            duration = now_ns - previous
            self._loop_samples += 1
            self._loop_last_ns = duration
            self._loop_max_ns = max(self._loop_max_ns, duration)
            self._loop_overruns += duration > self._budget_ns
        self._previous_loop_ns = now_ns

    def transaction_finished(self, start_ns: int, end_ns: int, *, succeeded: bool) -> None:
        self._valid_ns(start_ns)
        self._valid_ns(end_ns)
        if type(succeeded) is not bool or end_ns < start_ns:
            raise ValueError('transaction requires monotonic times and bool outcome')
        duration = end_ns - start_ns
        self._transaction_samples += 1
        self._transaction_failures += not succeeded
        self._transaction_last_ns = duration
        self._transaction_max_ns = max(self._transaction_max_ns, duration)

    def snapshot(self) -> dict:
        return {
            'scope': 'owner_lifetime',
            'loop_samples': self._loop_samples,
            'loop_period_s': self._loop_last_ns / 1e9 if self._loop_samples else None,
            'loop_max_s': self._loop_max_ns / 1e9 if self._loop_samples else None,
            'loop_overruns': self._loop_overruns,
            'loop_budget_s': self._budget_ns / 1e9,
            'transaction_samples': self._transaction_samples,
            'transaction_failures': self._transaction_failures,
            'transaction_latency_s': self._transaction_last_ns / 1e9 if self._transaction_samples else None,
            'transaction_max_s': self._transaction_max_ns / 1e9 if self._transaction_samples else None,
        }
