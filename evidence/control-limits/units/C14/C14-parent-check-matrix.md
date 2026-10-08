# Parent C14 behavioral checks

The settled source must satisfy these independent contracts. Existing negative constructor tests and all C07–C13 behavioral cases remain covered after required policy and telemetry migration.

- Driver and winding trips use their own configured inclusive thresholds. A cause union retains simultaneous communication, health, position, speed, effort and temperature failures per joint.
- Peak measured effort faults immediately. Continuous effort faults when confirmed fresh observed time above its configured limit accumulates to the dwell, including exact equality. Interrupted observations do not credit the unknown gap and do not discard previously established unsafe time. Below-limit samples end the current overload episode without clearing latched faults.
- A stale or missing sample cannot extend an observed overload or recovery interval. Stop, takeover, reconnect and healthy feedback cannot erase established faults or permit automatic rearm.
- Acknowledgment records permission to attempt recovery. It does not clear faults. Recovery requires all joints freshly sampled in Stop mode, below both reset temperatures and continuous effort throughout the full new dwell.
- Each joint owns its timers. One stale joint prevents global recovery. New causes during recovery latch and restart the healthy interval. Regression, nonfinite time or an unobserved long gap cannot complete recovery.
- Existing actual-write history, canonical packet cache, preparation-time equality, exact Fraction velocity/acceleration/lead/combined-effort authority, expiry and ownership epoch gates survive unchanged.
- Initial nonfault Check still requires complete fresh healthy Stop feedback. No timeout acknowledgment or motor-port access occurs in any parent check.

Parent inspection precedes checks and determines the actual required policy constructor. Fixed parent argv, raw results and source hashes are recorded even when checks fail. Reproduction helpers may migrate only their fixture arguments to the settled required policy API; changing an assertion requires an explicit correction record.

Parent planning correction. The first draft incorrectly called overload dwell contiguous. The frozen C14 assignment requires retained unsafe history across interruptions and a new contiguous interval only for healthy recovery. No executable test or production change relied on that mistaken planning phrase.
