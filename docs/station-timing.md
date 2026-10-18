# Station timing telemetry

`ArmManualSession.snapshot()` and the observational `GET /api/state` response expose a `timing` object. The existing per-joint `feedback_age_s` values report feedback age separately. Timing adds no bus owner, thread, command limit, or automatic action.

| Field | Meaning |
| --- | --- |
| `scope` | `owner_lifetime`. Counters persist across connections in this owner instance and restart with a new process. |
| `clock_source` | `host_monotonic_ns` for normal operation, or `injected_clock` in a fixture. |
| `loop_samples` | Number of measured intervals between consecutive owner-loop starts. The first start is an anchor. |
| `loop_period_s` | Latest measured interval, including dispatch, I/O and waiting. Unknown before an interval or after a clock failure. |
| `loop_max_s` | Largest measured interval in this owner lifetime. |
| `loop_budget_s` | Existing schedule-gap budget used to classify overruns, currently 0.08 seconds. This is a policy value, not a measurement. |
| `loop_overruns` | Measured intervals strictly greater than the reported budget. Equality is permitted. |
| `transaction_samples` | Completed exchange attempts, including failures and cancellations before a write. This is not a successful-write counter. |
| `transaction_failures` | Attempts that did not return a decoded exchange result. |
| `transaction_latency_s` | Latest measured exchange-attempt duration, excluding queue wait and the subsequent lock wait to publish counters. Unknown when its clock sample failed. |
| `transaction_max_s` | Largest available measured exchange-attempt duration. |
| `measurement_errors` | Unavailable or invalid loop intervals and transaction-duration samples. These do not count as successful measurements. |

Durations use integer monotonic nanoseconds internally and seconds in JSON. Empty duration series report `null`. A timing-clock failure invalidates its observation and breaks the loop's measurement anchor. It does not block owner actions, mask the original transport error, or change the independent authority clock, feedback checks, Stop, or motion limits. A later healthy timing sample begins a fresh interval.

`StationTiming` keeps counters, latest and maximum durations, and one loop anchor. It stores no growing trace. The owner updates it under its existing lock. HTTP serialization explicitly permits only the documented scalar fields.

Fixture runs establish software measurement behavior only. A fake adapter's latency does not establish physical RS485 latency, actuator response, stopping time, or achievable control rate. No speedup is claimed. A process restart creates a new owner and fresh timing counters.
