# Serial deadline verification

The transaction uses one monotonic budget from function entry. Read and write timeouts must use the remaining budget. A valid reply is accepted only after cleanup and before expiry. Reset and timeout configuration are synchronous operations. A driver or kernel operation that ignores its timeout cannot be interrupted by this function.

The actual backend is the pinned POSIX pyserial 3.5 implementation. Its read uses select with the configured timeout. Its write uses a positive finite timeout to bound readiness waits and EAGAIN retries. A zero write timeout may loop on EAGAIN because the final expiry check excludes nonblocking mode. The implementation therefore replaces zero or infinite write timeouts for the transaction, then restores the original values.

Deterministic checks cover logical budget accounting. Linux pseudo-terminal checks use real pyserial but no motor adapter. The missing-reply case repeats five times for each original timeout, and saturated-write cases exercise the finite write bound. The acceptance budget is 20 ms, with a 50 ms allowance for scheduling and synchronous cleanup. This is a timeout acceptance check, not a throughput benchmark or hard real-time guarantee. The raw logs record each observed elapsed value. Software tests do not establish electrical RS-485 behavior or actuator safety.
