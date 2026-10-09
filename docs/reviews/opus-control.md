# Independent control review (REVIEW-01)

| | |
| --- | --- |
| Reviewer | Independent Opus 5.5 collaborator invited by the user (not a Pstack worker) |
| Base | `9e03d918b4fb96ea58ea09b4c9e98c119a0ff32e` (`codex/controller-limits-station`, C07–C15) |
| Scope | Read-only review of `joint_reference.py`, `joint_contract.py`, `joint_controller.py`, `joint_protection.py`, `station_step.py`, `station_timing.py`, `scripts/benchmark_station_timing.py`, and the `im6014_serial.py` / `im6014_codec.py` changes since `244811b` |
| Environment | macOS 15.6.1 arm64, Anaconda Python 3.12.7, pyserial 3.5. Not the locked Linux/Python 3.12.14 environment. |
| I/O | Fake buses, in-memory transports and isolated PTYs only. No physical port was opened, and no runtime file was edited. |

Three review slices ran in parallel: reference/limits, controller/protection, and station/timing. The coordinating session then re-ran every reproduction cited below at this base and compared the outputs. The scripts are in [`opus-control/`](opus-control/). Each one inserts the repository root (four directories up) into `sys.path`, prints its evidence, and exits 0. Run them from the repository root with the selected interpreter, for example `python -B docs/reviews/opus-control/reference/r7_three_joint_hold.py`.

Labels: **REPRODUCED** means a script shows the behaviour. **TRACED** means an unambiguous source trace without a runnable reproduction. **SUSPECTED** means plausible but not demonstrated.

## Summary

The safety gates hold:

- **Packet bounds:** across 300 randomised 3-joint configurations, 123,579 packets decoded from the written bytes violated no C09–C12 bound.
- **Write gate:** a 400-seed model check covered 26,563 granted claims. It found no write after expiry, Stop or fault, or by a stale owner or generation. It also found no fault cleared without the acknowledged dwell check.
- **Codec:** 7 golden vectors and 200,000 random inputs produced no encoding change.

Most findings are therefore liveness and protection-coverage problems:

- **Shaping faults:** the reference shaper latches `constraint_fault` (Stop, owner revoked) while holding the measured pose and at the end of ordinary moves. On a gravity-loaded arm, Stop means torque off, so a spurious fault is itself a hazard.
- **Effort protection:** the continuous-effort protection never trips under realistic sustained overload.
- **Transport:** every serial transaction reconfigures the port several times.

| # | Severity | Finding | Where | Label |
|---|---|---|---|---|
| 1 | High | Shaper faults on reachable states: holding the measured pose, and every point-to-point move tried | `joint_reference.py:615-616` | REPRODUCED |
| 2 | High | C14 continuous-effort protection resets on any below-threshold sample; Stop/check erases history | `joint_protection.py:191-201` | REPRODUCED |
| 3 | High | Each exchange reconfigures the open port 4–13 times; busy-spins with timeout 0 | `im6014_serial.py:301`, `:371`, `:412-417` | REPRODUCED (counts); SUSPECTED (USB effects) |
| 4 | Medium | No braking look-ahead: ~v²/2A overshoot; goals at or near an inclusive limit fault | `joint_reference.py:430-444` | REPRODUCED |
| 5 | Medium | Station ignores `receive()` decisions: faults recorded as `OK` or blamed on the next joint | `station_step.py:207`, `:341` | REPRODUCED |
| 6 | Medium | A reply arriving after its deadline is accepted as the next transaction's reply; no bus-quiet guard | `im6014_serial.py:330`, `:368` | REPRODUCED (fake bus) |
| 7 | Medium | Protection latches and thermal hysteresis live only in one `JointCore`; a restarted station arms a hot motor | `joint_controller.py:146-151`, `:538-550` | REPRODUCED |
| 8 | Medium | Missed-slot counter off by one; benchmark passes with 100% overruns | `station_timing.py:462`; `scripts/benchmark_station_timing.py:270`, `:445-517` | REPRODUCED |
| 9 | Medium-low | An exception escaping `step()` wedges the tracker; a BaseException leaves the dispatch claim set | `station_step.py:104-308`; `im6014_serial.py:385` | REPRODUCED |
| 10 | Medium-low | `max_reference_gap_s` both expires the arm-time anchor and bounds the packet step | `joint_controller.py:555-564`, `:694-701`; `joint_reference.py:366-370` | REPRODUCED |
| 11 | Low | `prepare_refresh` handles only `ValueError` from the clock | `joint_controller.py:668-672` | REPRODUCED |
| 12 | Low | Write budget computed before `before_write`; a timed-out write may already be on the wire | `im6014_serial.py:371-374`, `:384-391` | REPRODUCED |
| 13 | Low | Test gaps: arrival/hold never exercised; three C14 recovery rules undetected by mutation | tests listed below | REPRODUCED |
| 14 | Low | Minor: unreachable trip thresholds accepted, record labels, windowed statistics | see below | REPRODUCED |

## 1. Shaper faults on states that satisfy every bound

**The rule:** `joint_reference.py:615-616` returns `CONSTRAINT_VIOLATION` when `v_desired != 0` and the projected position equals the confirmed one. That is, it fires whenever the nearest representable step is "no move" and the objective is nonzero. In these states the feasible interval is not empty, often hundreds of grid steps wide, and `q_prev` itself satisfies every closed bound. `docs/joint-api.md:140` asks for a Stop only "if no representable position step satisfies the closed bounds".

Two mechanisms trigger it:

- **Off-grid hold:** a goal off the 1.5138e-05 rad output grid can never be held. Within half a grid step, `(goal − q)/h ≠ 0` rounds to no move.
- **Velocity reversal:** at a turning point the projected step is below half a grid step. Grid-exact goals hit this too.

Reproductions:

`reference/r7_three_joint_hold.py`, holding the 3-joint measured pose:

```text
refresh 2: no plan; state=fault; faulted joints=[('shoulder', 'constraint'), ('elbow', 'constraint'), ('wrist', 'constraint')]; current_owner=None
[goal = first emitted grid pose (client-side workaround)] ... held for 500 refreshes; state=moving
```

`reference/r2_goal_sweep.py`, with h=10 ms, V=1, A=10 and ideal tracking:

```text
goal=+0.050: FAULT at step 18 (['constraint_fault']); q_ref=0.080928; overshoot=+0.0309 rad
goal=+0.300: FAULT at step 44 (['constraint_fault']); q_ref=0.339696; overshoot=+0.0397 rad
goal=-0.300: FAULT at step 44 (['constraint_fault']); q_ref=-0.339696; overshoot=+0.0397 rad
```

`reference/r3_diagnose_turning_point.py` shows, in exact Fractions, that the interval is non-empty (about 132 reachable grid steps) when the rule fires. The behaviour reproduces for A = 2, 3.7, 25 and 100, with ±5% jitter on h, and at 2 ms cadence. `reference/r6_fuzz_emitted_bounds.py` ended 171 of 300 random 3-joint runs in `constraint_fault` within 200 cycles, while decoding 0 bound violations.

**Impact:** a jog dashboard cannot hold a pose or finish a move without a client-side workaround. Each fault is Stop, which is torque off under gravity.

**Fix direction:**

- Accept `q_prev` when it lies inside the exact feasible interval.
- Fault only when no representable point exists in `[q_next_lo, q_next_hi]`, or on sustained lack of progress.
- Optionally shape toward the decoded admitted goal that `prepare_reference_joint` already computes.
- If you relax the rule, also bound the wire-velocity stream by the position and lead interval (`joint_reference.py:457-463` projects it only by V and A). Otherwise a reference parked at `q_max` could carry an outward `dq_wire`, and with `kp=0`, which is allowed, that is a pure velocity command.

## 2. Continuous-effort protection never trips under sustained overload

**The flaw:** `joint_protection.py:191-201` sets `overload_duration = 0` on any single sample with |τ| ≤ `continuous_effort_nm`. Normal torque ripple or noise therefore keeps the accumulator near zero. Stop → check → arm also erases it: `stop()` (`joint_controller.py:899-903`) and `begin_check()` (`:427-435`) feed Stop-mode samples with τ ≈ 0.

**The test that hides it:** `docs/joint-api.md:185` says Stop and checking "preserve confirmed overload duration". `test_controller_thermal.py:536` passes only because its helper (`_receive_overload`, `:158`) feeds 2× continuous torque *in Stop mode*, which a stopped motor does not report.

`controller/r1_continuous_effort_bypass.py` uses continuous 4 Nm, dwell 0.5 s, peak 8 Nm and 100 Hz:

```text
[dip] samples=3000 sim_time=30.00s time_above_continuous=27.00s mean_tau=7.140Nm ... max accumulated overload_duration_s=0.080 state=ARMED fault_latched=False
[noise] samples=6000 sim_time=60.00s mean_tau=4.403Nm fraction_above_continuous=0.67 fault_latched=False
[cycle] cycles=100 total time at 7.9Nm (~1.98x continuous)=45.0s duty=0.98 state=ARMED fault_latched=False
```

**Impact:** a gravity-loaded joint can run near peak effort indefinitely, with only the temperature trip as a backstop.

**Fix direction:** use a decaying integrator (I²t or a first-order thermal model) instead of a reset, carry it through Stop and check, and test with realistic Stop telemetry.

## 3. Every exchange reconfigures the serial port several times

**The cause:** the transaction assigns `bus.timeout` on every read pass (`im6014_serial.py:301`), `bus.write_timeout` (`:371`), and restores both (`:412-417`). In pyserial 3.5 each assignment on an open port runs `_reconfigure_port()`. With `exclusive=True` and a custom 6 Mbaud rate, that means `flock` + `tcgetattr` + `tcsetattr` + a custom-baud ioctl.

`station/repro_01_reconfigure_count.py` uses a `serial.Serial` subclass, so the setters are real pyserial code over simulated I/O. Part B records the real darwin `_reconfigure_port`. The coordinator's re-run:

```text
[S1 reply already buffered (benchmark fake model)] reconfigure_calls=4
[S2 reply lands 150us after write in one USB packet] reconfigure_calls=5
[S3 reply in 2 USB packets (150us, 400us)] reconfigure_calls=7
[S4 missing motor, port timeout=0.005 (im6014_port default)] elapsed=50.07ms reconfigure_calls=13
[S5 missing motor, original timeout=0 (tested in test_serial_deadline)] elapsed=20.06ms reconfigure_calls=8338
3-joint cycle, all replies like S2: 15 reconfigurations; with one missing motor in check_step: 23
```

- **Busy spin:** S5 spins because `_bounded_timeout(0, …)` returns 0. The agent's run recorded 8,427; the count depends on timing.
- **Darwin baud toggle:** if `tcgetattr` reports the actual 6 Mbaud, each reconfiguration emits `tcsetattr(speed=38400)` then `IOSSIOSPEED`. The first one falls between the write and the first read, while the reply is on the wire (TRACED).
- **Linux (SUSPECTED, not hardware-verified):** each TCSETS2 enters ftdi_sio `set_termios`, which can issue USB control transfers. `USER_MANUAL.md:156` already records control-transfer timeouts on this adapter. The deadline cannot interrupt a stalled setter, and reconfiguration time comes out of the reply budget.

**Fix direction:** set timeouts once at open, then wait with `select` on `fileno()` for the remaining budget and read only `in_waiting`.

## 4. No braking look-ahead

**The gap:** the interval at `joint_reference.py:430-444` has no stopping-distance term, and the objective `v_desired = (goal − q)/h + dq_goal` is one-step deadbeat. The reference therefore reaches the goal at speed and overshoots by about v²/2A, which is about 0.04 rad in finding 1's sweep. A goal at an inclusive limit is admitted but cannot be reached at speed.

`reference/r4_limit_approach.py`:

```text
[repo near-limit test continued]: q0=1.95 goal=2.0 q_max=2.0 V=2.0 A=4.0 h=0.1
  step 3: constraint_violation; q_prev=1.999998 v_prev=+0.1001
[goal == q_max from 0.0]: ... step 55: constraint_violation; q_prev=0.494557 v_prev=+0.9991
    v_lo=+0.8991 (accel lower +0.8991) v_hi=+0.5443 (position upper +0.5443) -> EMPTY
```

The first case continues the exact inputs of `test_controller_limits.py:2030`, which runs one step.

**Fix direction:** add an exact-rational braking constraint: stopping distance v²/2A plus v·h must fit before `q_min`/`q_max`, the lead edge and the goal. Use a braking-curve objective instead of the deadbeat one.

## 5. Station ignores `receive()` decisions

**The gap:** `station_step.py:207` and `:341` call `core.receive()` and discard the returned `Decision`.

`station/repro_03_receive_decision_ignored.py`:

```text
[J3 reply carries motor error flag errors=0x4]
   J1: outcome=OK ... J2: outcome=OK ... J3: outcome=OK
   tracker counters: transactions=6 successful=6 failed=0 write_cancelled=0
   core state=fault faults=J3(errors=4, position_limit=False)
[J1 reply carries motor error flag errors=0x4]
   J1: outcome=OK ... J2: outcome=WRITE_CANCELLED ... J3: outcome=SKIPPED
```

The fault is absent from the transaction history, or attributed to the joint after the faulting one. After `disconnect()`, `check_step` reports `OK` while `receive` returns `INVALID_STATE` (`station/repro_07_bookkeeping.py` [B]).

**Fix direction:** map a rejected `receive` to a typed outcome and end the cycle.

## 6. A late reply is accepted by the next transaction

**The gap:** the reset at `im6014_serial.py:368` discards only bytes that have already arrived, and the next write follows immediately. `feedback()` checks only the ID and that mode ∈ {0, 1}.

`station/repro_05_late_reply.py`, on a real-clock fake bus:

```text
[i] exchange #1: TransactionTimeoutError after 50.1 ms
    exchange #2: OK in 3.0 ms, reply position_rad=0.1000 (request #1 marker 0.1, request #2 marker 0.2)
[ii] J1: outcome=TIMEOUT duration=50.1 ms
     J2: outcome=INVALID_FEEDBACK duration=50.1 ms
```

- **Stale data:** a stale reply is accepted as fresh.
- **Misattribution:** a late J1 frame is blamed on J2.
- **Collision (TRACED):** on half-duplex RS-485 the next request can collide with a late responder.

**Fix direction:**

- After a timeout, wait a guard interval or drain until the line is quiet.
- Discard frames by ID.
- Check that the reply mode matches the request.

## 7. "Persistent" protection is per `JointCore` instance

**The gap:** `_faults` and `_protection_states` are in-memory only (`joint_controller.py:146-151`). The reset thresholds are consulted only when `_faults` is non-empty (`:538-550`). A fresh core therefore starts with no fault latches, no overload history and no thermal hysteresis. That covers both a station restart and the documented fix for sequence or generation exhaustion, "start a fresh core" (`docs/joint-api.md:108`, `:116`).

`controller/r2_restart_clears_latches.py`:

```text
core A: receive winding=105C -> temperature_out_of_bounds; state=FAULT winding_latch=True
core B (fresh): check+arm with J1..J3 winding=104C (> reset 90C) -> arm=accepted state=ARMED fault_latched=False
```

**Fix direction:** persist latched causes and protection state per motor ID, or require temperature ≤ reset and effort ≤ continuous at every `checked()`. At minimum, correct "persistent" in the C14 milestone text and the docs.

## 8. Timing statistics and benchmark

**The off-by-one:** `missed = (now − slot)//period + 1` (`station_timing.py:462`) counts the late slot as skipped, but `run_steps` runs it and re-anchors. Any oversleep, including on the first cycle, counts as one skipped slot. `test_station_timing.py:283-290` locks in the off-by-one.

`station/repro_02_missed_slots.py`:

```text
counters: cycles=10 overruns=0 missed_slots_skipped=9  (true skipped slots: 0)
Real clock, nominal 5 ms, 100 idle cycles: overruns=0 missed_slots_skipped=100; cycles late by >= 1 full period: 0
mean actual period 6.231 ms vs nominal 5.000 ms
```

**The benchmark:**

- `validate_repeat_result` checks work accounting but never timing.
- The committed receipt `evidence/control-limits/units/C15/C15-recovered-benchmark/…/timing-benchmark-receipt.json.gz` reports `passed`, with 50/50 overruns in each of its 5 repeats and mean periods of 16.9–19.6 ms against 5 ms nominal.
- A local run (`scripts/benchmark_station_timing.py --evidence-root …`) also passed, with 50/50 overruns in each of 5 repeats and mean periods of 14.75–14.78 ms.
- Its fake transport returns all 26 bytes at once, so it never pays for multi-read or reconfiguration.
- Zero-latency profiling (`station/profile_step.py`) shows a 3-joint cycle spending about 3.3 ms in `submit_target` and 4.2 ms in `step`, mostly the binary searches at `im6014_codec.py:149` and `:215`.

**Fix direction:**

- Count `floor((now − slot)/period)`.
- Fail or flag the benchmark on overruns or a missed rate.
- Model reply latency in the fake.

## 9. An escaped exception wedges the station

**The gap:** `station_step.py:104-308` has no `try/finally` around `start_cycle`/`end_cycle`. `im6014_serial.py:385` catches `Exception`, so a `BaseException` skips `on_write` and leaves the dispatch claim set.

`station/repro_04_wedge_after_escape.py`:

```text
[A] step() with clock fault: ValueError ...
    step() after clock recovered: RuntimeError: start_cycle called before the active cycle ended
[B] KeyboardInterrupt in bus.write: core._dispatch_claim set: True
    core.begin_check(acknowledge_fault=True): OK -> False ... core.begin_connect(): OK -> False
```

Only `tracker.reset()`, which erases history, or a new core recovers.

**Fix direction:** abort the cycle in a `finally`, and resolve the claim in a `finally` inside the exchange.

## 10. One gap parameter, two jobs

**The cause:** the arm-time anchor is stamped with the check observation (`joint_controller.py:555-564`) and never refreshed in READY or ARMED. `max_reference_gap_s` both expires that anchor and bounds the step to the next packet (`:694-701`; `joint_reference.py:366-370`).

`reference/r5_gap_coupling.py` and `controller/r5_misc.py` [b]:

```text
(a) submit at t=10.2: accepted; first prepare_refresh -> None; state=stopped; current_owner=None; fault_latched=False
(b) jump=0.8000 rad in one packet; h=0.800s; v_step=1.0000 <= V; |dv|=0.7003 <= A*h=8.00; state=moving
```

- **Tight value:** a gap of 0.05 s silently Stops the first command, and the client must reclaim, re-check and re-arm. The anchor also misses Stop-mode sag.
- **Loose value:** a gap of 1.0 s, which every repository test uses, lets a 0.8 s station hiccup emit a 0.8 rad step that passes the per-sample V/A certificates. Only the effort bound limits it.

**Fix direction:** re-anchor from the latest fresh Stop observation at arm or first prepare, and bound the per-packet step separately from the idle gap (for example V × the nominal period).

## 11–14. Lower-severity findings

**11. `prepare_refresh` clock errors.** `joint_controller.py:668-672` catches only `ValueError`, whereas `claim_dispatch` (`:771-775`) catches `Exception`. `controller/r3_prepare_clock_exception.py` shows an `OSError` leaving the core MOVING with owner and goal kept. Motion resumes when the clock recovers, with no Stop or acknowledgment. The final write gate still re-checks. Fix direction: catch `Exception` and `stop()`.

**12. Write budget.** `write_timeout` is computed before `before_write()` runs (`im6014_serial.py:371-374`), so the hook's time is not deducted. `station/repro_06_write_budget.py` [A] shows a 50 ms deadline exchange raising after 75.7 ms with a 20 ms hook. On a 115200-baud PTY [B], a write reported `failed` had already queued all 20 bytes. Fix direction: recompute the budget immediately before the write, and treat a timed-out write as possibly sent.

**13. Test gaps.**

- *Arrival and hold:* never exercised. `test_controller_limits.py:1506` holds at raw 0, exactly on the grid. `:631` builds grid-exact goals, and `:2030` runs one step.
- *Oracle:* `evidence/control-limits/units/C15/C14-exact-oracle.py` asserts that a "nonzero objective cannot silently stall", which codifies finding 1. Extended to 400 steps (`reference/r9_oracle_extended.py`), it chatters around the goal, with 198 velocity sign changes in 200 steps.
- *Mutation testing:* `controller/mutants.py` applies one mutation at a time to a temporary copy and runs the five controller suites.
  - **Survives (M6):** recovery no longer requires Stop mode, at `joint_protection.py:206`. That leaves `docs/joint-api.md:195` untested, and `controller/r4_recovery_mode_gap.py` shows the behaviour change (`fault_latched` → `accepted`).
  - **Survive (M4/M5):** removing the interval break in `stop()`/`disconnect()` (`joint_controller.py:901-903`, `:914-916`).
  - **Judged equivalent:** M1, M2, M8, M9, M10, M12 and M18 also survive. The slice reviewer judged them equivalent because other guards make those checks unreachable; that is not independently proven.

**14. Minor.**

- *Unreachable trip thresholds:* `ProtectionPolicy` accepts `driver_trip_c=130` and `winding_trip_c=300`, but the wire can report at most 127 °C and 255 °C (`im6014_codec.py:293`). Those trips are silently unreachable (`controller/r5_misc.py` [a]).
- *Record labels:* `check_step` labels its records with the next cycle index (`station_step.py:314`).
- *Windowed statistics:* min/max/mean cover only the last `capacity` samples while the counters are cumulative (`station_timing.py:546-553`).
- *Snapshot cost:* the windows use `list.pop(0)`, so `snapshot()` takes 2.7 ms at capacity 65536 (`station/micro_capacity.py`, `station/repro_07_bookkeeping.py`).

## Checked and holding up

- **C09–C12 bounds** (`reference/r6_fuzz_emitted_bounds.py`).
  - *Coverage:* 300 random 3-joint configurations, with reductions 0.6–7.3, direction ±1, offsets, h of 1.6–24 ms with jitter, lagging and noisy measurement, and nonzero feed-forward.
  - *Result:* 123,579 packets decoded from the written bytes. Checked for q limits, wire V/A, step V/A and lead: 0 violations.
  - *Also verified:* the `direction=−1` fallback sign, the monotonic `project_*_raw` searches, exact Fraction arithmetic, and the post-decode exact checks that catch outward float rounding.
- **C07/C08 write gate** (`controller/fuzz_gate.py 400`): 400 seeds × 3000 operations, covering 26,563 granted claims, 17,267 fault clears, 26,277 entries to ARMED and 7,773 to MOVING. There were no claims outside ARMED/MOVING with the current owner, an unexpired goal, and fresh, healthy, in-limit feedback after an arm. There were no fault clears without an acknowledgment after the latest latch and a shadow-verified dwell.
  - *Expiry:* takes effect at exact equality.
  - *Stale identity:* a stale generation, epoch or connection is rejected, and takeover revokes and Stops.
  - *Late receipts:* never restore MOVING.
  - *Exhaustion:* fails closed.
- **Effort algebra:** joint torque = s·N·output torque, kp and kd scale by N², and the inclusive exact comparison is correct in both directions. The mutation suite caught M3, M7, M11 and M13–M17.
- **Transport and codec:**
  - Within one exchange, a reply is never accepted after its deadline, and timeouts are restored on every path.
  - `PREWRITE_DEADLINE`, `FAILED` and `PARTIAL` reach `on_write`.
  - Clocks are monotonic and overrun arithmetic is exact.
  - The codec differential (`station/repro_08_codec_differential.py`) matched all 7 golden vectors (old = new = expected), produced 0 byte or accept/reject mismatches over 200,000 random inputs, and left `feedback()` unchanged on 50,000 CRC-valid frames.

## Commands and results

All runs used `/opt/anaconda3/bin/python3` (3.12.7) at `9e03d918`. Exit codes are as observed.

| Command | Result |
| --- | --- |
| `python -B -m unittest test_joint_mapping test_numerical_inputs test_controller test_controller_targets test_controller_expiry test_controller_owner test_controller_limits test_controller_lead test_controller_effort test_controller_thermal test_station_timing test_protocol test_diagnostic_capture` (run per module) | All exit 0: 12, 12, 27, 16, 25, 13, 55, 16, 20, 27, 30, 24 and 11 tests OK |
| `python -B -m unittest test_serial_deadline` | Exit 1: 11 run, 3 errors, `OSError: [Errno 25] Inappropriate ioctl for device`. macOS PTYs reject the 6 Mbaud IOSSIOSPEED (`station/probe_pty_iossiospeed.py`). |
| `python -B -m unittest test_bus_owner` | Exit 1: 17 run, 15 errors and 1 failure. The errors are `PortOwnershipError: Bus ownership requires root-owned sticky /tmp with mode 1777`; macOS `/tmp` is a symlink. |
| `bash scripts/check_offline_baseline.sh <python> <evidence>` | Does not start on macOS: BSD `realpath` has no `-e` |
| Every script under `opus-control/` (`python -B <script>`) | Exit 0. The outputs quoted above are the coordinator's re-runs. |
| `python -B scripts/benchmark_station_timing.py --evidence-root <dir>` | Exit 0, status `passed`, 50/50 overruns in each of 5 repeats, mean periods 14.75–14.78 ms against 5 ms nominal |
| `python -B docs/reviews/opus-control/reference/r6_fuzz_emitted_bounds.py` | Exit 0: `runs=300 emitted_packets_checked=123579 violations=0 runs_ending_in_fault=171` |
| `python -B docs/reviews/opus-control/controller/fuzz_gate.py 400` | Exit 0: `seeds=400 violations=0 stats={'grants': 26563, 'clears': 17267, 'armed': 26277, 'moving': 7773}` |
| `python -B docs/reviews/opus-control/controller/mutants.py` | Exit 0: baseline copy OK. Killed: M3, M7, M11, M13–M17. Survived: M1, M2, M4, M5, M6, M8, M9, M10, M12, M18 |

## Not checked

- **Not attempted:** physical serial ports, RS-485 timing, FTDI latency or USB behaviour, motor dynamics, gravity, clearance and emergency stopping time. Finding 3's USB consequences are therefore SUSPECTED.
- **Locked environment:** not used. Platform-dependent results above are macOS-specific.
- **Unreviewed:** the dashboard, bench UI, model conversion, and the arm team's `codex/arm-jog-dashboard` runtime.

## Related observation from the three-joint arm integration

This is outside REVIEW-01's code scope. Arm calibration work in an independent simulator assumes that the IM6014 reports an absolute output angle at power-up. `USER_MANUAL.md:73` and `:126` say the multi-turn state is not retained through power loss. The one recorded physical datum (`evidence/boundary/physical/stop-prechange.json`) shows a multi-turn accumulator, but no reading has been recorded across a power cycle. A short supported bench test would settle the joint-reference design for ARM-A03:

1. Mark the output and read it.
2. Move +1.3 turns while powered, power-cycle, and read.
3. Back-drive a known angle while unpowered, power up, and read.

## Coordination notes

- **File path:** `docs/development/tasks.tsv` names the output `docs/reviews/control.md`, while `docs/development/coordination.md` names `docs/reviews/opus-control.md`. This review uses the latter. The coordinator may rename it.
- **Registry:** `tasks.tsv` was not edited. The coordinator is its single writer.
- **Reproduction scripts:** the files under `opus-control/` are review evidence, not runtime code. Nothing imports them.
