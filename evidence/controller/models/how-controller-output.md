### Components Found

- [`MotorSession`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L131-L429) ([`motor_session.py`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py)): Coordinates GUI serial communication, runs a single background worker thread named `"IM6014 serial owner"`, serializes UI requests, manages lifecycle state transitions, samples rolling telemetry, and enforces watchdog/heartbeat constraints.
- [`DemoSerial`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L66-L130) ([`motor_session.py`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py)): In-memory packet simulator modeling motor IDs 0 and 1 with synthetic dynamics, error injections, and watchdog timeout latches without touching host serial ports.
- [`Dashboard`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L84-L545) ([`motor_dashboard.py`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py)): PySide6 desktop GUI managing controls, plots, diagnostics, UI heartbeats (via 100 ms timer), and user requests dispatched to `MotorSession`.
- Standalone CLI [`spin_5rpm.main`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L81-L141) & [`spin_5rpm.run`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L29-L79) ([`spin_5rpm.py`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py)): Direct command-line entry point that opens an exclusive physical serial port, runs bounded single-motor motion tests, verifies health gates, and guarantees a 3-packet Stop shutdown sequence.
- [`healthy`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L18-L27) ([`spin_5rpm.py`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py)): Strict runtime safety policy gate verifying mode match, zero error/warning bits, latched watchdog timeout, voltage (24–72 V), driver (-10–70 °C), winding (≤90 °C), and torque (±4 Nm).
- [`exchange`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L62-L108) ([`im6014_serial.py`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py)): Bounded synchronous serial transaction function that clears input buffers, writes a single command packet, dynamically manages monotonic deadlines, scans responses for valid headers (`0xFC 0xEE`), and restores driver timeouts on all exit branches.
- Codec functions [`command`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L23-L42), [`feedback`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L44-L57), and [`crc32`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L11-L20) ([`im6014_codec.py`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py)): Pure encoding and decoding functions converting between physical SI units (radians, rpm, Nm) and raw 20-byte command / 26-byte feedback frames using internal gear ratio 38/3.
- [`ForbiddenMotorIO`](file:///home/sid/Documents/code/im6014Workbench/test_startup.py#L64-L80) ([`test_startup.py`](file:///home/sid/Documents/code/im6014Workbench/test_startup.py)): Test sentinel interceptor that traps unauthorized attempts to open serial ports or execute packet transactions during disconnected startup.
- [`StartupVerifierTests`](file:///home/sid/Documents/code/im6014Workbench/test_startup_verifier.py#L30-L167) ([`test_startup_verifier.py`](file:///home/sid/Documents/code/im6014Workbench/test_startup_verifier.py)): Offline regression suite that verifies disconnected startup, validates JSON receipts, and injects deliberate faults into constructor sentinels and close handlers.
- [`FakeSerial`](file:///home/sid/Documents/code/im6014Workbench/fake_serial.py#L48-L215) & [`FakeClock`](file:///home/sid/Documents/code/im6014Workbench/fake_serial.py#L29-L46) ([`fake_serial.py`](file:///home/sid/Documents/code/im6014Workbench/fake_serial.py)): Deterministic offline mock serial transport modeling delays, fragmented byte delivery, and monotonic clock advances for unit testing.

---

### Flow

#### 1. Disconnected Startup and Shutdown Flow
1. **Entry**: User or script invokes [`motor_dashboard.main()`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L546-L555).
2. **Dashboard instantiation**: `main()` constructs [`Dashboard(demo)`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L84-L142).
   - In [`Dashboard.__init__`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L87), `self.session = MotorSession()` is instantiated.
   - In [`MotorSession.__init__`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L132-L155), `self.bus = None` is initialized. Daemon thread `self.thread` starts running [`_loop()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L400-L427).
   - The UI timer [`self.timer.start(100)`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L140) starts a 100 ms refresh cycle calling [`Dashboard.refresh()`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L471-L539).
   - In `refresh()`, [`self.session.ping()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L156-L157) updates `self.last_ui = time.monotonic()`.
   - Data state remains `state='Disconnected'`, `connected=False`, `bus=None`, `tx=0`, `rx=0`. No port open or transaction is attempted.
3. **Shutdown**: Window closes via [`Dashboard.closeEvent()`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L540-L544).
   - `self.timer.stop()` halts the UI refresh timer.
   - [`self.session.close()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L172-L176) calls `self.request('stop')`, sets `self.closing = threading.Event()`, and joins `self.thread.join(2)`.
   - In `MotorSession._loop()`, the `finally` block executes [`self._disconnect()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L242-L250), ensuring the worker terminates.

#### 2. GUI Connect & Check Lifecycle Flow
1. **User Action**: User selects adapter/demo and clicks "Connect" ([`Dashboard.connect_motor`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L362-L370)).
2. **Job Queue**: `connect_motor()` invokes [`self.session.request('connect', port=..., id=..., baud=..., demo=...)`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L164-L170), enqueueing `(self.generation, 'connect', values)` into [`self.jobs`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L134).
3. **Worker Processing**: In [`MotorSession._loop`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L410-L415), the job is dequeued and passed to [`_handle('connect', v)`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L271-L292).
   - If demo: `self.bus = DemoSerial()` ([`motor_session.py#L283`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L283)).
   - If real: `self.bus = serial.Serial(..., exclusive=True)` and `self.bus.open()` ([`motor_session.py#L285-L289`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L285-L289)).
   - `self._set(connected=True)` updates session data under `self.lock`.
4. **Immediate Check**: `_handle('connect', ...)` automatically calls [`self._check()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L251-L269).
   - Calls [`self._stop()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L215-L231) which dispatches a Stop transaction (`mode=0`) via [`_transact()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L186-L214).
   - `_transact()` calls [`im6014_serial.exchange(self.bus, packet, motor_id)`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L62-L108).
   - Returns feedback dictionary `f` and evaluates [`healthy(f, 0, allow_timeout=True)`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L18-L27).
   - If `f['timeout'] == 1`: transitions to `state='Timeout — disarmed'` ([`motor_session.py#L262`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L262)). User must explicitly trigger `ack` before arming.
   - If `f['timeout'] == 0`: transitions to `state='Ready · Stop'` ([`motor_session.py#L268`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L268)).

#### 3. Arming and Bounded Test Execution Flow
1. **Arm Request**: User checks "Motor secured / unloaded" and clicks "Arm one test" ([`Dashboard._controls`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L197)).
   - Enqueues `'arm'` via `self.session.request('arm', supported=True)`.
   - [`_handle('arm')`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L298-L308) validates:
     - `self.data['state'] == 'Ready · Stop'`
     - `supported is True`
     - Feedback age `now - self.last_reply <= 0.25 s`
     - UI heartbeat `now - self.last_ui <= 0.5 s`
     - `healthy(self.data['latest'], 0)`
   - Sets `self.arm_until = time.monotonic() + 10` and `state='Armed · motor stopped'`.
2. **Test Start**: User clicks "Run test" ([`Dashboard.run_test`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L415-L416)).
   - Enqueues `'test'` with parameters (`kind`, `rpm`, `kd`, `ff`, `torque`, `duration`, `position_deg`, etc.).
   - [`_handle('test')`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L309-L338) validates inputs with [`validate_test(v)`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L37-L64).
   - Verifies `self.data['armed'] is True`, `now <= self.arm_until`, feedback age `≤ 0.1 s`, UI heartbeat `≤ 0.5 s`.
   - Sends zero-torque arming transition transaction (`mode=1`) to account for firmware previous-mode reporting, then a second transaction confirming FOC mode 1.
   - Records `self.test` parameters and sets `state='Testing'`.
3. **Trajectory Execution**: Worker thread executes [`_tick()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L359-L390) every ~10 ms in `_loop()`.
   - Checks heartbeats: `now - self.last_ui > 0.5 s` or `now - self.last_reply > 0.1 s` raises `RuntimeError` (tripping fault).
   - If `elapsed >= t['duration']`: calls `_stop()`, `_check()`, logs completion, and returns to `Ready · Stop`.
   - Otherwise, calculates smoothstep/ramp targets, transmits packet via `_transact(1, ...)`, checks `healthy(f, 1, speed_limit=...)`, checks travel trip limits, and updates progress.

#### 4. Priority Stop, Fault Latching, and Disarm Flow
1. **Operator Stop Trigger**: User clicks Stop button, presses `Esc`, or calls `request('stop')`.
   - [`MotorSession.request('stop')`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L165-L169) locks `self.lock`, increments `self.generation += 1`, and sets `self.stop_requested.set()`.
   - The incremented generation automatically invalidates and discards all pending unhandled motion/arm requests in `self.jobs` ([`motor_session.py#L414`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L414)).
2. **Worker Interception**:
   - In `_transact()`: if `mode != 0` and `(self.stop_requested.is_set() or self.closing.is_set())`, raises `InterruptedError('Stop requested')` ([`motor_session.py#L187-L188`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L187-L188)).
   - In `_loop()`: `if self.stop_requested.is_set():` calls [`self._stop()`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L215-L231) which dispatches Stop packet (`mode=0`) up to two times to verify the motor confirms `mode == 0`.
   - Clears `stop_requested`, sets `state='Stopped · re-check required'`, and sets `armed=False`.
3. **Fault Trigger**: If any transaction raises a CRC error, deadline timeout, safety limit trip, or communication failure:
   - Worker catches `Exception` in `_loop()` and invokes [`self._fault(error)`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L232-L241).
   - `_fault()` calls `self._stop()`.
   - Sets `state='Fault — disarmed'`, `reason=str(error)`, `armed=False`.
   - In subsequent `_tick()` iterations, if the port remains open, background polling continues sending Stop transactions at 20 Hz (`now - self.last_poll >= 0.05`), but `healthy(f, 0)` is bypassed ([`motor_session.py#L394-L395`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L394-L395)) so telemetry updates without raising repeated exceptions.
   - The fault is latched; only an explicit user `_check()` or reconnect can transition out of `Fault — disarmed`.

#### 5. Standalone CLI Bus Ownership Lifecycle Flow
1. **Entry**: CLI execution via [`spin_5rpm.main()`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L81-L140).
2. **Bus Acquisition**: Lines 110–115 open `serial.Serial(port=args.port, baudrate=args.baud, timeout=0.005, write_timeout=0.05, exclusive=True)` directly in the main thread.
3. **Execution**: Calls [`spin_5rpm.run(bus, args, log)`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L29-L79).
   - Transacts Stop packet to verify initial communication.
   - If `first['timeout'] == 1` and `--ack-timeout` is given, sends `watchdog=False` acknowledge, then restores watchdog.
   - If `--arm` is NOT specified, exits cleanly at stopped state without moving.
   - If `--arm` is specified, sends zero-torque transition to FOC (`mode=1`), loops through 0.75 s ramps for `--duration`, and checks 0.1 s host watchdog.
4. **Shutdown Guarantee**: In `spin_5rpm.main()`'s `finally:` block ([`spin_5rpm.py#L120-L134`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L120-L134)):
   - If `bus.is_open`: unconditionally sends 3 Stop packets (`exchange(bus, command(args.id), args.id)`) spaced by 20 ms.
   - Records `stopped = (f['mode'] == 0)`.
   - Calls `bus.close()`.

---

### Files Read

- [`AGENTS.md`](file:///home/sid/Documents/code/im6014Workbench/AGENTS.md)
- [`docs/ROBOT_WORKBENCH_AGENT_PROMPT.md`](file:///home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md)
- [`docs/architecture.md`](file:///home/sid/Documents/code/im6014Workbench/docs/architecture.md)
- [`docs/milestones.tsv`](file:///home/sid/Documents/code/im6014Workbench/docs/milestones.tsv)
- [`docs/decisions.tsv`](file:///home/sid/Documents/code/im6014Workbench/docs/decisions.tsv)
- [`docs/bench-baseline.md`](file:///home/sid/Documents/code/im6014Workbench/docs/bench-baseline.md)
- [`docs/boundary-design.md`](file:///home/sid/Documents/code/im6014Workbench/docs/boundary-design.md)
- [`docs/b11-c06-workflow.md`](file:///home/sid/Documents/code/im6014Workbench/docs/b11-c06-workflow.md)
- [`/tmp/pstack-b11-c06-20261007-1mwdi1cr/frame.md`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/frame.md)
- [`/tmp/pstack-b11-c06-20261007-1mwdi1cr/how-controller-prompt.md`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/how-controller-prompt.md)
- [`/home/sid/.codex/plugins/cache/open-pstack/pstack/1.13.0/skills/how/references/explorer-prompt.md`](file:///home/sid/.codex/plugins/cache/open-pstack/pstack/1.13.0/skills/how/references/explorer-prompt.md)
- [`motor_session.py`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py)
- [`motor_dashboard.py`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py)
- [`spin_5rpm.py`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py)
- [`im6014_codec.py`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py)
- [`im6014_serial.py`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py)
- [`fake_serial.py`](file:///home/sid/Documents/code/im6014Workbench/fake_serial.py)
- [`test_startup.py`](file:///home/sid/Documents/code/im6014Workbench/test_startup.py)
- [`test_startup_verifier.py`](file:///home/sid/Documents/code/im6014Workbench/test_startup_verifier.py)
- [`test_dashboard.py`](file:///home/sid/Documents/code/im6014Workbench/test_dashboard.py)
- [`test_spin_5rpm.py`](file:///home/sid/Documents/code/im6014Workbench/test_spin_5rpm.py)
- [`test_protocol.py`](file:///home/sid/Documents/code/im6014Workbench/test_protocol.py)
- [`test_serial_deadline.py`](file:///home/sid/Documents/code/im6014Workbench/test_serial_deadline.py)
- [`scripts/check_offline_baseline.sh`](file:///home/sid/Documents/code/im6014Workbench/scripts/check_offline_baseline.sh)

---

### Boundaries

#### 1. Exact Owner / Open Sites
Currently, bus ownership is strictly isolated between two independent open sites:
- **Site 1 (GUI Worker Owner)**: [`motor_session.py#L282-L289`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L282-L289) inside `MotorSession._handle('connect')`:
  ```python
  if v.get('demo'):
      self.bus = DemoSerial()
  else:
      self.bus = serial.Serial(port=None, baudrate=baud, timeout=0.005,
                               write_timeout=0.05, exclusive=True)
      self.bus.port = v['port']
      self.bus.dtr = self.bus.rts = False
      self.bus.open()
  ```
  `self.bus` is held as an instance attribute of `MotorSession` and accessed exclusively by the `IM6014 serial owner` background thread.
- **Site 2 (Standalone CLI Owner)**: [`spin_5rpm.py#L110-L115`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L110-L115) inside `spin_5rpm.main()`:
  ```python
  bus = serial.Serial(port=None, baudrate=args.baud, timeout=0.005, write_timeout=0.05, exclusive=True)
  bus.port = args.port
  bus.dtr = False
  bus.rts = False
  bus.open()
  ```
  `bus` is held as a local variable in `main()` on the CLI process main thread.
- **Exclusivity Mechanism**: Both sites rely solely on `pyserial`'s `exclusive=True` flag (which sets the POSIX `TIOCEXCL` ioctl on Linux). There is currently no shared cross-process mutex or IPC station arbitrator.

#### 2. Plan C01–C06 Integration and Contract Preservation
Plan C01–C06 introduces the foundation for a shared controller:
- **C01 (One bus owner)**: "A second controller cannot independently open the same configured hardware."
  - When an initial shared controller abstraction is introduced, it must act as the sole hardware arbitrator.
  - To avoid breaking bench entry points (`motor_dashboard.py` and `spin_5rpm.py`), `MotorSession` and `spin_5rpm` must either delegate their open calls through this controller, or the controller must enforce an exclusive lock/token on the hardware target.
- **C02 (Controller states)**: "Illegal transitions produce no motor command."
  - Formalizes states (`Disconnected`, `Stopped`, `Armed`, `Moving`, `Fault`).
  - If a command arrives in an invalid state, it must fail cleanly at the controller boundary and produce zero packets on the bus.
- **C03 (Per-joint feedback age)**: "Fresh J1 feedback cannot conceal stale J2 feedback."
  - Currently, `MotorSession` tracks a single scalar `self.last_reply` ([`motor_session.py#L142`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L142)) and returns `age_s` for the active ID.
  - The shared controller must store feedback timestamps per joint ID (e.g. `feedback_ages: dict[int, float]`), ensuring any joint exceeding the age threshold triggers a stale-link fault.
- **C04 (Per-joint error status)**: "One motor fault identifies that motor and affects the appropriate build."
  - Fault status must be attributed specifically to the offending joint ID rather than setting a generic single-motor fault.
- **C05 (Bounded target storage)**: "Ten thousand updates leave bounded storage and the newest valid target."
  - Currently, `MotorSession.jobs` is an unbounded `queue.Queue()`.
  - The shared controller target buffer must have a fixed capacity (e.g. retaining only the newest target), preventing memory buildup or latency backlog under high-frequency setpoint updates.
- **C06 (Command sequence validation)**: "Duplicate and older commands cannot replace a newer target."
  - Targets must carry monotonically increasing sequence numbers. Any sequence number $\le$ the currently processed sequence number is discarded.

#### 3. How to Integrate Without Breaking Bench Entry Points
1. **Preserve `motor_session.py` interface and imports**:
   - `test_startup.py` ([lines 230–236](file:///home/sid/Documents/code/im6014Workbench/test_startup.py#L230-L236)) and `test_startup_verifier.py` ([lines 46–50](file:///home/sid/Documents/code/im6014Workbench/test_startup_verifier.py#L46-L50)) monkey-patch:
     - `motor_session.serial.Serial`
     - `motor_session.DemoSerial`
     - `motor_session.exchange`
   - If the shared controller moves the open site or exchange call into a new module, `motor_session.py` **must retain aliases** for `serial`, `DemoSerial`, and `exchange` (or preserve those exact patch points). Otherwise, offline startup verification and fault injection tests will fail immediately.
2. **Preserve `MotorSession` public API for `motor_dashboard.py`**:
   - `motor_dashboard.py` directly calls:
     `session.request('connect' | 'disconnect' | 'check' | 'ack' | 'arm' | 'test' | 'scan' | 'stop')`,
     `session.snapshot()`, `session.ping()`, `session.close()`.
   - `snapshot()` must continue returning the exact expected keys: `'state'`, `'connected'`, `'demo'`, `'id'`, `'port'`, `'baud'`, `'armed'`, `'latest'`, `'reason'`, `'tx'`, `'rx'`, `'crc_errors'`, `'reply_timeouts'`, `'rtt_ms'`, `'target_rpm'`, `'last_tx'`, `'last_rx'`, `'discovered'`, `'test_remaining'`, `'total_samples'`, `'stopped'`, `'connection'`, `'samples'`, `'events'`, `'age_s'`.
3. **Preserve `spin_5rpm.py` standalone script**:
   - `spin_5rpm.py` must retain its argument structure (`--port`, `--id`, `--baud`, `--arm`, `--ack-timeout`, etc.), its `healthy` function, its codec re-exports, and its return/exit codes.

#### 4. No-Rearm Constraints
The codebase adheres to a strict, non-negotiable safety invariant: **No automatic rearming or motion restart**.
1. **Disconnected to Connected**: Connect initiates Stop and status only; `armed` remains `False`.
2. **Latched Startup Timeout**: Latched `timeout=1` prevents arming; requires explicit `ack` while motor is in Stop (`mode=0`), followed by watchdog restoration.
3. **Explicit Arming Gate**: Arming requires explicit user interaction (`request('arm', supported=True)`), requires a healthy motor in Stop mode, valid physical safety checkbox, fresh feedback (< 250 ms), and active UI heartbeat (< 500 ms).
4. **Arm Lifetime**: Arm expires automatically after 10 seconds (`self.arm_until`).
5. **Single-Use Arm**: Arm is consumed by a single test execution. When a test completes, the session returns to `Ready · Stop` with `armed=False`.
6. **No Rearm on Stop**: Pressing Stop, invoking Esc, or stopping via test completion transitions to `Stopped · re-check required` or `Ready · Stop`; it **never** rearms.
7. **No Rearm on Fault Recovery**: If communications drop (timeout, CRC) or an error occurs, the session enters `Fault — disarmed`. If packets recover, motion is **never** resumed; the state remains `Fault — disarmed` until user manually checks connection.
8. **No Rearm on Dashboard Close**: Closing the dashboard disarms the session and shuts down the worker thread.

---

### Non-Obvious Things

1. **Monkey-Patching Sentinels in Startup Tests**:
   [`test_startup.py`](file:///home/sid/Documents/code/im6014Workbench/test_startup.py#L231-L236) and [`test_startup_verifier.py`](file:///home/sid/Documents/code/im6014Workbench/test_startup_verifier.py#L46-L50) explicitly check and patch attributes on the `motor_session` module:
   ```python
   patch.object(motor_session.serial, "Serial", io.serial)
   patch.object(motor_session, "DemoSerial", io.demo)
   patch.object(motor_session, "exchange", io.exchange)
   ```
   If `motor_session.py` were refactored to delegate to a controller and removed `import serial` or `from spin_5rpm import exchange`, the test fixtures in `test_startup_verifier.py` would fail with an `AttributeError`, breaking baseline verification.
2. **Source Fingerprint Gate in `test_startup.py`**:
   [`test_startup.py#L112-L120`](file:///home/sid/Documents/code/im6014Workbench/test_startup.py#L112-L120) and [`scripts/check_offline_baseline.sh#L81`](file:///home/sid/Documents/code/im6014Workbench/scripts/check_offline_baseline.sh#L81) contain a hardcoded list of repository files whose byte counts and SHA-256 digests are verified. Adding a new file (e.g. `motor_controller.py`) requires updating both `check_offline_baseline.sh` and `test_startup._fingerprint()`.
3. **Generation Invalidation on Stop**:
   In [`MotorSession.request('stop')`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L165-L169), `self.generation` is incremented under `self.lock`. Because jobs in `self.jobs` are tuples `(generation, action, values)`, any pending motion or arm jobs queued before the stop request are silently discarded in `_loop()` because their generation no longer matches `self.generation`.
4. **Persistent Polling During Latched Faults**:
   When `_fault()` is invoked, `self.bus` is NOT closed unless the OS port was already closed ([`motor_session.py#L235`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L235)). In fact, `_tick()` continues to send 20 Hz Stop packets (`mode=0`) via `_transact()` while in `Fault — disarmed` ([`motor_session.py#L391-L395`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L391-L395)), but skips calling `healthy(f, 0)`. This allows live voltage, temperature, and fault bits to display in the UI without re-raising exceptions in the loop.
5. **Firmware Mode Reporting Latency**:
   On the first packet transitioning from Stop (`mode=0`) to FOC (`mode=1`), the IM6014 firmware returns the *previous* state (`mode=0`) in its feedback packet. Both `spin_5rpm.run()` ([`spin_5rpm.py#L50-L51`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L50-L51)) and `MotorSession._handle('test')` ([`motor_session.py#L332-L335`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L332-L335)) deliberately send an initial zero-torque transition command and accept `mode=0` feedback, followed by a second transaction requiring `mode=1`.
6. **Watchdog Acknowledgment Handshake**:
   At power on, IM6014 firmware latches `timeout=1`. This timeout bit can **only** be cleared by sending a packet with `watchdog=False` in Stop mode (`mode=0`), followed immediately by a packet with `watchdog=True`. Attempting to clear the timeout in FOC mode (`mode=1`) is rejected by the codec ([`im6014_codec.py#L35-L36`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L35-L36)).

---

### Open Questions

1. **Inter-Process vs In-Process Scope for C01**:
   Does milestone C01 ("Enforce one bus owner: A second controller cannot independently open the same configured hardware") target cross-process arbitration (e.g. between `motor_dashboard.py` and `spin_5rpm.py` using OS-level lock files or a socket daemon) or in-process arbitration (a singleton controller rejecting a second instantiate/open attempt)?
2. **Multi-Joint Polling Cadence (C03)**:
   In a multi-joint station (J1–J6), synchronous RS-485 transactions take ~1–2 ms round-trip per joint. Polling 6 joints sequentially will increase loop transaction time to ~6–12 ms. How does this affect the 100 ms feedback watchdog and the current single-joint 50 ms polling interval in `MotorSession._tick()`?
3. **CLI Integration with Shared Controller**:
   Will `spin_5rpm.py` be modified in C01 to acquire hardware through the shared controller, or does `spin_5rpm.py` remain an independent legacy CLI baseline while a new station runner uses the controller?