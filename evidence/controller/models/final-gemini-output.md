## Adversarial Code Review: B11–B14 & C01–C06

**Review Target:** Commit `075973b02d909e22c2f120be6045a7cf65cb6665`  
**Scope:** Strict external joint mappings, numerical rejection before bus I/O, passive adapter configuration, bounded diagnostic capture, cooperating bus ownership, and transport-free fixed-joint controller lifecycle.

---

### 1. Critical & High-Priority Bugs and Logic Flaws

#### 1.1 Inverted Validation Ordering in `JointCore.submit_target`: Unnecessary CRC Work & Misleading Error Codes
* **File:** [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L560-L578) (lines 560–578)
* **Problem:** In [`JointCore.submit_target`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L543-L582), the core transforms every joint target via `spec.mapping.to_output()` and encodes it with `im6014_codec.command()` (which constructs the 20-byte wire frame and runs 32-bit hardware CRC loops across every joint) **before** verifying the command sequence watermark:
  ```python
  560:  try:
  561:      for spec, joint_target in zip(self._specs, target.joints):
  562:          output = spec.mapping.to_output(joint_target)
  563:          command(...)
  573:  except ValueError:
  574:      return self._rejected(DecisionCode.INVALID_ARGUMENT)
  575:
  576:  if target.sequence <= self._last_sequence:
  577:      return self._rejected(DecisionCode.DUPLICATE_OR_OLDER)
  ```
* **Impact:**
  1. **Performance penalty:** For every duplicate, stale, or retransmitted target packet (common in robotic control loops and lossy/jittery streams), the core burns CPU cycles performing floating-point conversions and full polynomial CRC calculations across all joints, only to discard the result at line 576.
  2. **Violated Decision Hierarchy:** If an older or duplicate target packet arrives with an out-of-range field (e.g., velocity or torque clamp violation), `submit_target()` returns `DecisionCode.INVALID_ARGUMENT` rather than `DecisionCode.DUPLICATE_OR_OLDER`. A sequence watermark is a front-line ordering gate; out-of-sequence targets must be rejected immediately without evaluating their payload.

#### 1.2 Unreachable Guards & Dead Decision Code (`DecisionCode.UNHEALTHY`) in State Machine
* **File:** [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L490-L503) (lines 497–498, 515–516, 551–552)
* **Problem:** In [`JointCore.checked()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L489-L504), [`arm()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L505-L519), and [`submit_target()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L543-L582), the code explicitly checks:
  ```python
  if any(not joint.healthy for joint in self._observations):
      return self._rejected(DecisionCode.UNHEALTHY)
  ```
  However, in [`JointCore.receive()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L431-L462) and [`communication_failed()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L463-L488), whenever an observation is unhealthy or communication fails, `_latch_joint_fault()` is immediately called:
  ```python
  self._state = State.FAULT
  ```
* **Impact:** 
  - Any time a joint becomes unhealthy, `self._state` has *already* transitioned to `State.FAULT`.
  - When `checked()` is subsequently invoked, line 490 (`if self._state is not State.CHECKING: return self._rejected(DecisionCode.INVALID_STATE)`) triggers first.
  - `DecisionCode.UNHEALTHY` can **never be returned by `checked()`**. It is completely dead code. In fact, [`test_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/test_controller.py#L377-L381) explicitly records that receiving an unhealthy packet during `CHECKING` causes `checked()` to return `DecisionCode.INVALID_STATE`.
  - In `arm()` and `submit_target()`, checking `any(not joint.valid ...)` is similarly unreachable because transitioning out of `CHECKING` into `READY` requires all joints to be valid, and observations are never invalidated while remaining in `READY`/`ARMED`/`MOVING`.

#### 1.3 `consume_target()` Leaves Stale Pending Target on Unhealthy Observation
* **File:** [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L583-L594) (lines 587–589)
* **Problem:** Look at the consumption logic:
  ```python
  586:  self.tick()
  587:  if self._state is State.FAULT or any(not joint.healthy for joint in self._observations):
  588:      return None
  589:  target = self._pending_target
  590:  self._pending_target = None
  ```
  If `any(not joint.healthy)` triggers on line 587, the function exits with `return None` **without clearing `self._pending_target`**.
* **Impact:** While `_latch_joint_fault` normally sets `self._pending_target = None` when a fault is latched, if an observation health check fails at line 587, `self._pending_target` is left retained. The requirement states: *"Submission/consumption must never release stale or unhealthy targets."* If `consume_target` drops execution due to unhealthy feedback, leaving the pending target sitting in memory risks releasing that stale target later if the state recovers or transitions. `self._pending_target = None` must be unconditionally enforced before returning `None`.

---

### 2. Security & Operating System Boundary Vulnerabilities

#### 2.1 Predictable Lock Directory in World-Writable `/tmp` Enables Local Denial-of-Service
* **File:** [`im6014_port.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/im6014_port.py#L14-L15) (line 14), lines 117–143, 173–183
* **Problem:** `_LOCK_DIRECTORY` is hardcoded as:
  ```python
  _LOCK_DIRECTORY = f'/tmp/im6014-port-owner-{os.getuid()}'
  ```
  In `_private_lock_fd()`, the code creates this directory and runs `_validate_private_directory()`, which enforces:
  ```python
  if before.st_uid != uid or stat.S_IMODE(before.st_mode) != 0o700:
      raise PortOwnershipError('Bus lock namespace must be owned by this user with mode 0700')
  ```
* **Impact:** On any multi-user Linux system or shared host, `/tmp` is world-writable with sticky bit (`0o1777`). Any unprivileged attacker, service account (e.g., `nobody`), or compromised container user can pre-create `/tmp/im6014-port-owner-<target_uid>` owned by themselves with mode 0700.
  - When the target user runs `open_managed_port`, `os.mkdir()` raises `FileExistsError` (silently swallowed).
  - Next, `_validate_private_directory` detects `before.st_uid != uid` and raises `PortOwnershipError`.
  - Because `/tmp` is sticky, the target user **cannot delete or rename the attacker's directory**.
  - The workbench is permanently locked out from opening *any* character device or serial port until an administrator intervenes. The design rejected `XDG_RUNTIME_DIR` (e.g. `/run/user/<uid>`) to prevent environment overrides, but created a textbook `/tmp` pre-creation DoS vulnerability.

#### 2.2 Unbounded Lockfile Leakage for Recycled & Virtual Devices
* **File:** [`im6014_port.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/im6014_port.py#L184) (line 184), line 207
* **Problem:** Lockfiles are named by major and minor character device numbers:
  ```python
  name = f'{os.major(device_number)}-{os.minor(device_number)}.lock'
  ```
  Line 207 mandates: `Never unlink lockfiles: all processes must keep locking the same inode.`
* **Impact:** For physical USB FTDI ports with static minor numbers, this is bounded. But for Linux PTYs (`/dev/pts/*`) or dynamically allocated tty devices (which cycle minor numbers over continuous integration runs, simulated unit tests, or reconnects), zero-byte `.lock` files accumulate indefinitely in `/tmp/im6014-port-owner-<uid>/`. Over long test cycles or automated fuzzing, thousands of inode entries leak without any garbage collection or rotation.

---

### 3. Data Model & Field Synchronization Defects

#### 3.1 Field Desynchronization in `JointFault` Between `communication_error` and `communication_errors`
* **File:** [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L115-L119) (lines 115–119), lines 360–371
* **Problem:** In [`JointFault`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L85-L120):
  ```python
  if self.communication_error is not None and not self.communication_errors:
      object.__setattr__(self, "communication_errors", (self.communication_error,))
  elif self.communication_errors and self.communication_error is None:
      object.__setattr__(self, "communication_error", self.communication_errors[-1])
  ```
  In `_latch_joint_fault()`:
  ```python
  if communication_error is not None and communication_error not in communication_errors:
      communication_errors += (communication_error,)
  fault = JointFault(
      ...,
      communication_error=communication_error or existing.communication_error,
      communication_errors=communication_errors,
  )
  ```
* **Impact:**
  - Suppose a joint experiences `TIMEOUT`, followed by `DRIVER_FAILURE`. `communication_errors` is `(TIMEOUT, DRIVER_FAILURE)`.
  - Next, an additional failure occurs: another `TIMEOUT`.
  - Line 361 evaluates `communication_error not in communication_errors`: since `TIMEOUT` is already in the tuple, `communication_errors` is NOT modified (still `(TIMEOUT, DRIVER_FAILURE)`).
  - Line 369 assigns `communication_error = TIMEOUT`.
  - Now, `fault.communication_error` is `TIMEOUT`, but `fault.communication_errors[-1]` is `DRIVER_FAILURE`. The single summary field contradicts the tail of the error history tuple.
  - Furthermore, `JointFault.__post_init__` performs no validation when both fields are provided, allowing arbitrary desynchronized records to be constructed.

#### 3.2 Sequence Number Watermark Saturation Permanently Wedges `JointCore`
* **File:** [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L216-L217) (lines 216–217), lines 576–577
* **Problem:** `Target` validates that `sequence` is an integer in `[0, (1 << 64) - 1]`. `JointCore` rejects any target with `target.sequence <= self._last_sequence`.
* **Impact:** If a client submits a target with `sequence = (1 << 64) - 1`, that target is accepted and sets `self._last_sequence = (1 << 64) - 1`. Because `(1 << 64) - 1` is the absolute maximum integer permitted by `Target`, no subsequent target can *ever* satisfy `target.sequence > self._last_sequence`.
  - Because sequence watermarks explicitly survive `stop()`, `disconnect()`, `reconnect()`, and recovery checks, the `JointCore` instance is permanently incapacitated and can never accept another motion target.
  - While generational rollover is deferred to C08, failing to document or handle sequence saturation (or rejecting `(1 << 64) - 1` as a reserved sentinel) leaves the state machine vulnerable to permanent lockup if an upstream generator wraps or initializes to `2**64 - 1`.

---

### 4. Concurrency, I/O & Maintainability Concerns

#### 4.1 Strict `age < 0.0` Check in Monotonic Clock Calculation Risks Spurious `ValueError`
* **File:** [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L389-L394) (lines 389–394)
* **Problem:** Look at `JointCore._age`:
  ```python
  @staticmethod
  def _age(received_at: float, now: float) -> float:
      age = now - received_at
      if not math.isfinite(age) or age < 0.0:
          raise ValueError("clock must be monotonic and feedback age finite")
      return age
  ```
* **Impact:** In high-rate real-time control loops or environments with sub-microsecond timer resolution and floating-point arithmetic (e.g., fast consecutive calls to `time.monotonic()` where `now` is queried in rapid succession), floating-point subtraction `now - received_at` can evaluate to `-0.0` or `-1e-17` due to IEEE-754 precision quirks. A zero-tolerance `age < 0.0` check raises a fatal `ValueError` that crashes `tick()`, `submit_target()`, or `snapshot()`. The check should clamp sub-epsilon negative differences (`if -1e-9 < age < 0: age = 0.0`) to avoid crashing on microsecond rounding noise.

#### 4.2 GUI Thread Hang in `MotorSession.close()` on Unresponsive Serial Driver
* **File:** [`motor_session.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/motor_session.py#L187-L194) (lines 187–194)
* **Problem:** In [`MotorSession.close()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/motor_session.py#L187-L194):
  ```python
  def close(self):
      self.request('stop')
      self.closing.set()
      self.thread.join(2)
      if not self.thread.is_alive() and self.port_owner:
          self._close_managed_port()
      return not self.thread.is_alive() and self.port_owner is None
  ```
* **Impact:** If the serial worker thread is blocked inside a synchronous operating system call (`bus.read()` or driver flush on an unplugged USB adapter), `self.closing.set()` cannot preempt it. The UI thread blocks for a full 2 seconds. If `join(2)` expires, `not self.thread.is_alive()` is False: `self._close_managed_port()` is skipped, leaving the device lease held and the background thread hung.

#### 4.3 `JointMap.to_output()` Omits Physical Protocol Bounds Checking
* **File:** [`joint_mapping.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_mapping.py#L64-L83) (lines 64–83)
* **Problem:** `JointMap.to_output()` converts joint coordinates to actuator space and instantiates `OutputTarget`. `OutputTarget.__post_init__` validates that values are finite and gains are non-negative, but enforces no protocol range boundaries (`rpm`, `tau_nm`).
* **Impact:** An `OutputTarget` with physical impossibilities (e.g. `tau_nm = 1e6` or `rpm = 1e9`) is happily constructed as "valid" by `joint_mapping`. Any non-codec consumer relying on `OutputTarget` will receive unvalidated, dangerous actuator commands.

---

### Summary Checklist of Recommended Corrections

| ID | Component | File | Recommended Fix |
|---|---|---|---|
| **1.1** | Target validation | [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L560) | Move `if target.sequence <= self._last_sequence:` **before** running kinematic conversions and CRC generation. |
| **1.2** | Decision codes | [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L497) | Remove dead `any(not joint.healthy)` checks in `checked()`, or defer `State.FAULT` transition until inspection. |
| **1.3** | Target consumption | [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L587) | Unconditionally clear `self._pending_target = None` when `consume_target()` rejects an unhealthy observation. |
| **2.1** | Lock directory | [`im6014_port.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/im6014_port.py#L14) | Use `XDG_RUNTIME_DIR` (`/run/user/<uid>`) with a secure fallback; prevent world-writable `/tmp` DoS pre-creation. |
| **2.2** | Lockfile leak | [`im6014_port.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/im6014_port.py#L184) | Implement a sweep/pruning mechanism for unreferenced dynamic PTY lockfiles. |
| **3.1** | Fault attribution | [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L115) | Sync `communication_error` with `communication_errors[-1]` consistently upon repeated failure occurrences. |
| **3.2** | Watermark limit | [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L216) | Reserve `(1 << 64) - 1` or define an explicit reset/epoch mechanism to prevent permanent controller wedging. |
| **4.1** | Timestamp calculation | [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/final-gemini-review/joint_controller.py#L389) | Tolerate sub-microsecond floating-point subtraction jitter (`-1e-9 < age < 0`) without raising `ValueError`. |