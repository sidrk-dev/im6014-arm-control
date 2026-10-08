# C07–C16 Controller and Station Design Candidate

## Problem

Milestones C01–C06 established a transport-free, thread-confined `JointCore` with per-joint freshness, attributed fault latches, one pending target, and monotonic sequence numbering, while `im6014_port.py` enforces an OS-level character device lease. However, the existing core lacks command expiry, ownership generations, kinematic bounds (position, velocity, acceleration), measured-position lead limits, combined effort monitoring, thermal/persistent effort fault latches, and station loop metrics. Furthermore, both existing bench callers (`motor_dashboard.py` and `spin_5rpm.py`) open the serial bus directly within the client process. Milestones C07–C16 must introduce a separate station daemon owning the bus refresh loop and device lease, while extending `JointCore` as the single, thread-confined authority for validation, reference generation, and fault management without leaking transport into the core or allowing client bypass.

---

## Usage (Caller's View)

### Call site 1: Client streaming commands with expiry and ownership token
```python
from station_client import StationClient, RequestedAction, JointTarget

client = StationClient.connect(socket_path="/tmp/im6014-station-1000/station.sock")
token = client.claim_ownership(owner_id="teleop_arm")

# Submit streaming target with 50 ms TTL in station monotonic domain
action = RequestedAction(
    sequence=101,
    owner_generation=token.generation,
    ttl_s=0.05,
    joints=(
        JointTarget(q_rad=0.15, dq_rad_s=0.2, tau_nm=0.0, kp=15.0, kd=0.5),
    ),
)
decision = client.submit_action(action)
assert decision.accepted, f"Submission rejected: {decision.code}"

snapshot = client.poll_snapshot()
print(f"Station state: {snapshot.state}, loop period: {snapshot.metrics.actual_period_s*1e3:.2f}ms")
```

### Call site 2: Human takeover and emergency stop revoking previous owner
```python
from station_client import StationClient

supervisor = StationClient.connect(socket_path="/tmp/im6014-station-1000/station.sock")
# Priority takeover increments owner generation and purges pending motion
new_token = supervisor.takeover(owner_id="operator_override")
assert new_token.generation > 1

# Delayed action from teleop_arm with generation 1 is now rejected atomically
# Immediate Stop disarms core and halts actuator refresh
stop_decision = supervisor.stop()
assert stop_decision.accepted
assert supervisor.poll_snapshot().state.name == "STOPPED"
```

### Call site 3: Station daemon execution loop owning serial exchange and core
```python
from station_daemon import StationDaemon
from joint_controller import JointCore, JointLimits
from im6014_port import PortConfig

daemon = StationDaemon(
    port_config=PortConfig("/dev/ttyUSB0", baud=6_000_000),
    limits=(JointLimits(q_min=-1.57, q_max=1.57, v_max=2.0, a_max=8.0, max_lead=0.2, max_effort=3.5),),
    nominal_period_s=0.010,  # 100 Hz refresh
)
daemon.run_until_interrupted()
```

---

## Shape

```mermaid
flowchart TD
    subgraph Station Process [Station Process (station_daemon.py)]
        IPC[IPC Server: Non-blocking UNIX Domain Socket] -->|RequestedAction| MGR[Ownership & Clock Gateway]
        MGR -->|Validated Generation & TTL| JC[JointCore (Thread-Confined Authority)]
        LOOP[Refresh Loop (100 Hz Timer)] -->|Loop Tick (now, dt)| JC
        JC -->|DispatchedReference| DISP[Pure Mapping & Codec Validation]
        DISP -->|Encoded Packets| MP[ManagedPort & im6014_serial]
        MP -->|Raw Bus Bytes| HW[(RS485 Bus / PTY Fixture)]
        HW -->|Feedback Frames| MP
        MP -->|Decoded Frames| JC
        JC -->|Snapshot & LoopMetrics| IPC
    end
    subgraph Client Process [Client Process (station_client.py)]
        CLI[Teleop / Dashboard / Agent] -->|Frame: Length + JSON| IPC
    end
```

### Load-Bearing Structural Decisions

1. **One Authority for Request, Limit, and Expiry**: `JointCore` remains transport-free and thread-confined. It is the sole authority validating `RequestedAction`, transforming it into an `AcceptedAction`, projecting it onto closed feasible kinematic intervals, and producing the `DispatchedReference`.
2. **Four Distinct Action Records** (*per boundary-discipline*):
   - `RequestedAction`: Raw client intent (sequence, generation, relative TTL, raw target).
   - `AcceptedAction`: Validated request accepted into the pending slot.
   - `DispatchedReference`: Kinematically shaped reference ($q, \dot{q}, \ddot{q}, \tau$) actually sent to the actuator codec at loop tick $k$.
   - `MeasuredState`: Actuator feedback received and converted to joint units at tick $k$.
3. **Monotonic Station Clock & Anti-Regression Guard**:
   Station monotonic clock (`time.monotonic`) is the only time source. The core stores `_last_now`. If `_now()` returns a value smaller than `_last_now`, the core latches a `clock_regressed` fault across all joints, disarms, and rejects all subsequent motions. A regressed clock cannot overwrite earlier observations.
4. **Feasibility Projection & Reversal / Zero-Crossing Feasibility**:
   For each joint $i$ at step $k$ with interval $\Delta t = t_k - t_{k-1}$:
   - **Position Enclosure**: $q \in [q_{\min}, q_{\max}]$. Out-of-bounds requests are rejected atomically (*DecisionCode.POSITION_OUT_OF_BOUNDS*).
   - **Measured Lead Enclosure**: $q \in [q_{\mathrm{meas}} - \Delta q_{\mathrm{lead\_max}}, q_{\mathrm{meas}} + \Delta q_{\mathrm{lead\_max}}]$. Violations are rejected atomically (*DecisionCode.LEAD_EXCEEDED*).
   - **Velocity & Acceleration Interval**: $\dot{q}_{\mathrm{feas}} \in [\dot{q}_{\mathrm{ref}} - a_{\max}\Delta t, \dot{q}_{\mathrm{ref}} + a_{\max}\Delta t] \cap [-v_{\max}, v_{\max}]$.
   - **Zero Crossing**: If $\dot{q}_{\mathrm{ref}} > 0$ and target $\dot{q} < 0$, the lower bound $\dot{q}_{\mathrm{ref}} - a_{\max}\Delta t$ enforces smooth deceleration across zero; velocity cannot instantaneously skip zero or reverse.
5. **Combined Effort Prediction & Persistent Fault Recovery**:
   - $\tau_{\mathrm{pred}} = \tau_{\mathrm{ff}} + K_p(q_{\mathrm{ref}} - q_{\mathrm{meas}}) + K_d(\dot{q}_{\mathrm{ref}} - \dot{q}_{\mathrm{meas}})$. If $|\tau_{\mathrm{pred}}| > \tau_{\max}$, the action is rejected (*DecisionCode.EFFORT_LIMIT_EXCEEDED*).
   - Thermal limits ($T_{\mathrm{driver}}, T_{\mathrm{winding}}$) and persistent effort ($|\tau_{\mathrm{meas}}| > \tau_{\mathrm{cont}}$ for $> \Delta t_{\mathrm{window}}$) latch unmaskable faults. Recovery strictly requires `begin_check(acknowledge_fault=True)` with healthy Stop feedback; incoming targets cannot clear latches.
6. **Station IPC & Independence**:
   Station daemon runs independently under `open_managed_port`. Communication uses length-prefixed JSON frames over a private Unix domain stream socket (`0700` directory). Client crash or disconnect immediately disarms the core (sets `STOPPED`) and revokes ownership. Reconnecting clients must re-authenticate and explicitly rearm; no automatic rearm is permitted.

---

## Synthesis Decision

### Selected Base Candidate: Integrated Reference-Core with Asynchronous Station Daemon (Candidate A)
Candidate A keeps kinematic limiting, lead evaluation, and effort prediction inside `JointCore` (backed by a pure math module `joint_kinematics.py`), while `station_daemon.py` owns the non-blocking event loop, device lease, and IPC gateway.

### Alternate Candidate Rejected: Split Safety-Filter Middleware with Synchronous RPC (Candidate B)
Candidate B proposed separating a `SafetyLimiter` middleware from `JointCore`, with synchronous request-reply RPC.
- **Why Candidate B was rejected**:
  - *Shallow modules and information leakage*: Both `SafetyLimiter` and `JointCore` had to track current joint reference state ($q_{\mathrm{ref}}, \dot{q}_{\mathrm{ref}}$), leading to split ownership.
  - *Bypass vulnerability*: A two-tier architecture allows a caller or daemon loop to call `consume_target()` directly, bypassing the filter.
  - *Jitter*: Synchronous socket handling in the bus thread caused packet deadline slips during client payload parsing.

---

## Tradeoffs Accepted

- **We accept non-realtime OS jitter in station loop metrics in exchange for a pure Python standard-library implementation**: We report loop overruns and latencies explicitly rather than requiring RT-PREEMPT kernel patches.
- **We accept discrete 100 Hz step limiting in exchange for avoiding complex continuous spline interpolation**: Reference velocity and acceleration are bounded over discrete intervals $\Delta t$, matching the bus packet rate.
- **We accept replacing `consume_target` with `dispatch_reference` in exchange for eliminating bypass paths**: All existing test call sites are migrated.

---

## Alternatives Considered

1. **Client-side timestamp expiry**: Rejected because client system clocks can drift, step, or be manipulated. Station monotonic time domain is the sole authority.
2. **Shared Memory IPC (POSIX shm + semaphores)**: Rejected due to complexity, lack of authenticated disconnect detection, and high risk of orphaned locks on client crash. Length-prefixed Unix stream sockets provide OS-enforced EOF notification on client death.

---

## Open Questions and Risks

1. *Timeout acknowledgment physical behavior*: Bench code sends Stop-mode acknowledgments for actuator timeouts, but vendor docs conflict. In fixture mode, deterministic actuators simulate clearing, but physical commissioning remains gated.
2. *RS485 turnaround time on multi-motor buses*: 6-motor polling at 100 Hz requires 10 ms cycle time (~1.6 ms/motor). If USB serial turnaround exceeds 2 ms, loop overruns will trigger.

---

## Next Implementation Step

Implement C07: Add `ttl_s`, `expires_at_s` to `Target`/`RequestedAction`, update `JointCore.submit_target` to reject expired commands, and ensure `JointCore.dispatch_reference` purges expired pending targets before dispatch.

---

## Module Map

| File | Ownership / Responsibility |
| --- | --- |
| `joint_kinematics.py` | Pure math: closed-interval kinematic feasibility, zero-crossing limiter, lead bounds, effort prediction. |
| `joint_controller.py` | State machine, ownership generations, expiry, 4 action records, monotonic clock validation, fault latches. |
| `station_daemon.py` | Independent process: owns `open_managed_port`, 100 Hz refresh loop, non-blocking IPC server, takeover/Stop logic. |
| `station_client.py` | Client library: connect to Unix socket, stream requests, handle generation tokens, receive snapshots. |
| `test_station_ipc.py` | Fixture/PTY tests: ownership takeover, client crash disarm, delayed command rejection, no second owner. |
| `test_controller_kinematics.py`| Unit tests: C07–C15 limits, reversals, lead limits, effort bounds, thermal faults, clock regressions. |

---

## Type and Signature Sketches

### `joint_kinematics.py`
```python
from dataclasses import dataclass

@dataclass(frozen=True)
class JointLimits:
    q_min: float
    q_max: float
    v_max: float
    a_max: float
    max_lead: float
    max_effort: float
    driver_temp_max_c: float = 85.0
    winding_temp_max_c: float = 100.0
    persistent_effort_threshold_nm: float = 3.0
    persistent_effort_duration_s: float = 1.0

    def __post_init__(self) -> None:
        if self.q_min >= self.q_max:
            raise ValueError("q_min must be strictly less than q_max")
        if self.v_max <= 0.0 or self.a_max <= 0.0 or self.max_lead <= 0.0 or self.max_effort <= 0.0:
            raise ValueError("v_max, a_max, max_lead, and max_effort must be positive")

def project_kinematic_step(
    q_target: float,
    dq_target: float,
    q_ref: float,
    dq_ref: float,
    q_meas: float,
    dt: float,
    limits: JointLimits,
) -> tuple[float, float, float]:
    """Projects requested target into closed feasible [position, velocity, acceleration] interval.
    Enforces smooth zero-crossing reversal deceleration."""
    raise NotImplementedError

def predict_combined_effort(
    tau_ff: float,
    kp: float,
    kd: float,
    q_ref: float,
    q_meas: float,
    dq_ref: float,
    dq_meas: float,
) -> float:
    """Computes tau_ff + kp*(q_ref - q_meas) + kd*(dq_ref - dq_meas)."""
    raise NotImplementedError
```

### `joint_controller.py` (Additions and Updates)
```python
from dataclasses import dataclass
from enum import Enum
from joint_mapping import JointTarget, JointSample
from joint_kinematics import JointLimits

class DecisionCode(str, Enum):
    # Existing codes preserved...
    ACCEPTED = "accepted"
    INVALID_STATE = "invalid_state"
    INVALID_ARGUMENT = "invalid_argument"
    EXPIRED = "expired"
    STALE_GENERATION = "stale_generation"
    POSITION_OUT_OF_BOUNDS = "position_out_of_bounds"
    LEAD_EXCEEDED = "lead_exceeded"
    EFFORT_LIMIT_EXCEEDED = "effort_limit_exceeded"
    CLOCK_REGRESSED = "clock_regressed"

@dataclass(frozen=True)
class RequestedAction:
    sequence: int
    owner_generation: int
    ttl_s: float
    joints: tuple[JointTarget, ...]

@dataclass(frozen=True)
class AcceptedAction:
    sequence: int
    owner_generation: int
    created_at_s: float
    expires_at_s: float
    joints: tuple[JointTarget, ...]

@dataclass(frozen=True)
class DispatchedReference:
    sequence: int
    owner_generation: int
    dispatched_at_s: float
    q_rad: tuple[float, ...]
    dq_rad_s: tuple[float, ...]
    tau_ff_nm: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]

@dataclass(frozen=True)
class LoopMetrics:
    nominal_period_s: float
    actual_period_s: float
    transaction_latency_s: float
    overruns: int

@dataclass(frozen=True)
class JointFault:
    # Retains existing fields...
    temperature_fault: bool = False
    persistent_effort_fault: bool = False
    clock_regressed: bool = False

class JointCore:
    def __init__(self, joints, limits: tuple[JointLimits, ...], feedback_max_age_s=0.5, clock=None):
        raise NotImplementedError

    def claim_ownership(self, owner_id: str) -> int:
        """Increments owner_generation, clears pending target, returns new generation."""
        raise NotImplementedError

    def revoke_ownership(self) -> None:
        """Increments owner_generation, clears pending target, forces STOPPED."""
        raise NotImplementedError

    def submit_target(self, action: RequestedAction) -> Decision:
        """Atomically validates sequence, generation, expiry, position bounds, lead, and effort."""
        raise NotImplementedError

    def dispatch_reference(self, now: float, dt: float) -> DispatchedReference | None:
        """Checks expiry of pending; shapes kinematic step; returns dispatched reference or None."""
        raise NotImplementedError
```

### `station_daemon.py`
```python
class StationDaemon:
    def __init__(self, port_config, limits, nominal_period_s=0.010, socket_path=None):
        raise NotImplementedError

    def run_until_interrupted(self) -> None:
        """Acquires ManagedPort, runs selectors event loop (100 Hz tick + non-blocking IPC)."""
        raise NotImplementedError
```

---

## Per-Milestone Implementation Sequence & Acceptance Tests

### Milestone C07: Command Expiry
- **Implementation**:
  - Add `ttl_s` to `RequestedAction` and `expires_at_s` to `AcceptedAction`.
  - In `JointCore.submit_target`: reject if `now >= expires_at_s` (`DecisionCode.EXPIRED`).
  - In `JointCore.dispatch_reference`: if pending target expired before dispatch, discard pending, record drop, hold reference.
- **Acceptance Tests**:
  - Test submitting target with negative or zero TTL rejects immediately.
  - Test valid target that sits in pending past `expires_at_s` is never returned by `dispatch_reference`.

### Milestone C08: Ownership Generations
- **Implementation**:
  - Core tracks `current_generation: int`. Add `claim_ownership(owner_id)` and `revoke_ownership()`.
  - `submit_target` checks `action.owner_generation == self._current_generation`.
  - Takeover increments generation, purges pending target, and revokes prior owner.
- **Acceptance Tests**:
  - Commands with older generation reject with `STALE_GENERATION`.
  - Human takeover increments generation; delayed commands from previous owner are rejected without disturbing state.

### Milestone C09: Joint Position Limits
- **Implementation**:
  - Integrate `JointLimits.q_min` and `q_max` into `JointCore`.
  - In `submit_target`: test each joint $q \in [q_{\min}, q_{\max}]$ inclusive.
  - Reject atomically with `POSITION_OUT_OF_BOUNDS` if any joint exceeds bounds.
- **Acceptance Tests**:
  - Exact boundary targets ($q == q_{\min}$ and $q == q_{\max}$) accept.
  - Target with $q_{\min} - \epsilon$ rejects; pending target remains unchanged.

### Milestone C10: Reference Speed Bounds on Target Jumps
- **Implementation**:
  - Implement velocity bounds in `project_kinematic_step`.
  - Enforce reference velocity $|\dot{q}_{\mathrm{ref}}| \le v_{\max}$.
  - Target jump across step $\Delta t$ cannot exceed $v_{\max} \Delta t$.
- **Acceptance Tests**:
  - Submit step jump in position: verify reference trajectory slopes match $v_{\max}$ across discrete ticks.

### Milestone C11: Reference Acceleration Bounds Across Changes
- **Implementation**:
  - Bound velocity changes: $|\dot{q}_{k} - \dot{q}_{k-1}| \le a_{\max} \Delta t$.
  - Enforce zero-crossing reversal: when flipping sign, velocity must decelerate through zero without stepping discontinuities.
- **Acceptance Tests**:
  - Step reversal from $+v_{\max}$ to $-v_{\max}$: verify velocity ramps down to 0 at rate $a_{\max}$ before reversing.

### Milestone C12: Bounded Lead from Measured Position
- **Implementation**:
  - Verify $|q_{\mathrm{target}} - q_{\mathrm{meas}}| \le \Delta q_{\mathrm{lead\_max}}$. Reject with `LEAD_EXCEEDED` if exceeded.
  - In `dispatch_reference`: project reference within $[q_{\mathrm{meas}} - \Delta q_{\mathrm{lead\_max}}, q_{\mathrm{meas}} + \Delta q_{\mathrm{lead\_max}}]$.
- **Acceptance Tests**:
  - If simulated arm is stalled ($q_{\mathrm{meas}}$ static), dispatched reference clamps at lead limit and refuses further divergence.

### Milestone C13: Combined Effort Prediction and Monitoring
- **Implementation**:
  - Implement `predict_combined_effort(tau_ff, kp, kd, ...)`.
  - In `submit_target`: reject if $|\tau_{\mathrm{pred}}| > \tau_{\max}$ (`EFFORT_LIMIT_EXCEEDED`).
  - In `receive()`: check $|\tau_{\mathrm{meas}}| \le \tau_{\max}$.
- **Acceptance Tests**:
  - Target with high $K_p$ and tracking error exceeding $\tau_{\max}$ rejects at submission.

### Milestone C14: Latched Temperature and Persistent Effort Faults
- **Implementation**:
  - Add `driver_temp_c` and `winding_temp_c` to `MotorHealth`.
  - Latch `temperature_fault` if temperature exceeds limit.
  - Latch `persistent_effort_fault` if effort exceeds threshold for window duration.
  - Enforce recovery requirement: `begin_check(acknowledge_fault=True)` requires normal temp/effort and Stop mode.
- **Acceptance Tests**:
  - Injected overheat latches FAULT. Submitting new targets rejects. Healthy feedback without ack fails. Acknowledged check with cool temperature restores READY.

### Milestone C15: Station Loop Timing Measurements
- **Implementation**:
  - In `StationDaemon`: compute `LoopMetrics` (period, transaction latency, overruns).
  - Include metrics in `Snapshot`.
- **Acceptance Tests**:
  - Inject sleep delay in fixture: verify `overruns` counter increments and `actual_period_s` reflects delay accurately.

### Milestone C16: Separate Station Process & Independence
- **Implementation**:
  - Implement `station_daemon.py` and `station_client.py`.
  - Exclude second owner using `open_managed_port`.
  - Client disconnection/crash handler sets core to STOPPED.
- **Acceptance Tests**:
  - Start station on PTY fixture. Connect Client 1. Kill Client 1 (`SIGKILL`). Verify station enters STOPPED.
  - Connect Client 2: verify state is STOPPED and robot is NOT automatically rearmed.
  - Start second station daemon on same port: verify rejection by device lease.

---

## Baseline, Fingerprint, and CI Integration

1. **`scripts/check_offline_baseline.sh`**:
   - Add new files to the `files=(...)` array:
     `joint_kinematics.py`, `station_daemon.py`, `station_client.py`, `test_station_ipc.py`, `test_controller_kinematics.py`.
   - Add milestone test runner:
     `run_logged milestone-c07-c16-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_controller_kinematics test_station_ipc`
2. **`test_startup.py`**:
   - Update `_fingerprint(root)` tuple with new module names and test files.
3. **Serial Caller Migration**:
   - All tests previously using `JointCore.consume_target()` are migrated to `JointCore.dispatch_reference()`. Superseded methods are deleted.