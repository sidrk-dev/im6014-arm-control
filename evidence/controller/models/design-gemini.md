# Candidate Design Package: B11–B14 & C01–C06

## Problem

Milestones B11–B14 and C01–C06 expand the IM6014 workbench from a single-motor, actuator-shaft bench prototype into a verified, multi-joint request and bus-coordination core. The core challenge is bridging pure robotic joint commands to raw actuator frames without leaking physical gearbox ratios across boundaries, while establishing strict cross-process bus exclusivity and multi-joint state arbitration. This must be accomplished while preserving existing dashboard and CLI entry points, byte-exact golden codec fixtures, and strict disconnected startup invariants. Crucially, C01–C06 must not introduce unvalidated live motion paths before downstream safety milestones (C07–C16 command expiry, generation tracking, and kinematic limiters) are verified.

---

## Usage (caller's view)

### 1. Pure External Mapping & Hardened Codec Boundary (B11, B12)
```python
from im6014_mapping import ReductionMapping, JointCommand
from im6014_codec import command

# Pure external reduction: N=3:1 reduction, reversed direction, zero offset
j1_map = ReductionMapping(ratio=3.0, direction=-1, offset_rad=0.0)

# Caller specifies canonical joint units: rad, rad/s, Nm, joint stiffness/damping
cmd = JointCommand(position=0.5236, velocity=1.0472, torque=1.5, kp=90.0, kd=18.0)
out = j1_map.to_actuator(cmd)

# Codec validates finite types, IDs, bench bounds, and internal gear ratio (38/3)
packet = command(
    motor_id=1,
    mode=1,
    rpm=out.rpm,
    kd=out.kd,
    torque=out.torque,
    watchdog=True,
    position=out.position,
    kp=out.kp,
)
```

### 2. Explicit Port Configuration & Cooperative Bus Lease (B13, C01)
```python
from im6014_adapter import PortConfig, inspect_port
from im6014_bus_lock import CooperativeBusOwner

# Passive inspection validates device node, channel hint, and baud without port opening
config = PortConfig(
    path="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0",
    baud=6000000,
    expected_channel="if03",
)
inspection = inspect_port(config)
if not inspection.is_valid:
    raise RuntimeError(f"Port validation failed: {inspection.reason}")

# Bus owner resolves canonical device rdev/inode, guarantees cross-process flock & in-process lock
owner = CooperativeBusOwner(config.path)
with owner.acquire() as lease:
    # Safe to construct serial.Serial; lease release guaranteed on crash/exit/error
    bus = lease.open_serial(baudrate=config.baud, timeout=0.005, write_timeout=0.05)
```

### 3. Station Controller Core & Bounded Target Streaming (C02–C06, B14)
```python
from im6014_controller_core import ControllerCore, JointTarget
from im6014_telemetry import DiagnosticRecorder

recorder = DiagnosticRecorder(maxlen=1000)
core = ControllerCore(joint_ids=(0, 1), recorder=recorder)

# Ingest feedback per joint; wrong IDs rejected, receive time taken monotonically
core.ingest_feedback(joint_id=0, raw_bytes=raw_j0, receive_mono=t_now)
core.ingest_feedback(joint_id=1, raw_bytes=raw_j1, receive_mono=t_now)

# Arming requires fresh, healthy feedback across all configured joints
core.arm(supported_acknowledged=True)

# 10,000 rapid updates store only the latest accepted target; older sequences rejected
core.submit_target(stream_id="teleop", seq=101, targets={0: JointTarget(rpm=5.0, kd=6.0), 1: JointTarget(rpm=0.0, kd=6.0)})
# Duplicate or older seq (e.g., seq=100) rejected without altering pending target
```

---

## Shape

The architecture uses an in-memory **Station Controller Core** paired with an OS-level **Cooperative Bus Lease**. It decouples command arbitration, state transitions, and diagnostic telemetry from raw serial I/O, ensuring no live motor dispatch can occur in illegal states.

```mermaid
flowchart TD
    Client[Dashboard UI / CLI / Tests] -->|Submit JointTarget| Core[ControllerCore]
    Client -->|Lifecycle Events: Arm/Stop/Close| Core
    Core -->|Target Mailbox O(1) & Monotonic Watermark| State[State & Freshness Arbiter]
    Transport[Serial Exchange / FakeSerial] -->|Raw Bytes & Monotonic Time| Core
    Core -->|Ingest & Attribute| Health[Per-Joint Health & Age Table]
    Health -->|Update Flags| State
    Core -->|Log Allowlisted TX/RX| Telemetry[DiagnosticRecorder]
    BusMgr[CooperativeBusOwner] -->|Stat Canonical Inode / flock| Lease[Managed Bus Lease]
    Lease --> Transport
```

### 1. Module Map

- `im6014_mapping.py` (B11): Pure mathematical conversions between canonical joint units ($\text{rad}$, $\text{rad/s}$, $\text{N}\cdot\text{m}$) and actuator output-shaft units ($\text{rad}$, $\text{RPM}$, $\text{N}\cdot\text{m}$, gains scaled by $1/N^2$).
- `im6014_codec.py` (B12): Strict boundary validation enforcing exact numeric types (`int` not `bool`, finite floats, bench bounds) and packing for wire conversion (`38/3` gearbox).
- `im6014_adapter.py` (B13): Passive OS/sysfs device inspection, channel hints (`FT4232HChannelD`), and 4/6 Mbps baud validation without opening ports.
- `im6014_telemetry.py` (B14): Fixed-size ring buffer recording allowlisted request/reply pairs, hex strings, timings, provenance, and diagnostic metrics (CRC/framing errors).
- `im6014_bus_lock.py` (C01): Cooperative file-descriptor locking (`fcntl.flock`) keyed on canonical device major/minor numbers and an in-process thread lock.
- `im6014_controller_core.py` (C02–C06): Authoritative request core managing state transitions, per-joint freshness, latched faults, O(1) target storage, and strictly increasing sequence watermarks.

### 2. Types and Signatures

```python
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Optional, Tuple, Set

class ControllerState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    STOPPED = auto()
    ARMED = auto()
    FAULTED = auto()
    TIMEOUT_LATCHED = auto()

@dataclass(frozen=True)
class ReductionMapping:
    ratio: float          # N > 0
    direction: int        # +1 or -1
    offset_rad: float     # q0 actuator offset

    def to_actuator_position(self, q_joint: float) -> float: ...
    def to_actuator_velocity(self, dq_joint: float) -> float: ...
    def to_actuator_torque(self, tau_joint: float) -> float: ...
    def to_actuator_gains(self, kp: float, kd: float) -> Tuple[float, float]: ...
    def to_joint_position(self, q_out: float) -> float: ...
    def to_joint_velocity(self, dq_out: float) -> float: ...
    def to_joint_torque(self, tau_out: float) -> float: ...

@dataclass(frozen=True)
class JointTarget:
    rpm: float = 0.0
    kd: float = 0.0
    torque: float = 0.0
    position: float = 0.0
    kp: float = 0.0

@dataclass(frozen=True)
class JointObservation:
    joint_id: int
    receive_mono: float
    raw_hex: str
    mode: int
    timeout: int
    driver_c: int
    winding_c: int
    voltage: float
    torque_nm: float
    rpm: float
    position_rad: float
    errors: int
    warnings: int
    is_healthy: bool

class ControllerCore:
    def __init__(self, joint_ids: Tuple[int, ...], recorder: Optional['DiagnosticRecorder'] = None): ...
    def request_transition(self, action: str, **kwargs) -> ControllerState: ...
    def arm(self, supported_acknowledged: bool, max_stale_s: float = 0.25) -> bool: ...
    def stop(self) -> None: ...
    def ingest_feedback(self, joint_id: int, raw_bytes: bytes, receive_mono: float) -> JointObservation: ...
    def submit_target(self, stream_id: str, seq: int, targets: Dict[int, JointTarget]) -> bool: ...
    def get_pending_target(self) -> Optional[Tuple[str, int, Dict[int, JointTarget]]]: ...
    def get_joint_age(self, joint_id: int, now_mono: float) -> Optional[float]: ...
    def snapshot(self) -> dict: ...
```

### 3. Controller State Machine & Transition Table

Transitions enforce immediate target clearing and disarming on any fault, disconnect, or Stop request:

| Current State | Input / Event | Guard Condition | Next State | Action / Side Effect |
|---|---|---|---|---|
| `DISCONNECTED` | `connect` | Valid adapter config | `CONNECTING` | Clear targets, reset sequence watermarks |
| `CONNECTING` | `connected_ok` | All configured joints reply | `STOPPED` | Mark disarmed, verify Stop feedback |
| `CONNECTING` | `connected_timeout`| Any joint flag `timeout==1` | `TIMEOUT_LATCHED` | Disarm, require explicit ack sequence |
| `CONNECTING` | `connected_fault` | Any joint has error/warning | `FAULTED` | Latch fault, retain offending ID |
| `CONNECTING` | `fail / abort` | Bus open error or timeout | `DISCONNECTED` | Release bus lease |
| `STOPPED` | `arm` | `supported==True`, all active joints fresh ($\le 0.25\,\text{s}$) & healthy | `ARMED` | Start 10s arm window; keep motor stopped |
| `STOPPED` | `submit_target`| Any | `STOPPED` | **Rejected**; no motor command generated |
| `STOPPED` | `feedback_fault`| Any joint reports error | `FAULTED` | Record offending joint and affected build |
| `ARMED` | `submit_target`| `seq > watermark` and all targets finite/valid | `ARMED` | Atomically replace pending target; advance watermark |
| `ARMED` | `submit_target`| `seq <= watermark` or invalid | `ARMED` | **Rejected**; retain current accepted target |
| `ARMED` | `arm_expired` | `now > arm_until` | `STOPPED` | Clear pending target; disarm |
| `ARMED` | `feedback_stale`| Any joint age $> 0.1\,\text{s}$ | `FAULTED` | Clear pending target; disarm |
| `ARMED` | `feedback_fault`| Any joint reports error/warning | `FAULTED` | Clear pending target; disarm; retain fault ID |
| `ARMED` / `STOPPED` | `stop` | None (operator priority) | `STOPPED` | Disarm; clear pending target; preserve watermark |
| Any except `DISCONNECTED` | `disconnect` | None | `DISCONNECTED` | Clear target; disarm; release bus lease |
| `TIMEOUT_LATCHED` | `ack_timeout` | Mode 0, watchdog toggled (Stop only) | `STOPPED` | Clear timeout latch if healthy; remain disarmed |

### 4. Ownership Lifetime, Lease Contract, and Failure Semantics

- **Advisory OS Exclusivity**: `CooperativeBusOwner` resolves the target path via `os.path.realpath` and `os.stat` to obtain `st_rdev` (character device major/minor) or `(st_dev, st_ino)`. It locks a dedicated lockfile (`/tmp/im6014_bus_{major}_{minor}.lock`) using `fcntl.flock(LOCK_EX | LOCK_NB)`. This covers symlinks, by-id aliases, and direct `/dev/ttyUSB*` paths.
- **In-Process Exclusion**: A process-wide registry guarded by `threading.Lock` prevents threads in the same process (or GUI and embedded CLI) from acquiring the same device.
- **Lease Boundary**: The lock is acquired strictly *before* serial port construction. If `serial.Serial.open()` fails, the lock is released immediately. On normal termination, exception, or process crash, the kernel closes the file descriptor and releases the `flock` automatically.
- **Contract Clarity**: Kernel `flock` guarantees exclusion among cooperating workbench components. It does not prevent uncooperative root utilities from issuing raw writes directly to the device node.

### 5. Target Buffer, Watermark, and Per-Joint Tracking

- **Bounded O(1) Mailbox**: `ControllerCore` holds a single `_pending_target` slot containing an immutable snapshot `Tuple[str, int, Dict[int, JointTarget]]`. Incoming streaming targets overwrite this single record. Ten thousand rapid updates occupy $O(1)$ memory.
- **Strictly Increasing Sequence**: Every target stream maintains a watermark `_watermarks[stream_id]`. A new target is accepted if and only if `seq > _watermarks[stream_id]`. Duplicate or older sequences are dropped with an error counter increment, leaving the current valid target intact.
- **Sequence Retention**: Calling `stop()` or transitioning to `FAULTED` clears `_pending_target` to `None`, but **retains** `_watermarks[stream_id]`. Delayed or out-of-order packets from the past cannot execute.
- **Independent Joint Freshness**: Per-joint records update exclusively when a frame's decoded ID matches the joint. A fresh reply on J1 updates only J1's monotonic receive timestamp; missing or stale J2 frames remain visible. Freshness and health are strictly separated: a frame received at $t_{\text{now}}$ with fault bits set updates freshness but immediately trips the core into `FAULTED`.

---

## Synthesis decision

This design adopts the **Station Controller Core Engine** architecture. It implements a self-contained, domain-authoritative in-memory state engine and target mailbox, cleanly decoupled from transport mechanisms. 

It explicitly rejects both a loose "driver-level state dictionary" (which leaks transition invariants across callers) and a premature "station IPC daemon" (which adds unnecessary socket serialization and process management before C07–C16 safety limiters are built). The chosen shape provides deep interface abstraction while running entirely in-process under fake transports and deterministic clocks.

---

## Tradeoffs accepted

- We accept pure in-process execution for C01–C06 without a separate Unix socket/IPC broker in exchange for zero IPC overhead, deterministic unit-test execution via fake clocks, and strict isolation from unverified live motion loops.
- We accept advisory lockfiles (`/tmp/im6014_bus_*.lock`) keyed on device numbers in exchange for robust cross-process alias detection that does not modify system `/dev` permissions.
- We accept O(1) latest-target replacement (dropping intermediate setpoints) in exchange for bounded memory and preventing stale command accumulation during high-rate teleoperation.
- We accept storing raw hex frames in diagnostic records in exchange for bounded memory consumption ($1000$ records $\approx 500\,\text{kB}$) without leaking arbitrary environment strings.

---

## Alternatives considered

1. **Minimal Typed State Object + Driver Boundary**:
   - *Description*: Expose a `ControllerState` dataclass modified directly inside `im6014_serial` or `MotorSession`.
   - *Why it lost*: Extremely shallow interface. It forces callers to coordinate sequence checks, error bit checking, and target invalidation manually. It scatters safety invariants across the GUI and CLI, making it trivial for an agent or caller to bypass state checks and dispatch motion in an illegal state.
2. **Standalone Station Daemon with Socket IPC (Premature Physical Station)**:
   - *Description*: Spawn a background daemon holding the serial port, exposing a Unix Domain Socket with JSON-RPC.
   - *Why it lost*: Excessive interface and operational complexity. It pretends the data reducer is already a physical station before C07 (command expiry), C08 (ownership generations), and C09–C13 (kinematic and torque limits) exist. It breaks startup interception tests (`test_startup.py`), impairs offline fake transport testing, and introduces socket reconnection failure modes.

---

## Open questions and risks

1. *How should multi-joint queries be sequenced on a single RS485 bus?* Does downstream station scheduling poll joints sequentially (round-robin) within one control cycle, or will multi-drop command interleaving be introduced in C15?
2. *Will the FTDI channel hint require platform-specific sysfs fallback?* On Linux, udev `/dev/serial/by-id` provides stable interface paths (`-if03-port0`), but if run on systems lacking udev, how should passive adapter identification fall back?

---

## Next implementation step & milestone acceptance plan

The immediate next step is to implement `im6014_mapping.py` with pure `ReductionMapping` arithmetic and verify round trips for ratios $1, 3, 6$ across positive and negative signs.

### Milestone Acceptance Plan (B11–B14, C01–C06)

- **B11 (Mapping)**: Unit tests verify forward and inverse conversions for $N \in \{1, 3, 6\}$, $s \in \{+1, -1\}$, and non-zero $q_0$. Verify gain scaling ($1/N^2$). Confirm zero imports of `im6014_codec.RATIO`.
- **B12 (Codec Boundary)**: Test rejection of `bool` IDs, floats, strings, negative IDs, and IDs $>14$. Verify `math.isnan` and `math.isinf` rejection on all float inputs. Assert that pre-scaled values exceeding struct boundaries throw errors before packing. Keep existing bench limits ($0 \le K_d \le 6, 0 \le K_p \le 30, |\tau| \le 3.5$).
- **B13 (Port Configuration)**: Assert passive inspection correctly flags mismatching baud ($4\,\text{Mbps}$ or $6\,\text{Mbps}$ only) and validates historical FTDI hints without calling `Serial.open()`. Verify non-FTDI paths (e.g. PTYs) pass with explicit config.
- **B14 (Diagnostics)**: Verify allowlisted `TransactionRecord` generation for fake, demo, and PTY runs. Confirm exclusion of environment variables and credentials. Verify ring buffer bounds at 1,000 entries.
- **C01 (Bus Exclusivity)**: Test second-owner rejection across two independent processes and two distinct symlinks pointing to the same PTY node. Assert immediate lock release upon simulated open exception.
- **C02 (Controller States)**: Feed all invalid state-action pairs into `ControllerCore`; verify return code / exception and assert zero output packets generated.
- **C03 (Per-Joint Age)**: Feed fake packets for J0 only; assert J0 receive age updates while J1 age monotonically increases past stale threshold. Assert wrong ID does not refresh receive time.
- **C04 (Per-Joint Faults)**: Ingest packet with error bit set for J1; assert J1 latches error, core transitions to `FAULTED`, and build status reports disarmed. Unknown bits must be preserved in hex representation.
- **C05 (Bounded Target Storage)**: Submit 10,000 distinct valid targets through `submit_target`; verify pending buffer length is strictly 1 and matches the 10,000th target. Submit invalid target; verify 10,000th target remains intact.
- **C06 (Sequence Checks)**: Submit sequence numbers $1, 5, 5, 2, 6$. Assert sequences $1, 5, 6$ are accepted; duplicate $5$ and stale $2$ are rejected without erasing the pending target. Verify watermark $6$ persists after `stop()`.