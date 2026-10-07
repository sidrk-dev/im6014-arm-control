### Components Found

- **`im6014_codec.command`** ([`im6014_codec.py:23-42`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L23-L42)): Pure function that validates numerical parameters against protocol encodability and bench bounds, converts actuator output-shaft units to internal motor register values using internal ratio `RATIO = 38/3`, and serializes a 20-byte wire command packet with CRC-32.
- **`im6014_codec.feedback`** ([`im6014_codec.py:44-57`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L44-L57)): Pure function that validates 26-byte framing, header, CRC-32, motor ID, and mode, and parses raw registers into a dictionary of physical actuator-output units and error flags.
- **`im6014_codec.crc32`** ([`im6014_codec.py:11-20`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L11-L20)): Pure checksum function implementing Unitree's reversed-byte 32-bit CRC (`0x04C11DB7`) over 4-byte-aligned words.
- **`im6014_serial.exchange`** ([`im6014_serial.py:62-108`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L62-L108)): Single-transaction synchronous bus exchange that flushes input, bounds write and read timeouts within an explicit monotonic deadline, writes a 20-byte packet, parses the incoming stream, and restores serial settings in a `finally` block.
- **`im6014_serial._receive`** ([`im6014_serial.py:22-60`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L22-L60)): Stream parser that scans for the `0xFC, 0xEE` response header, processes candidates via `im6014_codec.feedback`, increments CRC and rejection metrics on malformed frames, and advances byte-by-byte on corruption.
- **`fake_serial.FakeSerial`** ([`fake_serial.py:48-215`](file:///home/sid/Documents/code/im6014Workbench/fake_serial.py#L48-L215)): PySerial-compatible deterministic mock bus that schedules scripted response chunks, advances a fake monotonic clock, models read/write delays and timeouts, and records immutable writes.
- **`fake_serial.FakeClock`** ([`fake_serial.py:29-46`](file:///home/sid/Documents/code/im6014Workbench/fake_serial.py#L29-L46)): Monotonic time reference driven strictly by simulated I/O delays or test sleep calls.
- **`motor_dashboard.Dashboard.refresh_ports`** ([`motor_dashboard.py:343-361`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L343-L361)): Passive port enumerator calling `serial.tools.list_ports.comports()` that inspects OS device descriptors and prioritizes FT4232H Channel D without opening ports or transmitting packets.
- **`spin_5rpm.healthy`** ([`spin_5rpm.py:18-27`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L18-L27)): Safety gate asserting mode consistency, zero error/warning bits, zero latched timeout, voltage within [24, 72] V, temperatures within limits, and torque under 4 Nm.
- **`motor_session.DemoSerial`** ([`motor_session.py:66-130`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L66-L130)): Packet-level stateful simulator modeling single-actuator and dual-actuator response packets for disconnected UI testing.

---

### Flow

#### 1. Command Formulation and Encoding (Downlink)
1. **Initiation**: The application layer requests motor action (e.g., [`motor_session.py:186`](file:///home/sid/Documents/code/im6014Workbench/motor_session.py#L186) `MotorSession._transact` or [`spin_5rpm.py:33`](file:///home/sid/Documents/code/im6014Workbench/spin_5rpm.py#L33) `run.transact`).
2. **Encoding Call**: Calls `im6014_codec.command(motor_id, mode, rpm, kd, torque, watchdog, position, kp)` in [`im6014_codec.py:23`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L23).
3. **Numerical Validation**:
   - Validates `0 <= motor_id <= 14` and `mode in (0, 1)` ([`im6014_codec.py:24`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L24)).
   - Validates `math.isfinite()` on all numerical fields ([`im6014_codec.py:26`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L26)).
   - Enforces mathematical encodability bounds: `RPM_MIN <= rpm <= RPM_MAX` and `POSITION_MIN <= position <= POSITION_MAX` ([`im6014_codec.py:28, 32`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L28)).
   - Enforces bench policy bounds: `0 <= kd <= 6`, `0 <= kp <= 30`, `-3.5 <= torque <= 3.5` ([`im6014_codec.py:30`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L30)).
   - Enforces mode constraints: Mode 0 requires all targets to be 0; Mode 1 requires `watchdog=True` ([`im6014_codec.py:35`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L35)).
4. **Internal Gear Reduction and Quantization**:
   - Scales output-shaft torque to rotor: `round(torque / RATIO * 2560)` (int16).
   - Scales output-shaft speed to rotor: `round(rpm / 60 * RATIO * 64)` (int16).
   - Scales output-shaft position to rotor counts: `round(position / math.tau * RATIO * 32768)` (int32).
   - Scales output-shaft stiffness and damping to rotor: `round(kp / RATIO**2 * 12800)` and `round(kd / RATIO**2 * 51200)` (int16).
5. **CRC Calculation**: Pre-appends header `b'\xfe\xee'` and control byte `motor_id | mode << 4 | int(watchdog) << 7`, passes the 16 bytes to `crc32()` ([`im6014_codec.py:11`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L11)), and packs the resulting 32-bit CRC.
6. **Return**: Returns a 20-byte `bytes` object to caller.

#### 2. Serial Bus Exchange and Stream Resynchronization
1. **Entry**: Caller passes `bus`, `packet`, `motor_id`, and `deadline_s` to `im6014_serial.exchange()` in [`im6014_serial.py:62`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L62).
2. **Deadline Establishment**: Checks `deadline_s > 0` and `math.isfinite()`, calculates `deadline = time.monotonic() + deadline_s` ([`im6014_serial.py:70-72`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L70-L72)).
3. **Buffer Flushing & Write**: Flushes stale input buffer with `bus.reset_input_buffer()`. Binds `bus.write_timeout` to the remaining deadline, writes the 20-byte packet, and verifies that `len(packet)` bytes were written ([`im6014_serial.py:79-87`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L79-L87)).
4. **Stream Read Loop**: Hands execution to `_receive()` ([`im6014_serial.py:22`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L22)):
   - Sets `bus.timeout = min(original_timeout, remaining_deadline)`.
   - Reads available chunks: `bus.read(min(26, max(1, bus.in_waiting)))`.
   - Locates `b'\xfc\xee'` header. Drops leading noise bytes up to header ([`im6014_serial.py:41`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L41)).
   - If at least 26 bytes are available, slices `data[:26]` and invokes `im6014_codec.feedback()`.
   - On `RuntimeError` (checksum failure, wrong mode, or wrong motor ID), increments rejection metrics, drops 1 byte from the stream, and searches for the next header ([`im6014_serial.py:46-50`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L46-L50)).
5. **Restoration**: On success or exception, `finally` in `exchange()` restores the original `timeout` and `write_timeout` attributes on `bus` ([`im6014_serial.py:95-106`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L95-L106)).
6. **Return**: Decoded dictionary returned to caller.

#### 3. Feedback Decoding (Uplink)
1. **Length & Header Verification**: In `im6014_codec.feedback()` ([`im6014_codec.py:44`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L44)), checks `len(raw) == 26` and `raw[:2] == b'\xfc\xee'`.
2. **CRC Verification**: Computes `crc32(raw[2:22])` and compares with unpacked uint32 from `raw[22:26]` ([`im6014_codec.py:47`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L47)).
3. **Payload Extraction**: Unpacks `<BbBBhhiIHBB` across bytes 2 to 21 ([`im6014_codec.py:49`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L49)).
4. **Motor ID and Mode Gate**: Validates `m & 15 == motor_id` and `(m >> 4) & 7 in (0, 1)` ([`im6014_codec.py:50`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L50)).
5. **Physical Unit Conversion**:
   - `voltage = v / 2` (Volts).
   - `torque_nm = tq / 2560 * RATIO` (Actuator output-shaft Nm).
   - `rpm = sp / 64 / RATIO * 60` (Actuator output-shaft rpm).
   - `position_rad = pos / 32768 / RATIO * math.tau` (Actuator output-shaft radians).
   - `raw = raw.hex()`.
6. **Return**: Returns a dictionary with keys `id`, `mode`, `timeout`, `driver_c`, `winding_c`, `voltage`, `torque_nm`, `rpm`, `position_rad`, `errors`, `warnings`, `raw`.

#### 4. Passive Port Enumeration
1. **Trigger**: Dashboard initialization ([`motor_dashboard.py:250`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L250)) or operator clicking "Refresh" ([`motor_dashboard.py:166`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L166)).
2. **Mode Check**: In `refresh_ports()` ([`motor_dashboard.py:343`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L343)), checks if Demo mode is selected; if so, displays `"Demo bus · IDs 0 and 1"` and exits without system queries.
3. **Passive OS Query**: Invokes `serial.tools.list_ports.comports()` ([`motor_dashboard.py:351`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L351)), inspecting Linux sysfs/udev attributes. No ports are opened, no baud rates set, no RTS/DTR lines asserted.
4. **Channel D Prioritization**: Sorts detected ports placing FTDI chips with serial `FTC2RQ7H` and USB location ending in `.3` (Channel D) first ([`motor_dashboard.py:351`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L351)).
5. **Symlink Resolution**: Matches `/dev/serial/by-id/...` against resolved real devices ([`motor_dashboard.py:353`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L353)).
6. **UI Population**: Populates `self.port` QComboBox with device descriptions, storing port paths and attaching serial/USB hardware metadata in tooltips.

---

### Files Read

- `/home/sid/.codex/plugins/cache/open-pstack/pstack/1.13.0/skills/how/references/explorer-prompt.md`
- `/home/sid/Documents/code/im6014Workbench/AGENTS.md`
- `/home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md`
- `/home/sid/Documents/code/im6014Workbench/docs/architecture.md`
- `/home/sid/Documents/code/im6014Workbench/docs/milestones.tsv`
- `/home/sid/Documents/code/im6014Workbench/docs/boundary-design.md`
- `/home/sid/Documents/code/im6014Workbench/docs/decisions.tsv`
- `/home/sid/Documents/code/im6014Workbench/docs/b11-c06-workflow.md`
- `/tmp/pstack-b11-c06-20261007-1mwdi1cr/frame.md`
- `/tmp/pstack-b11-c06-20261007-1mwdi1cr/how-protocol-prompt.md`
- `/tmp/pstack-b11-c06-20261007-1mwdi1cr/how-controller-prompt.md`
- `/home/sid/Documents/code/im6014Workbench/im6014_codec.py`
- `/home/sid/Documents/code/im6014Workbench/im6014_serial.py`
- `/home/sid/Documents/code/im6014Workbench/fake_serial.py`
- `/home/sid/Documents/code/im6014Workbench/spin_5rpm.py`
- `/home/sid/Documents/code/im6014Workbench/motor_session.py`
- `/home/sid/Documents/code/im6014Workbench/motor_dashboard.py`
- `/home/sid/Documents/code/im6014Workbench/test_protocol.py`
- `/home/sid/Documents/code/im6014Workbench/test_serial_deadline.py`
- `/home/sid/Documents/code/im6014Workbench/test_spin_5rpm.py`
- `/home/sid/Documents/code/im6014Workbench/test_dashboard.py`
- `/home/sid/Documents/code/im6014Workbench/test_startup.py`
- `/home/sid/Documents/code/im6014Workbench/scripts/check_offline_baseline.sh`

---

### Boundaries

- **Wire / Transport Boundary**:
  - Input: Raw bytes from pyserial `bus.read()`.
  - Output: 20-byte wire commands passed to `bus.write()`.
  - Invariants: Bounded monotonic deadline, input buffer cleared before write, dynamic timeout restoration on all exit paths ([`im6014_serial.py:73-106`](file:///home/sid/Documents/code/im6014Workbench/im6014_serial.py#L73-L106)).
- **Protocol Codec Boundary (`im6014_codec.py`)**:
  - Downlink Input: Actuator output-shaft targets (`rpm`, `position` in radians, `torque` in Nm, `kp`, `kd`).
  - Downlink Output: 20-byte binary packet with CRC-32.
  - Uplink Input: 26-byte binary response with CRC-32.
  - Uplink Output: Dictionary of actuator output-shaft measurements (`position_rad`, `rpm`, `torque_nm`, `voltage`, temperatures, faults).
  - Invariant: Exclusively owns internal reduction `RATIO = 38/3`. Does not know about external arm transmissions, joint directions, zero offsets, or multi-joint kinematics.
- **Controller & Mapping Boundary (Target for B11 / C01-C06)**:
  - Input from Robot Level: Canonical arm joint state/actions ($q_{\mathrm{joint}} \in \text{rad}$, $\dot{q}_{\mathrm{joint}} \in \text{rad/s}$, $\tau_{\mathrm{joint}} \in \text{Nm}$, $K_{p,\mathrm{joint}}$, $K_{d,\mathrm{joint}}$).
  - Output to Protocol Codec: Actuator output-shaft targets ($q_{\mathrm{out}} \in \text{rad}$, $\text{rpm}$, $\tau_{\mathrm{out}} \in \text{Nm}$, $K_{p,\mathrm{out}}$, $K_{d,\mathrm{out}}$).
  - Reverse for Feedback: Actuator output-shaft measurements $\to$ Canonical arm joint measurements.

---

### Where a Pure External Reduction Map Fits Without Double Converting

#### The Mathematical Mapping Contract
As specified in [`docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:186-207`](file:///home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md#L186-L207):
Let $q_{\mathrm{joint}}$ be canonical joint position (radians), $\dot{q}_{\mathrm{joint}}$ be canonical joint velocity (rad/s), and $\tau_{\mathrm{joint}}$ be canonical joint effort (Nm). Let $N$ be the external transmission reduction ratio (e.g., $N=1$ for J1, $N=6$ for shoulder J2, $N=3$ for elbow J3), $s \in \{+1, -1\}$ be the transmission direction sign, and $q_0$ be the zero offset.

The forward mapping from canonical joint coordinates to actuator output-shaft coordinates ($q_{\mathrm{out}}, \dot{q}_{\mathrm{out}}, \tau_{\mathrm{out}}$) is:
$$q_{\mathrm{out}} = q_0 + s N q_{\mathrm{joint}}$$
$$\dot{q}_{\mathrm{out}} = s N \dot{q}_{\mathrm{joint}}$$
$$\tau_{\mathrm{out}} = \frac{s \tau_{\mathrm{joint}}}{N}$$
$$K_{p,\mathrm{out}} = \frac{K_{p,\mathrm{joint}}}{N^2}, \quad K_{d,\mathrm{out}} = \frac{K_{d,\mathrm{joint}}}{N^2}$$

The inverse mapping from actuator output-shaft measurements to canonical joint coordinates is:
$$q_{\mathrm{joint}} = \frac{s (q_{\mathrm{out}} - q_0)}{N}$$
$$\dot{q}_{\mathrm{joint}} = \frac{s \dot{q}_{\mathrm{out}}}{N}$$
$$\tau_{\mathrm{joint}} = s N \tau_{\mathrm{out}}$$

#### Architectural Placement and Preventing Double Conversion
1. **Strict Layer Separation**:
   - The physical system has **two cascaded reductions**:
     1. Internal planetary gearbox: $N_{\mathrm{int}} = 38/3$.
     2. External link transmission: $N_{\mathrm{ext}} \in \{1, 3, 6\}$.
   - `im6014_codec.py` **already applies $N_{\mathrm{int}} = 38/3$** on wire registers. Its inputs and outputs are strictly in **actuator output-shaft units** (`position_rad`, `rpm`, `torque_nm`).
   - The external reduction map must sit **strictly above `im6014_codec.py`** and **strictly below the multi-joint controller / station layer**.
2. **What Must Never Happen**:
   - `im6014_codec.py` must never accept or reference $N_{\mathrm{ext}}$, $s$, or $q_0$.
   - The external reduction map must never reference `RATIO = 38/3`.
   - If an upper layer maps joint angles to actuator output angles, and the codec were modified to also map them, double conversion would occur.
3. **Velocity Unit Boundary**:
   - Canonical robot joint velocity is in $\text{rad/s}$ ([`docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:138`](file:///home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md#L138)).
   - Actuator output velocity in joint mapping is $\dot{q}_{\mathrm{out}} = s N \dot{q}_{\mathrm{joint}}$ in $\text{rad/s}$.
   - Currently, `im6014_codec.command()` accepts `rpm`, where `round(rpm / 60 * RATIO * 64)` converts to rotor registers.
   - Therefore, the conversion $\text{rpm} = \dot{q}_{\mathrm{out}} \times \frac{60}{2\pi}$ must occur at the boundary between joint mapping and `im6014_codec.command`, preventing double scaling by $2\pi$ or 60.

---

### Non-Obvious Things

1. **Bench Limits Hardcoded Inside Protocol Codec**:
   [`im6014_codec.py:30-31`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L30-L31) rejects commands with `kd > 6`, `kp > 30`, or `abs(torque) > 3.5 Nm`. These are historic bench safety clamps, not hardware encoding limits (the 16-bit register can mathematically encode torque up to $\pm 162$ Nm). Having bench policy inside the protocol encoder conflates protocol encoding with supervisory control limits.
2. **Speed Unit Inconsistency**:
   While canonical robot state and action contracts specify radians and radians per second ([`docs/ROBOT_WORKBENCH_AGENT_PROMPT.md:138`](file:///home/sid/Documents/code/im6014Workbench/docs/ROBOT_WORKBENCH_AGENT_PROMPT.md#L138)), the codec's position is in radians (`position_rad`), but its velocity is in revolutions per minute (`rpm`).
3. **Reversed Word CRC-32**:
   Unitree's CRC-32 operates on 4-byte words, but reverses the byte order within each 4-byte chunk before shifting: `for byte in data[start:start + 4][::-1]` ([`im6014_codec.py:16`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L16)). Slicing non-multiple-of-4 byte sequences raises `ValueError`.
4. **Passive Enumeration Does Not Probe Hardware**:
   Clicking "Refresh" on the dashboard calls `list_ports.comports()`, which reads OS sysfs/udev records without opening `/dev/ttyUSB*` or toggling RS485 transceiver lines. Port opening only occurs when the operator explicitly clicks "Connect" ([`motor_dashboard.py:362`](file:///home/sid/Documents/code/im6014Workbench/motor_dashboard.py#L362)).
5. **Mode 0 vs Mode 1 Mutual Exclusivity**:
   In `im6014_codec.command` ([`im6014_codec.py:35`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L35)), Stop (Mode 0) strictly forbids any nonzero command argument (`rpm or kd or torque or position or kp`), while FOC (Mode 1) strictly forbids `watchdog=False`. Acknowledging a startup timeout can only be done in Mode 0 with `watchdog=False`.
6. **Linux Serial `exclusive=True` is Not Multi-Process Coordination**:
   `exclusive=True` in PySerial issues `ioctl(fd, TIOCEXCL)`. While this prevents concurrent opens of the same device node on Linux, it does not manage command ownership, generation tracking, or multi-process RPC as required by C01.

---

### Open Questions and Gaps for B11–B14

1. **B12 Type Validation vs Value Validation**:
   [`im6014_codec.py:24`](file:///home/sid/Documents/code/im6014Workbench/im6014_codec.py#L24) checks `0 <= motor_id <= 14`, but does not enforce `isinstance(motor_id, int)`. Non-integer numbers like `1.5` pass the inequality and trigger a downstream `TypeError` when packed into `bytes`. B12 requires systematic rejection of non-integer IDs and non-bytes decoder input.
2. **Separation of Protocol Limits from Bench Limits**:
   For B12, should `im6014_codec.py` reject values based purely on 16-bit/32-bit register overflow (e.g. `abs(torque) > 162.1 Nm`), while moving the $\pm 3.5$ Nm and Kp/Kd bench limits to the controller supervisor (C09-C13)?
3. **B14 Diagnostic Capture Format**:
   Currently, raw packets are surfaced as hex strings in dictionary keys (`raw=raw.hex()`) and displayed in a UI text box (`self.packets.setPlainText(...)`). B14 calls for comparing raw packets and decoded fields without exposing credentials. The exact persistence format (e.g. redaction-safe JSON schema vs binary trace) needs an agreed-upon structure before implementation.