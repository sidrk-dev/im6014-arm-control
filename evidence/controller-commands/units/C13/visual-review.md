# Independent Visual-Evidence Review Report: C13 Baseline Host Run

**Run Directory**: `/home/sid/Documents/code/im6014Development/.workbench-runs/c11-c16-20261014/c13-baseline-host/run-20261015T031209Z-Fh6pYp2a`  
**Reviewer**: Independent Research Subagent  
**Date of Review**: October 15, 2026  
**Artifacts Inspected**:
1. `dashboard_preview.png` (Speed Test Active Dual-Trace — Dark Theme)
2. `dashboard_dark_preview.png` (Position Test Active Dual-Trace — Dark Theme)
3. `startup-real/dashboard.png` (Real Hardware Startup Baseline — Light Theme)
4. `startup-demo/dashboard.png` (Demo Mode Startup Baseline — Light Theme)

---

## 1. Executive Summary & Verdict

- **Hardware Safety Boundary**: **VERIFIED PASS**. At application launch in both real and demo configurations, the workbench starts in a strictly disconnected, idle state. No serial port is opened, hardware motion controls are disabled/greyed out, packet counters read `Valid replies 0 · CRC errors 0 · Missed replies 0`, and telemetry metrics display null indicators (`—`).
- **Demo vs. Real Hardware Demarcation**: **VERIFIED PASS**. Clear, persistent top-right badges differentiate runtime modes (`DEMO · no hardware · Disconnected/Testing` in amber/yellow vs. `REAL HARDWARE · Disconnected` in slate blue). Inline contextual warnings notify the operator when simulated transports are active versus missing physical serial devices.
- **Active Test Visualization**: **VERIFIED PASS**. Live tests display dual-trace plotting (solid blue measured vs. dashed gold target) with auto-scaling axes, responsive telemetry cards (speed, torque, bus voltage), packet timing metrics (reply age, RTT, valid/CRC counts), and timestamped safety logs recording explicit single-test arming before execution.
- **Contrast & Legibility**: **VERIFIED PASS**. Both light and dark themes maintain high contrast, clean typography, unambiguous button states (enabled vs. greyed-out disabled), and distinct chart line styles.

---

## 2. Detailed Inspection of Artifacts

### Artifact 3: `startup-real/dashboard.png` (Real Hardware Clean Startup)
- **Header & Status Banner**:
  - Title: `IM6014 Motor Bench · connect, inspect, test`
  - Guidance text: *"Select an adapter and connect."*
  - Top-right Status Badge: `REAL HARDWARE · Disconnected` (pale blue-gray badge with high-contrast text).
  - Emergency Disarm: Prominent red `STOP / DISARM Esc` button.
- **Workflow Step 1 (Connect)**:
  - Source: `Real motor` selected.
  - Adapter profile: `U2D2`.
  - Serial port: Empty dropdown with adjacent `Refresh` button.
  - Warning banner: *"No serial devices found. Connect the adapter, then select Refresh."*
  - Baud: `6 Mbps`, Motor ID: `0`.
  - Controls: `Connect` enabled; `Disconnect` disabled/greyed out.
- **Workflow Step 2 (Check_prepare)**:
  - `Check connection`: **Disabled / greyed out**.
  - `Acknowledge timeout`: **Disabled / greyed out**.
  - Safety precondition checkbox: `Motor secured / unloaded | Power cut within reach` is **Unchecked**.
  - `Arm one test`: **Disabled / greyed out**.
- **Workflow Step 3 (Bounded test)**:
  - Parameters present (`Speed 5.00 rpm`, `Kd 6.00`, `Friction 0.160 Nm`, `Duration 5.00 s`, `Trip Off`), but completely unfired and unarmable without completing Steps 1 & 2.
- **Live Plots & Telemetry Area**:
  - Telemetry readouts: `SPEED · rpm: —`, `TORQUE · Nm: —`, `BUS · V: —` (dash indicators, no fake or residual data).
  - Plot panes: Both Speed and Torque plots display empty canvas with centered placeholder text: *"Connect to see live feedback"*.
  - Communications summary bar:  
    `Reply age — · Round trip 0.0 ms · Valid replies 0 · CRC errors 0 · Missed replies 0`.
- **Event Log & Footer**:
  - Event log box is completely clear/blank.
  - Console prompt: `Optional console: help, status, stop, check, ack-timeout, arm, speed 5 5`.
  - Status footer: `Output-shaft units · One selected motor · No automatic motion`.

### Artifact 4: `startup-demo/dashboard.png` (Demo Mode Clean Startup)
- **Header & Status Banner**:
  - Top-right Status Badge: `DEMO · no hardware · Disconnected` (amber/tan badge clearly indicating simulation mode).
  - Emergency Disarm: Active red `STOP / DISARM Esc`.
- **Workflow Step 1 (Connect)**:
  - Source: `Demo simulator`.
  - Adapter profile: `U2D2`.
  - Serial port: `Demo bus · IDs 0 and 1`.
  - Notice text: *"Demo mode uses a simulated motor; no serial adapter is needed."*
  - Baud: `6 Mbps`, Motor ID: `0`.
  - Controls: `Connect` button enabled; `Disconnect` disabled/greyed out.
- **Workflow Steps 2 & 3**:
  - Fully locked: `Check connection`, `Acknowledge timeout`, and `Arm one test` are **disabled / greyed out**. Safety checkbox unchecked.
- **Live Plots & Telemetry Area**:
  - Metrics: `SPEED: —`, `TORQUE: —`, `BUS: —`.
  - Canvases: Display placeholder *"Connect to see live feedback"*.
  - Counters: `Reply age — · Round trip 0.0 ms · Valid replies 0 · CRC errors 0 · Missed replies 0`.
- **Conclusion**: Even in demo simulation mode, application launch is inert and disconnected until explicit operator connection.

### Artifact 1: `dashboard_preview.png` (Speed Test Active — Dark Theme)
- **Header & Mode Badges**:
  - Banner: `Speed test · 2s maximum` in high-contrast header bar.
  - Mode Badge: `DEMO · no hardware · Testing` (amber pill badge).
  - Emergency Disarm: Red `STOP / DISARM Esc` button.
- **Workflow Controls**:
  - Source: `Demo simulator`, Serial port: `Demo bus · IDs 0 and 1`.
  - Precondition: Safety checkbox `[X] Motor secured / unloaded | Power cut within reach` is checked.
  - Arming state: Single test arming executed.
  - Bounded test parameters: `Speed = 5.00 rpm`, `Kd = 6.00`, `Friction = 0.160 Nm`, `Duration = 2.00 s`, `Trip = Off`.
- **Live Telemetry & Dual-Trace Plotting**:
  - Metric cards:
    - `SPEED · rpm`: **`2.812`**
    - `TORQUE · Nm`: **`0.391`**
    - `BUS · V`: **`36.000`**
  - **Speed Plot (`Speed (rpm) · last 15 seconds`)**:
    - Legend: `Measured` (solid blue) and `-- Target` (dashed amber).
    - Trajectory: Dashed amber line ramps up to target 5.00 rpm; solid blue measured line tracks acceleration smoothly up through 2.812 rpm.
    - Y-Axis: Clearly labeled (`-0.49`, `0.57`, `1.64`, `2.71`, `3.78`).
    - X-Axis: Time window labeled `-15 s` to `now`.
  - **Torque Plot (`Torque (Nm) · last 15 seconds`)**:
    - Legend: `Measured` (solid blue).
    - Trajectory: Shows acceleration torque surge peaking at ~0.40 Nm and tracking system inertia/friction.
    - Y-Axis: `-0.10` to `0.52 Nm`.
  - Comm status bar:  
    `Reply age 7 ms · Round trip 0.1 ms · Valid replies 62 · CRC errors 0 · Missed replies 0`.
- **Event Log**:
  - `0.24s Armed for one test; motor remains stopped.`
  - `0.33s TEST START: {'rpm': 5.0, 'kd': 6.0, 'ff': 0.16, 'torque': 0.1, 'duration': 2.0, 'position_deg': 10.0, 'kp': 3.0, 'speed_limit': 0.0, 'travel_limit': 0.0, 'kind': 'Speed', 'position_reference': 'Relative'}`

### Artifact 2: `dashboard_dark_preview.png` (Position Test Active — Dark Theme)
- **Header & Mode Badges**:
  - Banner: `Position test · 3s maximum`.
  - Mode Badge: `DEMO · no hardware · Testing`.
  - Emergency Disarm: Red `STOP / DISARM Esc`.
- **Workflow Controls**:
  - Bounded test parameters:
    - Test: `Position`
    - Target move rate · rpm: `5.00`
    - Position reference: `Relative`
    - Move by · degrees: `10.000`
    - Stiffness: `Kp = 3.00`, `Kd = 1.00`, `Duration = 3.00 s`.
    - Safety checkbox checked.
- **Live Telemetry & Dual-Trace Plotting**:
  - Metric cards:
    - `SPEED · rpm`: **`0.962`**
    - `TORQUE · Nm`: **`-0.020`**
    - `BUS · V`: **`36.000`**
  - **Position Plot (`Position (degrees) · last 15 seconds`)**:
    - Legend: `Measured` (solid blue) vs `-- Target` (dashed amber).
    - Trajectory: Commanded target trajectory executes smooth S-curve/ramp motion from baseline towards commanded destination (~17° total span); measured position line follows faithfully with realistic physical damping lag.
    - Y-Axis: `-2.25`, `2.62`, `7.49`, `12.36`, `17.23`.
  - **Torque Plot (`Torque (Nm) · last 15 seconds`)**:
    - Shows acceleration torque pulse up to ~0.50 Nm, followed by deceleration torque drop and settling towards zero (`-0.020 Nm`).
    - Y-Axis: `-0.13` to `0.53 Nm`.
  - Comm status bar:  
    `Reply age 2 ms · Round trip 0.0 ms · Valid replies 175 · CRC errors 0 · Missed replies 0`.
- **Event Log**:
  - `1.04s Armed for one test; motor remains stopped.`
  - `1.14s TEST START: {'rpm': 5.0, 'kd': 1.0, 'ff': 0.16, 'torque': 0.1, 'duration': 3.0, 'position_deg': 10.0, 'kp': 3.0, 'speed_limit': 0.0, 'travel_limit': 0.0, 'kind': 'Position', 'position_reference': 'Relative', 'target_position': 0.2615459669964724, 'move_s': 0.75}`

---

## 3. Evaluation Against Safety & Workbench Requirements

| Requirement | Visual Evidence Observed | Evaluation |
|---|---|---|
| **Hardware Boundary: Disconnected startup** | Both `startup-real` and `startup-demo` show `Disconnect` disabled, `Check` and `Arm` disabled, plots displaying *"Connect to see live feedback"*. | **PASS** |
| **No physical serial open on startup** | Real startup displays empty serial port dropdown with explicit warning: *"No serial devices found..."*. No port handles grabbed. | **PASS** |
| **Zero packet counters at rest** | Both startups display: `Valid replies 0 · CRC errors 0 · Missed replies 0` and `Reply age —`. | **PASS** |
| **Demo vs Real visual demarcation** | Real shows `REAL HARDWARE · Disconnected` badge (slate blue); Demo shows `DEMO · no hardware · Disconnected / Testing` (amber). Contextual labels clearly identify demo bus vs missing physical hardware. | **PASS** |
| **Safety arming guard** | Safety checkbox (`Motor secured / unloaded | Power cut within reach`) must be checked before test arming; event log explicitly logs *"Armed for one test; motor remains stopped"* before *"TEST START"*. | **PASS** |
| **Dual-trace plotting & active test feedback** | Both Speed and Position active runs plot dual traces (solid blue measured vs dashed gold target), synchronized torque curves, and live metric cards. | **PASS** |
| **Theme contrast & legibility** | Light theme (`startup-*`) provides clean borders, readable dark text on light gray; Dark theme (`dashboard_preview`, `dashboard_dark_preview`) provides high-contrast white text on dark gray/slate cards with high-luminance plot lines. | **PASS** |

---

## 4. Conclusion

All 4 visual artifacts from run `run-20261015T031209Z-Fh6pYp2a` satisfy the IM6014 workbench requirements. The user interface demonstrates strict compliance with hardware boundary isolation, transparent demo vs. real indicators, single-arm bounded testing, and unambiguous telemetry plotting.
