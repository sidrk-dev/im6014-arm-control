# Existing J1 bench baseline

## Entry points

The source baseline is commit `69c8219d764a24d03dadf1df011efb027f0f8996` from sidrk-dev/im6014Development. This repository preserves that history. The user's original checkout and added plan remain unchanged.

| Entry point | Configuration | Dependency | Current evidence |
| --- | --- | --- | --- |
| `./start_dashboard.sh --demo` | Demo source selected; starts disconnected | Python, PySide6, pyserial | Existing offline suite passes. A10 independently checks startup and shutdown. |
| `./start_dashboard.sh` | Real source selected; starts disconnected | Same | Source shows no port open until Connect. A10 checks both source selections. |
| `python3 spin_5rpm.py` | `--port`, `--id`, `--baud`, `--rpm`, `--kd`, `--torque-ff`, `--duration` | Python, pyserial | Existing command and packet fixtures pass. Invoking the CLI sends Stop/status even without `--arm`. It is not an offline diagnostic. |
| `python3 test_spin_5rpm.py` | Fixed packet and fake transport fixtures | Python, pyserial | Parent ran it. See baseline log. |
| `QT_QPA_PLATFORM=offscreen python3 test_dashboard.py` | DemoSerial and mocked serial constructors | Python, PySide6, pyserial | Parent ran it. 18 grouped checks passed. |

The historic default is motor ID 0 at 6 Mbps on the FTDI Channel D path stored in `spin_5rpm.PORT`. Device identity, live communication, J1 mounting, calibration, and permitted motion are not verified by this run. The inherited README's physical result is historical source material, not a fresh test.

## Control packet audit

`spin_5rpm.command()` only encodes bytes. `spin_5rpm.exchange()` resets the input buffer, writes a packet, and reads feedback. The following paths reach that boundary.

| Operation | Source path | Transmission and stop effect |
| --- | --- | --- |
| Import modules or start disconnected dashboard | `Dashboard.__init__` → `MotorSession.__init__` → `_loop` | Bus is `None`. No serial constructor, open, packet write, or measured motor data. Port enumeration reads device metadata. |
| Connect | `connect_motor` → `request('connect')` → `_handle` → `_check` → `_stop` | Opens selected port. Sends mode-0 Stop/status. Stop requests zero targets and can release torque. |
| Check connection | `_handle('check')` → `_check` | Sends Stop then Stop/status validation. Not read-only. |
| Acknowledge timeout | `_handle('ack')` → `_check(acknowledge=True)` | Sends Stop packets and a watchdog-disabled mode-0 packet, then restores watchdog-enabled Stop/status. Explicit user acknowledgment required. |
| Connected idle polling | `_tick`, no active test | Sends mode-0 Stop/status approximately every 0.05 s when the bus is open. This is control traffic, not passive telemetry. |
| Arm | `_handle('arm')` | No immediate packet. Checks supported setup, health, freshness, and heartbeat. Connected idle Stop polling continues. |
| Run a test | `_handle('test')` and `_tick` | Enters mode 1 and repeatedly sends speed, position, damping, or torque targets. Requires fresh arming. |
| Scan IDs | `scan_ids` → `_handle('scan')` | Sends individual mode-0 Stop/status to IDs 0–14, then rechecks selected ID. No broadcast. Affects other connected motors. |
| Stop button or Esc | `request('stop')` → worker `_stop` | Invalidates queued generations, disarms, sends Stop if connected, and attempts confirmation. If disconnected, sends nothing. |
| Supported-setup box cleared while armed | `support_changed` → `request('stop')` | Same Stop path. |
| Test duration completed | `_tick` → `_stop` → `_check` | Sends Stop and verifies health while disarmed. |
| Fault or worker exception | `_fault` → `_stop` | Attempts Stop and latches disarmed fault. Stop confirmation can fail; it does not prove torque is off. |
| Interrupted operation | `_loop` handles `InterruptedError` → `_stop` | Attempts Stop. |
| Disconnect | `_disconnect` → `_stop` → bus close | Sends Stop before releasing a connected port. |
| Window close or worker finalization | `closeEvent` → `session.close`; `_loop` finally → `_disconnect` | Connected shutdown attempts Stop and closes port. Disconnected shutdown has no packet. |
| Bench CLI without `--arm` | `main` → port open → `run` | Sends initial Stop/status and final Stop packets. It does not request FOC motion. Still a control operation. |
| Bench CLI with `--arm` | `run` | After health checks, sends mode-1 arming and speed targets. Finally attempts three Stop exchanges. |
| CLI error, Ctrl-C, or normal end | `main` finally | If port is open, attempts three Stop exchanges, then closes port and records result. |

These are unloaded bench semantics. They are not a supported shutdown procedure for a future gravity-loaded shoulder. No physical connect, scan, acknowledgment, motion, or Stop packet was executed during A01-A10.
