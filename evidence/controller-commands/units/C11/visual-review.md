# Independent Visual-Evidence Review: Milestone C11 Baseline Run

**Run Directory**: `/home/sid/Documents/code/im6014Development/.workbench-runs/c11-c16-20261014/c11-baseline-host/run-20261015T022706Z-QHcHWoUK`  
**Reviewer**: Independent Visual Review Subagent (`research`)  
**Date**: October 15, 2026  

---

## Executive Summary

A comprehensive visual inspection of all four visual artifacts (`dashboard_preview.png`, `dashboard_dark_preview.png`, `startup-real/dashboard.png`, and `startup-demo/dashboard.png`) was completed.

1. **Safety & Hardware Boundary**: Zero physical hardware I/O occurred. Real-hardware startup (`startup-real/dashboard.png`) starts strictly disconnected with no serial adapter open, no serial devices bound, and all arming/testing controls disabled.
2. **Demo Mode Disconnected State**: The initial startup demo mode (`startup-demo/dashboard.png`) begins strictly disconnected with all arming, check, and motion controls disabled, requiring explicit user connection before any simulation begins.
3. **Active Test Visual Evidence**: The active testing captures (`dashboard_preview.png` and `dashboard_dark_preview.png`) clearly display the yellow `DEMO · no hardware · Testing` badge, simulated 36.000 V bus, simulated responses (66 and 178 valid replies, 0 CRC errors, 0 missed replies), explicit test start logs, and accurate dual-trace target vs. measured plotting for speed and relative position control.
4. **Theme & Contrast Verification**: Both light and dark UI themes provide high contrast, legible typography, distinct warning/status badges, and clear button state demarcations.

---

## Detailed Image-by-Image Inspection

### 1. `dashboard_preview.png` (Active Speed Test - Dark Theme)
- **Badge & Connection State**: Top-right status badge displays `DEMO · no hardware · Testing` in bold black text on gold badge. Red `STOP / DISARM Esc` button active. Subtitle: `Speed test · 2s maximum`. Connected to `Demo simulator`.
- **Adapter Profile & Serial Configuration**: `U2D2` profile, `Demo bus · IDs 0 and 1`, baud 6 Mbps, motor ID 0.
- **Control Status**: Safety precondition checked. Active speed test running (5.00 rpm target, Kd 6.00, friction 0.160 Nm, duration 2.00s).
- **Telemetry & Plots**: Speed plot shows measured blue curve ramping toward target amber dashed curve (2.961 rpm current). Bus 36.000 V. Reply age 7 ms, 66 valid replies, 0 CRC errors, 0 missed replies.
- **Hardware Boundary**: Pure software simulation (`DEMO · no hardware`).

### 2. `dashboard_dark_preview.png` (Active Position Test - Dark Theme)
- **Badge & Connection State**: Status badge `DEMO · no hardware · Testing`, red STOP/DISARM button active. Subtitle: `Position test · 3s maximum`. Connected to `Demo simulator`.
- **Adapter Profile & Serial Configuration**: `U2D2` profile, `Demo bus · IDs 0 and 1`, baud 6 Mbps, motor ID 0.
- **Control Status**: Safety precondition checked. Position test running (relative 10.0 deg, 5.00 rpm move rate, Kp 3.00, Kd 1.00).
- **Telemetry & Plots**: Position plot shows smooth measured tracking curve following 10 deg target displacement. Torque bump to ~0.47 Nm during acceleration, settling to -0.020 Nm. Reply age 2 ms, 178 valid replies, 0 CRC errors, 0 missed replies.
- **Hardware Boundary**: Pure software simulation (`DEMO · no hardware`).

### 3. `startup-real/dashboard.png` (Startup Real Mode - Light Theme)
- **Badge & Connection State**: Grey/blue badge `REAL HARDWARE · Disconnected`. Red STOP/DISARM button present. Subtitle: `Select an adapter and connect.`. Source: `Real motor`. Disconnected (`Connect` button enabled, `Disconnect` greyed out).
- **Adapter Profile & Serial Configuration**: `U2D2` profile, serial port dropdown blank (`No serial devices found. Connect the adapter, then select Refresh.`).
- **Control Status**: All controls disabled (`Check connection`, `Acknowledge timeout`, `Arm one test` all disabled; precondition unchecked; log empty).
- **Telemetry & Plots**: `SPEED · rpm: —`, `TORQUE · Nm: —`, `BUS · V: —`. Graphs display `Connect to see live feedback`. Counters: 0 valid replies, 0 CRC errors, 0 missed replies.
- **Hardware Boundary**: Zero physical I/O performed. Safe disconnected default verified.

### 4. `startup-demo/dashboard.png` (Startup Demo Mode - Light Theme)
- **Badge & Connection State**: Yellow badge `DEMO · no hardware · Disconnected`. Red STOP/DISARM button present. Subtitle: `Select an adapter and connect.`. Source: `Demo simulator`. Disconnected (`Connect` enabled, `Disconnect` greyed out).
- **Adapter Profile & Serial Configuration**: `U2D2` profile, `Demo bus · IDs 0 and 1`.
- **Control Status**: All controls disabled on launch (`Check connection`, `Acknowledge timeout`, `Arm one test` all disabled; precondition unchecked; log empty).
- **Telemetry & Plots**: All readouts placeholder dashes. `Connect to see live feedback` displayed. Counters: 0 valid replies, 0 CRC errors, 0 missed replies.
- **Hardware Boundary**: Simulation not started automatically. Safe disconnected default verified.

---

## Conclusion
Visual evidence confirms strict adherence to workbench safety invariants:
- Default disconnected state on startup with all controls disabled.
- Clear visual distinction between real and simulated modes.
- Accurate dual-trace plotting and reply counter accounting.
- Zero physical hardware I/O occurred.
