# IM6014 robot workbench

Work alongside the construction of a custom six-joint IM6014 arm. The browser dashboard controls a single base motor or three daisy-chained motors, with a three-joint CAD view. The original bench dashboard and verified communication/controller baseline remain available.

The original bench source and history come from [sidrk-dev/im6014Development](https://github.com/sidrk-dev/im6014Development). The workbench lives in one repository. See [component boundaries](docs/architecture.md), the [full milestone plan](docs/ROBOT_WORKBENCH_AGENT_PROMPT.md), and the [milestone ledger](docs/milestones.tsv).

## Move the arm from the browser

```bash
./start_arm.sh
```

Open **http://127.0.0.1:8768/**. Connect → set the current pose → confirm your setup → enable jogging. Hold the on-screen joint buttons or use A/D for J1, W/S for J2 and Q/E for J3. No held Space or RB is required. J2/J3 are virtual in the default J1-only profile; select the three-motor profile while disconnected to commission all three. See [arm quickstart](docs/arm-quickstart.md) for unique motor IDs, mechanical support, individual zero references and travel limits. Esc / STOP disarms. Use `./start_arm.sh --demo` for a USB-free preview.

The adapter profile defaults to U2D2. An FT232H-class candidate label does not verify the board or motor link. The ordered U2D2 has not arrived; see [U2D2 connection and verification](docs/u2d2.md) before physical use.

Read the [short arm guide](docs/arm-quickstart.md) for first connection, software zero, joint limits, PD tuning, keyboard/Xbox mapping and troubleshooting. The interface follows the plain ARM2027 layout with the updated three-joint CAD model. Arbitrary finite parameter drafts are editable; applied physical settings retain the bench limits.

The original Qt bench dashboard remains available through `start_dashboard.sh`. The multijoint milestone controller remains a separate development track with no production station integration; the browser J1 controller does not depend on it.

## Install the verified Linux environment

The hash lock covers Linux x86_64 with glibc 2.34 or later. The selected interpreter is Python 3.12. Other platforms are not validated by this baseline.

```bash
git clone https://github.com/sidrk-dev/im6014-workbench.git
cd im6014-workbench
python3.12 -m venv .venv
.venv/bin/python -m pip install --require-hashes -r requirements.lock
.venv/bin/python -m pip check
```

`requirements.txt` preserves the original direct pins. `requirements.lock` adds the exact Qt component versions and wheel hashes. Install the lock for the verified environment.

## Run the offline baseline

```bash
scripts/check_offline_baseline.sh
```

The driver runs packet and protocol fixtures, serial deadline checks through isolated pseudo-terminals, the existing 18-group dashboard suite, negative startup-verifier checks, and disconnected startup checks for both source selections. It uses copied source in temporary storage. It does not modify the checkout's preview images. Each run produces new logs, screenshots, and startup receipts under `evidence/runs`. The command prints the run directory.

The parent must inspect the rendered screenshots before recording A10 as verified. A test's success alone does not establish that its display is useful.

The startup gate checks behavior with the supplied interpreter and records its versions. For reproducible runs, use the locked installation above and compare the receipt's interpreter and package versions with the recorded baseline. Version pinning is checked during installation and evidence review.

## Launch the disconnected dashboard

```bash
source .venv/bin/activate
./start_dashboard.sh --demo
```

Demo mode starts disconnected. Click Connect only when you want simulated telemetry. To select real hardware while remaining disconnected, omit `--demo`. Starting the app does not open the motor port.

Connect, Check, Scan, timeout acknowledgment, and Stop transmit control packets after a physical connection. They are not passive diagnostics. Physical operation requires explicit authorization and the setup procedures in [USER_MANUAL.md](USER_MANUAL.md).

## Current bench controls

- Speed in signed output-shaft rpm, with acceleration and deceleration ramps.
- Relative or absolute position in degrees, smooth targets, and adjustable Kp/Kd.
- Bounded torque pulses and damping tests.
- Telemetry, plots, packet traces, fault checks, and JSON/CSV exports.
- Priority Stop/disarm through the button or Esc. Each test needs fresh arming.

Displayed motion quantities use output-shaft units. The packet boundary applies the internal 38/3 reduction. The pure `joint_mapping` API now converts external joint units; the bench controls continue to use output-shaft units.

## Joint mapping and controller core

The transport-free `JointCore` owns a fixed ordered joint set, lifecycle decisions, per-joint feedback ages, attributed latched faults, an identity-checked owner grant, configured closed position intervals, one validated pending target and a retained monotonic sequence watermark. Arm rejects a measured pose outside those intervals; target submission rejects the complete requested vector before mapping or codec checks when any joint position is outside its interval. The intervals apply in joint radians and do not clip requests or certify physical travel. Stop, the first attributed fault and disconnect revoke the grant. `JointMap` converts joint position, speed, torque and gains before codec validation. Read [the joint API](docs/joint-api.md) and [the controller design](docs/controller-design.md).

Run one software milestone check with `.venv/bin/python -B scripts/check_milestone.py C09`. Results and source hashes are recorded in its printed evidence directory. The core has no production serial runner; packet emission is demonstrated by test-only fake adapters. Live station integration remains later work.

## Verification boundaries

The A01-A10 evidence is in [evidence/baseline](evidence/baseline). The B01-B10 evidence is in [evidence/boundary](evidence/boundary). The later batch verifies unchanged command bytes, strict decoder validation, stream recovery, requested motor identity, deadline behavior and output units. Software checks use fake transport, independent CRC fixtures, isolated pseudo-terminals and disconnected startup. Read [the transport design](docs/boundary-design.md) and [deadline limits](docs/serial-deadline.md).

The inherited bench README reported a historic 5.002 rpm physical result. The B01-B10 batch performed a separately authorized Stop/status probe at 36 V and received CRC-valid ID 0 feedback. The firmware timeout flag was latched, so no FOC command or motion test followed. Final Stop mode was confirmed. This is communication and Stop evidence, not calibration, torque-off, arm safety or a new speed result. See [the raw capture](evidence/boundary/physical/stop-prechange.json).

Stop releases torque on the bench and can let a gravity-loaded joint fall. A gravity-loaded arm needs its own supported shutdown procedure. The current bench tool is not the future six-joint station controller.

MakerMods Lab is the planned operator interface for E01-E10. Its exact inspected revision and Python compatibility are recorded in [architecture.md](docs/architecture.md). It is not installed or integrated here. Six-joint simulation, recording, VLA training, and cloud inference remain later milestones.
