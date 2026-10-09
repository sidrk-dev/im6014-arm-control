# IM6014 Workbench

A Linux workbench for Unitree IM6014 actuators. The current application is a single-motor Qt bench dashboard. The controller library also supplies joint mappings, exact bounded references, protection history, managed serial ownership, and station timing.

The active priority is a three-joint arm dashboard with keyboard and Xbox jogging. Its model and operator controls are being developed; there is no working arm launcher yet. Impedance and gravity control follow verified manual control and physical commissioning.

## Install

The verified environment is Python 3.12 on Linux x86_64 with glibc 2.34 or later.

```bash
git clone https://github.com/sidrk-dev/im6014-workbench.git
cd im6014-workbench
python3.12 -m venv .venv
.venv/bin/python -m pip install --require-hashes -r requirements.lock
.venv/bin/python -m pip check
```

The default branch contains the landed B11-C06 work. Check [coordination](docs/development/coordination.md) before choosing a base for newer controller or arm work.

## Start the bench with fake motors

```bash
source .venv/bin/activate
./start_dashboard.sh --demo
```

The dashboard starts disconnected. **Connect** in demo mode enables simulated telemetry. Omitting `--demo` selects real hardware while remaining disconnected. Starting the application does not open a physical port.

The bench supports output-shaft speed/position tests, bounded torque and damping tests, telemetry plots, traces, and exports. Physical operation requires explicit authorization and the setup procedure in the [bench manual](USER_MANUAL.md). A gravity-loaded arm needs support when torque is released.

## Check the software

```bash
scripts/check_offline_baseline.sh
```

The driver uses copied source, fake transport, isolated PTYs, and offscreen Qt startup. It prints a fresh directory under `evidence/runs/` with logs, receipts, and captures. Inspect the captures before claiming UI verification.

## Arm model and commissioning

The operator's latest URDF source was `mainAssem (3).zip`; the later `mainAssem (4).zip` download contains two 3MF print parts and no URDF. The selected arm model has 76 links and 75 joints, with the driven chain `revolute_2`, `revolute_1`, `revolute_3`; the forearm clamp joint is passive.

The arm design sets J1's intended travel to 180 degrees total, from -90 to +90 around a provisional zero. No arm motion path is implemented yet, and the zero and direction have not been commissioned on hardware. The operator reports that the J2 screw obstruction was removed in the updated CAD; J2/J3 physical travel still requires commissioning. Three IM6014 motors use provisional IDs 0/1/2 with no external reduction. The codec applies the motor's internal 38/3 ratio. IDs, direction, gains, timeout, and loaded-arm behavior are not physically verified. CAD inertials and visual meshes do not validate gravity or collision control.

## Documentation

- [Bench manual](USER_MANUAL.md): current operator procedures.
- [Contributing](CONTRIBUTING.md): setup, checks, and review.
- [Coordination](docs/development/coordination.md) and [tasks](docs/development/tasks.tsv): active ownership and collaborator handoffs.
- [Architecture](docs/architecture.md) and [joint API](docs/joint-api.md): control and transport boundaries.
- [Layout plan](docs/development/layout-plan.md): the package migration and caller inventory.
- [Milestones](docs/milestones.tsv) and [decisions](docs/decisions.tsv): actual acceptance and evidence.

The original bench source comes from [im6014Development](https://github.com/sidrk-dev/im6014Development). Development checkout and the user plan remain separate from this private workbench.
