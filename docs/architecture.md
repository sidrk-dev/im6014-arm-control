# Repository ownership and component boundaries

## One repository

Use one private `sidrk-dev/im6014-workbench` repository for the bench, future controller integration, workbench adapters, and milestone evidence. These components currently share one developer, one control contract, and one verification cycle. Separate repositories or Git submodules would add version coordination without an independent release boundary.

The bench history is inherited from `sidrk-dev/im6014Development` at `69c8219d764a24d03dadf1df011efb027f0f8996`. The `bench-upstream` remote records that source. Publish workbench changes to `origin`. Review upstream changes before incorporating them into the protocol boundary.

Inspect external foundations at exact revisions and record the revision in evidence. Add a submodule only when a component needs an independent release and its interface is stable. Do not add placeholder submodules for unimplemented milestones.

## Current dependency direction

| Component | Owns | Imports or calls |
| --- | --- | --- |
| `start_dashboard.sh` | Resolve application path and start Python | `motor_dashboard.main` |
| `motor_dashboard.py` | Controls, display, plots, exports, UI heartbeat | `MotorSession`; codec constants; Qt; passive serial enumeration |
| `motor_session.py` | One GUI serial-owner worker, requests, demo transport, bench lifecycle | Bench aliases in `spin_5rpm.py`; pyserial |
| `im6014_codec.py` | Pure packet encoding, strict feedback decoding, CRC and internal reduction | Python standard library |
| `im6014_serial.py` | One request, bounded stream scan, timeout budgets, restoration and optional bounded capture | Pure feedback decoder; Python standard library |
| `im6014_port.py` | Passive configuration and cooperating device lease | OS metadata; injected serial factory |
| `joint_controller.py` | Transport-free lifecycle and per-joint freshness | Frozen mapping records; Python standard library |
| `joint_mapping.py` | Pure external joint units and gain conversion | Python standard library |
| `spin_5rpm.py` | Bench aliases, health checks and standalone bounded CLI | Codec and serial boundary; pyserial |
| `fake_serial.py` | Scripted replies and deterministic clock for offline checks | Python standard library; never imported by production |
| Offline tests and startup verifier | Fixtures, rejection guards, visible evidence | Production modules; never imported by production |
| Baseline driver | Run audited checks on exact temporary source copies | Tests and selected Python interpreter |
| `docs/milestones.tsv` | Persistent acceptance results with source snapshots | Evidence paths; no runtime dependency |

The GUI worker and standalone CLI acquire the same cooperating device lease through `im6014_port.open_managed_port`. Aliases of a character device collide by `st_rdev`. The fixed private per-user namespace ignores TMPDIR and XDG settings. A second cooperating opener rejects before serial construction, and unconfirmed close retains ownership. Raw clients can ignore this advisory lock. Neither entry point is yet a complete station process for a six-joint arm.

B01-B10 extract the existing codec into pure functions and the serial exchange into a separate boundary. The bench aliases preserve callers and startup interception. Build profiles and the shared station remain later work. A01-A10 baseline records describe their earlier source revisions.

## MakerMods integration boundary

MakerMods Lab is the planned E01-E10 operator interface. Current inspection revision is `64defd4e5e9641199b1af5b90587080f721374ba`. Its [pinned pyproject](https://github.com/makermods-robotics/makermodslab/blob/64defd4e5e9641199b1af5b90587080f721374ba/pyproject.toml) requires Python `>=3.12,<3.14` and pins its LeRobot fork to `5ce2fe41a63647c20b2ab6e84bcb4cc7295a9b21`.

This repository uses Python 3.12 for its clean bench environment. MakerMods and LeRobot are not installed, integrated, or verified by A01-A10. Their later custom-arm adapter must submit requests through the shared station. It must not open a second serial connection.

## Evidence identities

`evidence/baseline` contains the committed parent-run snapshot. Later verification runs use fresh directories under `evidence/runs`, which are ignored. Fake motor outputs are marked as simulation. Disconnected startup contains no measured motor state. Hardware commissioning remains pending.
