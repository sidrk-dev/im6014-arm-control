# Repository ownership and component boundaries

## One repository

Use one private `sidrk-dev/im6014-workbench` repository for the bench, future controller integration, workbench adapters, and milestone evidence. These components currently share one developer, one control contract, and one verification cycle. Separate repositories or Git submodules would add version coordination without an independent release boundary.

The bench history is inherited from `sidrk-dev/im6014Development` at `69c8219d764a24d03dadf1df011efb027f0f8996`. The original repository is a read-only provenance remote named `bench-upstream`. Do not merge unrelated upstream code into the protocol boundary without review.

Inspect external foundations at exact revisions and record the revision in evidence. Add a submodule only when a component needs an independent release and its interface is stable. Do not add placeholder submodules for unimplemented milestones.

## Current dependency direction

| Component | Owns | Imports or calls |
| --- | --- | --- |
| `start_dashboard.sh` | Resolve application path and start Python | `motor_dashboard.main` |
| `motor_dashboard.py` | Controls, display, plots, exports, UI heartbeat | `MotorSession`; codec constants; Qt; passive serial enumeration |
| `motor_session.py` | One GUI serial-owner worker, requests, demo transport, bench lifecycle | Packet encoding and exchange in `spin_5rpm.py`; pyserial |
| `spin_5rpm.py` | Packet codec, exchange, health checks, standalone bounded bench CLI | Python standard library and pyserial |
| Offline tests and startup verifier | Fixtures, rejection guards, visible evidence | Production modules; never imported by production |
| Baseline driver | Run audited checks on exact temporary source copies | Tests and selected Python interpreter |
| `docs/milestones.tsv` | Persistent A01-A10 results | Evidence paths; no runtime dependency |

The GUI's worker owns its bus. The standalone CLI is an alternative bus owner. Neither is yet a complete station process for a six-joint arm. Never run both against the same physical adapter. Serial exclusivity is an existing Linux port flag, not proof of the later C01 cross-process controller contract.

Keep protocol conversion inside the existing codec for this baseline. Extracting the communication boundary, introducing build profiles, and implementing a shared station belong to B, C, and D milestones. A01-A10 do not change the motor control implementation.

## MakerMods integration boundary

MakerMods Lab is the planned E01-E10 operator interface. Current inspection revision is `64defd4e5e9641199b1af5b90587080f721374ba`. Its [pinned pyproject](https://github.com/makermods-robotics/makermodslab/blob/64defd4e5e9641199b1af5b90587080f721374ba/pyproject.toml) requires Python `>=3.12,<3.14` and pins its LeRobot fork to `5ce2fe41a63647c20b2ab6e84bcb4cc7295a9b21`.

This repository uses Python 3.12 for its clean bench environment. MakerMods and LeRobot are not installed, integrated, or verified by A01-A10. Their later custom-arm adapter must submit requests through the shared station. It must not open a second serial connection.

## Evidence identities

`evidence/baseline` contains the committed parent-run snapshot. Later verification runs use fresh directories under `evidence/runs`, which are ignored. Fake motor outputs are marked as simulation. Disconnected startup contains no measured motor state. Hardware commissioning remains pending.
