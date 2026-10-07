# IM6014 workbench instructions

Read `docs/ROBOT_WORKBENCH_AGENT_PROMPT.md`, `docs/milestones.tsv`, and `docs/architecture.md` before changing the workbench. Preserve the working bench entry points.

Use the operator's configured Pstack routes. Do not copy model choices into a second editable routing configuration. The parent owns Git operations, package installations, test execution, and the physical communication boundary. Model reviewers receive supplied artifacts and do not establish physical results.

Default to fake transport and disconnected startup. Opening the motor port, Connect, Check, Scan, timeout acknowledgment, Stop, and motion are hardware control operations. A software-development request does not authorize them. Obtain explicit live-hardware scope before physical I/O.

Implement one milestone behavior at a time. Update the ledger with actual parent-run results, source revision, and evidence paths. Never turn a model's assertion or a screenshot filename into a passing result. Inspect screenshots. Keep software, simulated, physical, and cloud evidence separate.

Use `scripts/check_offline_baseline.sh` for offline regression checks after installation. Read the script and supplied tests before execution. Never replace fake transport with a physical serial port in these checks.
