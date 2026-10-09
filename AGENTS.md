# Workbench contributor instructions

Before editing, read [coordination](docs/development/coordination.md), the [task registry](docs/development/tasks.tsv), [architecture](docs/architecture.md), and [milestones](docs/milestones.tsv). Check the current branch and working tree. The user prioritizes a three-joint keyboard/Xbox dashboard; the remaining milestone queue is paused.

## Own an isolated scope

Agree on a task, base commit, allowed files, and acceptance checks with the integration coordinator. Use a separate checkout or worktree. Do not edit another contributor's directory, rename files an active worker needs, or overwrite unrelated work. The coordinator updates the registry and integrates changes.

Dispatched Pstack workers obey their prepared contract. Their parent owns Git mutations, installations, checks, and hardware operations. Independent user-approved collaborators can use their own branches and tools and submit draft PRs. Their account and model choices are their own. Keep the operator's model sheet as the single routing configuration.

## Preserve control authority

Keep one station/Core authority and one managed serial-port owner. Preserve exact Fraction reference histories, separate decoded wire velocity, actual per-joint write outcomes, canonical packet caches, and persistent protection/recovery rules. Preserve the bench launcher while introducing the arm interface.

Use fake I/O, isolated PTYs, and disconnected startup for development. Opening a physical port, Connect, Check, Scan, timeout acknowledgment, Stop, and motion require explicit live-hardware scope. The assembled arm is not commissioned; the timeout acknowledgment remains unresolved.

## Finish with evidence

Use installed Pstack poteto-mode for engineering work. Ground behavior with how/why, agree on types and ownership before changes, and finish each unit with actual checks. Retain failures and inspect configured review findings against source. Apply no-comments/deslop, maintain the decision trail, and obtain fresh independent verification before guarded integration.

Run the applicable checks through the parent when the worker contract requires it. The offline baseline is `scripts/check_offline_baseline.sh`. Keep raw logs, source hashes, and inspected captures. Separate software, simulation, hardware, and cloud results. A model report or screenshot filename does not prove success. Merge only the independently verified commit with green checks on that exact head.
