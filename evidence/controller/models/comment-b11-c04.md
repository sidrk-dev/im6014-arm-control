# Comment review through C04

Scope: `/home/sid/Documents/code/im6014Workbench`, base `d8d810888f76c676acaa1d54bc9e790249592f45`, frozen head `0ef01e9`. Reviewed added or changed comments and docstrings in source, tests, and scripts. All locations below refer to the frozen head.

Proposed deletions: 11 comment/docstring blocks, 12 physical lines, across 6 files. No source edits were made.

## Exact deletions

| File and lines | Delete verbatim | Evidence and reason |
| --- | --- | --- |
| `im6014_port.py:1` | `"""Passive configuration checks for the existing IM6014 serial entry points."""` | Module-purpose narration. The module also opens and leases ports through `open_managed_port`, so this sentence no longer describes its complete responsibility. |
| `im6014_port.py:125` | `"""Open and validate a fixed private directory without following links."""` | Private helper `_validate_private_directory`; its name and `os.O_NOFOLLOW` at line 135 express the operation. It defines no public API contract. |
| `im6014_port.py:155` | `"""Require the fixed namespace parent to be the trusted sticky /tmp."""` | Private helper `_validate_tmp_root`; literal `/tmp`, root UID, and `0o1777` checks at lines 156-163 express the requirement. |
| `im6014_port.py:183` | `"""Acquire a never-unlinked lockfile keyed by character-device st_rdev."""` | Private helper `_private_lock_fd`; the device-number argument and `os.major`/`os.minor` filename at line 194 express the key. The separate comment at line 217 covers the external locking constraint and survives. |
| `im6014_port.py:285-286` | Both lines shown below. | Narrates the implementation's implicit exception attribute. `error.managed_port = managed` at line 289 is the actual mechanism; this is our exception contract and needs a structural name or explicit contract, rather than prose defending it. See the flag below. |
| `motor_session.py:303` | `# Do not keep polling a bus whose close was not confirmed.` | Narrates `self.bus = None` at line 304 after `_close_managed_port`. The owner is already retained by `_close_managed_port` on failure at lines 285-295. |
| `test_bus_owner.py:1` | `"""Offline cross-process serial ownership checks using Linux pseudo-terminals."""` | Test-module narration; `PseudoTerminal` and ownership test names state the fixture and behavior. |
| `test_bus_owner.py:31` | `"""Serial-shaped fake that opens only a supplied PTY for fstat checks."""` | Test-helper narration. `FakeSerial.open` directly opens `self.open_path or self.port`; the comment is neither a public API contract nor an enforced PTY restriction. |
| `test_controller.py:25` | `"""Test-only owner that emits fake Stop frames after accepted decisions."""` | Test-helper narration; `_write_stops`, `decision.accepted`, and `writes` make the behavior visible at lines 27-60. |
| `test_diagnostic_capture.py:1` | `"""B14 allowlist, bounds, provenance, and caller integration checks."""` | Test-module narration. Named test cases provide the specific contracts. |
| `test_port_config.py:1` | `"""Offline B13 checks for explicit, passive port configuration."""` | Test-module narration. `B13PortConfig` and its individual test names already state the scope. |

Delete this block from `im6014_port.py:285-286` without replacement:

```python
        # Keep an unconfirmed owner reachable through the original exception so
        # callers can retry cleanup instead of dropping the lock fd.
```

## MUST KILL flags

- `MUST KILL im6014_port.open_managed_port` at `im6014_port.py:284-292`: make failed-close owner retention explicit in the exception contract or extract a named retention operation. The current implementation adds `managed_port` to an arbitrary original exception and silently ignores failure to attach it. Preserve original-error propagation and retryable lease retention when the parent evaluates a structural change. This is a comment-review flag, not a claim that a tested failure was observed.

## Kept comments and skips

Kept 15 blocks under the skill's exceptions:

- `im6014_port.py:24,47,51,55,63-69,83-87,246-251,400-405`: public configuration, exception, ownership, close, open, and passive-inspection contracts.
- `im6014_port.py:217`: `# Never unlink lockfiles: all processes must keep locking the same inode.` This concerns the external Linux `flock`/file-identity mechanism. The production path opens one persistent per-device lockfile at line 198 and locks that descriptor at line 219; cleanup closes descriptors without unlinking the path. This was retained on static inspection of the actual acquisition and release paths. No new runtime proof was performed.
- `im6014_serial.py:46,56,79,224`: public timeout, invalid-feedback, bounded diagnostic-ring, and injectable exchange contracts.
- `joint_controller.py:1-5,261`: public caller thread-confinement, transport ownership, immutable snapshot, and fixed joint-order contracts.

No changed comments were found in `im6014_codec.py`, `joint_mapping.py`, `scripts/check_milestone.py`, `scripts/check_offline_baseline.sh`, `spin_5rpm.py`, `test_dashboard.py`, `test_joint_mapping.py`, `test_numerical_inputs.py`, or `test_startup.py`.

Excluded `docs/` and `evidence/`, including verbatim reviewer artifacts. C05/C06 in the isolated writer checkout are outside this review. No changed suppressions, commented-out code, legal headers, or issue/RFC links required judgment. No tests, package operations, Git writes, physical I/O, checkout edits, or subagents were used. Read-only source extraction used `git show` at the frozen head and zero-context diff ranges from the specified base.
