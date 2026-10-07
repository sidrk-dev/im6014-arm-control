# B01–B10 comment review

Reviewed `/home/sid/Documents/code/im6014Workbench` at `7684e0b92b9ef4c0121074b8ebf39e77b1114b7e` against `c5132702a164e4f2f611e30de914f24d8d52739a`. Applied the prepared worker contract and `agents/comment-sicko.md`. This is a report only.

Nine authored runtime, test, and script files were inspected: `fake_serial.py`, `im6014_codec.py`, `im6014_serial.py`, `motor_session.py`, `spin_5rpm.py`, `scripts/check_offline_baseline.sh`, `test_protocol.py`, `test_serial_deadline.py`, and `test_startup.py`.

Source files touched: none. Artifact written: this file. Recommended deletion count: four docstring blocks, four source lines. Retain five public API docstring blocks. MUST KILL flags: zero.

## Exact deletions

All locations below refer to the reviewed commit. Delete each complete line without replacing its prose.

| File and line | Exact text | Reason |
| --- | --- | --- |
| `fake_serial.py:1` | `"""Scripted, deterministic serial fake for offline boundary tests."""` | Module-purpose narration. The callable contracts are retained below. |
| `im6014_serial.py:1` | `"""One-owner, single-request IM6014 stream transaction."""` | Module summary duplicates the actionable public `exchange` contract. |
| `test_protocol.py:1` | `"""Offline IM6014 communication-boundary acceptance checks."""` | Test-suite narration rather than a public API contract. |
| `test_serial_deadline.py:1` | `"""Deadline behavior on scripted transports and isolated Linux pseudo-terminals."""` | Test-suite narration rather than a public API contract. |

## Exact keeps and scoped proof

| File and lines | Symbol | Keep clause and source proof |
| --- | --- | --- |
| `fake_serial.py:21` | `FakeChunk` | Public API contract. It specifies immutable chunks and delay relative to the previous chunk. `@dataclass(frozen=True)` and `__post_init__` at lines 19–28 establish immutability and bytes conversion. `_schedule_response` at lines 124–131 accumulates each chunk's delay. |
| `fake_serial.py:32` | `FakeClock` | Public API contract. Callers need to know that waiting uses explicit fake time. `monotonic`, `sleep`, and `advance` at lines 39–47 expose that behavior; reads and writes call `clock.advance`. |
| `fake_serial.py:51–62` | `FakeSerial` | Public API contract. It specifies accepted response shapes, recording semantics, mutable controls, and reset behavior. `_transaction` at lines 102–106 normalizes transactions; `write` at lines 140–156 snapshots every write and consumes a reply only after a complete successful write; `reset_input_buffer` at lines 133–138 clears ready/scheduled input without clearing `_responses`. The fake returns supplied bytes and does not generate motor packets. |
| `fake_serial.py:110` | `FakeSerial.writes` | Public API contract. The property returns a tuple at line 111, while `write` captures `bytes(data)` at lines 140–142. This distinguishes attempted writes from successful writes and prevents callers from changing the recorded data. |
| `im6014_serial.py:61–67` | `exchange` | Public API contract. It states caller ownership and driver preconditions, the limit of synchronous cancellation, and timeout restoration before success. The body resets once at line 77 and writes once at line 82. It bounds supported timeout settings, restores both in `finally` at lines 93–104, checks the deadline after restoration at line 105, and only then returns. These synchronous calls contain no cancellation mechanism. The docstring makes this limit explicit instead of claiming that arbitrary drivers can be interrupted. |

These keeps use the public API contract exception. They do not rely on an unverified firmware claim or a proposed application refactor.

## MUST KILL flags

None. No added comment explains a hidden workaround in owned application code, and no added suppression disables a correctness or safety rule. No refactor target is inferred from comment-free code.

## Skips and limits

- Existing comments in `spin_5rpm.py`, `motor_session.py`, and `test_startup.py` are unchanged by this diff and are outside this deletion list. The old comment in the removed `spin_5rpm.exchange` implementation is already deleted by the reviewed diff.
- `im6014_codec.py` adds no comments or docstrings. The edited shell script adds no prose comments. Its shebang is an interpreter directive.
- Preserved vendor CRC source, raw model outputs, raw logs, binary evidence, evidence data, and standalone design/workflow documents were excluded as assigned.
- Inspection used Git reads and source reads only. No tests, package operations, Git mutations, source edits, serial access, or hardware commands were performed.
