# Function/module candidate for B01-B10

## Problem

Extract the existing codec and repair the synchronous serial transaction without changing the bench command API, feedback dictionaries, controls, or ownership. `MotorSession` remains the GUI's only bus owner. The CLI remains an alternative owner. Each exchange has one request and one local receive buffer. Source grounding is the parent-supplied `grounding.md` and completed `how.md`, plus inspected workbench files and the installed pyserial POSIX implementation. This candidate is a design, with no implementation, test execution, or hardware I/O by this runner.

## Usage, written before the types

Existing bench callers keep working:

```python
import spin_5rpm as bench

packet = bench.command(0)                     # Existing Stop bytes.
f = bench.exchange(bus, packet, 0, deadline_s=0.05)
bench.healthy(f, 0)                           # Bench policy belongs to the caller.
print(f['position_rad'], f['raw'])
```

The GUI keeps a module-level exchange name, so its existing sentinel and monkeypatch work:

```python
# motor_session.py
from im6014_codec import RATIO, command, crc32
from im6014_serial import exchange
from spin_5rpm import PORT, healthy

# Inside the existing _transact method:
packet = command(motor_id, mode, rpm, kd, torque, watchdog, position, kp)
f = exchange(self.bus, packet, motor_id)
f.update(time_s=now - self.started, target_rpm=rpm)
# Existing ID-selected state update follows only after successful exchange.
```

An offline consumer can use the pure codec without importing pyserial or the CLI:

```python
from im6014_codec import feedback

f = feedback(bytes.fromhex(
    'fcee801d1b48000000006cc4db000000000000000000f179457d'), 0)
assert f['id'] == 0 and f['mode'] == 0
# The timeout bit remains 1. Decoding never clears it or declares health.
```

## Shape

Two new production modules own distinct knowledge. `im6014_codec.py` owns byte layout, CRC, internal reduction, signedness, and exact packet acceptance. Its functions are deterministic and do no I/O. `im6014_serial.py` owns reset, timeout settings, the single write, candidate search, and the transaction deadline. It never opens, closes, reconnects, arms, clears faults, or retries a command. This is boundary-discipline: the codec parses external bytes once, and the transaction handles the serial boundary once. No boundary object holds a bus or parser state between calls.

Keep `command`'s current restrictions in the extracted pure function during B01. Name them as the existing bench command contract: individual IDs, Stop/FOC, zero Stop targets, FOC watchdog, Kd 0-6, Kp 0-30, and feedforward torque within ±3.5 Nm. These restrictions are policy, not firmware capabilities or rated limits. Representable speed/position ranges, scaling, packing, and CRC are protocol facts. `healthy` remains in `spin_5rpm.py` and owns measured mode, faults, warnings, timeout, voltage, temperature, torque, and optional speed gates. B01 does not introduce a second permissive encoder or relax those restrictions. A later policy redesign can move them as one explicit change.

The interface is small because the caller needs only its existing `command` and `exchange` calls. The exchange hides fragmented reads, noise trimming, overlapping candidate search, timeout changes, and failure diagnostics. Existing bytes APIs and the `raw` dictionary field remain for compatibility and diagnostics; adding a domain object would force every sample, plot, export, and test through a new representation. The only shared mutable object is the caller-owned bus, under the existing one-owner rule. All parser state is per call, per separate-before-serializing-shared-state.

## Synthesis decision

Pending the parent's comparison. This candidate proposes extracted functions with a transaction-local buffer. It introduces no object-owned boundary, station, registry, new CLI, or dependency.

## Tradeoffs accepted

- We accept the bench restrictions inside the pure encoder during extraction in exchange for one command path and unchanged caller behavior.
- We accept the existing dictionary and raw-packet API in exchange for byte fixtures, in-place sample enrichment, and exports remaining compatible.
- We accept a conditional synchronous deadline guarantee in exchange for retaining one serial worker. Python cannot interrupt a stuck OS ioctl or an arbitrary fake that ignores its timeout contract.

## Alternative considered

A `MotorBoundary(bus)` object with persistent parser state can hide transport settings behind methods. It also creates another lifetime owner alongside `MotorSession`, makes stale buffer handling survive transactions, and requires adapters to preserve the current function calls and patch point. Its extra state does not improve this one-request, one-reply contract. The function candidate hides the same exchange complexity with no construction or close protocol.

## Open questions and risks

- Will the parent accept `deadline_s` as a budget enforced by bounded read/write calls plus measured OS and scheduling overhead, rather than a hard real-time guarantee for reset and timeout-setting ioctls? The inspected pyserial code makes the stronger guarantee unavailable to this design.
- Should rejected CRC candidates followed by a valid reply count as a CRC error in the dashboard? This candidate keeps current counters as failed-transaction counters, avoiding new feedback keys or a callback API.
- Can a delayed reply from the same ID arrive after reset and be mistaken for the next reply? The protocol has no transaction identifier; reset and one outstanding request reduce this risk but cannot prove freshness by sequence number.

## Next implementation step

Extract the codec with compatibility exports, add both modules to the source-copy and fingerprint lists, and have the parent rerun the unchanged packet checks before proceeding to B02.

## Module and signature sketch

| File | Responsibility and changes |
| --- | --- |
| `im6014_codec.py` | Extract `RATIO`, representability constants, `crc32`, `command`, and `feedback`. Preserve validation, rounding, bytes, dictionary keys, exception families, and existing error text. No serial/time/CLI imports. |
| `im6014_serial.py` | Extract and repair `exchange`. Import `feedback` and pyserial's write-timeout exception. Use private clock/sleep bindings for deterministic offline tests. No bus ownership. |
| `spin_5rpm.py` | Re-export the existing names directly from the extracted modules. Retain `PORT`, `healthy`, `run`, `main`, existing CLI settings and final Stop sequence. Remove moved bodies rather than duplicate them. |
| `motor_session.py` | Import pure functions and exchange into its existing namespace. Retain lookup of `exchange`, `serial.Serial`, and `DemoSerial` in that namespace. Keep the worker, lifecycle, and disarmed fault latch. Give `DemoSerial` `write_timeout = 0` and make its empty-read sleep honor a clamped finite `timeout`. |
| `motor_dashboard.py` | Existing controls and plots use the same dictionary. In B09, honor `session.close()`'s boolean before accepting window close. A failed join must leave the window visible and allow its timer to report/retry shutdown. |
| `fake_serial.py` | Reusable test support only. Record writes and release scripted chunks per transaction. Production never imports it. |
| `test_boundary.py` | Parent-run B01-B10 fixtures and deterministic transaction checks. No physical port paths. |
| `test_serial_deadline.py` | Parent-run POSIX pseudo-terminal checks against the pinned actual pyserial implementation. No motor adapter access. |
| `scripts/check_offline_baseline.sh`, `test_startup.py` | Copy every new runtime/test support file into the scratch source and fingerprint them. Add boundary and pseudo-terminal checks to the driver. Preserve historical reports; new reports list the expanded source set. |

The compatibility names in `spin_5rpm` are aliases to the implementations, not a second implementation. Existing `motor_session.exchange` patching still intercepts every GUI transaction. Existing `motor_session.serial.Serial` and `DemoSerial` constructor sentinels still intercept construction. Tests can still use `spin_5rpm.command`, `feedback`, `crc32`, `exchange`, and all constants. Data flows through at most `MotorSession -> exchange -> feedback`.

```python
# im6014_codec.py, proposed declarations only.
from typing import TypedDict

class Feedback(TypedDict):
    id: int
    mode: int
    timeout: int
    driver_c: int
    winding_c: int
    voltage: float
    torque_nm: float
    rpm: float
    position_rad: float
    errors: int
    warnings: int
    raw: str

RATIO = 38 / 3
RPM_MIN = -32768 / 64 / RATIO * 60
RPM_MAX = 32767 / 64 / RATIO * 60
# POSITION_MIN/MAX retain the current expressions verbatim.

def crc32(data: bytes) -> int:
    raise NotImplementedError

def command(motor_id: int, mode: int = 0, rpm: float = 0,
            kd: float = 0, torque: float = 0, watchdog: bool = True,
            position: float = 0, kp: float = 0) -> bytes:
    raise NotImplementedError

def feedback(raw: bytes, motor_id: int) -> Feedback:
    raise NotImplementedError

# im6014_serial.py, same public argument names/order/default as today.
def exchange(bus, packet: bytes, motor_id: int,
             deadline_s: float = 0.05) -> Feedback:
    raise NotImplementedError

# spin_5rpm.py, retained public policy function.
def healthy(f, mode, allow_timeout=False, speed_limit=0):
    raise NotImplementedError
```

`Feedback` is a static description, not a runtime wrapper. `feedback` returns an ordinary fresh dictionary in the current insertion order. Time/target/demo/export fields remain additions made by existing callers. Exact feedback validation stays strict: length 26, header `fc ee`, CRC over bytes 2-21, requested ID, and supported mode 0 or 1. A valid nonzero fault/timeout packet is decoded, then the caller applies `healthy`. Do not require feedback mode to equal the requested command mode inside `exchange`; the existing zero-torque FOC transition explicitly permits the preceding Stop mode.

## Stream algorithm sketch

Use a 26-byte maximum read and a local buffer. After each scan, retained undecoded bytes are at most 25. The next append therefore holds at most 51 bytes. Keep bounded tail diagnostics separately if required; never accumulate the entire noisy stream in a log string.

```text
deadline = monotonic_at_entry + deadline_s
validate that deadline_s is finite and strictly positive before bus I/O
save timeout and optional write_timeout
try:
    reset input buffer once
    check deadline before transmitting
    if write_timeout exists:
        use min(original positive finite value, remaining), otherwise remaining
    check deadline again after the setting operation
    write packet once
    check deadline after write; reject a short/zero write without retransmission

    while budget remains:
        set read timeout to min(original finite value, remaining), or remaining
        check deadline again after the setting operation
        capture in_waiting, then check deadline after that property access
        read min(26, max(1, captured_in_waiting)) bytes
        check deadline before accepting/decoding the returned bytes
        append the bounded read
        repeatedly:
            find fc ee
            if absent:
                retain a trailing fc only, discard other noise
                break to read
            discard bytes preceding the header
            if fewer than 26 remain: break to read
            candidate = first 26 bytes
            call strict feedback(candidate, requested_id)
            if CRC or ID/mode validation rejects candidate:
                record a bounded rejection count/last reason
                discard only the first byte, then search again
                continue
            check deadline after decode
            stage candidate dictionary as the result
            stop reading
        if an immediate read returns empty:
            sleep at most min(1 ms, remaining), rather than busy-spin
    if no staged result: raise the bounded deadline failure
finally:
    restore every changed timeout, independently attempting both restorations
after successful restoration, check deadline before returning the staged result
```

Checking the clock also inside the candidate loop prevents a continuous noisy stream from postponing expiry. Discarding one byte on rejection preserves an overlapping header inside a damaged candidate. A foreign valid frame, invalid supported-mode field, or damaged frame can precede a good requested-ID frame in the same read. None updates `last_reply`, `latest`, samples, or RX counts. Exact decoder length rejection and stream framing differ: `feedback(valid_frame + extra_bytes)` rejects the oversized input, while `exchange` may extract a valid frame from a larger stream.

Only catch the decoder's specified validation `RuntimeError` cases as candidate rejection. Unexpected codec or transport failures abort the transaction. Preserve `Incomplete serial write`. Terminal no-valid-reply errors contain `deadline`, bounded received-tail hex, and a CRC rejection summary containing `CRC` only when such a candidate occurred. This preserves the existing `_transact` string-based counters; a failed transaction with both reasons increments both. A valid later reply succeeds and does not turn discarded candidates into a fault. Convert a pyserial write timeout to a `RuntimeError` that contains `deadline` and names the write phase. No subsequent write resumes that packet.

## Deadline and transport contract

The budget starts at function entry, before reset or timeout changes. The implementation checks it before and after reset, timeout setting, write, read, decoding, and restoration. If reset consumes the budget, send nothing. If a write exhausts it, read nothing. A result completed at or after expiry never returns as fresh feedback.

For the normal pyserial bus, clamp read timeouts to the remaining budget, preserving read timeout zero as nonblocking. Clamp a positive finite write timeout; temporarily replace an infinite or zero write timeout with the remaining positive budget. The inspected POSIX write loop can retry EAGAIN without an expiry check when its write timeout is zero, so preserving zero here would undermine B09 on a saturated device. Recompute remaining time after setters. Do not call `flush`, whose POSIX implementation uses `tcdrain`. Restore the original settings on success, validation failure, incomplete write, timeout, transport exception, and interruption. If restoration fails, do not return the staged result. Preserve a primary exception and attach the restoration failure; with no primary error, raise a restoration failure. Normalize timeout-setting and restoration failures to `RuntimeError`, so the worker faults rather than treating a transport `ValueError` as rejected operator input.

The inspected POSIX pyserial `read` uses a monotonic `Timeout` and `select` with time remaining. `write` has a finite-timeout path and can raise `SerialTimeoutException`. Input reset calls `tcflush`; timeout setters can reconfigure the port. Those calls and restoration have no independently interruptible deadline. Thus the return bound is the requested budget plus observed scheduler/syscall/cleanup overhead, under a responsive OS and a transport honoring its timeout settings. A blocked reset cannot be rescued by checking a clock afterwards. A hard wall-clock bound for arbitrary blocking drivers needs a process boundary or different I/O contract, beyond B01-B10.

The legacy test fake has `timeout = 0` and no `write_timeout`. Continue to accept it under its existing immediate-write contract. Detect absent write timeout without creating an attribute, and do not claim that this makes arbitrary write methods bounded. `DemoSerial` can expose `write_timeout = 0` directly. The reusable fake exposes both timeout properties. A provided transport that refuses a required timeout change fails before the write, and restoration still runs.

## Fake shape

```python
# fake_serial.py, proposed test-support declarations only.
from dataclasses import dataclass
from typing import Iterable

@dataclass(frozen=True)
class Chunk:
    data: bytes
    delay_s: float = 0

@dataclass(frozen=True)
class Transaction:
    chunks: tuple[Chunk, ...] = ()
    write_count: int | None = None   # None means full write; 0/partial are explicit.
    reset_delay_s: float = 0
    write_delay_s: float = 0
    reset_error: Exception | None = None
    write_error: Exception | None = None
    read_error: Exception | None = None

class FakeClock:
    def monotonic(self) -> float:
        raise NotImplementedError
    def sleep(self, seconds: float) -> None:
        raise NotImplementedError

class FakeSerial:
    timeout: float | None
    write_timeout: float | None
    is_open: bool
    writes: list[bytes]

    def __init__(self, transactions: Iterable[Transaction] = (), *,
                 clock: FakeClock | None = None,
                 timeout: float | None = 0,
                 write_timeout: float | None = 0):
        raise NotImplementedError
    @property
    def in_waiting(self) -> int:
        raise NotImplementedError
    def reset_input_buffer(self) -> None:
        raise NotImplementedError
    def write(self, packet: bytes) -> int:
        raise NotImplementedError
    def read(self, size: int) -> bytes:
        raise NotImplementedError
    def close(self) -> None:
        raise NotImplementedError
```

A reset clears the current active receive queue and peeks at the next script's reset settings without consuming that script. The next write records its exact immutable bytes and consumes exactly one script. Reads split chunks at `size` and preserve the remainder. No script means a complete write with no reply. For deterministic B09 tests, patch the serial module's private monotonic/sleep bindings to the same fake clock. Write delay honors finite `write_timeout`, and pending chunk delay honors finite `timeout`; reaching either bound advances the clock accordingly and returns empty or raises the specified write timeout. Reset delay intentionally advances without a timeout to prove that the exchange notices expiry before writing. Reset/write/read errors are explicit injection points. This fake does not generate expected packets through the production codec. Keep `DemoSerial` as the existing UI dynamics simulator, rather than turn this scripted fake into a motion model.

## Existing callers, faults, and shutdown

`MotorSession._transact` still increments TX before the request and only changes RX, `last_reply`, and samples after a successful exchange. Terminal candidate rejection follows its existing exception path into `_fault`, which clears tests and arms, attempts Stop, and preserves the fault latch. Scanning can continue on deadline failures as its existing code does. Decoder faults do not clear motor faults, timeout bits, or mode restrictions. The CLI's `healthy` checks and final Stop confirmation remain authoritative for their own lifecycle.

The exchange never closes the port while another caller might use it. Shutdown waits for the one in-flight exchange, then the existing owner performs its explicit Stop exchanges and closes its own bus. Each Stop exchange gets a separate budget, so the whole close sequence is longer than one `deadline_s`. GUI `_stop` can require two replies; CLI cleanup deliberately sends three Stop requests with sleeps. Those are existing safety actions, not automatic retries of a failed motion request. Parent-run tests must show that a missing reply during motion or final Stop still disarms, records an unconfirmed Stop where appropriate, and terminates the worker under the supported transport contract. Do not claim that successful worker shutdown proves physical torque removal. Honor `MotorSession.close()`'s false return in `Dashboard.closeEvent` rather than hide a surviving owner.

## Parent verification by milestone

| Milestone | Required evidence against the implemented artifact |
| --- | --- |
| B01 | Unchanged bench fixtures, including Stop `feee80000000000000000000000000001d99fb6e`, remain identical through old imports and new pure imports. Pure codec import needs no serial/Qt and performs no I/O. Independently check complete fixed command bytes and CRC using a separately written word-feed reference or archived implementation. |
| B02 | Fixed raw feedback `fcee801d1b48000000006cc4db000000000000000000f179457d` decodes ID 0, mode 0, timeout 1, driver 29 C, winding 27 C, 36 V, zero torque/rpm/errors/warnings, position 218.02713164107217 rad, and unchanged raw hex. Keep synthetic fixture and separately captured physical evidence identities distinct. Inspect capture provenance before claiming physical agreement. |
| B03 | Flip every byte in bytes 2-25 of a fixed valid frame by a chosen bit and assert rejection. Also cover independent CRC vectors and complete-word enforcement. Header corruption belongs to B04. Expected frames/CRCs must not come from `command` or `crc32` under test. |
| B04 | Every length 0-25, 27, and longer; wrong/broken headers; concatenated frames passed directly to decoder; unchanged dictionary keys, types, raw hex, unsigned faults/winding/voltage, and signed driver/effort/position. |
| B05 | Fake records the exact packet once; scripts consume once per write; reset discards old bytes only; multiple calls have separate replies; reads respect requested size; empty/zero/partial-write paths and close state are observable. |
| B06 | Each fixed valid response arrives byte by byte, at every split offset, and across a split `fc`/`ee` header. It decodes once without a second write. |
| B07 | Noise longer than the buffer cap, isolated trailing `fc`, bad CRC then good frame, embedded overlapping header, unsupported-mode then good frame, and multiple candidates in one read. Instrument maximum retained bytes without adding production API options. Terminal diagnostics stay bounded. |
| B08 | Foreign-ID before correct-ID succeeds; foreign-only expires; unsupported mode cannot return. Through `_transact`, neither case refreshes requested-ID state until a correct packet succeeds. A valid previous Stop mode during the existing FOC transition remains allowed. |
| B09 | Fake-clock budget covers reset, write, read, candidate search, restoration, `None` and zero timeouts, small deadlines, late full replies, missing bytes, write exceptions, and restoration exceptions. Invalid deadline fails before I/O. Confirm one write and restored settings on every path. Separately run pinned pyserial on a pseudo-terminal with no responder, delayed/fragmented responder, initially infinite/long read timeout, and bounded write timeout. Record actual elapsed time, requested budget, and explicit tolerance, including cleanup. An honest upper-bound assertion must use measured elapsed time, not just exception text. A saturated PTY can test write timeout with a test-only large payload; the production exchange still sends normal 20-byte packets in bench use. Run existing disarmed fault/Stop/close checks plus a missing-reply shutdown case and the rejected-close boolean branch. |
| B10 | Independently derive and freeze signed output-unit fixtures from the vendor scaling, with both directions and quantization: ±5 rpm encodes ±68, ±60 rpm encodes ±811, ±pi/2 position encodes ±103765, ±0.1 Nm feedforward encodes ±20, Kp 3 encodes 239, Kd 1 encodes 319, plus representable signed extrema. Decoder raw speed ±608 must produce ±45 rpm; raw position ±311296 must produce ±3*pi/2 rad. Use independent expected constants and half-LSB tolerances. Internal ratio remains 38/3; no external joint ratio/offset/direction is added. |

After each behavior the parent runs its focused check before the next behavior. The final full driver still runs packet, dashboard, startup verifier, and both disconnected startup modes, now with expanded copied-source/fingerprint coverage and the new checks. Inspect actual startup renders. Record source revision and evidence in the milestone ledger. This runner verified source and design consistency only; implementation acceptance, elapsed bounds, and physical results remain parent work.
