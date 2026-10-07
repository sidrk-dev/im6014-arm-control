# IM6014 communication boundary design

## Problem and usage

B01-B10 need a testable codec and reliable single-request transaction. The dashboard and CLI keep their existing public functions, dictionary fields, worker ownership and health gates.

```python
from im6014_codec import command, feedback
from im6014_serial import exchange

packet = command(0)
state = exchange(bus, packet, 0, deadline_s=0.05)
```

`spin_5rpm` re-exports these exact functions for bench compatibility. `motor_session.exchange` remains the actual lookup used by the worker, preserving startup interception. No new controller owner is introduced.

## Shape

`im6014_codec.py` owns existing CRC, command encoding, strict feedback decoding and the internal38/3 conversion. All moved bodies retain their behavior. Existing bench encoding limits are retained compatibility policy, not actuator ratings. `spin_5rpm.py` retains port defaults, health policy and CLI lifecycle.

`im6014_serial.py` owns one input reset, one write, a bounded transaction-local candidate buffer and serial timeout restoration. A strict26byte decoder rejects complete malformed frames. The stream scanner advances one byte after a rejected candidate and preserves a final header prefix. It accepts only the requested ID and supported mode before expiry, including legitimate previous-mode feedback during existing FOC transitions. It never opens, closes, clears faults, acknowledges a motor timeout, retries a motion write or stores targets.

`fake_serial.py` is test support. It records immutable writes and releases per-transaction scripted chunks. A deterministic clock models delayed chunks and transport timeouts. The dashboard dynamics demo remains separate.

Read and write timeouts use the remaining monotonic budget and are restored on all outcomes. A zero or infinite write timeout is temporarily replaced with a positive finite budget because pinned POSIX pyserial can loop on EAGAIN when configured with zero. A synchronous reset, timeout setter, or driver that ignores its contract cannot be forcibly interrupted. Deadline claims cover supported timeout behavior plus explicitly measured OS scheduling and cleanup overhead.

## Synthesis decision

Sol's extracted-function candidate is the base. Gemini's transceiver-object candidate supplied a distinct comparison. Muse recommended the function candidate. The parent read both complete designs and checked the serial source. No structural graft is needed because both recommend one-byte resynchronization and deadline-scoped I/O.

The object candidate adds a per-exchange object and duplicate codec checks without improving bus ownership. Its optional expected-ID decoder, unbounded diagnostic accumulation, reset-cleared fake future replies and timeout pseudocode defects are rejected. Muse's suggestion to hide CRC noise during missing-ID scans is also rejected because actual CRC rejection must remain visible in existing failed-transaction counters. No successful recovered transaction is counted as a terminal CRC fault.

Sol's unrelated GUI closeEvent change is deferred. This batch changes the communication boundary and preserves the already-verified dashboard lifecycle. Existing fault/close tests remain acceptance gates. The proposed larger type catalog is reduced to the state actually needed by the fake.

## Verification

Each B unit gets a parent-run check and exact source fingerprint before advancing. Expected decoder values and CRCs use fixed captured or explicitly generated data and independently calculated vendor references. B09 also exercises actual pinned pyserial on isolated pseudo-terminals. All new runtime/test modules join copied-source checks and receipt fingerprints. Final product startup screenshots are inspected.

## Tradeoffs and next step

The existing feedback dictionary and bench command constraints stay compatible. External transmission mapping remains B11. Sequence numbers and a hard real-time process boundary remain outside B01-B10. First extract the unchanged codec and verify the seven saved complete commands and inherited tests.
