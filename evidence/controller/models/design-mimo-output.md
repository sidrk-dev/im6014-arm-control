## Scores

**1. Canonical joint units / conversion ownership — S:4, G:3**
- **S**: `JointMap` is pure/external-only; formulas match spec (`q_out=q0+s*N*q`, `tau_out=s*tau/N`, gains `1/N²`), `direction` is real ±1 excluding bools, reduction>0, finite checks, overflow→ValueError; codec keeps `38/3` and bounds Kd 0..6 / Kp 0..30 / ±3.5 Nm. Deduction: no explicit `dq_out=s*N*dq` rad/s stage — rad/s→rpm ownership (mapping vs codec) is implicit in `rpm_out=s*N*dq*60/(2π)`. Graft: name the rad/s intermediate.
- **G**: Mapping math exists, but the core's canonical `JointTarget` is `rpm/position/kd/kp` (output units) — joint units are not canonical in C02–C06. Usage calls `to_actuator(cmd)` while the shape defines `to_actuator_position/velocity/...` (internal contradiction); output velocity unit (rpm vs rad/s) ambiguous; core has no embedded mapping (no `JointSpec`), so the joint/output seam is undefined; no bool-direction/overflow rules stated.

**2. Acquisition, alias/crossprocess, lifetime, injection points — S:5, G:3**
- **S**: `flock` keyed on `st_rdev` in one namespace so direct+symlink aliases collide; never unlink lockfile; FD held for full bus lifetime; `fstat` recheck against replacement races; acquire before constructor; release on failed open/close/crash; **close-unconfirmed retains the lease** (kills dangling second-open path); GUI worker / CLI context lifetimes named; no acquisition at import, enumeration, inspection or dashboard startup; `serial_factory` looked up at call time preserves startup mocks; advisory "cooperating clients only" stated; `exclusive=True` advisory only.
- **G**: Lock-before-construction, immediate release on failed open, kernel release on crash, in-process registry, cooperating caveat — all present. Missing: no injectable factory (`lease.open_serial` constructs internally → startup mock patch points unaddressed), no close-unconfirmed/GUI-worker lifetime rule, `/tmp` lockfile unlink risk unaddressed, no "never unlink", no acquisition-at-startup statement, no typed alternate/fixture specs, key note (namespace) absent.

**3. Transitions, per-joint age/error, no auto recovery — S:5, G:3**
- **S**: Full table with catch-all "reject, no motor command or target replacement"; gating CONNECTED→CHECKING→READY→ARMED; fault recovery only via explicit `begin_check(acknowledge_fault=True)`; healthy feedback alone cannot clear FAULT; age stamped from injected clock **after validation** (never caller timestamp); wrong ID/corrupt/decode failure cannot refresh; unknown bits + timeout retained; affected build from active vector; freshness⊥health; configurable `feedback_max_age_s`; honest note that software Stop ≠ physical confirmation.
- **G**: Arming guards and fault/Stop/stale clearing are right, and wrong-ID non-refresh is stated — but **FAULTED has no exit row** (dead end or undefined recovery), `arm_until` 10s auto-expiry is a premature expiry mechanism (C07), caller supplies `receive_mono` (spoofable input), hardcoded 0.25s/0.1s thresholds, no catch-all reject row, `TIMEOUT_LATCHED` ack clears "if healthy" (health-conditioned auto-clear).

**4. Bounded immutable storage + watermark — S:5, G:3**
- **S**: Frozen dataclasses, owned tuples, exact vector match (no absent joints/zero padding), one `Target` slot; `submit` validates whole vector then updates pending+watermark together; invalid updates preserve both; `consume()` clears pending, retains watermark; watermark retained after Stop/fault/reconnect/disconnect; **one immutable `stream_id` per core** (bounded), bool excluded from sequence; thread-confined; caller-mutation tested; snapshots copy.
- **G**: Single O(1) slot and watermark retention after Stop are correct, and invalid seq preserves pending — but the "immutable snapshot" contains a mutable `Dict`, targets are dict-keyed (partial vectors / no fixed joint order), `_watermarks[stream_id]` is an unbounded multi-stream map with premature cross-stream semantics (C08), no `consume()`/post-consumption retention, `snapshot() -> dict` (untyped, mutable), caller-mutation untested.

**5. API coherence, bench bytes/aliases, single owner path — S:5, G:3**
- **S**: Typed `Decision` (acceptance + fixed rejection code) and one lifecycle vocabulary; wire packets excluded from the core interface; **both** bench open sites share one managed boundary; codec goldens/display strings unchanged; startup fingerprints + baseline copied-source lists updated; fixture provenance explicit; module map clean; grounded source evidence with file:line citations (codec/session/CLI/plan).
- **G**: `request_transition(action: str, **kwargs)` is stringly-typed and duplicates `arm/stop` (dual transition paths); core ingests `raw_bytes` (wire/codec knowledge inside the core) and returns `raw_hex`; `JointCommand`/`to_actuator` inconsistency; `snapshot()->dict`; no statement that the two existing open sites share one boundary; no startup-fingerprint/baseline integration steps; **no source line citations**.

**6. Staging / acceptance, no premature motion/IPC — S:5, G:4**
- **S**: Per-milestone parent acceptance table; core cannot open ports or call `exchange`; no production runner through C06; explicitly rejects the station daemon because C07–C16 (expiry, generations, limits, independence) are unfinished; names remaining station-IPC/refresh scope and warns against calling the reducer a physical station; no profile/schema framework, no rate reports; design-only statement; fake clock/transport + parent PTY checks, no physical ports.
- **G**: Strong acceptance list and rejects both bad alternatives, no IPC/profile introduced — but the 10s arm expiry is premature (C07), no explicit "core cannot open a port or dispatch" guarantee, no explicit remaining-IPC scope section (only open questions), no fingerprint integration, and the name "Station Controller Core" risks conflation with a completed physical station.

**Totals: S = 29/30, G = 19/30.**

## Base choice

**design-sol.md** as the whole architecture (typed joint core + managed port boundary). It is the only candidate that is coherent end-to-end on unit ownership, lease lifetime, transition completeness, and bounded immutable storage, and the only one with grounded source evidence and explicit no-motion/no-IPC staging.

## Grafts from G into S

1. Adopt G's transition-table column split (Event | Guard | Action) and its explicit offending-joint latch row as a table row, not just prose.
2. Adopt G's concrete acceptance vectors: codec IDs `>14`/negative, "zero imports of `im6014_codec.RATIO`", B13 non-FTDI/PTY-with-explicit-config pass, and C06 sequence `1,5,5,2,6 → accept 1,5,6; watermark 6 after stop()`.
3. Adopt G's distinction of timeout as a named fault reason code (keep it inside `FAULT`, do not add a `TIMEOUT_LATCHED` state).
4. Adopt G's explicit "in-process registry covers GUI + embedded CLI in one process" phrasing alongside the cross-process flock claim.

## Corrections to S (small, pre-merge)

1. Add an explicit `dq_out = s*N*dq_joint` rad/s stage and name who owns rad/s→rpm (mapping, as external algebra) so B11's formula is literal.
2. Enumerate `Decision` rejection codes (bad_type, nonfinite, overflow, out_of_bounds, wrong_id, stale, invalid_state, bad_sequence, duplicate/older, vector_mismatch).
3. State that `inspect_port`'s `observed_metadata` is caller/OS-sourced (stat/sysfs/udev only), never serial traffic, so inspection cannot regress into an open.

## If G were used as base (required corrections)

Joint-unit `JointTarget` + embedded per-joint mapping; typed `Decision` methods, kill `request_transition(str)`; add catch-all reject and an acknowledged FAULTED recovery row, delete `arm_until`; core stamps its own clock, fixed-order owned tuples not dicts, single bounded stream watermark, frozen `Snapshot`, add `consume()`; injectable `serial_factory` at both open sites, never-unlink + close-unconfirmed lease rules; typed alternate/fixture port specs; exact-boolean watchdog validation; `SourceIdentity` type with per-record byte caps (≤20/≤26) and no exception/credential strings; forbid core port open/`exchange`; add file:line evidence and fingerprint/baseline steps.
