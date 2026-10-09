# Manual arm station API

The three-joint station backend is verified with a synthetic packet fixture. Keyboard/Xbox controls, the final write guard, and physical commissioning are separate pending units. This module does not provide an assembled-arm commissioning path.

## Ownership

`ArmStation` has one executor that owns Core, transport exchanges, Stop and close. Use either `start_loop()` for background execution or `step()` from one bound thread for deterministic fixture work. Other threads submit typed requests and read immutable published views. They do not access Core.

Register a session with `register_session()`. The accepted Connect reserves station ownership. Other registered sessions can request Stop. A confirmed Disconnect releases ownership for a new client. Failed initial Connect releases its reservation. Unconfirmed Stop or close retains ownership; a new client cannot silently take it.

The fixture lifecycle is Connect, Check, then Arm. Check requires fresh mode 0 feedback for all three motors. An explicit acknowledged fault recovery preserves Core's fault and protection history. Arm admits a measured stationary goal. It starts energized hold and requires an observed neutral release followed by a new deadman edge before jogging.

## Input and observations

`issue_ticket(session, sequence)` returns the sole canonical input ticket. `JogIntent` carries three normalized axes, deadman/focus state, a device epoch, sequence and that ticket. Server issuance time establishes expiry. Caller edits to expiry, replay, delayed receipt, a changed epoch or lost focus cannot extend authority.

Core owns bounded goals, exact per-joint reference histories, command packing and write outcomes. Each joint's elapsed reference time is used independently. Ordinary deadman release captures the complete confirmed position once and holds it while the input lease remains fresh. Stop revokes ingress and attempts mode 0 independently for every joint.

`snapshot()` reads the published immutable view without reading Core on the caller thread. Requested axes, admitted goal, dispatched and confirmed references, measured feedback, logical Stop, per-joint Stop confirmation, and transport closure are separate observations. Internal session metadata must be redacted by the future HTTP serializer.

Requests return an immutable receipt. `receipt(request_id)` returns a `ReceiptLookup` with `present`, `expired`, or `unknown` status. General receipts, active lifecycle work and terminal lifecycle receipts have separate fixed capacities. Clients must handle expiry rather than wait indefinitely for old results.

## Failure and shutdown

`close()` queues executor-owned Stop and transport close. `stop_loop()` also routes through that shutdown path. Failed close retains the executor for an explicit retry. A caller does not inherit Core execution when the background worker stops.

An unexpected executor failure revokes authority, attempts bounded mode 0 transactions for all motors and attempts transport close. Broken Core time cannot suppress those packet attempts or establish fresh confirmation. The snapshot exposes actual per-motor attempts and confirmed closure separately.

These are software observations. A reply proves only the protocol evidence received during that attempt. It does not establish physical clearance, torque release, stopping distance or a safety-rated emergency stop.

## Fixture and commissioning boundaries

`ArmPacketFixture` emulates IDs 0, 1 and 2 with the canonical codec. J1 uses the generated model's intended ±90 degree bounds around a provisional zero. J2/J3 limits and gains are explicitly synthetic. The fixture can inject missing/wrong-mode feedback, partial writes and failed close.

`PhysicalDraft` validation and `compute_zero_output()` are pure helpers. Physical Connect returns a commissioning refusal and acquires no motor port. The next unit must require explicit direction, measured travel, reference, gains, protection and timeout facts before binding physical Core. No saved pose is restored automatically.

## Check

From the repository root, run the registered offline unit:

```bash
.venv/bin/python -B scripts/check_milestone.py ARM-A02 --python .venv/bin/python
```

The full offline baseline also runs the station tests in a byte-identical scratch copy. Current evidence includes 28 focused station tests, 293 controller/timing regressions, and 17 independent parent checks. The configured Muse and Gemini final reviews found no actionable regressions. MiMo's HTTP 502 failure is retained as a reviewer dropout. No assembled-arm hardware or physical Xbox connection was tested.
