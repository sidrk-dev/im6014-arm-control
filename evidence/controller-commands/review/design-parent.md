# Separate command gate and reference generator

## Problem and usage

An alternative keeps the existing Target as a payload. A frozen CommandEnvelope contains that payload, a controller-issued owner token, and an absolute monotonic deadline. A CommandGate accepts envelopes and emits valid payloads into JointCore. A ReferenceGenerator accepts the payload and measured pose, then returns a bounded reference at each dispatch time.

The station would claim an owner through the gate, arm the core explicitly, submit an envelope, poll the gate, and pass its output to the reference generator. It then maps the generated reference to motor units. Stop, fault, and disconnect must revoke the gate token and clear both the core slot and reference generator state.

## Shape

OwnerToken identifies the controller instance, owner, and generation. CommandEnvelope stores the token, expires_at, and Target. JointLimits stores the joint position interval and velocity cap. ReferenceGenerator owns one ordered reference vector and its last dispatch time. CommandGate owns the active token and envelope.

The generator seeds its reference from measured joint positions when the core arms. It rejects a seed outside the configured interval. Each dispatch limits the reference displacement by velocity times elapsed dispatch time. It does not bank elapsed time before a new target arrived. Desired velocity is bounded too. Repeated polls continue towards the newest target only until its deadline.

## Tradeoffs and screening

The shapes are individually small and transport-free. However, the station must coordinate three owners of mutable state on every lifecycle event. A caller could bypass CommandGate by calling the old JointCore.submit_target directly. Clearing one slot would leave another active trajectory. This fails the split-ownership and two-ways-to-do-one-task screening criteria.

I would choose the unified core instead if it can expose one required command contract and one bounded-reference consumer. The separate-layer shape remains an independent candidate for comparison. Its useful contribution is the distinction between the accepted goal and dispatched reference, including the target-arrival timestamp.

## Checks

Use fake monotonic time to test exact deadline equality, queue expiry, previous-owner delayed delivery, reconnect, failed takeover, atomic vector rejection, positive and negative position bounds, repeated dispatch steps, long idle before target arrival, zero elapsed time, and clock regression after valid feedback. Packet assertions inspect actual bounded references rather than accepting a reported limiter flag.
