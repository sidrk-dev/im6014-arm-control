# J1 release validation

The browser controls one physical IM6014 (ID 0, Channel D, 6M baud). J2/J3 remain virtual. Startup is disconnected. Finite parameter drafts do not change the applied motor limits.

## Checks

- 40 focused Python checks passed on host loopback with fake motor I/O.
- Both Node input and preset test files passed.
- Two allocated real pySerial PTY checks passed; no motor was attached to those PTYs.
- Nine existing offline baseline groups passed, including 18 legacy dashboard groups.
- An independent nonwriter checked 500 CAD poses and 401 Figure-8 targets against the actual joint transforms. This is not a collision or physical position certificate.

## Physical results

All runs used the single secured unloaded J1 with a reachable physical E-stop. The earlier Kd 0.1 run tripped the 1.5 rpm measured-speed protection and ended in confirmed Stop and disconnect. Its failure is retained.

With Kp 15, Kd 3, a 0.5 rpm command limit and 1 Nm effort ceiling, a run reached 30.06°, then a continuation preserving that software reference reached 88.53°. Both ended without errors, with actual mode-0 Stop confirmation and successful disconnect. Command speed is not a guaranteed physical transient maximum.

The visible browser flow was exercised through Connect, timeout acknowledgment in Stop, software zero, Enable, reload recovery, Check, Stop and Disconnect. Held-key physical motion and a physical Xbox controller were not established by browser automation. Physical movement was exercised through the same production dashboard HTTP ticket/input route. Pure reducer and request tests cover the input mappings.

## Review and retained evidence

Muse, MiMo and Gemini reviewed the selected source; the parent triaged their findings and corrected accepted defects. A prior MiMo HTTP 502 remains a failed seat. Independent Sol verification and original failed/successful logs are retained outside this source repository at the workstation run's J1-simple-final-evidence directory. Do not publish raw authenticated session captures.

This release does not establish assembled-arm clearance, gravity support, motor ID setup for J2/J3, power-cycle absolute reference behavior, or a firmware PID tune. Those require separate commissioning.
