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

## Contributor update and usability correction

The later contributor UI commit `7250547` added a serial dropdown, workflow tracker and gauges. Its reported 41 checks and physical auto-selection were contributor claims, not parent verification of that artifact. Parent reproduction found ownership bypass, a missing post-Enable neutral gate, an identity fallback and misleading connection/control guidance. That artifact was not accepted as a physical control release.

The correction removes `/api/claim`; an independently opened tab creates an observer. A successful explicit Connect establishes ownership. Browser Duplicate Tab or an opener can copy sessionStorage and reuse the same authenticated owner session; use one controlling tab and open observers independently. Direct keys, on-screen buttons and selected standard-controller axes request movement only after explicit Enable and a new neutral observation. Space and RB are no longer required. Stop, Esc, B, lost focus, expired input, stale feedback and physical command limits retain their authority.

Serial dropdown and Refresh enumerate bounded device metadata without opening or probing a port. The verified Channel D uses its stable by-id path when present. Missing Channel D is reported even if unrelated serial ports exist. Enumeration is not a healthy motor reply. Calibration requires measured Stop feedback on every connection; a stored software zero is not trusted after reconnect.

### Checkpoint before final PR review

The first repaired artifact passed 30 manual and 15 HTTP checks, two Node groups, six independent parent checks and two actual allocated pySerial PTY cases. A fake browser session completed Connect, timeout acknowledgment, zero declaration, Enable, automatic Figure-8 following on simulated J1, confirmed Stop and observer isolation. Momentary key injection did not establish sustained keyboard motion; physical Xbox motion was not demonstrated. Browser screenshot capture was unavailable.

Muse and Gemini reviewed that frozen artifact. Accepted presentation, controller selection, transient-session and bounded-port findings were corrected. The revised source passed 45 Python checks, three Node groups, six independent checks, two bounded port/static-module checks and two allocated PTY cases. The first revised presentation test had an incorrect zero-label expectation and caught a real uncleared hidden-gauge fill; both corrections and the failed log are retained. A parent PTY invocation passed its two cases but omitted its receipt argument; the corrected invocation passed and wrote the receipt. An actual visible browser session followed the Figure-8 on simulated J1 to −7.73°, then confirmed mode-0 Stop. Hidden-tab ticket expiry and browser input timeouts are retained; no protection deadlines were relaxed. A stale post-Stop preview caption was reproduced and corrected. MiMo's HTTP 502 is a retained failed seat, with no retry or substitute for that artifact. Final-artifact checks and independent verification must supersede this provisional record before shipping.

At the latest passive hardware observation, the USB Channel D adapter was absent. The old server had stale feedback and repeated serial I/O errors; cached voltage and angles were not fresh communication proof. It was stopped. No motor movement or fresh physical connection was performed during this usability correction. Replug the USB adapter and use Refresh before attempting Connect; J2/J3 remain virtual.

### Final software release

Private PR #5 merged as `8b6a66d` after exact-head green checks and a fresh independent nonwriter Sol PASS+NOTES verdict. The merged tree matches the reviewed candidate. Muse and Gemini completed the final review; their findings were triaged. MiMo returned HTTP 502 with no review output and was neither retried nor substituted. Two additional independent fake production-HTTP cases passed. Merged-main CI passed.

The local dashboard was launched disconnected; passive startup still found no Channel D adapter. This release does not claim a new physical connection or physical Xbox/held-key result. Three nonblocking display/documentation notes are addressed in this follow-up: the zero-marker color, separate velocity-recipe scope and browser-duplicate owner-session wording.
