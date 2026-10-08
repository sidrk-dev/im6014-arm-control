Review of B11-B14 + C01 only. No tools used. C02-C06 not demanded.

BLOCKING DEFECTS

1. im6014_port.py:inspect_port + open_managed_port -- unknown identity opens any char device on default route
   What: inspect_port returns "unknown/incomplete ... identity is not verified" instead of raising when observed is None/partial. open_managed_port calls inspect_port(config) then unconditionally proceeds to _character_device + _private_lock_fd + factory.
   Repro: PortConfig("/dev/pts/N",6000000) with defaults expected_serial=FTC2RQ7H/expected_channel=3. With ports=() or live enumeration missing PTY, observed=None, merge with by-id hint None -> returns unknown string. open_managed_port then locks rdev and calls serial_factory, sets bus.port, bus.open, fstat check passes because fd rdev==path rdev. Any PTS/ttyUSB opens without FTDI assertion.
   Violates docs/joint-api.md "bench GUI and CLI default to recorded FTDI serial and interface 3" and docs/controller-design.md "Known wrong ... reject". Unknown becomes bypass.
   Tests miss it: test_port_config.py:test_unknown_default_path_stays_unknown_and_does_not_open only checks inspect_port does not call constructor, never checks open_managed_port rejects. test_bus_owner.py:C01BusOwnership always uses expected_serial=None/expected_channel=None for PTY opens, so never exercises default+unknown.
   Smallest fix: make open distinguish verified vs unverified. E.g. have inspect_port return (report, verified_bool) or check string, and in open_managed_port raise PortOwnershipError if config.expected_serial is not None and not verified. Keep alternate None/None path allowed to be unverified. Preserve injectable-factory failure tests by still allowing unverified only when expected is None.

2. im6014_port.py:ManagedPort.close -- getattr default True leaks lease for minimal mocks
   What: 
     try: if getattr(self.bus,'is_open',True): return False
   If bus has close()+fileno()+open() but no is_open attribute, close() always returns False even after bus.close() succeeded. Lock fd never closed. Same-process retry and cross-process open always PortBusyError until process exit.
   Repro: factory returns object with port/ba dtr/rts/open/fileno/close, no is_open, open_path=PTY. open_managed_port succeeds (verified True via fstat). owner.close() -> bus.close() succeeds -> getattr->True -> return False. Second open_managed_port same PTY -> PortBusyError. owner.close() again -> same False.
   Docs say "Constructor mocks remain injectable". No test covers minimal mock; test_bus_owner.py:FakeSerial always defines is_open.
   Smallest fix: default to closed when attribute missing: getattr(self.bus,'is_open',False) or explicit hasattr check only when present. E.g. if hasattr(bus,'is_open') and bus.is_open: return False.

NOTES / WEAK TESTS (non-blocking for this batch)

- joint_mapping.py:JointMap.to_output/from_output intermediate overflow false-positive. _multiply(sample.rpm, tau) then _divide(...,60) overflows for representable final. Example JointMap(1e308,1,0).from_output(OutputSample(0,1e308,0)) mathematically dq~0.1047 finite but _multiply 1e308*tau -> inf -> ValueError. Same for to_output output_rad_s*60/tau. Safe direction (fail-closed) and far outside joint rad/s, but algebra proof incomplete. Fix: divide first (rpm/60*tau). Test only uses small values, does not catch.

- B11 weak anchor: test_joint_mapping.py:B11JointMapping.test_mapping_does_not_clip_speed_and_codec_still_rejects_it only proves speed clipping absence. docs/joint-api.md example produces tau -4 Nm exceeding +-3.5 Nm policy, but no test asserts mapping preserves torque and codec rejects torque. Add torque analog.

- B11 round-trip single vector: test_round_trip_for_ratios_directions_and_offsets uses one JointTarget(-0.42,1.25,-2.5,8,3) and one OutputSample. No zero-gain, zero-velocity, or zero-torque round-trip. Power invariant partially covers but not exactness.

- B13 missing wrong-adapter case: no test for manufacturer/product non-FTDI with matching serial/channel, VID mismatch, or open_managed_port with default expected + unknown PTY (see blocking 1). _metadata adapter logic untested.

- B14 missing provenance edges: test_diagnostic_capture.py never tests source=physical with port='' rejection, capacity 0/257 rejection, clock returning nan/inf or ended<started inversion. capture_exchange uses capture._now() twice; second _now ValueError in finally would mask original exchange error and lose record. No test.

- C01 missing lease cases: no test for lockfile mode 0644 rejection, /tmp mode/uid change, same-rdev reuse after PTY recycle, or minimal mock without is_open (see blocking 2). Tests use production namespace /tmp/im6014-port-owner-UID with real PTYs; parallel runs share namespace but flock makes them serialise to Busy, not isolated.

- im6014_serial.py:_bounded_timeout precedence `if original is None or writing and original==0` relies on and-over-or. Correct but fragile; add parens. Behavior change of write_timeout 0 -> remaining deserves comment.

API REGRESSIONS

- No breaking regression found in this batch: spin_5rpm.py:run retains run(bus,args,log) uncaptured path, capture opt-in only; test_cli_run_log_capture_is_opt_in verifies. motor_session.py:snapshot adds packet_capture key with owned copy; json-serialisable, readers ignoring extra keys unaffected. im6014_port.py:open_managed_port keeps serial_factory injection; both bench open sites pass serial.Serial factory per design. ManagedPort context PortCloseError is new API, not overriding old behavior.
