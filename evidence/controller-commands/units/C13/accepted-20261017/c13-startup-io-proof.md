# Disconnected offline startup proof

The initial host baseline request was rejected before launch because its `--source real` test name could imply physical probing. No rejected command was executed through another path.

Source inspection of the exact release copy establishes that `test_startup.py` uses `real` only to select the disconnected UI source selector. Lines 229-246 patch `dashboard.enumerate_ports` to an empty tuple, replace `motor_session.serial.Serial` with `ForbiddenMotorIO.serial`, and replace `DemoSerial` and `exchange` with equivalent rejecting sentinels before calling `dashboard.main()`.

`ForbiddenMotorIO` (lines 63-78) raises on any open or exchange attempt. The report validator (lines 29-51) requires both pre/post snapshots disconnected and disarmed, no bus, zero packet counters, and zero serial-open, demo-open or exchange attempts. Close completes inside the same patches. The imported dashboard/session modules define classes and functions; their actual entrypoint is guarded by `__main__`. The constructor's only enumeration call is the patched symbol. These checks start an offscreen isolated Qt test application, not a persistent motor service.

This establishes a strictly offline startup test despite its UI mode label. No device probe, serial open, status/Stop command, fault clear, motion or motor-service change is authorized. A fresh approval review may inspect this source before allowing the exact baseline. If it remains rejected, this test stays blocked; no alternate execution path is permitted.
