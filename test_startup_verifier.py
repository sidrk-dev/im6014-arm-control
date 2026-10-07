import copy
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from test_startup import ForbiddenMotorIO, _cleanup_after_main, validate_report


def passing_report():
    before = {
        "bus_is_none": True, "state": "Disconnected", "connected": False, "armed": False,
        "tx": 0, "rx": 0, "source_selected_demo": False, "window_visible": True,
        "timer_active": True, "worker_alive": True, "event_loop_tick_seen": True,
    }
    after = dict(before, window_visible=False, timer_active=False, worker_alive=False,
                 event_loop_tick_seen=False)
    return {
        "source": "real", "system_exit": True, "exit_code": 0, "main_exception": None,
        "argv_restored": True, "window_captured": True, "callback_ran": True,
        "callback_completed": True, "callback_errors": [], "pre": before, "post": after,
        "close_returned": True,
        "sentinel_counts": {"serial_open": 0, "demo_open": 0, "exchange": 0},
        "screenshot": {"saved": True, "width": 1, "height": 1, "bytes": 1, "sha256": "0" * 64},
    }


class StartupVerifierTests(unittest.TestCase):
    def test_faults_fail_actual_startup_process(self):
        script = '''
import sys
from pathlib import Path
from unittest.mock import patch
import motor_dashboard
import motor_session
from test_startup import run

fault, evidence = sys.argv[1:]
original = motor_dashboard.Dashboard
original_main = motor_dashboard.main

class FaultyDashboard(original):
    def __init__(self, demo=False):
        if fault in ("serial_open", "demo_open", "exchange"):
            attempt = {"serial_open": motor_session.serial.Serial,
                       "demo_open": motor_session.DemoSerial,
                       "exchange": motor_session.exchange}[fault]
            try:
                attempt()
            except RuntimeError:
                pass
        super().__init__(demo)

    def grab(self):
        if fault == "screenshot":
            raise RuntimeError("injected screenshot failure")
        return super().grab()

    def close(self):
        if fault == "close":
            return True
        return super().close()

def faulty_main():
    if fault == "missing_window":
        raise SystemExit(0)
    try:
        original_main()
    except SystemExit:
        if fault == "exit_code":
            raise SystemExit(2)
        raise

with (patch.object(motor_dashboard, "Dashboard", FaultyDashboard),
      patch.object(motor_dashboard, "main", faulty_main)):
    raise SystemExit(run("real", Path(evidence)))
'''
        cases = {
            "serial_open": "forbidden serial_open attempt count was 1",
            "demo_open": "forbidden demo_open attempt count was 1",
            "exchange": "forbidden exchange attempt count was 1",
            "screenshot": "injected screenshot failure",
            "close": "dashboard window remained visible after close",
            "missing_window": "Dashboard constructor did not produce a window",
            "exit_code": "application exit code was 2",
        }
        env = dict(os.environ, QT_QPA_PLATFORM="offscreen", PYTHONDONTWRITEBYTECODE="1")
        with tempfile.TemporaryDirectory(prefix="startup-faults-") as temp:
            for fault, diagnostic in cases.items():
                with self.subTest(fault=fault):
                    evidence = Path(temp) / fault
                    result = subprocess.run(
                        [sys.executable, "-c", script, fault, str(evidence)],
                        cwd=Path(__file__).resolve().parent, env=env,
                        capture_output=True, text=True, timeout=30,
                    )
                    self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                    self.assertIn(diagnostic, result.stderr)
                    self.assertFalse((evidence / "startup-result.json").exists())
                    print(f"PASS: injected {fault} rejected; exit {result.returncode}; no startup receipt",
                          flush=True)

    def test_valid_report_passes(self):
        validate_report(passing_report())

    def test_caught_forbidden_io_attempts_still_fail(self):
        for method, count_key in (("serial", "serial_open"), ("demo", "demo_open"),
                                  ("exchange", "exchange")):
            with self.subTest(kind=count_key):
                sentinels = ForbiddenMotorIO()
                try:
                    getattr(sentinels, method)()
                except RuntimeError:
                    pass
                report = passing_report()
                report["sentinel_counts"] = sentinels.counts
                with self.assertRaisesRegex(AssertionError, count_key):
                    validate_report(report)

    def test_invalid_lifecycle_evidence_never_passes(self):
        failures = (
            ("missing constructor/window", lambda r: r.update(window_captured=False)),
            ("nonzero application exit", lambda r: r.update(exit_code=2)),
            ("main returned without SystemExit", lambda r: r.update(system_exit=False)),
            ("callback failure", lambda r: r.update(callback_errors=["injected failure"])),
            ("worker still alive", lambda r: r["post"].update(worker_alive=True)),
            ("timer still active", lambda r: r["post"].update(timer_active=True)),
        )
        for label, damage in failures:
            with self.subTest(reason=label):
                report = copy.deepcopy(passing_report())
                damage(report)
                with self.assertRaises(AssertionError):
                    validate_report(report)

    def test_cleanup_cannot_hide_worker_alive_after_normal_close(self):
        class Thread:
            alive = True

            def is_alive(self):
                return self.alive

        class Session:
            def __init__(self):
                self.thread = Thread()

            def close(self):
                self.thread.alive = False

        class Window:
            def __init__(self):
                self.session = Session()

        report = passing_report()
        report["post"]["worker_alive"] = True
        state = {"callback_completed": True, "callback_errors": [], "post": report["post"],
                 "close_returned": True}
        window = Window()
        _cleanup_after_main(window, state)
        self.assertFalse(window.session.thread.is_alive())
        self.assertTrue(state["post"]["worker_alive"])
        with self.assertRaisesRegex(AssertionError, "worker remained alive"):
            validate_report(report)


if __name__ == "__main__":
    unittest.main(verbosity=2)
