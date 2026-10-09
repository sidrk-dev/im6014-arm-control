import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import traceback


def _need(condition, message):
    if not condition:
        raise AssertionError(message)


def validate_report(report):
    _need(report.get("system_exit") is True, "application main did not exit through SystemExit")
    _need(not report.get("callback_errors"), "event-loop callback or close raised an exception")
    _need(report.get("exit_code") == 0, f"application exit code was {report.get('exit_code')!r}")
    _need(not report.get("main_exception"), "application main raised an exception")
    _need(report.get("argv_restored") is True, "sys.argv was not restored")
    _need(report.get("window_captured") is True, "Dashboard constructor did not produce a window")
    _need(report.get("callback_ran") is True, "event-loop verification callback did not run")
    _need(report.get("callback_completed") is True, "event-loop verification callback did not complete")

    source = report.get("source")
    _need(source in ("real", "demo"), f"invalid source mode: {source!r}")
    pre, post = report.get("pre") or {}, report.get("post") or {}
    expected_demo = source == "demo"
    for name, observation in (("pre-close", pre), ("post-close", post)):
        _need(observation.get("bus_is_none") is True, f"{name}: session bus was not None")
        _need(observation.get("state") == "Disconnected", f"{name}: dashboard was not Disconnected")
        _need(observation.get("connected") is False, f"{name}: session reported connected")
        _need(observation.get("armed") is False, f"{name}: session was armed")
        _need(observation.get("tx") == 0 and observation.get("rx") == 0,
              f"{name}: packet counters were not zero")
        _need(observation.get("source_selected_demo") is expected_demo,
              f"{name}: selected source did not match {source}")
    _need(pre.get("window_visible") is True, "dashboard window was not visible")
    _need(pre.get("timer_active") is True, "dashboard UI timer was not active")
    _need(pre.get("worker_alive") is True, "serial-owner worker was not alive before close")
    _need(pre.get("event_loop_tick_seen") is True, "normal UI timer did not update in the event loop")
    _need(post.get("window_visible") is False, "dashboard window remained visible after close")
    _need(post.get("timer_active") is False, "dashboard UI timer remained active after close")
    _need(post.get("worker_alive") is False, "serial-owner worker remained alive after close")
    _need(report.get("close_returned") is True, "normal window close did not succeed")

    counts = report.get("sentinel_counts") or {}
    for name in ("serial_open", "demo_open", "exchange"):
        _need(counts.get(name) == 0, f"forbidden {name} attempt count was {counts.get(name)!r}")

    screenshot = report.get("screenshot") or {}
    _need(screenshot.get("saved") is True, "rendered dashboard screenshot was not saved")
    _need(screenshot.get("width", 0) > 0 and screenshot.get("height", 0) > 0,
          "rendered dashboard screenshot has invalid dimensions")
    _need(screenshot.get("bytes", 0) > 0, "rendered dashboard screenshot is empty")
    digest = screenshot.get("sha256", "")
    _need(len(digest) == 64 and all(c in "0123456789abcdef" for c in digest),
          "rendered dashboard screenshot has an invalid SHA-256")


class ForbiddenMotorIO:
    def __init__(self):
        self.counts = {"serial_open": 0, "demo_open": 0, "exchange": 0}

    def reject(self, kind, *args, **kwargs):
        self.counts[kind] += 1
        raise RuntimeError(f"offline startup verifier blocked {kind}")

    def serial(self, *args, **kwargs):
        return self.reject("serial_open", *args, **kwargs)

    def demo(self, *args, **kwargs):
        return self.reject("demo_open", *args, **kwargs)

    def exchange(self, *args, **kwargs):
        return self.reject("exchange", *args, **kwargs)


def _observe(window, event_loop_tick_seen=False):
    snap = window.session.snapshot()
    return {
        "bus_is_none": window.session.bus is None,
        "state": snap["state"],
        "connected": snap["connected"],
        "armed": snap["armed"],
        "tx": snap["tx"],
        "rx": snap["rx"],
        "source_selected_demo": window.source.currentIndex() == 1,
        "window_visible": window.isVisible(),
        "timer_active": window.timer.isActive(),
        "worker_alive": window.session.thread.is_alive(),
        "event_loop_tick_seen": bool(event_loop_tick_seen),
    }


def _cleanup_after_main(window, state):
    if window is None or not hasattr(window, "session") or not window.session.thread.is_alive():
        return
    try:
        if not state["callback_completed"] and hasattr(window, "timer"):
            state["close_returned"] = bool(window.close())
            state["post"] = _observe(window, False)
        else:
            window.session.close()
    except BaseException:
        state["callback_errors"].append("post-main cleanup failed\n" + traceback.format_exc())


def _fingerprint(root):
    names = ("motor_dashboard.py", "motor_session.py", "spin_5rpm.py", "im6014_codec.py", "im6014_serial.py", "fake_serial.py", "USER_MANUAL.md",
             "test_spin_5rpm.py", "test_dashboard.py", "test_startup.py", "test_startup_verifier.py", "test_protocol.py", "test_serial_deadline.py", "evidence/boundary/vendor-crc-vectors.json", "joint_mapping.py", "test_joint_mapping.py", "test_numerical_inputs.py", "im6014_port.py", "test_port_config.py", "test_diagnostic_capture.py", "test_bus_owner.py", "joint_contract.py", "joint_reference.py", "joint_controller.py", "test_controller.py", "test_controller_targets.py", "test_controller_expiry.py", "test_controller_owner.py", "test_controller_limits.py", "test_controller_lead.py", "test_controller_effort.py", "joint_protection.py", "test_controller_thermal.py", "station_timing.py", "station_step.py", "test_station_timing.py", "scripts/benchmark_station_timing.py")
    files = {}
    for name in names:
        path = root / name
        data = path.read_bytes()
        files[name] = {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    return files


def run(source, evidence_dir):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    evidence_dir = Path(evidence_dir).expanduser().resolve()
    evidence_dir.mkdir(parents=True, exist_ok=False)
    screenshot_path = evidence_dir / "dashboard.png"
    result_path = evidence_dir / "startup-result.json"
    root = Path(__file__).resolve().parent
    original_argv = sys.argv
    original_argv_values = list(original_argv)
    main_argv = [str(root / "motor_dashboard.py")]
    if source == "demo":
        main_argv.append("--demo")

    io = ForbiddenMotorIO()
    state = {
        "app": None,
        "window": None,
        "initial_ping": None,
        "callback_ran": False,
        "callback_completed": False,
        "callback_errors": [],
        "pre": None,
        "post": None,
        "close_returned": False,
        "screenshot": None,
    }

    try:
        from PySide6.QtCore import QTimer
        from PySide6.QtWidgets import QApplication
        import motor_dashboard as dashboard
        import motor_session

        original_dashboard = dashboard.Dashboard
        original_application = dashboard.QApplication

        def add_error(label):
            state["callback_errors"].append(label + "\n" + traceback.format_exc())

        def close_after_inspection():
            if state["callback_ran"]:
                return
            state["callback_ran"] = True
            window = state["window"]
            app = state["app"]
            try:
                if window is None:
                    raise AssertionError("Dashboard constructor did not produce a window")
                app.processEvents()
                event_tick = (state["initial_ping"] is not None and
                              window.session.last_ui > state["initial_ping"])
                state["pre"] = _observe(window, event_tick)
                pixmap = window.grab()
                _need(not pixmap.isNull(), "dashboard grab returned a null pixmap")
                _need(pixmap.width() > 0 and pixmap.height() > 0, "dashboard grab has invalid dimensions")
                _need(pixmap.save(str(screenshot_path), "PNG"), "could not save dashboard PNG")
                image = screenshot_path.read_bytes()
                state["screenshot"] = {
                    "path": str(screenshot_path),
                    "saved": screenshot_path.is_file(),
                    "width": pixmap.width(),
                    "height": pixmap.height(),
                    "bytes": len(image),
                    "sha256": hashlib.sha256(image).hexdigest(),
                }
            except BaseException:
                add_error("startup observation failed")
            finally:
                try:
                    app.setQuitOnLastWindowClosed(False)
                    if window is not None:
                        state["close_returned"] = bool(window.close())
                        state["post"] = _observe(window, False)
                except BaseException:
                    add_error("normal window close failed")
                state["callback_completed"] = True
                app.exit(1 if state["callback_errors"] else 0)

        def watchdog():
            if state["callback_completed"]:
                return
            state["callback_errors"].append("startup event-loop watchdog expired")
            app = state["app"]
            window = state["window"]
            try:
                if window is not None:
                    app.setQuitOnLastWindowClosed(False)
                    state["close_returned"] = bool(window.close())
                    state["post"] = _observe(window, False)
            except BaseException:
                add_error("watchdog cleanup failed")
            app.exit(1)

        class CapturingDashboard(original_dashboard):
            def __init__(self, demo=False):
                state["window"] = self
                super().__init__(demo)
                state["initial_ping"] = self.session.last_ui
                QTimer.singleShot(350, close_after_inspection)

        def capture_application(*args, **kwargs):
            app = original_application(*args, **kwargs)
            state["app"] = app
            app.setQuitOnLastWindowClosed(False)
            QTimer.singleShot(15000, watchdog)
            return app

        from unittest.mock import patch
        with patch.object(dashboard, "Dashboard", CapturingDashboard), \
             patch.object(dashboard, "QApplication", capture_application), \
             patch.object(dashboard.list_ports, "comports", return_value=[]), \
             patch.object(motor_session.serial, "Serial", io.serial), \
             patch.object(motor_session, "DemoSerial", io.demo), \
             patch.object(motor_session, "exchange", io.exchange):
            sys.argv = main_argv
            system_exit = False
            exit_code = None
            main_exception = None
            try:
                dashboard.main()
            except SystemExit as error:
                system_exit = True
                exit_code = 0 if error.code is None else error.code
            except BaseException:
                main_exception = traceback.format_exc()
            finally:
                sys.argv = original_argv
                argv_restored = sys.argv is original_argv and sys.argv == original_argv_values

            _cleanup_after_main(state["window"], state)

        report = {
            "created_at": datetime.now(timezone.utc).isoformat(),
            "source": source,
            "argv": main_argv,
            "python": {"executable": sys.executable, "version": sys.version},
            "packages": {
                "PySide6": importlib.metadata.version("PySide6"),
                "pyserial": importlib.metadata.version("pyserial"),
            },
            "source_fingerprint": _fingerprint(root),
            "system_exit": system_exit,
            "exit_code": exit_code,
            "main_exception": main_exception,
            "argv_restored": argv_restored,
            "window_captured": state["window"] is not None,
            "callback_ran": state["callback_ran"],
            "callback_completed": state["callback_completed"],
            "callback_errors": state["callback_errors"],
            "pre": state["pre"],
            "post": state["post"],
            "close_returned": state["close_returned"],
            "sentinel_counts": dict(io.counts),
            "screenshot": state["screenshot"],
        }
        validate_report(report)
        with result_path.open("x", encoding="utf-8") as result_file:
            json.dump(report, result_file, indent=2)
            result_file.write("\n")
        print(f"PASS: {source} dashboard startup and shutdown; evidence: {evidence_dir}", flush=True)
        return 0
    except BaseException as error:
        print(f"FAIL: {source} dashboard startup verification: {error}", file=sys.stderr, flush=True)
        if "report" in locals():
            print(json.dumps(report, indent=2), file=sys.stderr, flush=True)
        else:
            traceback.print_exc()
        return 1
    finally:
        sys.argv = original_argv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("real", "demo"), required=True)
    parser.add_argument("--evidence-dir", required=True)
    args = parser.parse_args()
    return run(args.source, args.evidence_dir)


if __name__ == "__main__":
    raise SystemExit(main())
