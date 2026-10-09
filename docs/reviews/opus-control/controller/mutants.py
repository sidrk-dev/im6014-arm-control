"""Mutation probe: does the repo's own controller suite detect each injected defect?

Copies the repository's root *.py files into a temporary scratch directory,
applies ONE textual mutation per run (exact single-occurrence
match at line N), and runs the five controller suites with fake I/O only.
The repository itself is never modified.
"""
import pathlib
import shutil
import subprocess
import sys
import tempfile

WT = pathlib.Path(str(__import__("pathlib").Path(__file__).resolve().parents[4]))
OUT = pathlib.Path(tempfile.mkdtemp(prefix="im6014-mutants-"))
PY = sys.executable
SUITES = ["test_controller", "test_controller_expiry", "test_controller_owner",
          "test_controller_effort", "test_controller_thermal"]

# (id, file, line_no, old, new, description)
MUTANTS = [
    ("M1", "joint_controller.py", 549,
     "if not (self._recovery_authorized and below_recovery and dwell_satisfied):",
     "if not (below_recovery and dwell_satisfied):",
     "checked(): recovery no longer requires acknowledgment flag"),
    ("M2", "joint_controller.py", 549,
     "if not (self._recovery_authorized and below_recovery and dwell_satisfied):",
     "if not (self._recovery_authorized and dwell_satisfied):",
     "checked(): drop reset-temperature/continuous-effort recovery bound"),
    ("M3", "joint_controller.py", 209,
     "state.reset_timing_gap() for state in self._protection_states",
     "state for state in self._protection_states",
     "claim_owner(): takeover no longer breaks protection intervals"),
    ("M4", "joint_controller.py", 902,
     "state.reset_timing_gap() for state in self._protection_states",
     "state for state in self._protection_states",
     "stop(): no longer breaks protection intervals"),
    ("M5", "joint_controller.py", 915,
     "state.reset_timing_gap() for state in self._protection_states",
     "state for state in self._protection_states",
     "disconnect(): no longer breaks protection intervals"),
    ("M6", "joint_protection.py", 206,
     "health.mode == 0",
     "health.mode in (0, 1)",
     "recovery candidate no longer requires Stop mode"),
    ("M7", "joint_protection.py", 165,
     "driver_temp = health.driver_c >= spec.protection.driver_trip_c",
     "driver_temp = health.driver_c > spec.protection.driver_trip_c",
     "driver trip boundary inclusive -> exclusive"),
    ("M8", "joint_controller.py", 785,
     "if not self._all_fresh(now):",
     "if False:",
     "claim_dispatch(): drop explicit all-fresh check"),
    ("M9", "joint_controller.py", 879,
     "and owner_tokens_match(self._current_owner, plan.target.owner)",
     "and True",
     "dispatch_result(): COMPLETE receipt no longer checks current owner before MOVING"),
    ("M10", "joint_controller.py", 469,
     "self._recovery_authorized,",
     "True,",
     "receive(): recovery dwell accumulates without acknowledgment"),
    ("M11", "joint_controller.py", 434,
     "state.reset_timing_gap() for state in self._protection_states",
     "state for state in self._protection_states",
     "begin_check(): no longer restarts recovery interval"),
    ("M12", "joint_controller.py", 780,
     "if not owner_tokens_match(self._prepared_plan.target.owner, self._current_owner):",
     "if False:",
     "claim_dispatch(): drop owner check at final gate"),
    ("M13", "joint_controller.py", 372,
     "and now >= self._admitted_target.window.expires_at_s",
     "and now > self._admitted_target.window.expires_at_s",
     "_housekeeping(): expiry equality no longer expires the admitted goal"),
    ("M14", "joint_controller.py", 776,
     "if self._housekeeping(now) is not None:",
     "if False:",
     "claim_dispatch(): skip housekeeping (expiry/stale) at final gate"),
    ("M15", "joint_controller.py", 472,
     "if not status.healthy:",
     "if False:",
     "receive(): unhealthy telemetry no longer latches a fault"),
    ("M16", "joint_controller.py", 425,
     "if bool(self._faults) and not acknowledge_fault:",
     "if False:",
     "begin_check(): fault acknowledgment no longer required"),
    ("M17", "joint_protection.py", 203,
     "continuous_fault = overload_duration >= continuous_dwell",
     "continuous_fault = overload_duration > continuous_dwell",
     "continuous dwell equality no longer trips"),
    ("M18", "joint_controller.py", 764,
     "if self._state is State.FAULT or bool(self._faults):",
     "if False:",
     "claim_dispatch(): drop early fault guard"),
]


def copy_tree():
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    for p in WT.glob("*.py"):
        shutil.copy2(p, OUT / p.name)


def run_suites():
    r = subprocess.run([PY, "-B", "-m", "unittest", *SUITES], cwd=OUT,
                       capture_output=True, text=True, timeout=600)
    tail = (r.stderr.strip().splitlines() or [""])[-1]
    return r.returncode, tail


def main():
    copy_tree()
    rc, tail = run_suites()
    print(f"baseline copy: exit={rc} {tail}")
    for mid, fname, line_no, old, new, desc in MUTANTS:
        copy_tree()
        path = OUT / fname
        lines = path.read_text().splitlines(keepends=True)
        if old not in lines[line_no - 1]:
            print(f"{mid}: SKIP pattern not at {fname}:{line_no}")
            continue
        lines[line_no - 1] = lines[line_no - 1].replace(old, new, 1)
        path.write_text("".join(lines))
        rc, tail = run_suites()
        verdict = "KILLED" if rc != 0 else "SURVIVED"
        print(f"{mid}: {verdict:8s} exit={rc} [{tail}] {fname}:{line_no} {desc}")
    shutil.rmtree(OUT)


if __name__ == "__main__":
    sys.exit(main())
