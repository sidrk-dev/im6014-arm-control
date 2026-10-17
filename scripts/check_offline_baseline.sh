#!/usr/bin/env bash
set -u
set -o pipefail

if (( $# > 2 )); then
    printf 'Usage: %s [absolute-python] [evidence-root]\n' "$0" >&2
    exit 2
fi

caller_dir="$(pwd -P)"
script_path="$(realpath -e -- "$0")" || exit 2
repo_dir="$(cd -- "$(dirname -- "$script_path")/.." && pwd -P)"
python_arg="${1:-$repo_dir/.venv/bin/python}"
evidence_arg="${2:-$repo_dir/evidence/runs}"
case "$python_arg$evidence_arg" in
    *$'\n'*|*$'\t'*) printf 'Paths cannot contain tabs or newlines.\n' >&2; exit 2 ;;
esac

case "$python_arg" in
    /*) ;;
    *) python_arg="$caller_dir/$python_arg" ;;
esac
python_dir_arg="$(dirname -- "$python_arg")"
python_name="$(basename -- "$python_arg")"
python_dir="$(cd -- "$python_dir_arg" 2>/dev/null && pwd -P)" || {
    printf 'Python interpreter does not exist: %s\n' "$python_arg" >&2
    exit 2
}
python_path="$python_dir/$python_name"
if [[ ! -f "$python_path" || ! -x "$python_path" ]]; then
    printf 'Python interpreter must be an executable file: %s\n' "$python_path" >&2
    exit 2
fi

case "$evidence_arg" in
    /*) ;;
    *) evidence_arg="$caller_dir/$evidence_arg" ;;
esac
mkdir -p -- "$evidence_arg" || {
    printf 'Could not create evidence root: %s\n' "$evidence_arg" >&2
    exit 2
}
evidence_root="$(realpath -e -- "$evidence_arg")" || exit 2
if [[ "$evidence_root" == / || "$evidence_root" == "$repo_dir" ]]; then
    printf 'Refusing unsafe evidence root: %s\n' "$evidence_root" >&2
    exit 2
fi
case "$evidence_root/" in
    "$repo_dir/"*)
        case "$evidence_root/" in
            "$repo_dir/evidence/runs/"|"$repo_dir/evidence/runs/"*) ;;
            *) printf 'Evidence inside the repository must be under evidence/runs: %s\n' "$evidence_root" >&2; exit 2 ;;
        esac
        ;;
esac

run_dir="$(mktemp -d -- "$evidence_root/run-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXXXX")" || exit 2
scratch="$(mktemp -d -- "$run_dir/.source-XXXXXXXX")" || exit 2
cleanup() {
    rm -rf -- "$scratch"
}
trap cleanup EXIT

printf 'check\texit_code\n' > "$run_dir/checks.tsv"
failed=0
record_check() {
    local name="$1"
    local code="$2"
    printf '%s\t%s\n' "$name" "$code" >> "$run_dir/checks.tsv"
    if (( code != 0 )); then failed=1; fi
}
run_logged() {
    local name="$1"
    shift
    "$@" > "$run_dir/$name.log" 2>&1
    local code=$?
    record_check "$name" "$code"
    return 0
}

files=(motor_dashboard.py motor_session.py spin_5rpm.py im6014_codec.py im6014_serial.py im6014_transport.py fake_serial.py test_spin_5rpm.py test_dashboard.py test_startup.py test_startup_verifier.py test_protocol.py test_serial_deadline.py evidence/boundary/vendor-crc-vectors.json USER_MANUAL.md joint_mapping.py test_joint_mapping.py test_numerical_inputs.py im6014_port.py test_port_config.py test_diagnostic_capture.py test_bus_owner.py joint_controller.py joint_records.py test_controller.py test_controller_targets.py test_controller_ownership.py test_controller_limits.py test_controller_velocity.py joint_reference.py test_controller_acceleration.py test_controller_lead.py test_controller_effort.py test_controller_faults.py)
for name in "${files[@]}"; do
    mkdir -p -- "$scratch/$(dirname -- "$name")" || exit 2
    if ! cp -- "$repo_dir/$name" "$scratch/$name" || ! cmp -s -- "$repo_dir/$name" "$scratch/$name"; then
        printf 'Could not create a byte-identical baseline copy: %s\n' "$name" >&2
        exit 2
    fi
done

if "$python_path" -c 'import sys; print(sys.version)' > "$run_dir/python-version.txt" 2>&1; then
    record_check interpreter 0
else
    code=$?
    record_check interpreter "$code"
fi

cd -- "$scratch" || exit 2
run_logged packet-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" test_spin_5rpm.py
run_logged protocol-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_protocol
run_logged serial-deadline-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_serial_deadline
run_logged milestone-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_joint_mapping test_numerical_inputs test_port_config test_diagnostic_capture test_bus_owner test_controller test_controller_targets test_controller_ownership test_controller_limits test_controller_velocity test_controller_acceleration test_controller_lead test_controller_effort test_controller_faults
run_logged dashboard-tests env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 "$python_path" test_dashboard.py
for name in dashboard_preview.png dashboard_dark_preview.png dashboard_test_results.json; do
    if [[ -f "$scratch/$name" ]]; then
        if ! cp -- "$scratch/$name" "$run_dir/$name"; then record_check "copy-$name" 1; fi
    fi
done

run_logged startup-verifier-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" test_startup_verifier.py
run_logged startup-real env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 \
    "$python_path" test_startup.py --source real --evidence-dir "$run_dir/startup-real"
run_logged startup-demo env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 \
    "$python_path" test_startup.py --source demo --evidence-dir "$run_dir/startup-demo"

printf 'Evidence: %s\n' "$run_dir"
cat "$run_dir/checks.tsv"
exit "$failed"
