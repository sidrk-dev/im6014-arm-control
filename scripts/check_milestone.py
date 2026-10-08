import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess


CHECKS = {
    'B11': 'test_joint_mapping',
    'B12': 'test_numerical_inputs',
    'B13': 'test_port_config',
    'B14': 'test_diagnostic_capture',
    'C01': 'test_bus_owner',
    'C02': 'test_controller.ControllerStateTests',
    'C03': 'test_controller.FeedbackAgeTests',
    'C04': 'test_controller.JointFaultTests',
    'C05': 'test_controller_targets.LatestTargetTests',
    'C06': 'test_controller_targets.CommandSequenceTests',
    'C07': 'test_controller_expiry',
    'C08': 'test_controller_owner',
}


def fingerprint(repo):
    paths = sorted(repo.glob('*.py')) + [
        repo / 'scripts/check_offline_baseline.sh',
        repo / 'scripts/check_milestone.py', repo / 'USER_MANUAL.md',
    ]
    return {str(p.relative_to(repo)): {
        'bytes': p.stat().st_size,
        'sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
    } for p in paths}


def main():
    repo = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description='Run one audited offline milestone check.')
    parser.add_argument('unit', choices=CHECKS)
    parser.add_argument('--python', type=Path, default=repo / '.venv/bin/python')
    parser.add_argument('--evidence-root', type=Path, default=repo / 'evidence/runs')
    args = parser.parse_args()
    root = args.evidence_root.resolve()
    if root == repo or root == Path('/') or repo in root.parents and repo / 'evidence/runs' != root and repo / 'evidence/runs' not in root.parents:
        parser.error('Evidence inside the repository must be under evidence/runs')
    out = root / (args.unit + '-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True, exist_ok=False)
    before = fingerprint(repo)
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    command = [str(args.python.absolute()), '-B', '-m', 'unittest', '-v', CHECKS[args.unit]]
    result = subprocess.run(command, cwd=repo, capture_output=True, text=True,
                            env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1', QT_QPA_PLATFORM='offscreen'))
    (out / 'stdout.log').write_text(result.stdout)
    (out / 'stderr.log').write_text(result.stderr)
    unchanged = before == fingerprint(repo)
    (out / 'receipt.json').write_text(json.dumps({
        'unit': args.unit, 'command': command, 'cwd': str(repo), 'baseHead': head,
        'sourceFiles': before, 'sourceUnchanged': unchanged,
        'exitCode': result.returncode, 'recordedAt': datetime.now(timezone.utc).isoformat(),
        'physicalIO': 'none; parent reviewed fixtures or pseudo-terminals before execution',
    }, indent=2) + '\n')
    print(result.stdout, end='')
    print(result.stderr, end='')
    print('Receipt:', out / 'receipt.json')
    return result.returncode if unchanged else 2


if __name__ == '__main__':
    raise SystemExit(main())
