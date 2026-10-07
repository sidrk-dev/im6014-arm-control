"""Read-only audit of the frozen checkout and the independent baseline run."""
from pathlib import Path
import csv
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
import re
import subprocess
import sys

ROOT = Path('/tmp/pstack-a01-a10-20261006-bv_ho4fx/verifier')
OUT = Path('/tmp/pstack-a01-a10-20261006-bv_ho4fx/final-verification')
RUN = OUT / 'run-20261007T033511Z-9hzNOvAI'
ORIGINAL = Path('/home/sid/Documents/code/im6014Development')
HEAD = '91634b433b3b917458fa51d2fbe14bceb8c57299'
BASE = '7a6881e6da6884fa1e796a662e9e3f347b7d349c'
BENCH = '69c8219d764a24d03dadf1df011efb027f0f8996'
COMMANDS = []
FAILURES = []

def need(condition, message):
    if not condition:
        FAILURES.append(message)

def digest(data):
    return hashlib.sha256(data).hexdigest()

def read_json(path):
    return json.loads(path.read_text())

def save(name, data):
    (OUT / name).write_text(json.dumps(data, indent=2) + '\n')

def command(argv, cwd=ROOT, input_data=None):
    result = subprocess.run(argv, cwd=cwd, env=dict(os.environ, GIT_OPTIONAL_LOCKS='0'),
                            input=input_data, capture_output=True)
    COMMANDS.append({'argv': argv, 'cwd': str(cwd), 'exitCode': result.returncode,
                     'stderr': result.stderr.decode(errors='replace')})
    need(result.returncode == 0, 'Read-only command failed: ' + repr(argv))
    return result.stdout

diff = command(['git', 'diff', '--binary', '--full-index', BASE, HEAD])
(OUT / 'actual.diff').write_bytes(diff)
patch_id_output = command(['git', 'patch-id', '--stable'], input_data=diff).decode().strip()
patch_id = patch_id_output.split()[0]
actual_head = command(['git', 'rev-parse', 'HEAD']).decode().strip()
actual_base = command(['git', 'rev-parse', BASE]).decode().strip()
need(actual_head == HEAD, 'Checkout HEAD differs from frozen candidate')
need(actual_base == BASE, 'Resolved patch base differs from supplied base')
status = command(['git', 'status', '--porcelain=v1']).decode()
need(status == '', 'Verifier checkout is not clean')
mode = command(['git', 'ls-files', '--stage', 'scripts/check_offline_baseline.sh']).decode().strip()
need(mode.startswith('100755 '), 'Offline driver mode is not 100755')
need(os.access(ROOT / 'scripts/check_offline_baseline.sh', os.X_OK), 'Offline driver is not executable')
preserved = ['USER_MANUAL.md', 'dashboard_dark_preview.png', 'motor_dashboard.py',
             'motor_session.py', 'requirements.txt', 'spin_5rpm.py', 'start_dashboard.sh',
             'test_dashboard.py', 'test_spin_5rpm.py']
preservation = {}
for name in preserved:
    historical = command(['git', 'show', f'{BENCH}:{name}'])
    checkout = (ROOT / name).read_bytes()
    original = (ORIGINAL / name).read_bytes()
    preservation[name] = {'benchSHA256': digest(historical), 'checkoutSHA256': digest(checkout),
                          'originalSHA256': digest(original),
                          'checkoutMatchesBench': checkout == historical,
                          'originalMatchesBench': original == historical}
    need(checkout == historical and original == historical, 'Inherited file differs from bench: ' + name)
original_head = command(['git', 'rev-parse', 'HEAD'], ORIGINAL).decode().strip()
original_status = command(['git', 'status', '--porcelain=v1'], ORIGINAL).decode()
repository_receipt = read_json(ROOT / 'evidence/baseline/repository.json')
need(original_head == BENCH, 'Original checkout HEAD changed')
need(original_status == repository_receipt['sourceStatus'], 'Original checkout status changed')
plan = ORIGINAL / repository_receipt['planOriginalName']
plan_hash = digest(plan.read_bytes())
need(plan_hash == repository_receipt['planSha256'], 'Original plan SHA changed')
need((ROOT / repository_receipt['importedPlan']).read_bytes() == plan.read_bytes(), 'Imported plan differs from original')
git_audit = {'head': actual_head, 'patchBase': actual_base, 'benchBase': BENCH,
             'stablePatchID': patch_id, 'patchIDRawOutput': patch_id_output,
             'actualDiffSHA256': digest(diff), 'checkoutStatus': status,
             'driverIndexRecord': mode, 'preservedFiles': preservation,
             'originalCheckout': {'head': original_head, 'status': original_status,
                                  'planSHA256': plan_hash}}
save('git-audit.json', git_audit)

expected_checks = ['interpreter', 'packet-tests', 'dashboard-tests', 'startup-verifier-tests',
                   'startup-real', 'startup-demo']
checks = list(csv.DictReader((RUN / 'checks.tsv').open(), delimiter='\t'))
need([r['check'] for r in checks] == expected_checks, 'Six check identifiers differ')
need(all(r['exit_code'] == '0' for r in checks), 'A baseline check did not exit 0')
dashboard_result = read_json(RUN / 'dashboard_test_results.json')
dashboard_log = (RUN / 'dashboard-tests.log').read_text()
dashboard_lines = [line for line in dashboard_log.splitlines() if line.startswith('PASS:')]
need(len(dashboard_result['checks']) == len(dashboard_lines) == 18, 'Dashboard group count is not 18')
need(dashboard_result['motor_io_performed'] is False, 'Dashboard report does not mark no motor I/O')
need('18 grouped checks passed. No real motor I/O.' in dashboard_log, 'Missing dashboard summary')
verifier_log = (RUN / 'startup-verifier-tests.log').read_text()
# unittest prefixes the first printed case with the test label on that same line.
cases = re.findall(r'PASS: injected (\w+) rejected; exit 1; no startup receipt$', verifier_log, re.M)
expected_cases = ['serial_open', 'demo_open', 'exchange', 'screenshot', 'close', 'missing_window', 'exit_code']
need(cases == expected_cases, 'Seven explicit failure-case lines differ')
need('Ran 5 tests' in verifier_log and verifier_log.rstrip().endswith('OK'), 'Five verifier tests did not pass')
packages = {name: importlib.metadata.version(name) for name in
            ['pyserial', 'PySide6', 'PySide6_Addons', 'PySide6_Essentials', 'shiboken6']}
lock = dict(re.findall(r'^(\S+)==(\S+) --hash=sha256:[0-9a-f]{64}$',
                      (ROOT / 'requirements.lock').read_text(), re.M))
need(packages == lock and len(lock) == 5, 'Actual selected packages differ from five-package hash lock')
need(sys.version_info[:3] == (3, 12, 14), 'Selected interpreter is not 3.12.14')
save('execution-audit.json', {
    'driverCommand': ['scripts/check_offline_baseline.sh', sys.executable, str(OUT)],
    'driverToolExitCode': 0,
    'driverExitEvidence': 'Host exec session 82181 completed with exit_code 0, chunk 4c5399',
    'runDirectory': str(RUN), 'checks': checks, 'dashboardGroups': len(dashboard_lines),
    'verifierTests': 5, 'explicitFaultCases': cases, 'python': sys.version,
    'pythonExecutable': sys.executable, 'packages': packages, 'packagesMatchHashLock': packages == lock,
    'temporaryAndCacheRoots': {name: os.environ.get(name) for name in
       ['TMPDIR', 'XDG_CACHE_HOME', 'XDG_CONFIG_HOME', 'XDG_DATA_HOME', 'XDG_RUNTIME_DIR', 'PIP_CACHE_DIR']},
    'exception': 'Legacy /tmp/dashboard_small.png explicitly authorized; excluded from evidence',
})

names = ['motor_dashboard.py', 'motor_session.py', 'spin_5rpm.py', 'USER_MANUAL.md',
         'test_spin_5rpm.py', 'test_dashboard.py', 'test_startup.py', 'test_startup_verifier.py']
actual_fingerprints = {name: {'bytes': (ROOT / name).stat().st_size,
                              'sha256': digest((ROOT / name).read_bytes())} for name in names}
artifact_map = read_json(ROOT / 'evidence/baseline/startup/artifact-map.json')
parent_receipt = read_json(ROOT / 'evidence/baseline/startup/parent-verification.json')
need(artifact_map['codeRevision'] == parent_receipt['verifiedCodeRevision'], 'Parent code revisions disagree')
historical_fingerprints = {}
for name in names:
    data = command(['git', 'show', f"{artifact_map['codeRevision']}:{name}"])
    historical_fingerprints[name] = {'bytes': len(data), 'sha256': digest(data)}
need(historical_fingerprints == actual_fingerprints, 'Parent verified source bytes differ from frozen checkout')
mode_artifacts = {}
for source in ['real', 'demo']:
    fresh = read_json(RUN / f'startup-{source}/startup-result.json')
    mapped = artifact_map['modes'][source]
    for path_key in ['receipt', 'screenshot']:
        path = Path(mapped[path_key])
        need(not path.is_absolute() and '..' not in path.parts and (ROOT / path).is_file(),
             f'Committed {source} {path_key} mapping is not a valid relative file')
    committed = read_json(ROOT / mapped['receipt'])
    fresh_png = RUN / f'startup-{source}/dashboard.png'
    committed_png = ROOT / mapped['screenshot']
    fresh_sha = digest(fresh_png.read_bytes())
    committed_sha = digest(committed_png.read_bytes())
    need(fresh_sha == fresh['screenshot']['sha256'], f'Fresh {source} screenshot hash mismatch')
    need(committed_sha == committed['screenshot']['sha256'] == mapped['screenshotSHA256'],
         f'Committed {source} screenshot mapping hash mismatch')
    need(str(fresh_png) == fresh['screenshot']['path'], f'Fresh {source} screenshot path mismatch')
    need(committed['screenshot']['path'] == mapped['historicalScreenshotPath'],
         f'Committed {source} historical screenshot path mismatch')
    for label, receipt in [('fresh', fresh), ('committed', committed)]:
        need(receipt['source_fingerprint'] == actual_fingerprints == mapped['sourceFingerprints'],
             f'{label} {source} eight fingerprints differ')
        need(receipt['source'] == source, f'{label} {source} source mismatch')
        for phase in ['pre', 'post']:
            state = receipt[phase]
            need(state['bus_is_none'] is True and state['state'] == 'Disconnected'
                 and state['connected'] is False and state['armed'] is False
                 and state['tx'] == state['rx'] == 0
                 and state['source_selected_demo'] is (source == 'demo'),
                 f'{label} {source} {phase} disconnected lifecycle mismatch')
        need(receipt['pre']['timer_active'] is True and receipt['pre']['worker_alive'] is True
             and receipt['pre']['window_visible'] is True and receipt['pre']['event_loop_tick_seen'] is True,
             f'{label} {source} pre-close was not live')
        need(receipt['post']['timer_active'] is False and receipt['post']['worker_alive'] is False
             and receipt['post']['window_visible'] is False and receipt['close_returned'] is True,
             f'{label} {source} close failed')
        need(receipt['sentinel_counts'] == {'serial_open': 0, 'demo_open': 0, 'exchange': 0},
             f'{label} {source} forbidden I/O occurred')
        need(receipt['system_exit'] is True and receipt['exit_code'] == 0 and receipt['argv_restored'] is True
             and not receipt['callback_errors'] and not receipt['main_exception']
             and receipt['window_captured'] is True and receipt['callback_ran'] is True
             and receipt['callback_completed'] is True, f'{label} {source} execution failed')
        need(receipt['screenshot']['bytes'] == (fresh_png if label == 'fresh' else committed_png).stat().st_size,
             f'{label} {source} screenshot byte count differs')
        need(receipt['screenshot']['width'] == 1200 and receipt['screenshot']['height'] == 970,
             f'{label} {source} screenshot dimensions differ')
    mode_artifacts[source] = {
        'freshReceipt': str(RUN / f'startup-{source}/startup-result.json'),
        'freshScreenshot': str(fresh_png), 'freshScreenshotSHA256': fresh_sha,
        'committedReceipt': mapped['receipt'], 'committedScreenshot': mapped['screenshot'],
        'committedScreenshotSHA256': committed_sha,
        'freshPre': fresh['pre'], 'freshPost': fresh['post'], 'sentinelCounts': fresh['sentinel_counts'],
        'visualInspection': 'Both fresh and committed actual PNGs opened through view_image. Correct source badge, Disconnected, blank telemetry, disabled check/ack/arm/run controls; readable layout.',
    }
save('artifact-audit.json', {'sourceFingerprints': actual_fingerprints,
                            'parentVerifiedCodeRevision': artifact_map['codeRevision'],
                            'modes': mode_artifacts,
                            'note': 'Fresh PNG hashes need not equal historical PNG hashes; each matches its own actual bytes and receipt. All eight source hashes match.'})

model_index = read_json(ROOT / 'evidence/baseline/models/observed-executions.json')
model_audit = []
for record in model_index['records']:
    path = ROOT / record['evidence']
    need(path.is_file(), 'Model evidence pointer missing: ' + record['evidence'])
    receipt = read_json(path)
    entry = {'descriptor': record['model'], 'evidence': record['evidence'], 'indexStatus': record['status'],
             'receiptStatus': receipt['status'], 'receiptSHA256': digest(path.read_bytes())}
    if receipt.get('provider') in ['opencode', 'antigravity']:
        descriptor = f"{receipt['provider']}:{receipt['model']}@{receipt['effort']}"
        need(descriptor == record['model'], 'External descriptor mismatch: ' + record['model'])
        need(receipt['status'] == 'complete' and receipt['exitCode'] == 0 and receipt['processStarted'] is True,
             'External model did not complete: ' + record['model'])
        need(receipt['modelVerified'] is False and receipt['modelEvidence'] in ['pinned-request', 'pinned-argv'],
             'External model evidence overstates verification: ' + record['model'])
        need(receipt['argv'][receipt['argv'].index('--model') + 1] == receipt['model'], 'Pinned model argument differs')
        if receipt['provider'] == 'opencode':
            need(receipt['exactRequestModel'] == receipt['model'] and receipt['apiSpend'] == 'approved'
                 and receipt['submissionCount'] == 1 and receipt['rejectedPermissions'] == 0,
                 'OpenCode reviewer route differs')
            need(receipt['argv'][0] == '/home/sid/.local/bin/pstack-opencode-review', 'Wrong OpenCode reviewer launcher')
        else:
            need(receipt['apiSpend'] == 'deny', 'Gemini API spend differs')
        entry.update({'modelVerified': receipt['modelVerified'], 'modelEvidence': receipt['modelEvidence'],
                      'apiSpend': receipt['apiSpend'], 'exitCode': receipt['exitCode']})
    else:
        need('requested' in (receipt.get('evidence', '') + receipt.get('modelEvidence', '') + record.get('route', '')),
             'Native receipt does not identify requested flags')
        entry['evidenceLimit'] = 'Parent native dispatch attestation. Requested model and effort; no independent model identity or hidden reasoning attestation.'
    if record.get('output'):
        output = ROOT / record['output']
        need(digest(output.read_bytes()) == record['outputSHA256'], 'Model output hash mismatch')
        entry['outputSHA256'] = digest(output.read_bytes())
    if receipt.get('outputSHA256'):
        need(digest((ROOT / receipt['output']).read_bytes()) == receipt['outputSHA256'], 'Native worker output hash mismatch')
    if record.get('reviewPromptSHA256'):
        prompt = Path(receipt['promptPath'])
        entry['reviewPromptFileAvailable'] = prompt.is_file()
        if prompt.is_file():
            entry['reviewPromptSHA256'] = digest(prompt.read_bytes())
            need(entry['reviewPromptSHA256'] == record['reviewPromptSHA256'], 'Final review prompt hash mismatch')
    model_audit.append(entry)
need(len({r['model'] for r in model_index['records']}) == 5, 'Model index does not include five distinct requested descriptors')
save('model-audit.json', {'records': model_audit,
                         'note': 'No model route was rerun. External records remain pinned requests/backend reports with modelVerified false. Native records remain parent attestations. Luna partial handoff is preserved.'})

ledger = list(csv.DictReader((ROOT / 'docs/milestones.tsv').open(), delimiter='\t'))
need([row['id'] for row in ledger] == [f'A{i:02d}' for i in range(1, 11)], 'Ledger is not exactly A01-A10')
support = {
    'A01': 'Original HEAD/status and plan SHA match recorded source; imported plan is byte identical.',
    'A02': 'Launcher, Dashboard/main, MotorSession connect path, and alternate CLI traced; historic bench claim distinguished.',
    'A03': 'Committed raw packet/dashboard logs and grouped JSON agree. Fresh final driver independently passes both; 18 groups.',
    'A04': 'Session exchange owner, every handle/tick/stop/disconnect/close path, and CLI initial/final control traffic checked against source.',
    'A05': 'Environment values are historical observations; NVIDIA-specific raw GLX output reports dedicated 8188 MB and no unified memory. CUDA remains false.',
    'A06': 'Committed clean-install/freeze/pip-check logs support parent attestation. Actual selected Python is 3.12.14; all five metadata versions equal the hash lock. No install rerun.',
    'A07': 'Actual imports, GUI worker ownership and alternate CLI serial owner match architecture. Future components remain pending.',
    'A08': 'Exactly ten TSV rows in order, one software verified status per row, with nonempty expected/actual/evidence/review/revision/dependency fields.',
    'A09': 'Five exact requested descriptors present. Raw external completed receipts and native attestations are honestly labeled; partial Luna outcome retained; reviews triaged against source.',
    'A10': 'Fresh six-check driver exits all 0, 18 groups, five verifier tests, seven explicit cases; both startup lifecycle reports and actual PNGs verified. All eight fingerprints and committed mappings match.',
}
for row in ledger:
    need(row['status'] == 'software verified', 'Unexpected ledger status: ' + row['id'])
    for field in ['expected', 'actual', 'evidence', 'parent_review', 'revision', 'remaining_dependency']:
        need(bool(row[field]), f'Ledger {row["id"]} field {field} is empty')
    pointers = [value.strip() for value in row['evidence'].split(';')]
    for value in pointers:
        path = Path(value)
        need(not path.is_absolute() and '..' not in path.parts and (ROOT / path).exists(),
             f'Ledger {row["id"]} evidence pointer missing or unsafe: {value}')
    row['independentAudit'] = support[row['id']]
    row['evidencePointersExist'] = all((ROOT / value).exists() for value in pointers)
need('- [ ] Phase E: Verify and hand back.' in (ROOT / 'docs/workflow.md').read_text(), 'Expected Phase E pending state changed')
save('ledger-audit.json', {'rows': ledger, 'count': len(ledger),
                         'workflowPhaseE': 'Expected pending until parent GitHub landing/handback; not a source defect',
                         'boundary': 'No physical serial I/O, connected operations, motor communication, torque-off, calibration, J1 mounting, CUDA compute, or future arm milestones established.'})
save('audit-summary.json', {'createdAt': datetime.now(timezone.utc).isoformat(), 'head': HEAD,
                           'base': BASE, 'stablePatchID': patch_id, 'failures': FAILURES,
                           'readOnlyCommandExits': COMMANDS,
                           'result': 'PASS' if not FAILURES else 'FAIL'})
print(json.dumps({'head': HEAD, 'base': BASE, 'stablePatchID': patch_id, 'failures': FAILURES,
                  'checks': checks, 'dashboardGroups': 18, 'verifierTests': 5,
                  'explicitFaultCases': cases, 'outputRoot': str(OUT)}, indent=2))
raise SystemExit(1 if FAILURES else 0)
