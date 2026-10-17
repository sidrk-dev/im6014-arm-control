import json,sys,unittest
from pathlib import Path
from unittest.mock import patch

import arm_dashboard
import im6014_port

with patch.object(im6014_port, 'enumerate_ports', side_effect=AssertionError('real enumeration forbidden')) as backend, \
        patch.object(arm_dashboard, 'enumerate_ports', return_value=[]) as ports:
    suite = unittest.defaultTestLoader.discover('tests', pattern='test_arm_*.py')
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    backend.assert_not_called()
    record = {'testsRun':result.testsRun,'successful':result.wasSuccessful(),
              'realEnumerationCalls':backend.call_count,'fakePortSnapshots':ports.call_count,
              'physicalIO':'no motor ports;demo/fake exchanges and ephemeral HTTP only'}
Path('/tmp/c15-codex-evidence-20261017/guarded-arm-result.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record))
raise SystemExit(not result.wasSuccessful())
