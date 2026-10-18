import json,sys,unittest
from pathlib import Path
from unittest.mock import patch
from serial.tools import list_ports
import im6014_port

# Existing port-classification tests replace this supplied list with their own fixtures.
# All platform discovery and serial construction remain replaced before test loading.
with patch.object(list_ports, 'comports', return_value=[]) as ports, \
        patch.object(im6014_port, '_by_id_alias', return_value=None), \
        patch('serial.Serial', side_effect=AssertionError('physical serial construction forbidden')) as serial:
    suite = unittest.defaultTestLoader.discover('tests', pattern='test_arm_*.py')
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    serial.assert_not_called()
    record = {'testsRun':result.testsRun,'successful':result.wasSuccessful(),
              'physicalSerialConstructorCalls':serial.call_count,'defaultFakePortSnapshots':ports.call_count,
              'discovery':'comports supplied fixtures; by-id lookup replaced; no host serial metadata discovery',
              'physicalIO':'no motor ports; demo/fake exchanges and ephemeral HTTP only'}
Path('/tmp/controller-release-review-20261017/guarded-arm-result.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record))
raise SystemExit(not result.wasSuccessful())
