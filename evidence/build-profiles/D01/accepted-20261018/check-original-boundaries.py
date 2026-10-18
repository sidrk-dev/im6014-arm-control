import importlib.util
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
sys.path.insert(0, str(root))
import build_profile as bp
import test_build_profile as fixture

raw = fixture.profile_dict()
text = json.dumps(raw)
text = text.replace('"motor_id": 1', '"motor_id": 0, "motor_id": 1')
results = {}
try:
    bp.parse_json(text)
    results['duplicateKeyAccepted'] = True
except ValueError:
    results['duplicateKeyAccepted'] = False
raw = fixture.profile_dict()
raw['joints'][0]['bus'] = ' bus0 '
try:
    bp.from_dict(raw)
    results['paddedBusAccepted'] = True
except ValueError:
    results['paddedBusAccepted'] = False
print(json.dumps(results))
