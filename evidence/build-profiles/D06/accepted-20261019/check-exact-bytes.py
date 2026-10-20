import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]).resolve()))
from dataclasses import replace
from test_validation_invalidation import context,result
from validation_records import ConfigurationArtifact,ValidationStatus
c=context();a=replace(c,tool=ConfigurationArtifact(b'{"tool":"x"}', 'synthetic fixture'))
b=replace(a,tool=ConfigurationArtifact(b'{ "tool":"x" }', 'synthetic fixture'))
assert result(a).invalidate_changed(b).status is ValidationStatus.PENDING
print('Exact-byte whitespace-only configuration change invalidates selected validation.')
