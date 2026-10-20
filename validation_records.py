from dataclasses import dataclass, replace
from enum import Enum
import hashlib
import json
import re

from build_profile import BuildProfile


class ValidationDependency(Enum):
    PROFILE = 'profile'
    TOOL = 'tool'
    MOUNTING_TRANSFORM = 'mounting_transform'


class ValidationStatus(Enum):
    PENDING = 'pending'
    PASSED = 'passed'
    FAILED = 'failed'


def _text(value, name):
    if type(value) is not str or not value.strip() or value != value.strip():
        raise ValueError(f'{name} must be a non-empty unpadded string')


@dataclass(frozen=True)
class ConfigurationArtifact:
    data: bytes | None
    source: str

    def __post_init__(self):
        _text(self.source, 'configuration source')
        if self.data is not None and (type(self.data) is not bytes or not self.data):
            raise ValueError('configuration data must be non-empty immutable bytes or None for unknown')

    def fingerprint(self) -> str | None:
        if self.data is None:
            return None
        content = json.dumps({'source': self.source,
                              'sha256': hashlib.sha256(self.data).hexdigest()}, sort_keys=True)
        return hashlib.sha256(content.encode('utf-8')).hexdigest()


@dataclass(frozen=True)
class ValidationContext:
    profile: BuildProfile
    tool: ConfigurationArtifact
    mounting_transform: ConfigurationArtifact

    def __post_init__(self):
        if type(self.profile) is not BuildProfile:
            raise ValueError('profile must be a BuildProfile')
        if any(type(value) is not ConfigurationArtifact
               for value in (self.tool, self.mounting_transform)):
            raise ValueError('tool and mounting_transform must be ConfigurationArtifact records')

    def fingerprint(self, dependency: ValidationDependency) -> str | None:
        if dependency is ValidationDependency.PROFILE:
            return hashlib.sha256(self.profile.to_json().encode('utf-8')).hexdigest()
        if dependency is ValidationDependency.TOOL:
            return self.tool.fingerprint()
        if dependency is ValidationDependency.MOUNTING_TRANSFORM:
            return self.mounting_transform.fingerprint()
        raise ValueError('dependency must be a ValidationDependency')


@dataclass(frozen=True)
class ValidationRecord:
    check_id: str
    dependencies: frozenset[ValidationDependency]
    status: ValidationStatus = ValidationStatus.PENDING
    evidence: str | None = None
    input_hashes: tuple[tuple[ValidationDependency, str], ...] = ()

    def __post_init__(self):
        _text(self.check_id, 'check ID')
        if (type(self.dependencies) is not frozenset or not self.dependencies
                or any(not isinstance(item, ValidationDependency) for item in self.dependencies)):
            raise ValueError('dependencies must be a non-empty frozenset of ValidationDependency')
        if ValidationDependency.PROFILE not in self.dependencies:
            raise ValueError('every check must bind its build profile')
        if not isinstance(self.status, ValidationStatus):
            raise ValueError('status must be a ValidationStatus')
        if type(self.input_hashes) is not tuple:
            raise ValueError('input_hashes must be an immutable tuple')
        keys = []
        for pair in self.input_hashes:
            if type(pair) is not tuple or len(pair) != 2:
                raise ValueError('input hashes must contain immutable dependency/digest pairs')
            dependency, digest = pair
            if not isinstance(dependency, ValidationDependency):
                raise ValueError('input hash key must be a ValidationDependency')
            if type(digest) is not str or re.fullmatch('[0-9a-f]{64}', digest) is None:
                raise ValueError('input hash must be a lowercase SHA256 digest')
            keys.append(dependency)
        if len(set(keys)) != len(keys):
            raise ValueError('input hash dependencies must be unique')
        if self.status is ValidationStatus.PENDING:
            if self.evidence is not None or self.input_hashes:
                raise ValueError('pending validation cannot carry a completed result')
        else:
            _text(self.evidence, 'result evidence reference')
            if set(keys) != self.dependencies:
                raise ValueError('completed validation must bind every declared dependency')
        object.__setattr__(self, 'input_hashes', tuple(sorted(
            self.input_hashes, key=lambda pair: pair[0].value)))

    def report_result(self, context: ValidationContext, *,
                      status: ValidationStatus, evidence: str) -> 'ValidationRecord':
        if type(context) is not ValidationContext:
            raise ValueError('context must be a ValidationContext')
        if status not in (ValidationStatus.PASSED, ValidationStatus.FAILED):
            raise ValueError('a reported result must be passed or failed')
        hashes = tuple((item, context.fingerprint(item)) for item in self.dependencies)
        if any(digest is None for _, digest in hashes):
            raise ValueError('required configuration is unknown; validation remains pending')
        return replace(self, status=status, evidence=evidence, input_hashes=hashes)

    def invalidate_changed(self, context: ValidationContext) -> 'ValidationRecord':
        if type(context) is not ValidationContext:
            raise ValueError('context must be a ValidationContext')
        if self.status is ValidationStatus.PENDING:
            return self
        if any(context.fingerprint(dependency) != digest
               for dependency, digest in self.input_hashes):
            return ValidationRecord(self.check_id, self.dependencies)
        return self
