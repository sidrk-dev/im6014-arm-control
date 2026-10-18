from dataclasses import dataclass
import hashlib
import re

from build_profile import BuildProfile


def _profile_digest(profile: BuildProfile) -> str:
    if not isinstance(profile, BuildProfile):
        raise ValueError('profile must be a BuildProfile')
    return hashlib.sha256(profile.to_json().encode('utf-8')).hexdigest()


def _artifact_digest(artifact: bytes) -> str:
    if type(artifact) is not bytes or not artifact:
        raise ValueError('calibration artifact must be non-empty immutable bytes')
    return hashlib.sha256(artifact).hexdigest()


@dataclass(frozen=True)
class CalibrationBinding:
    calibration_id: str
    source: str
    profile_sha256: str
    artifact_sha256: str

    def __post_init__(self):
        for value in (self.calibration_id, self.source):
            if type(value) is not str or not value.strip() or value != value.strip():
                raise ValueError('calibration ID and source must be non-empty unpadded strings')
        for digest in (self.profile_sha256, self.artifact_sha256):
            if type(digest) is not str or re.fullmatch('[0-9a-f]{64}', digest) is None:
                raise ValueError('calibration binding digests must be lowercase SHA256 hex strings')

    def require_match(self, profile: BuildProfile, artifact: bytes) -> None:
        if self.profile_sha256 != _profile_digest(profile):
            raise ValueError('calibration belongs to a different build revision or configuration')
        if self.artifact_sha256 != _artifact_digest(artifact):
            raise ValueError('calibration artifact content differs from its binding')


def bind_calibration(profile: BuildProfile, artifact: bytes, *,
                     calibration_id: str, source: str) -> CalibrationBinding:
    return CalibrationBinding(calibration_id, source, _profile_digest(profile), _artifact_digest(artifact))
