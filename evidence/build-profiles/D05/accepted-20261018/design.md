# D05 calibration artifact binding

Base accepted/exact-green D04 5a4fed1. Root implements personally.

A: Immutable CalibrationBinding stores calibration ID/source plus SHA256 of canonical full profile and exact opaque artifact bytes. require_match(profile, artifact) rejects changed build/revision/backend/identity/mapping/locked mass/provenance or changed artifact. No file I/O, artifact parsing, calibration calculation or live application.
B: Duplicate selected identity fields or apply saved zeros. Rejected: drift when schema grows, omitted config, or unsafe live restoration.

Choose A. Record references an existing calibration artifact, it does not certify its contents or invent a calibration-data schema. Unknown facts remain unverified; matching metadata is never readiness/permission. No apply/restore/arm method and no owner import. Existing runtime must still obtain fresh session references, health and explicit enable. Strict nonempty immutable bytes and explicit unpadded ID/source; exact lowercase64hex digests. Records only; persistence/parser and actual calibration engine are separate.

Checks: sameprofile/artifact match; changedrevision/build/backend/actuator/map/measurement/locked mass reject; changedartifact reject; frozen/strictinputs; owner/Core unchanged.
