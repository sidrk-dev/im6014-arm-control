# D03 provisional later builds

Parent implements personally; existing Pstack architect/review workflow continues. Base exact-green D02 99545c1.

A: Add backend and exact joint names to canonical immutable BuildDefinition; use one provisional_build factory for named later stages. Existing J1 factories keep explicit inputs.
B: Eight bespoke factories/types with parallel validation. Rejected: repeats identities and physical-vs-simulation checks.

Choose A. Later physical stages J1-J2 through J1-J6 plus tool build; nominal/measured simulation identities remain distinct. No definition claims commissioning. A measured-simulation NAME is a workflow target, not evidence all entries are measured. Each joint mapping/bus/ID/identity must be provided by caller through D01. Missing body mass/link length become explicit unknown Measurement values with units/source, never assumed nominal numbers. Preserve explicit declared/measured entries and provenance. Definition backend, exact ordered joint set and build identity must match BuildPlan. No runtime/owner/transport integration, passive bodies D04, calibration identity D05, invalidation D06.

Check all definitions, required inputs, unknown serialization roundtrip, geometry provenance, wrong backend/build/jointset, duplicate bus IDs; all D01/D02 regression. Complete full baseline and current fingerprints/captures before acceptance.
