# Combined C11–C16 software review correction

The combined review found a test that changed feedback before planning and never exercised the final measured-lead veto. The corrected test first requires a real planned candidate, then supplies fresh shifted feedback before release. Zero gains isolate the lead guard. It verifies no issue, retained target/ownership, and recovery equivalent to an uninterrupted control core.

The corrected source passes 164 controller tests, all nine offline baseline groups (256 milestone tests and 19 legacy Qt groups), and 79 separately guarded arm tests. These groups overlap and are not summed. Removing only the final lead veto fails this test at the issue assertion. Four saved legacy Qt captures were inspected; both disconnected startup modes match 39 source hashes and record zero opens/exchanges. This is not a new arm-dashboard visual verification or physical commissioning.

The original combined review and its correction addendum are preserved. Gemini completed with no findings; factual prose corrections appear in review-triage.json. Muse HTTP 400 is a dropout, not approval. The earlier bad test-module invocation and sandbox baseline failures are retained. Host baseline passes without changing authority or hardware boundaries.

Runtime code is byte-identical to 71a0a5f. This packet establishes the combined software source review after correcting its single verification gap. PR10 dependency handling, exact corrected-head CI, guarded merge, deployment and physical tests remain separate gates. JointCore is still transport-free and separate from the production manual owner.
