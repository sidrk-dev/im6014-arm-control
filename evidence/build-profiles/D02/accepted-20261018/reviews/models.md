# pstack model configuration

Provider-qualified per-role choices. Read the installed pstack provider-dispatch reference before dispatching. Every documented role remains present. Native aliases use the current Codex parent and retain its MCP access. Codex Sol stays at xhigh. Fixed-effort OpenCode and Antigravity routes use default.

# budget: large (xhigh)
# access: {"provider":"codex","funding":"included","capacity":"unknown","apiSpend":"deny","provenance":"user","plan":"ChatGPT Plus"}
# access: {"provider":"antigravity","funding":"included","capacity":"unknown","apiSpend":"deny","provenance":"user","plan":"Google $20 Pro plan"}
# access: {"provider":"opencode","funding":"included","capacity":"high","apiSpend":"approved","provenance":"user","plan":"Zen free models","note":"User expects unlimited free-model capacity; this is advisory and does not establish executable access or a charge guarantee."}
# continuation: {"on":["permission","handoff"],"max":2}

feature, refactoring: opencode:opencode/deepseek-v4.1-flash-free@default
bug-fix: opencode:opencode/deepseek-v4.1-flash-free@default
perf-issue: opencode:opencode/deepseek-v4.1-flash-free@default
hillclimb: opencode:opencode/deepseek-v4.1-flash-free@default
judgment and prose: codex:gpt-6.1-sol@xhigh
hardest tasks: codex:gpt-6.1-sol@xhigh
how explorer: opencode:opencode/deepseek-v4.1-flash-free@default
how explainer: codex:gpt-6.1-sol@xhigh
why investigators, synthesizer: inherit-parent
reflect tooling, judgment, divergent, synthesizer: inherit-parent
arena runners: opencode:opencode/deepseek-v4.1-flash-free@default, codex:gpt-6.1-sol@xhigh
arena cross-judge pool: antigravity:gemini-3.8-flash-high@default, opencode:opencode/muse-spark-1.3-contributor-free@default
swarm workers: opencode:opencode/deepseek-v4.1-flash-free@default
architect runners: opencode:opencode/deepseek-v4.1-flash-free@default, codex:gpt-6.1-sol@xhigh
interrogate reviewers: antigravity:gemini-3.8-flash-high@default, opencode:opencode/muse-spark-1.3-contributor-free@default

# OpenCode reviewer override
Muse reviewers use /home/sid/.local/bin/pstack-opencode-review with the complete supplied material inline, --model opencode/muse-spark-1.3-contributor-free, --api-spend approved, and unique prompt/output/receipt paths. Every tool request ends that review. All other external worker roles use installed provider-dispatch.

This sheet controls Codex-hosted worker routing. It does not replace the current chat model or install OpenCode as a supported Pstack parent. The separate proposed OpenCode coordinator uses opencode/deepseek-v4.1-flash-free. Codex Sol remains the quota-limited collaborator. No paid fallback or reset-credit redemption is configured.
