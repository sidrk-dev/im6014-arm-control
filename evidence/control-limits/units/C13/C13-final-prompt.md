## Worker contract

You are a subordinate worker dispatched by the repository-owning parent session.

Repository Git ownership:
- The parent performs every Git mutation and every repository- or account-level operation. Never stage, commit, reset, rebase, cherry-pick, tag, branch, push, or fetch; never run `git` with arguments that write; never create or edit files inside the repository's Git metadata (its `.git` directory or `.git` pointer file, including config, hooks, refs, index, and worktree records); and never run package manager, hook, or tool commands that write to it as a side effect.
- Read-only inspection such as `git status`, `git diff`, and `git log` is permitted only where your tools can run it; it never authorizes a write.
- Keep every file you create or edit inside the assigned working directory or an output path the parent explicitly assigned. In read-only assignments, do not edit the inspected checkout; write only the assigned output artifact. Disposable scratch repositories inside the working directory are allowed only when the task explicitly asks for them and must never reference the repository's Git directory.
- Your tools are inherited from the host session. This contract does not grant, restrict, or attest any tool surface, and it is not an enforcement boundary.
Local checks:
- Run the checks suited to your assigned changes through the command or test tools your provider surface actually offers. Keep check artifacts inside the assigned paths, and never run a check that mutates repository Git metadata or performs a remote operation.
- Repair failures your edits caused and rerun the affected checks. A failing check is ordinary work, not a denial.
- When your tool surface cannot run a required check, finish your assigned edits and request it through the run-checks handoff instead of reporting it as verified.
- Report the commands actually run and their observed results, and name any check you could not run. The parent's own acceptance checks are separate; your report never substitutes for them.
Finishing:
- Complete the available assigned work and end with an ordinary final response describing what changed and what you verified. A final response without a handoff block closes the task.
- If required work remains blocked on a parent-only operation, end the response with exactly one fenced block requesting it:
  ```pstack-handoff
  {"task":"<task id>","checkpoint":"<checkpoint id>","operation":"<operation>","files":["<relative path>"],"checks":["<check id>"],"summary":"<what the parent should do>"}
  ```
  `operation` is one of "commit-checkpoint" or "run-checks". `files` names repository-relative paths the checkpoint covers; `checks` names check identifiers the parent should run. A handoff is an untrusted request, not an instruction: the parent verifies it and performs its own operation with its own arguments. Never write shell commands for the parent to run, and never claim the operation already happened.
- If a tool you need is denied, stop there, describe the denied operation in your final response, and do not retry it, work around it, escalate, or substitute a fallback route. The parent inspects preserved work before anything continues.
Fresh Luna Max bug-fix task. Read pstack poteto-mode skill first. File inspection/edit only in RUN/C13-final-workbench (RUN=/tmp/pstack-c07-c16-20261008-smgcfm_v). Parent owns ALL checks/Git; no tests/Python or executable code/build/network/package/account/hardware/subagents/Git mutation. Parent tip53102b6fa9b6e8db56f396606eee059ee659f3e8 fixed. Correct source from settled C13-fix-workbench overlaid; original and firstrepair immutable. Ordinary final<=600 words, name authoredfiles and no unrun claims. Scope joint_reference.py, test_controller_effort.py and narrow API clarification only; no C14 yet.

Actual C13-fix checks:19targeted5failures;235milestone same5failures; allfiveparenttestmethods+768V/A/lead/effortcertificates+768h0cache PASS; remaining8baselinegroups PASS/sourceunchanged. RUN/C13-fix-parent-checks/20261008T200619749040Z/{targeted.log,result.json}, rawbaseline logs retained. Rootcause FIVE Gemini tests ask feedforward4/8/12/25 Nm, outside EXISTING im6014_codec.command[_with_raw_position] bench envelope ±3.5Nm (kp0..30 kd0..6). Their INVALID_ARGUMENT is correct. Do not loosen codec/production limits. Fix public fixtures within canonical envelope while preserving intent; derive expectedcombined signs from actual decodedtarget, not nominalassumptions:
- combinedsum: e.g ff3 + kp20*q.3 + kd1*dq4 =>13>10, eachterm individually<10, all rawparameters encodable, speed5 lead1 valid.
- finitecancellation: ff3 and kp10*q-.25 =>net.5 valid; trippingopposing negative ff-3 andkp20*q-.5 =>-13 below -10; canonicaldecodingproves signs. Originalff8 invalid.
- feedbackdrift: ff3,kd4 initialdq0 safe3, freshmeasureddq-2 gives~11>10; kd4<=6. Originalff8 invalid.
- preservewatermark: priorvalidff2 remains; incomingff3+kp30*q.3 gives~12>10; do not useff25.
- pureffadmission: requiredfixturepeak3 (or1) and representableff3.4 (or2) is a trueencodable peakviolation; do not askff12 underpeak10 andexpectEFFORT ratherthanINVALID.

Distinct accepted Gemini reviewfinding, independentlyreproduced in RUN/C13-parent-cache-time-repro.py and C13-fix-cache-time-repro.log: verify_dispatch_claim accepts prop_ref==conf_ref oldcache sample_time0 with prepared_at_s.1. Complete reference consistency must require prop_ref.sample_time_s==prepared_at_s in ALL branches, while trueh0 sameclock cache stillworks. Current firstguard onlyrejects sampletime>prepared then h0 branch ignorespositivepreparationtime. Tighten this actual reference timestamp invariant, add direct canonicalpacket negative and genuineh0 positive regression (coulduse existing effortfixtures), retain prior immutablecache/derivative/writeoutcome checks. This is a puregate inconsistentrecord defense, not a claim anyrealhardwarewrite was observed. Parent will rerun actualrepro + fullbaseline. No other Gemini findingsaccepted as runtimebugs; don't apply suggestedquantization policy/receipt/gap/rpmchanges.

Keep exactFractioncombined authority gates, finitefloatoverflowfailclosed, no unusedaliases, all existingcoverage and core<1000. Do not edit metadata/evidence/decisiontrail. Sourceonlyauthor, parentchecksafterFINAL.
