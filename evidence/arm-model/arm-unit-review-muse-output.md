## Findings

### 1. [critical] J1 conversion can emit duplicate `<limit>` elements
**Location**: `tools/build_manual_arm.py`, `build_manual_arm()` J1 branch (`revolute_2`)
**Finding**: Code copies every child of the source joint (`for elem in joint: j_copy.append(elem)`) then unconditionally appends a new `<limit>`. If the source `revolute_2` is already `revolute` with a `<limit>`, output has two `<limit>` elements.
**Evidence**: No removal/replacement of existing `limit`, `dynamics`, `calibration` before append. Test only does `j1.find('limit')`, which returns the first match, so a duplicate passes while the URDF is invalid/ambiguous — parser takes first, ignores the intended ±π/2, or rejects. The happy path only works because current v3 `revolute_2` happens to be `continuous` with no limit. This is exactly the “source happened to work” trap.
**Suggestion**: Remove existing `limit`/`dynamics`/`safety_controller` children before appending the canonical limit, and assert exactly one limit afterwards.

### 2. [critical] Cleaned URDF shares mutable `Element` objects with source tree
**Location**: `tools/build_manual_arm.py`, cleaned URDF generation (inertial, visual `origin`/`material`, joint children)
**Finding**: `link_copy.append(inertial)`, `v_copy.append(orig)`, `v_copy.append(mat)`, `j_copy.append(elem)` append the original `Element` objects, not copies. `ET.indent(cleaned_tree)` then mutates shared `text`/`tail`.
**Evidence**: `xml.etree` elements have no parent pointer; appending does not clone. Source and cleaned trees alias the same objects. Follow-on mutation of one view affects the other. The “unchanged origin” test compares `src_orig.get('xyz') == clean_orig.get('xyz')` on aliased objects, so it is vacuous — it cannot fail by construction. This directly defeats the stated “exported transform preserved” verification.
**Suggestion**: `copy.deepcopy()` every appended element.

### 3. [critical] glTF parser accepts non-unit quaternions and degenerate scales
**Location**: `tools/arm_gltf.py`, `load_gltf_mesh()` node TRS handling
**Finding**: Rotation is accepted as any finite 4-vector, scale as any finite 3-vector. No unit-length check, no zero/negative-scale rejection.
**Evidence**: Zero quaternion yields a zero rotation matrix and collapses all positions to translation without error. Zero scale silently produces degenerate triangles. Non-unit quaternion introduces unintended scale/shear. glTF requires normalized quaternions; fail-closed parser must reject. Current v3 happens to be valid, so tests never exercise this.
**Suggestion**: Reject non-normalized quaternions (e.g. `|norm-1| > 1e-6`) and zero scales; decide explicitly on mirrored (negative determinant) transforms.

### 4. [warning] `byteStride > 252` arbitrarily rejects valid glTF
**Location**: `tools/arm_gltf.py`, `bufferView` validation
**Finding**: `stride > 252` is rejected. glTF 2.0 has no 252-byte maximum; interleaved vertices with many attributes can legitimately exceed it.
**Evidence**: The bound is not in the spec and is not derived from v3 data. A future exporter with e.g. 64-byte strides fails closed for the wrong reason. Fail-closed is good, but the boundary is invented.
**Suggestion**: Enforce `stride % 4 == 0` and `stride >= elem_bytes` plus buffer-bounds checks; drop the 252 cap or ground it in a documented limit.

### 5. [warning] Archive member matching is still suffix-based, not exact
**Location**: `tools/build_manual_arm.py`, `archive_member()`
**Finding**: Match is `name == suffix or name.endswith('/' + suffix)`. A zip containing only `export/mainassem/urdf/mainassem.urdf` matches and is accepted, even though the extracted `source_dir` layout is `mainassem/urdf/...`.
**Evidence**: Docs claim “one unambiguous full relative path.” Code allows an arbitrary leading prefix. If the archive has a top-level folder, verification compares bytes of differently-rooted trees and can pass while provenance (`archive_file`, relative paths) is misleading. Triage item “exact relative ZIP paths” is only half-fixed.
**Suggestion**: Require exact equality with the expected relative path, or explicitly document and test the allowed prefix.

### 6. [warning] Empty geometry silently accepted
**Location**: `tools/arm_gltf.py`, end of `load_gltf_mesh()`; `tools/build_manual_arm.py` STL/bin paths
**Finding**: A glTF with no reachable meshes returns `(0,3)` arrays without error. Downstream `write_binary_stl` writes a 0-triangle STL and bin generation can emit 0-vertex/0-triangle entries.
**Evidence**: Missing geometry (wrong scene roots, stripped meshes) becomes a “successful” build with empty assets. No test asserts non-empty visuals for retained links. Fail-open on the most dangerous case — silent model disappearance.
**Suggestion**: Reject empty `parts` unless explicitly allowed, and assert every retained visual yields `>0` triangles.

### 7. [warning] Unbounded buffers / recursion — malicious or corrupt glTF DoS
**Location**: `tools/arm_gltf.py`, buffer decode + `walk_node()` recursion
**Finding**: No maximum `byteLength`, `count`, vertex/triangle cap before allocation; node traversal is recursive.
**Evidence**: `base64.b64decode` + `np.frombuffer`/`np.ndarray` allocate as declared. A corrupt `count: 2**31` with matching `byteLength` OOMs the converter. Deep `children` chains overflow Python recursion. Offline does not mean trusted — the input is a CAD export zip.
**Suggestion**: Add explicit byte/count caps grounded in v3 sizes, and make `walk_node` iterative or set a depth limit.

### 8. [warning] LOD dedup keeps arbitrary orientation for coincident faces
**Location**: `tools/build_manual_arm.py`, `cluster_mesh_for_visualization()`
**Finding**: `np.unique(np.sort(remapped, axis=1), ...)` treats opposite-winding duplicates as identical and keeps the first encountered orientation.
**Evidence**: Two-sided thin walls represented as coincident opposite faces lose one side with no warning. `max_vertex_displacement` does not capture this topological loss, and docs correctly say displacement is “not a Hausdorff bound,” but say nothing about winding loss. For visualization-only this may be acceptable, but it is an undocumented semantic change.
**Suggestion**: Document winding behavior or preserve both windings when normals oppose.

### 9. [warning] `MATERIAL_MAP` silent `hardware` fallback misclassifies structure
**Location**: `tools/build_manual_arm.py`, `MATERIAL_MAP` + `link_materials[n] = MATERIAL_MAP.get(mesh_name, 'hardware')`
**Finding**: Any mesh filename not in the 12-entry hint map becomes `hardware`, determining `meshes/{body}.{mat}.bin` grouping.
**Evidence**: New fasteners, revised part names, or the `Part_1_12`/`Part_1_7`-style overflow silently lump structural links/frames into `hardware`. Downstream render consumers cannot distinguish “intentionally hardware” from “unknown.” Triage dismissed this as “visualization color,” but bin grouping is now part of the schema contract.
**Suggestion**: Warn/log unmapped meshes in `model.json` provenance, or fail on unknown visual stems.

### 10. [warning] World-floor derivation assumes non-empty base, obscure failure otherwise
**Location**: `tools/build_manual_arm.py`, `base_pts_cad_mm = np.concatenate(base_source_triangles_cad_mm)`
**Finding**: No guard for empty `base_source_triangles_cad_mm` or empty `link_tris_cad_mm[n]`.
**Evidence**: If base visuals were ever all dropped or a link had no visuals, `np.concatenate([])` raises `ValueError: need at least one array` — unrelated to the real invariant. Edge case is reachable if `none_2` handling changes.
**Suggestion**: Explicit `if not base_source_triangles_cad_mm: raise ValueError(...)` with a domain message.

### 11. [warning] README contradicts itself on J1 zero + asserts unverified physical facts
**Location**: `D01-edit/README.md`, “Arm model and commissioning”
**Finding**: “initially -90 to +90 around commissioned zero” vs. same section “IDs, direction, zero, gains, timeout, and loaded-arm behavior are not physically verified.” Both cannot hold. “J2's obstructing screw was removed” is a physical-world claim with no hardware evidence in scope (`physicalIO: none`).
**Evidence**: Intent demands truthful documented claims and “do not infer physical safety from green checks.” A reader planning wire management or commissioning will read “commissioned zero” and “screw was removed” as established facts.
**Suggestion**: Say “provisional zero, uncommissioned” and either ground the screw claim with a dated photo/work order reference or remove it.

### 12. [warning] Coordination doc retains reset-credit language claimed as removed
**Location**: `D01-edit/docs/development/coordination.md`, “Hand off actual state”
**Finding**: Paragraph on “scheduled quota wake,” “ten percent / two percent,” “already consumed its one authorized reset credit. The two remaining credits are not authorized” survives, while `D01-parent-corrections.json` claims “Removed invented reset-credit tool reference.”
**Evidence**: The correction log and the actual file disagree. The quota/reset model is provider-specific, unverifiable from the repo, and will go stale. It also reads as an instruction to future agents.
**Suggestion**: Delete the quota/reset paragraph or replace with “query fresh usage; checkpoint before stopping” with no invented credit counts.

### 13. [warning] Task-collision guard is by convention, not structure
**Location**: `D01-edit/docs/development/tasks.tsv` + `coordination.md`
**Finding**: M01, ARM-A02/A03/A04, OPUS-01/02, REFACTOR-01 share `base_sha 9e03d9...` and overlapping doc-adjacent paths. Separation (“distinct checkout,” “coordinator is single writer,” “scope must be accepted”) is prose.
**Evidence**: Nothing prevents OPUS-02 (`docs/user/arm-jogging.md`) from being written before ARM-A04 exists — the very “do not document planned controls as working” hazard the registry names. Nothing prevents two workers from both branching off a stale base after M01 lands. The “keep integration head fixed” rule serializes all work without a lock or check.
**Suggestion**: Make `tasks.tsv` status transitions machine-checked (blocked until dependency landed), and add a CI check that `docs/user/arm-jogging.md` references only existing launcher/UI paths.

### 14. [nit] Tests over-fit to exact v3 numbers, brittle maintenance
**Location**: `tools/test_build_manual_arm.py`, `test_hash_and_provenance`, `test_dense_visual_lod...`
**Finding**: Hardcoded `archive_sha256`, `urdf_sha256`, `source_triangles == 1226332`, `displacement == 0.0004566... ±1e-7`, `yaw == 3.35 ±0.1`.
**Evidence**: Any legitimate re-export (new CAD revision, different compression, LOD grid tweak) breaks a dozen assertions that test identity, not behavior. Docs say the converter “does not enforce a hard-coded archive digest,” yet the test suite does.
**Suggestion**: Keep one explicit “pinned v3 identity” test, and convert the rest to behavioral bounds (`<=50000`, `displacement <= cell*sqrt(3)`, `yaw < pos_threshold_derived`).

### 15. [nit] `get_body` encodes joint order in list position
**Location**: `tools/build_manual_arm.py`, `RobotModel.get_body()`
**Finding**: Returns `f'j{actuated_joints.index(j_name)+1}'`, so `['revolute_2','revolute_1','revolute_3']` order defines j1/j2/j3. `joint_specs` separately maps `motor_id 0/1/2` to the same order.
**Evidence**: Reordering `actuated_chain` silently reassigns bodies and motor IDs with no type error. Two parallel orderings must stay in sync by inspection.
**Suggestion**: Single canonical `ORDERED_JOINTS = [('revolute_2','J1 base',0), ...]` dict, derive both body map and `model.json` joints from it.
