# Adversarial Code Review: M01 Arm Model Generator & D01 Contributor Documentation

**Review Scope:**
- [`tools/build_manual_arm.py`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py)
- [`tools/arm_gltf.py`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/arm_gltf.py)
- [`tools/manual-arm-requirements.txt`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/manual-arm-requirements.txt)
- [`tools/test_build_manual_arm.py`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/test_build_manual_arm.py)
- [`docs/arm-model.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/docs/arm-model.md)
- [`D01-edit/AGENTS.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/AGENTS.md)
- [`D01-edit/README.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/README.md)
- [`D01-edit/CONTRIBUTING.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/CONTRIBUTING.md)
- [`D01-edit/docs/development/coordination.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/docs/development/coordination.md)
- [`D01-edit/docs/development/layout-plan.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/docs/development/layout-plan.md)
- [`D01-edit/docs/development/tasks.tsv`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/docs/development/tasks.tsv)

---

## 1. Executive Assessment

While the repaired manual arm generator passes 28 unit tests and 355 standard forward kinematics and asset validation checks on the supplied `cad-export-v3` grounding data, **a passing test suite on a single fixed fixture has masked substantial mathematical, geometric, and architectural defects**.

Specifically:
1. The new LOD clustering logic unconditionally destroys surface topology on thin-walled and back-to-back geometries by sorting triangle vertex indices without regard to cyclic winding order or normal direction, leaving visual holes and orphaned vertices.
2. The custom glTF parser omits quaternion normalization, fails to correct triangle winding under reflection matrices, and ignores byte-offset alignment rules.
3. The generated `arm.urdf` violates physical simulation semantics by specifying `effort="0"` and `velocity="0"` on J1 (rendering it immovable in standard ROS/MoveIt planners) while retaining `type="continuous"` on J2 and J3 (claiming infinite collision-free rotation on joints that physically cannot rotate 360 degrees).
4. The contributor documentation and CI pipelines are decoupled: CI does not install the NumPy model dependencies or run model unit tests, `scripts/check_milestone.py` cannot run M01 checks, and ephemeral LLM session quota instructions leaked into project documentation.

---

## 2. Visualization Mesh LOD & Geometry Destruction

### 2.1 Destructive Vertex Sorting Destroys Opposite-Winding Faces & Thin-Walled Surfaces
In [`build_manual_arm.py:125-127`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L125-L127):
```python
    if len(remapped):
        _, first = np.unique(np.sort(remapped, axis=1), axis=0, return_index=True)
        remapped = remapped[np.sort(first)]
```

- **Defect:** `np.sort(remapped, axis=1)` sorts the three vertex indices of each triangle in scalar ascending order. A front-facing triangle `[0, 1, 2]` and a back-facing triangle `[0, 2, 1]` both sort to `[0, 1, 2]`. Consequently, `np.unique` treats them as duplicates and discards one.
- **Impact:** In CAD models, brackets, sheet-metal plates, and ribs have front and back faces. When clustered into 0.4 mm cells (`VISUAL_CLUSTER_CELL_METERS = 0.0004`), opposite vertices collapse into the same grid cells. This code permanently strips one side of the surface. If rendered with back-face culling (standard in WebGL, Three.js, and OpenGL), the geometry will be completely invisible from one side, resulting in gaping visual holes.
- **Misleading Documentation:** [`docs/arm-model.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/docs/arm-model.md) asserts that it *"remaps faces, removes index-degenerate faces, and keeps the first orientation of each duplicate face."* It does **not** preserve orientation: it treats opposite faces as duplicates, discarding valid opposing polygons based entirely on input file line order. Face deduplication must be based on cyclic permutation `[(v0, v1, v2), (v1, v2, v0), (v2, v0, v1)]`, not scalar sorting.

### 2.2 Unreferenced "Orphan" Centroids Injected into Output Metadata
In [`build_manual_arm.py:108-117`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L108-L117):
- `inverse` is computed over all occupied grid cells via `np.unique(grid, axis=0)`.
- `centroids` has length equal to the number of occupied cells.
- When degenerate triangles are filtered (`nondegenerate = (remapped[:, 0] != remapped[:, 1]) & ...`), any centroid whose incident triangles collapsed into degenerate lines or points is **never referenced** in `remapped`.
- `cluster_mesh_for_visualization` returns `centroids.astype(np.float32)` directly.
- **Impact:** `mesh_metadata` and [`model.json`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L295) declare `generated_vertices` equal to `len(centroids)`, which includes phantom unreferenced vertices that do not exist in the visual topology.

### 2.3 Unhandled Degeneracy in 1-Micron Deduplication of Binary Render Meshes
In [`build_manual_arm.py:441-447`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L441-L447):
```python
    q = np.round(T.reshape(-1, 3) * 1000.0).astype(np.int64)
    _, first, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    pos = T.reshape(-1, 3)[first].astype('<f4')
    idx_bytes = inv.reshape(-1).astype('<u4').tobytes()
```
- In Section 11, vertices are rounded to 1 micron (`* 1000.0`).
- If two vertices of any triangle in `T` are closer than 1 micron, `inv` assigns them identical indices.
- Unlike Section 5, Section 11 performs **no check for degenerate triangles** (`inv[:, 0] == inv[:, 1]`, etc.).
- **Impact:** Degenerate triangles with zero surface area are written into `meshes/*.bin`. When rendering in Three.js or calculating face normals, downstream consumers will divide by zero or process zero-area faces.

### 2.4 Rigid Hard-Coded LOD Limits Cause Brittle Failure
- `MAX_VISUAL_TRIANGLES = 50000` and `VISUAL_CLUSTER_CELL_METERS = 0.0004` are fixed constants.
- If an updated CAD export yields 50,001 triangles at 0.4 mm, [`build_manual_arm.py:279-284`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L279-L284) raises `ValueError` and terminates without attempting adaptive decimation or cell-size coarsening.

---

## 3. glTF 2.0 Parser Robustness & Transformation Errors

### 3.1 Unnormalized Quaternions Skew and Scale Node Transforms
In [`arm_gltf.py:188-196`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/arm_gltf.py#L188-L196):
```python
    rotation = _finite_array(nd.get('rotation', [0.0, 0.0, 0.0, 1.0]), (4,), f'node {node_idx} rotation')
    x, y, z, w = rotation
    local_m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
    ]
```
- **Defect:** This standard rotation matrix derivation strictly assumes a unit quaternion ($\|q\| = 1$). `arm_gltf.py` never validates that $\|q\| > 0$ or normalizes `rotation`.
- **Impact:** Any CAD exporter emitting unnormalized quaternions (or a degenerate `[0, 0, 0, 0]` vector) causes `local_m` to produce a non-orthogonal matrix that introduces artificial shearing, scaling, or distortion into link geometries.

### 3.2 Reflection Matrices Invert Triangle Winding and STL Normal Orientation
In [`arm_gltf.py:271-274`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/arm_gltf.py#L271-L274):
```python
    pos_trans = pos_arr @ accum_m[:3, :3].T + accum_m[:3, 3]
    if not np.all(np.isfinite(pos_trans)):
        raise ValueError(...)
    parts.append((pos_trans, idx_arr))
```
- **Defect:** If a glTF node contains a negative scale (e.g. mirrored part with `scale: [-1, 1, 1]`) or a transformation matrix with negative determinant ($\det(M) < 0$), spatial reflection reverses triangle winding order.
- **Impact:** `arm_gltf.py` does not check $\det(M)$ or flip `idx_arr` winding. When [`write_binary_stl`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L143-L148) computes surface normals via `np.cross(v1 - v0, v2 - v0)`, the calculated normals point **inward**, corrupting lighting, backface rendering, and raycasting.

### 3.3 Missing BufferView and Accessor Alignment Validation
- In [`arm_gltf.py:108-112`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/arm_gltf.py#L108-L112) and [`143-144`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/arm_gltf.py#L143-L144), `stride` is checked for divisibility by 4, but `bv_offset` and `a_offset` are **never validated for alignment**.
- Under the glTF 2.0 specification (§3.6.2.2), accessors and buffer views must align to multiples of their component type (4 bytes for float32/uint32). Misaligned byte offsets passed to `np.ndarray(..., buffer=...)` can cause bus errors or unaligned SIMD memory traps on strict architectures.

### 3.4 Unhandled Recursion in Node Graph Cycle Detection
- In [`arm_gltf.py:208-219`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/arm_gltf.py#L208-L219), `check_node_tree` uses recursive depth-first search.
- For deep assemblies with $> 1000$ hierarchical nodes, it raises an unhandled Python `RecursionError` rather than a caught `ValueError`.

---

## 4. Kinematics, URDF Invariants & Provenance Integrity

### 4.1 Incompatible URDF Limits: J1 `effort="0"` and `velocity="0"` Break Planners
In [`build_manual_arm.py:350-356`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L350-L356):
```python
    lim_elem = ET.SubElement(j_copy, 'limit', {
        'lower': str(-math.pi / 2.0),
        'upper': str(math.pi / 2.0),
        'effort': '0',
        'velocity': '0'
    })
```
- In standard URDF semantics, `velocity="0"` means maximum velocity is 0 rad/s, and `effort="0"` means maximum torque is 0 Nm.
- Physics engines and trajectory planners (MoveIt, PyBullet, Gazebo, Pinocchio, Drake) parsing this URDF will treat J1 as an unactuated, immovable joint or divide by zero during time-parameterization calculations.
- If limits are uncommissioned, standard URDF convention requires either setting realistic dummy ceilings or omitting velocity/effort constraints, rather than declaring the actuator incapable of motion.

### 4.2 J2 and J3 Emitted as `continuous` in Kinematic URDF
In [`build_manual_arm.py:358-360`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L358-L360):
- J2 (shoulder) and J3 (elbow) retain their CAD export `type="continuous"`.
- A continuous joint in URDF has $[-\infty, +\infty]$ travel without mechanical limits.
- The physical arm's shoulder and elbow have finite physical travel; rotating them continuously will cause mechanical collision and tear internal motor harnesses. Any planning or collision verification tool importing `arm.urdf` directly will incorrectly presume infinite travel is safe.

### 4.3 XML Node Duplication Risk on `revolute_2`
In [`build_manual_arm.py:349-356`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L349-L356):
```python
    for elem in joint:
        j_copy.append(elem)
    lim_elem = ET.SubElement(j_copy, 'limit', ...)
```
- All existing children of `joint` are copied into `j_copy`, followed by adding `lim_elem`. If a source URDF already had a `<limit>` tag on `revolute_2`, `arm.urdf` will contain duplicate `<limit>` elements, invalidating the XML schema.

### 4.4 Inconsistent Collision Geometry Attributes
- In visual elements ([line 330](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L330)): `scale` is copied (`scale = geom.find('mesh').get('scale', '1 1 1')`).
- In collision elements ([line 343](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L343)): `scale` is omitted entirely. While unity scale is verified upstream, dropping the scale attribute on collision shapes while keeping it on visual shapes creates an unnecessary schema divergence.

---

## 5. Material Mapping & Binary Asset Classification

### 5.1 Stale Configuration & Structural Frame Fallback to Hardware
In [`build_manual_arm.py:30-44`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/build_manual_arm.py#L30-L44):
```python
MATERIAL_MAP = {
    'NONE.gltf': 'motor',
    'Part_1.gltf': 'link',
    'Part_1_1.gltf': 'link',
    'Part_1_12.gltf': 'link',  # Dead: does not exist in v3 CAD export
    'Part_1_2.gltf': 'tube',
    'Part_1_6.gltf': 'frame',
    'Part_1_9.gltf': 'frame',
    'Part_1_10.gltf': 'frame',
    'Part_1_11.gltf': 'frame',
    ...
}
```
- `Part_1_12.gltf` is dead configuration from an obsolete CAD export.
- `Part_1_7.gltf` and `Part_1_8.gltf` exist in `cad-export-v3` but are omitted from `MATERIAL_MAP`.
- At line 406: `link_materials[n] = MATERIAL_MAP.get(mesh_name, 'hardware')`.
- Structural arm brackets (`Part_1_7` and `Part_1_8`) silently fall back to `'hardware'` and are bundled into `base.hardware.bin` and `j2.hardware.bin` instead of frame/link assets, distorting material groupings in the generated binaries.

---

## 6. Test Suite Isolation & CI Verification Blindspots

### 6.1 Disconnected Python Environments Break Contributor Execution
- [`D01-edit/README.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/README.md) and [`D01-edit/CONTRIBUTING.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/CONTRIBUTING.md) document installing `.venv` using [`requirements.lock`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/requirements.lock).
- `requirements.lock` contains PySide6 and pyserial, but **omits `numpy`**.
- `tools/build_manual_arm.py` requires `numpy==2.3.5` (pinned separately in [`tools/manual-arm-requirements.txt`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/manual-arm-requirements.txt)).
- A contributor following the project setup guide will immediately hit `ModuleNotFoundError: No module named 'numpy'` when running model tools or tests. Neither `README.md` nor `CONTRIBUTING.md` references `tools/manual-arm-requirements.txt`.

### 6.2 Zero CI Coverage for Arm Model Generation
- [`.github/workflows/offline-baseline.yml`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/.github/workflows/offline-baseline.yml) only installs `requirements.lock` and executes `scripts/check_offline_baseline.sh`.
- `scripts/check_offline_baseline.sh` does not run [`test_build_manual_arm.py`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/tools/test_build_manual_arm.py).
- **Impact:** Model generation code is completely unverified in CI. A breaking regression or syntax error in `build_manual_arm.py` or `arm_gltf.py` will still yield green CI badges.

### 6.3 Milestone Audit Script Cannot Run or Fingerprint M01
- [`CONTRIBUTING.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/CONTRIBUTING.md) instructs contributors to run milestone validation via `scripts/check_milestone.py`.
- [`scripts/check_milestone.py:10-30`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/M01-fixture-workbench/scripts/check_milestone.py#L10-L30) only accepts `B11..C15`. It rejects `M01`.
- `scripts/check_milestone.py:34` fingerprints only `*.py` in the repository root, ignoring the `tools/` directory entirely.

---

## 7. Coordination & Documentation Governance (`D01-edit`)

### 7.1 Ephemeral Session Quota Artifacts Embedded into Contributor Docs
In [`D01-edit/docs/development/coordination.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/docs/development/coordination.md):
> *"A scheduled quota wake does not prove a reset. Query fresh usage before resuming. At ten percent remaining prepare the checkpoint and job inventory; at two percent settle or stop owned writers safely and schedule the next normal reset. This run already consumed its one authorized reset credit. The two remaining credits are not authorized."*

- While `D01-parent-corrections.json` noted removing an "invented reset-credit tool reference", ephemeral AI runtime session quota rules were left baked directly into the repository's permanent contributor coordination guide. This creates confusion for human and external contributors.

### 7.2 High-Risk Task Collision on Shared Dispatch Layer (`ARM-A03`)
In [`D01-edit/docs/development/tasks.tsv`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/docs/development/tasks.tsv):
- Task `ARM-A03` assigns `station_step.py` to the Arm team.
- `station_step.py` is an active core module verified under milestone C15 and audited by `scripts/check_offline_baseline.sh`.
- Permitting the arm team to modify `station_step.py` directly violates the stated policy in `coordination.md` (*"Do not edit another contributor's directory, rename files an active worker needs, or overwrite unrelated work"*) and threatens regression of single-motor bench invariants.

### 7.3 Model-Coupled Task Identifiers in Shared Registry
- In `tasks.tsv`, tasks are named `OPUS-01` and `OPUS-02` while owner is listed as `unassigned`.
- [`AGENTS.md`](file:///home/sid/Documents/code/im6014Workbench-runs/arm-priority-20261009/D01-edit-workbench/AGENTS.md) explicitly mandates: *"Keep the operator's model sheet as the single routing configuration. Do not copy model choices into a second editable routing configuration."* Hardcoding vendor model names into shared repository task IDs violates this boundary discipline.

---

## 8. Summary of Findings & Action Items

| ID | Component | Severity | Issue Summary | Recommended Remediation |
|---|---|---|---|---|
| **M01-01** | `build_manual_arm.py` | **High** | Scalar vertex sorting in `cluster_mesh_for_visualization` discards opposite-facing polygons and thin walls. | Deduplicate triangles by cyclic permutation; preserve opposing face orientations. |
| **M01-02** | `arm_gltf.py` | **Medium** | Unnormalized quaternions and negative-determinant reflections distort transforms and invert STL normals. | Normalize quaternions; check matrix determinant and invert triangle indices when $\det(M) < 0$. |
| **M01-03** | `build_manual_arm.py` | **Medium** | J1 URDF has `effort="0"` / `velocity="0"`; J2/J3 are marked `continuous`. | Set realistic non-zero velocity/effort ceilings; constrain J2/J3 or document simulator incompatibility. |
| **M01-04** | `build_manual_arm.py` | **Low** | Centroid clustering leaves unreferenced orphan vertices in mesh metadata. | Filter vertex list to only those referenced by surviving triangles before computing counts and metadata. |
| **M01-05** | `build_manual_arm.py` | **Low** | 1-micron rounding in binary render packing leaves degenerate triangles. | Filter degenerate triangle indices before writing `meshes/*.bin`. |
| **M01-06** | `build_manual_arm.py` | **Low** | `MATERIAL_MAP` contains dead `Part_1_12` and omits `Part_1_7`/`Part_1_8` (falling back to hardware). | Remove `Part_1_12`; add `Part_1_7` and `Part_1_8` as frame components. |
| **D01-01** | `README.md` / CI | **High** | `requirements.lock` omits `numpy`; CI does not execute arm generator unit tests. | Document `manual-arm-requirements.txt` in `README.md` / `CONTRIBUTING.md`; add model test step to CI. |
| **D01-02** | `tasks.tsv` | **Medium** | `ARM-A03` assigns core `station_step.py` to Arm team. | Require arm dispatch guards to use extension adapters rather than mutating shared C15 core code. |
| **D01-03** | `coordination.md` | **Low** | Leaked AI session quota management text in contributor guide. | Remove quota reset instructions from `docs/development/coordination.md`. |
| **D01-04** | `tasks.tsv` | **Low** | Model-specific task identifiers (`OPUS-01`, `OPUS-02`). | Rename to generic role-based IDs (`REVIEW-01`, `DOC-01`). |