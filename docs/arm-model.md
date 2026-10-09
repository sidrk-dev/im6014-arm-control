# Manual arm model generator

`tools/build_manual_arm.py` converts the v3 CAD export into a kinematic URDF, portable STL visualization meshes, binary rendering meshes, and `model.json`. It is an offline model converter. The generated `arm.urdf` is not validated for physics simulation or motion planning. The output does not establish physical clearance, collision validity, dynamics, calibration, simulation effort limits, or live motor command authority.

## Run

Create a separate converter environment from the workbench root:

```bash
python3 -m venv .model-venv
.model-venv/bin/python -m pip install --only-binary=:all: -r tools/manual-arm-requirements.txt
.model-venv/bin/python -m pip check
```

Provide the extracted source, original archive, and a fresh empty output directory:

```bash
.model-venv/bin/python tools/build_manual_arm.py \
  --source _arm_inputs/cad-export-v3 \
  --archive "_arm_inputs/mainAssem (3).zip" \
  --output arm_assets
```

The output directory must be empty or absent. The converter does not overwrite existing files. Its only dependency is the pinned NumPy version in `tools/manual-arm-requirements.txt`; dashboard code does not import converter dependencies.

## Accepted input

- The source has one URDF at `mainassem/urdf/mainassem.urdf`, with 76 links and 75 joints. The URDF graph must have unique names, known parent and child links, one parent per child, no cycles, and one reachable root. Joint types are limited to `fixed`, `revolute`, and `continuous`.
- The active joint order is `revolute_2`, `revolute_1`, `revolute_3`. `revolute_4` is the only passive moving joint. Other source joints are fixed.
- Mesh references must use `package://mainassem/meshes/<file>`, resolve inside the source package, and match one unambiguous exact source-relative path in the archive byte for byte. Duplicate ZIP paths, ambiguous source URDFs, other package names, and collision-only meshes fail explicitly.
- URDF visual and collision shapes must be meshes with unity scale. A link may have at most one visual element. Other geometry forms and multiple visuals are unsupported.
- glTF buffers must be embedded base64 data. Sparse accessors are unsupported; positions must use aligned `FLOAT VEC3`, indices must use aligned unsigned `SCALAR`, and counts, offsets, strides, buffer bounds, node graphs, scene roots, and node transforms are validated before geometry is read. `byteStride` must be a multiple of four from 4 through 252 bytes, matching the glTF 2.0 schema. Quaternions must be unit length; matrices must be affine and nonsingular; mirrored transforms are supported with triangle winding preserved. Inputs without reachable triangles fail explicitly.

`model.json` records SHA-256 digests for the archive and source URDF supplied to the converter. The converter compares each selected URDF and mesh file byte for byte with its archive entry; it does not enforce a hard-coded archive digest. The independent M01 source check verifies that the supplied inputs match the expected v3 source.

## Generated model

- `arm.urdf` preserves the 76-link, 75-joint structure and source joint origins. `revolute_4` becomes fixed with its origin preserved. J1 (`revolute_2`) has one canonical ±π/2 display bound for user-directed wire management, centered on a provisional, uncommissioned zero. Its URDF effort and velocity values are zero display placeholders, not physics or planning parameters or actuator ceilings; do not infer or guess numeric ceilings from them. J2 and J3 retain their continuous CAD types; their actual physical travel limits are commissioning-only.
- The fixed-base motor visual `none_2` is retained. The moving duplicate `none_1` visual is removed only when its source mesh matches `none_2`, its source bytes match the archive, its CAD position is within 1 mm, and the URDF graph places it on J1 while `none_2` belongs to the base. The link and joints remain in the URDF. Provenance records the actual source parent names and transforms.
- The `joints` array is the canonical ordered joint list in `model.json`. Joint origins and axes are derived from the source frames. The world floor uses full-resolution retained base visual geometry; display LOD never determines the world origin.
- STL files use binary STL in meters. Their `model.json` entries and the `cleaned_urdf` entry declare `units: "m"`. The top-level `units: "mm"` describes joint origins and binary rendering positions; binary rendering files store little-endian float32 positions in millimeters followed by uint32 triangle indices, grouped by body and material. All generated mesh paths are relative to `arm.urdf`.
- Binary rendering meshes round vertices to a 1-micron grid, remove triangles whose indices collapse after rounding, compact unused vertices, and report counts for the packed output. Material names are display classifications; unmapped source mesh names are listed in `cleanup_provenance` and use the `hardware` fallback without assigning a mechanical role.

For a source mesh with more than 50,000 triangles, display LOD clusters the original vertices by `floor(point / 0.0004 m)`, replaces each occupied cell with its vertex centroid, remaps faces, removes index-degenerate faces, and removes duplicate faces with the same winding while preserving opposite winding. Small source meshes remain unchanged. The converter rejects any nonempty dense mesh whose generated count is zero or exceeds 50,000 before it creates output files. This cap applies per unique source mesh; body render meshes may contain multiple instances of that mesh.

Each LOD mesh records source and generated vertex/triangle counts, the method and grid size, and the measured maximum source-vertex-to-output-centroid displacement. This vertex displacement is not a Hausdorff bound or a collision-clearance guarantee. `model.json` keeps `placeholder_inertials: true`, `dynamics_validated: false`, and `collision_validated: false`.

## Input-dependent checks

The generator tests include synthetic glTF and URDF cases that run without the ignored `_arm_inputs` directory. Tests that build against the retained v3 archive and compare source frames, hashes, and geometry skip when those inputs are unavailable. The parent M01 verification supplies them explicitly and runs the full-source checks.

Run converter tests with `.model-venv/bin/python -B -m unittest -v tools.test_build_manual_arm`. CI runs synthetic parser, mesh and URDF checks in its own converter job. Source-dependent tests skip in CI because the private raw export is retained outside the repository. The local M01 certificate includes the full retained v3 build and source comparisons.
