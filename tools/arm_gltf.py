import base64
import json
from pathlib import Path

import numpy as np


def _nonnegative_int(value, description: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{description} must be a nonnegative integer')
    return value


def _finite_array(value, shape: tuple[int, ...], description: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as e:
        raise ValueError(f'{description} must contain finite numbers') from e
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f'{description} must have shape {shape} and contain finite numbers')
    return result


def load_gltf_mesh(gltf_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load mesh primitives from a single glTF 2.0 file.

    Returns:
        points: (N, 3) float32 array in mesh coordinates (meters).
        triangles: (M, 3) int64 array of vertex index triplets.
    """
    try:
        content = gltf_path.read_text(encoding='utf-8')
        g = json.loads(content)
    except Exception as e:
        raise ValueError(f"Failed to read glTF JSON from {gltf_path}: {e}") from e

    asset = g.get('asset') if isinstance(g, dict) else None
    if not isinstance(asset, dict) or asset.get('version') not in ('2.0', 2.0):
        raise ValueError(f"Unsupported glTF asset version in {gltf_path}")

    if g.get('extensionsRequired'):
        raise ValueError(f"Unsupported glTF required extensions in {gltf_path}: {g['extensionsRequired']}")

    buffers = g.get('buffers', [])
    if not isinstance(buffers, list):
        raise ValueError(f'glTF buffers must be an array in {gltf_path}')
    bufs: list[bytes] = []
    for b_idx, b in enumerate(buffers):
        if not isinstance(b, dict):
            raise ValueError(f'Invalid buffer {b_idx} in {gltf_path}')
        uri = b.get('uri')
        if not isinstance(uri, str) or not uri.startswith('data:'):
            raise ValueError(
                f"External buffers and network URIs are rejected in {gltf_path}: buffer {b_idx}"
            )
        if ';base64,' not in uri:
            raise ValueError(f"Buffer {b_idx} in {gltf_path} is not base64 encoded data URI")
        _, b64data = uri.split(';base64,', 1)
        try:
            buf_bytes = base64.b64decode(b64data, validate=True)
        except (ValueError, TypeError) as e:
            raise ValueError(f'Invalid base64 buffer {b_idx} in {gltf_path}') from e
        byte_length = _nonnegative_int(b.get('byteLength'), f'buffer {b_idx} byteLength')
        if len(buf_bytes) != byte_length:
            raise ValueError(
                f'Buffer {b_idx} byteLength mismatch in {gltf_path}: expected {byte_length}, got {len(buf_bytes)}'
            )
        bufs.append(buf_bytes)

    buffer_views = g.get('bufferViews', [])
    accessors = g.get('accessors', [])
    nodes = g.get('nodes', [])
    meshes = g.get('meshes', [])
    scenes = g.get('scenes', [])
    if not all(isinstance(items, list) for items in (buffer_views, accessors, nodes, meshes, scenes)):
        raise ValueError(f'glTF bufferViews, accessors, nodes, meshes, and scenes must be arrays in {gltf_path}')

    view_ranges: list[tuple[int, int, int, int]] = []
    for bv_idx, bv in enumerate(buffer_views):
        if not isinstance(bv, dict):
            raise ValueError(f'Invalid bufferView {bv_idx} in {gltf_path}')
        buf_idx = _nonnegative_int(bv.get('buffer'), f'bufferView {bv_idx} buffer')
        if buf_idx >= len(bufs):
            raise ValueError(f'Invalid buffer index {buf_idx} in bufferView {bv_idx}')
        offset = _nonnegative_int(bv.get('byteOffset', 0), f'bufferView {bv_idx} byteOffset')
        length = _nonnegative_int(bv.get('byteLength'), f'bufferView {bv_idx} byteLength')
        stride = 0
        if 'byteStride' in bv:
            stride = _nonnegative_int(bv['byteStride'], f'bufferView {bv_idx} byteStride')
            if stride == 0 or stride < 4 or stride > 252 or stride % 4:
                raise ValueError(f'Invalid byteStride {stride} in bufferView {bv_idx}')
        if offset + length > len(bufs[buf_idx]):
            raise ValueError(f'bufferView {bv_idx} exceeds buffer {buf_idx} bounds')
        view_ranges.append((buf_idx, offset, length, stride))

    def read_accessor(acc_idx: int, *, four_byte_aligned: bool = False) -> np.ndarray:
        acc_idx = _nonnegative_int(acc_idx, 'accessor index')
        if acc_idx >= len(accessors):
            raise ValueError(f"Accessor index out of range: {acc_idx}")
        a = accessors[acc_idx]
        if not isinstance(a, dict):
            raise ValueError(f'Invalid accessor {acc_idx}')
        if 'sparse' in a:
            raise ValueError(f'Sparse accessor {acc_idx} is unsupported')
        bv_idx = a.get('bufferView')
        if bv_idx is None:
            raise ValueError(f"Invalid bufferView reference in accessor {acc_idx}")
        bv_idx = _nonnegative_int(bv_idx, f'accessor {acc_idx} bufferView')
        if bv_idx >= len(buffer_views):
            raise ValueError(f"Invalid bufferView reference in accessor {acc_idx}")
        buf_idx, bv_offset, bv_len, byte_stride = view_ranges[bv_idx]
        buf = bufs[buf_idx]

        comp_type_map = {
            5120: np.int8,
            5121: np.uint8,
            5122: '<i2',
            5123: '<u2',
            5125: '<u4',
            5126: '<f4',
        }
        comp_type = _nonnegative_int(a.get('componentType'), f'accessor {acc_idx} componentType')
        if comp_type not in comp_type_map:
            raise ValueError(f"Unsupported componentType {comp_type} in accessor {acc_idx}")
        dtype = comp_type_map[comp_type]

        type_dim_map = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
        a_type = a.get('type')
        if a_type not in type_dim_map:
            raise ValueError(f"Unsupported accessor type {a_type} in accessor {acc_idx}")
        n = type_dim_map[a_type]
        count = _nonnegative_int(a.get('count'), f'accessor {acc_idx} count')
        a_offset = _nonnegative_int(a.get('byteOffset', 0), f'accessor {acc_idx} byteOffset')
        start_offset = bv_offset + a_offset
        component_bytes = np.dtype(dtype).itemsize
        elem_bytes = component_bytes * n
        if a_offset % component_bytes:
            raise ValueError(f'Accessor {acc_idx} byteOffset is not aligned to its component type')
        if start_offset % component_bytes:
            raise ValueError(f'Accessor {acc_idx} absolute byteOffset is not aligned to its component type')
        if four_byte_aligned and start_offset % 4:
            raise ValueError(f'Accessor {acc_idx} POSITION data must be 4-byte aligned')
        if a_offset > bv_len or start_offset > len(buf):
            raise ValueError(f'Accessor {acc_idx} byteOffset exceeds bufferView bounds')
        if byte_stride and byte_stride < elem_bytes:
            raise ValueError(f'byteStride {byte_stride} < element size {elem_bytes} in accessor {acc_idx}')

        if count == 0:
            return np.empty((0, n) if n > 1 else 0, dtype=dtype)

        if byte_stride and byte_stride != elem_bytes:
            total_needed = (count - 1) * byte_stride + elem_bytes
            if start_offset + total_needed > len(buf) or a_offset + total_needed > bv_len:
                raise ValueError(f"Accessor {acc_idx} bounds overflow with byteStride")
            shape = (count, n) if n > 1 else (count,)
            strides = (byte_stride, np.dtype(dtype).itemsize) if n > 1 else (byte_stride,)
            arr = np.ndarray(shape, dtype=dtype, buffer=buf, offset=start_offset, strides=strides)
        else:
            total_needed = count * elem_bytes
            if start_offset + total_needed > len(buf) or a_offset + total_needed > bv_len:
                raise ValueError(f"Accessor {acc_idx} bounds overflow")
            arr = np.frombuffer(buf, dtype=dtype, count=count * n, offset=start_offset)
            if n > 1:
                arr = arr.reshape(-1, n)

        if np.issubdtype(np.dtype(dtype), np.floating) and not np.all(np.isfinite(arr)):
            raise ValueError(f"Non-finite values (NaN/Inf) detected in accessor {acc_idx}")

        if 'min' in a:
            exp_min = np.array(a['min'], dtype=float)
            act_min = arr.min(axis=0) if arr.ndim > 1 else arr.min()
            if np.any(act_min < exp_min - 1e-4):
                raise ValueError(f"Accessor {acc_idx} values violate declared min bounds")
        if 'max' in a:
            exp_max = np.array(a['max'], dtype=float)
            act_max = arr.max(axis=0) if arr.ndim > 1 else arr.max()
            if np.any(act_max > exp_max + 1e-4):
                raise ValueError(f"Accessor {acc_idx} values violate declared max bounds")
        return arr

    parents = [0] * len(nodes)
    node_children: list[list[int]] = []
    local_matrices: list[np.ndarray] = []
    for node_idx, nd in enumerate(nodes):
        if not isinstance(nd, dict):
            raise ValueError(f'Invalid node {node_idx} in {gltf_path}')
        children = nd.get('children', [])
        if not isinstance(children, list):
            raise ValueError(f'Node {node_idx} children must be an array')
        valid_children = []
        for child_idx in children:
            child_idx = _nonnegative_int(child_idx, f'node {node_idx} child index')
            if child_idx >= len(nodes):
                raise ValueError(f'Node index out of range: {child_idx}')
            parents[child_idx] += 1
            if parents[child_idx] > 1:
                raise ValueError(f'Node {child_idx} has multiple parents')
            valid_children.append(child_idx)
        node_children.append(valid_children)

        local_m = np.eye(4, dtype=np.float64)
        if 'matrix' in nd:
            if any(key in nd for key in ('translation', 'rotation', 'scale')):
                raise ValueError(f'Node {node_idx} cannot combine matrix with TRS transforms')
            local_m = _finite_array(nd['matrix'], (16,), f'node {node_idx} matrix').reshape(4, 4).T
            if not np.array_equal(local_m[3], [0.0, 0.0, 0.0, 1.0]):
                raise ValueError(f'Node {node_idx} matrix must be affine')
        else:
            rotation = _finite_array(nd.get('rotation', [0.0, 0.0, 0.0, 1.0]), (4,), f'node {node_idx} rotation')
            if abs(float(np.linalg.norm(rotation)) - 1.0) > 1e-6:
                raise ValueError(f'Node {node_idx} rotation quaternion must be unit length')
            x, y, z, w = rotation
            local_m[:3, :3] = [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
            ]
            scale = _finite_array(nd.get('scale', [1.0, 1.0, 1.0]), (3,), f'node {node_idx} scale')
            if np.any(scale == 0.0):
                raise ValueError(f'Node {node_idx} scale components must be nonzero')
            translation = _finite_array(nd.get('translation', [0.0, 0.0, 0.0]), (3,), f'node {node_idx} translation')
            local_m[:3, :3] = local_m[:3, :3] @ np.diag(scale)
            local_m[:3, 3] = translation
        if not np.all(np.isfinite(local_m)):
            raise ValueError(f'Node {node_idx} transform is non-finite')
        if np.linalg.slogdet(local_m[:3, :3])[0] == 0.0:
            raise ValueError(f'Node {node_idx} transform is singular')
        local_matrices.append(local_m)
        if 'mesh' in nd:
            mesh_idx = _nonnegative_int(nd['mesh'], f'node {node_idx} mesh')
            if mesh_idx >= len(meshes):
                raise ValueError(f'Mesh index out of range: {mesh_idx}')

    state = [0] * len(nodes)

    def check_node_tree(node_idx: int):
        if state[node_idx] == 1:
            raise ValueError(f'Cycle detected in glTF node tree at node {node_idx}')
        if state[node_idx] == 2:
            return
        state[node_idx] = 1
        for child_idx in node_children[node_idx]:
            check_node_tree(child_idx)
        state[node_idx] = 2

    for node_idx in range(len(nodes)):
        check_node_tree(node_idx)

    if scenes:
        scene_idx = _nonnegative_int(g.get('scene', 0), 'glTF scene index')
        if scene_idx >= len(scenes) or not isinstance(scenes[scene_idx], dict):
            raise ValueError(f'Invalid scene index {scene_idx} in {gltf_path}')
        scene_roots = scenes[scene_idx].get('nodes', [])
        if not isinstance(scene_roots, list):
            raise ValueError(f'Scene {scene_idx} nodes must be an array')
    else:
        scene_roots = [idx for idx, count in enumerate(parents) if count == 0]
    root_nodes = []
    for root_node in scene_roots:
        root_node = _nonnegative_int(root_node, 'scene root node')
        if root_node >= len(nodes):
            raise ValueError(f'Invalid scene root node {root_node} in {gltf_path}')
        if parents[root_node] != 0:
            raise ValueError(f'Scene root node {root_node} has a parent')
        if root_node in root_nodes:
            raise ValueError(f'Duplicate scene root node {root_node}')
        root_nodes.append(root_node)

    parts: list[tuple[np.ndarray, np.ndarray]] = []

    def walk_node(node_idx: int, parent_m: np.ndarray):
        nd = nodes[node_idx]
        accum_m = parent_m @ local_matrices[node_idx]
        if not np.all(np.isfinite(accum_m)):
            raise ValueError(f'Accumulated transform is non-finite at node {node_idx}')
        determinant_sign = np.linalg.slogdet(accum_m[:3, :3])[0]
        if determinant_sign == 0.0:
            raise ValueError(f'Accumulated transform is singular at node {node_idx}')

        if 'mesh' in nd:
            mesh_idx = nd['mesh']
            mesh = meshes[mesh_idx]
            if not isinstance(mesh, dict) or not isinstance(mesh.get('primitives', []), list):
                raise ValueError(f'Invalid mesh {mesh_idx} in {gltf_path}')
            for p_idx, prim in enumerate(mesh.get('primitives', [])):
                if not isinstance(prim, dict):
                    raise ValueError(f'Invalid primitive {p_idx} in mesh {mesh_idx}')
                mode = _nonnegative_int(prim.get('mode', 4), f'mesh {mesh_idx} primitive {p_idx} mode')
                if mode != 4:
                    raise ValueError(
                        f'Unsupported primitive mode {mode} in mesh {mesh_idx} primitive {p_idx}; TRIANGLES (mode 4) required'
                    )
                attrs = prim.get('attributes', {})
                if not isinstance(attrs, dict):
                    raise ValueError(f'Invalid attributes in mesh {mesh_idx} primitive {p_idx}')
                if 'POSITION' not in attrs:
                    raise ValueError(f'Primitive {p_idx} missing POSITION attribute in mesh {mesh_idx}')
                pos_idx = _nonnegative_int(attrs['POSITION'], f'mesh {mesh_idx} primitive {p_idx} POSITION accessor')
                if pos_idx >= len(accessors) or not isinstance(accessors[pos_idx], dict):
                    raise ValueError(f'Invalid POSITION accessor in mesh {mesh_idx} primitive {p_idx}')
                pos_accessor = accessors[pos_idx]
                if pos_accessor.get('componentType') != 5126 or pos_accessor.get('type') != 'VEC3':
                    raise ValueError(f'POSITION accessor must be FLOAT VEC3 in mesh {mesh_idx} primitive {p_idx}')
                pos_arr = read_accessor(pos_idx, four_byte_aligned=True).astype(np.float64)

                if 'indices' in prim:
                    index_idx = _nonnegative_int(prim['indices'], f'mesh {mesh_idx} primitive {p_idx} indices accessor')
                    if index_idx >= len(accessors) or not isinstance(accessors[index_idx], dict):
                        raise ValueError(f'Invalid indices accessor in mesh {mesh_idx} primitive {p_idx}')
                    index_accessor = accessors[index_idx]
                    if index_accessor.get('componentType') not in (5121, 5123, 5125) or index_accessor.get('type') != 'SCALAR':
                        raise ValueError(f'Indices accessor must be unsigned SCALAR in mesh {mesh_idx} primitive {p_idx}')
                    flat_indices = read_accessor(index_idx)
                    if len(flat_indices) % 3:
                        raise ValueError(f'Index count in mesh {mesh_idx} primitive {p_idx} is not divisible by 3')
                    idx_arr = flat_indices.reshape(-1, 3).astype(np.int64)
                    if len(idx_arr) > 0 and idx_arr.max() >= len(pos_arr):
                        raise ValueError(
                            f'Triangle index out of bounds in mesh {mesh_idx} primitive {p_idx}: '
                            f'{idx_arr.max()} for {len(pos_arr)} vertices'
                        )
                else:
                    if len(pos_arr) % 3 != 0:
                        raise ValueError(f'Non-indexed vertex count {len(pos_arr)} not divisible by 3')
                    idx_arr = np.arange(len(pos_arr), dtype=np.int64).reshape(-1, 3)

                pos_trans = pos_arr @ accum_m[:3, :3].T + accum_m[:3, 3]
                if not np.all(np.isfinite(pos_trans)):
                    raise ValueError(f'Non-finite transformed positions in mesh {mesh_idx} primitive {p_idx}')
                if determinant_sign < 0.0:
                    idx_arr = idx_arr[:, [0, 2, 1]]
                parts.append((pos_trans, idx_arr))

        for child_idx in node_children[node_idx]:
            walk_node(child_idx, accum_m)

    for root_node in root_nodes:
        walk_node(root_node, np.eye(4, dtype=np.float64))

    if not parts or not any(len(indices) for _, indices in parts):
        raise ValueError(f'glTF contains no reachable triangle geometry in {gltf_path}')

    all_p = np.concatenate([p for p, _ in parts])
    offsets = np.cumsum([0] + [len(p) for p, _ in parts[:-1]])
    all_i = np.concatenate([i + off for (_, i), off in zip(parts, offsets)])
    points = all_p.astype(np.float32)
    if not np.all(np.isfinite(points)):
        raise ValueError(f'glTF positions cannot be represented as finite float32 values in {gltf_path}')
    return points, all_i.astype(np.int64)
