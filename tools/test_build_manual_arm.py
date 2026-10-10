#!/usr/bin/env python3

import base64
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))
if str(WORKSPACE_ROOT / 'tools') not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT / 'tools'))

from tools.build_manual_arm import (
    build_manual_arm,
    load_gltf_mesh,
    write_binary_stl,
    cluster_mesh_for_visualization,
    _archive_member,
    _copy_j1_with_display_limit,
    _pack_millimeter_render_mesh,
    RobotModel,
    rpy,
    pose,
    R_UP,
)


def make_synthetic_gltf(
    positions: np.ndarray,
    indices: np.ndarray = None,
    node_matrix: list[float] = None,
    node_trs: dict = None,
    byte_stride: int = 0,
    component_type: int = 5126,
    position_type: str = 'VEC3',
    index_component_type: int = 5125,
    buffer_uri: str = None,
    primitive_mode: int = 4
) -> dict:
    pos_data = positions.astype('<f4')
    num_verts = len(pos_data)

    if byte_stride > 0 and byte_stride > 12:
        record_dtype = np.dtype([('pos', '<f4', (3,)), ('pad', f'V{byte_stride - 12}')])
        records = np.zeros(num_verts, dtype=record_dtype)
        records['pos'] = pos_data
        pos_bytes = records.tobytes()
        bv_pos_stride = byte_stride
    else:
        pos_bytes = pos_data.tobytes()
        bv_pos_stride = None

    idx_bytes = b''
    if indices is not None:
        idx_dtype = {5121: 'u1', 5123: '<u2', 5125: '<u4'}.get(index_component_type, '<u4')
        idx_data = indices.astype(idx_dtype)
        idx_bytes = idx_data.tobytes()

    combined_buf = pos_bytes + idx_bytes
    if buffer_uri is None:
        b64 = base64.b64encode(combined_buf).decode('ascii')
        buf_uri = f"data:application/octet-stream;base64,{b64}"
    else:
        buf_uri = buffer_uri

    buffer_views = [
        {
            "buffer": 0,
            "byteOffset": 0,
            "byteLength": len(pos_bytes),
        }
    ]
    if bv_pos_stride:
        buffer_views[0]["byteStride"] = bv_pos_stride

    accessors = [
        {
            "bufferView": 0,
            "byteOffset": 0,
            "componentType": component_type,
            "count": num_verts,
            "type": position_type,
            "min": [float(x) for x in pos_data.min(axis=0)] if num_verts > 0 else [0, 0, 0],
            "max": [float(x) for x in pos_data.max(axis=0)] if num_verts > 0 else [0, 0, 0],
        }
    ]

    attributes = {"POSITION": 0}
    primitives = [{"attributes": attributes, "mode": primitive_mode}]

    if indices is not None:
        buffer_views.append({
            "buffer": 0,
            "byteOffset": len(pos_bytes),
            "byteLength": len(idx_bytes),
        })
        accessors.append({
            "bufferView": 1,
            "byteOffset": 0,
            "componentType": index_component_type,
            "count": len(indices.reshape(-1)),
            "type": "SCALAR",
            "min": [int(indices.min())] if len(indices) > 0 else [0],
            "max": [int(indices.max())] if len(indices) > 0 else [0],
        })
        primitives[0]["indices"] = 1

    node_dict = {"mesh": 0}
    if node_matrix is not None:
        node_dict["matrix"] = node_matrix
    elif node_trs is not None:
        node_dict.update(node_trs)

    return {
        "asset": {"version": "2.0"},
        "buffers": [{"byteLength": len(combined_buf), "uri": buf_uri}],
        "bufferViews": buffer_views,
        "accessors": accessors,
        "meshes": [{"primitives": primitives}],
        "nodes": [node_dict],
        "scenes": [{"nodes": [0]}],
        "scene": 0
    }


class TestSyntheticGltfAndValidation(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp(prefix='m01_synth_'))

    def tearDown(self):
        shutil.rmtree(self.tmp_dir, ignore_errors=True)

    def test_gltf_local_scene_transforms(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        tris = np.array([[0, 1, 2]], dtype=np.int64)

        trs = {
            "translation": [10.0, 20.0, 30.0],
            "scale": [2.0, 2.0, 2.0]
        }
        gltf_data = make_synthetic_gltf(verts, tris, node_trs=trs)
        gltf_path = self.tmp_dir / 'test_trs.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')

        pts_out, tris_out = load_gltf_mesh(gltf_path)
        self.assertEqual(len(pts_out), 3)
        self.assertEqual(len(tris_out), 1)

        np.testing.assert_allclose(pts_out[0], [10.0, 20.0, 30.0], rtol=1e-5)
        np.testing.assert_allclose(pts_out[1], [12.0, 20.0, 30.0], rtol=1e-5)
        np.testing.assert_allclose(pts_out[2], [10.0, 22.0, 30.0], rtol=1e-5)

    def test_gltf_column_major_matrix_transform(self):
        verts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
        tris = np.array([[0, 0, 0]], dtype=np.int64)
        col_major_mat = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            5.0, 7.0, 0.0, 1.0
        ]
        gltf_data = make_synthetic_gltf(verts, tris, node_matrix=col_major_mat)
        gltf_path = self.tmp_dir / 'test_mat.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')

        pts_out, _ = load_gltf_mesh(gltf_path)
        np.testing.assert_allclose(pts_out[0], [6.0, 7.0, 0.0], rtol=1e-5)

    def test_gltf_rejects_invalid_quaternions_and_singular_or_nonaffine_matrices(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        tris = np.array([[0, 1, 2]], dtype=np.int64)
        singular_matrix = [
            0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ]
        nonaffine_matrix = [
            1.0, 0.0, 0.0, 0.25,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ]
        cases = (
            (make_synthetic_gltf(verts, tris, node_trs={'rotation': [0.0, 0.0, 0.0, 0.0]}), 'unit length'),
            (make_synthetic_gltf(verts, tris, node_trs={'rotation': [0.0, 0.0, 0.0, 2.0]}), 'unit length'),
            (make_synthetic_gltf(verts, tris, node_trs={'scale': [0.0, 1.0, 1.0]}), 'must be nonzero'),
            (make_synthetic_gltf(verts, tris, node_matrix=singular_matrix), 'singular'),
            (make_synthetic_gltf(verts, tris, node_matrix=nonaffine_matrix), 'must be affine'),
        )
        for index, (gltf_data, error) in enumerate(cases):
            with self.subTest(error=error):
                gltf_path = self.tmp_dir / f'test_invalid_transform_{index}.gltf'
                gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')
                with self.assertRaises(ValueError) as cm:
                    load_gltf_mesh(gltf_path)
                self.assertIn(error, str(cm.exception))

    def test_gltf_mirrored_transforms_preserve_triangle_orientation(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        tris = np.array([[0, 1, 2]], dtype=np.int64)
        mirrored_matrix = [
            -1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ]
        cases = (
            make_synthetic_gltf(verts, tris, node_trs={'scale': [-1.0, 1.0, 1.0]}),
            make_synthetic_gltf(verts, tris, node_matrix=mirrored_matrix),
        )
        for index, gltf_data in enumerate(cases):
            gltf_path = self.tmp_dir / f'test_mirrored_{index}.gltf'
            gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')
            points, faces = load_gltf_mesh(gltf_path)
            normal = np.cross(points[faces[0, 1]] - points[faces[0, 0]], points[faces[0, 2]] - points[faces[0, 0]])
            self.assertGreater(normal[2], 0.0)

    def test_gltf_rejects_misaligned_buffer_and_accessor_offsets(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        tris = np.array([[0, 1, 2]], dtype=np.int64)

        def canceling_position_offsets(data):
            buffer_uri = data['buffers'][0]['uri']
            buffer_bytes = base64.b64decode(buffer_uri.split(',', 1)[1])
            shifted_bytes = b'\0' * 4 + buffer_bytes
            data['buffers'][0]['byteLength'] = len(shifted_bytes)
            data['buffers'][0]['uri'] = (
                'data:application/octet-stream;base64,'
                + base64.b64encode(shifted_bytes).decode('ascii')
            )
            data['bufferViews'][0].update({'byteOffset': 2, 'byteLength': 38})
            data['accessors'][0]['byteOffset'] = 2
            data['bufferViews'][1]['byteOffset'] += 4

        cases = (
            (make_synthetic_gltf(verts, tris), lambda data: data['accessors'][0].update({'byteOffset': 2})),
            (make_synthetic_gltf(verts, tris, index_component_type=5123), lambda data: data['accessors'][1].update({'byteOffset': 1})),
            (make_synthetic_gltf(verts, tris), lambda data: data['bufferViews'][0].update({'byteOffset': 2})),
            (make_synthetic_gltf(verts, tris), canceling_position_offsets),
        )
        for index, (gltf_data, mutate) in enumerate(cases):
            with self.subTest(offset_case=index):
                mutate(gltf_data)
                gltf_path = self.tmp_dir / f'test_alignment_{index}.gltf'
                gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')
                with self.assertRaises(ValueError) as cm:
                    load_gltf_mesh(gltf_path)
                self.assertRegex(str(cm.exception), 'align')

    def test_gltf_requires_reachable_triangle_geometry(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        cases = (
            make_synthetic_gltf(verts, np.empty((0, 3), dtype=np.int64)),
            make_synthetic_gltf(verts),
        )
        cases[1]['nodes'] = [{}]
        for index, gltf_data in enumerate(cases):
            gltf_path = self.tmp_dir / f'test_empty_geometry_{index}.gltf'
            gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'no reachable triangle geometry'):
                load_gltf_mesh(gltf_path)

    def test_gltf_byte_stride_keeps_schema_252_byte_limit(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        valid = make_synthetic_gltf(verts, byte_stride=252)
        gltf_path = self.tmp_dir / 'test_stride_252.gltf'
        gltf_path.write_text(json.dumps(valid), encoding='utf-8')
        points, faces = load_gltf_mesh(gltf_path)
        self.assertEqual(points.shape, (3, 3))
        self.assertEqual(faces.shape, (1, 3))

        invalid = make_synthetic_gltf(verts, byte_stride=252)
        invalid['bufferViews'][0]['byteStride'] = 256
        gltf_path.write_text(json.dumps(invalid), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'Invalid byteStride 256'):
            load_gltf_mesh(gltf_path)

    def test_gltf_byte_stride_interleaved(self):
        verts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
        tris = np.array([[0, 1, 0]], dtype=np.int64)
        gltf_data = make_synthetic_gltf(verts, tris, byte_stride=24)
        gltf_path = self.tmp_dir / 'test_stride.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')

        pts_out, tris_out = load_gltf_mesh(gltf_path)
        self.assertEqual(len(pts_out), 2)
        np.testing.assert_allclose(pts_out[0], [1.0, 2.0, 3.0], rtol=1e-5)
        np.testing.assert_allclose(pts_out[1], [4.0, 5.0, 6.0], rtol=1e-5)

    def test_gltf_interleaved_accessor_needs_no_final_padding(self):
        verts = np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ], dtype=np.float32)
        gltf_data = make_synthetic_gltf(verts, byte_stride=24)
        buffer_uri = gltf_data['buffers'][0]['uri']
        buffer_bytes = base64.b64decode(buffer_uri.split(',', 1)[1])
        buffer_bytes = buffer_bytes[:-12]
        gltf_data['buffers'][0]['byteLength'] = len(buffer_bytes)
        gltf_data['buffers'][0]['uri'] = (
            'data:application/octet-stream;base64,'
            + base64.b64encode(buffer_bytes).decode('ascii')
        )
        gltf_data['bufferViews'][0]['byteLength'] = len(buffer_bytes)

        gltf_path = self.tmp_dir / 'test_stride_no_final_padding.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')
        pts_out, tris_out = load_gltf_mesh(gltf_path)

        np.testing.assert_allclose(pts_out, verts, rtol=1e-5)
        np.testing.assert_array_equal(tris_out, [[0, 1, 2]])

    def test_gltf_rejects_nonfinite(self):
        verts = np.array([[0.0, float('nan'), 1.0]], dtype=np.float32)
        gltf_data = make_synthetic_gltf(verts)
        gltf_path = self.tmp_dir / 'test_nan.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')

        with self.assertRaises(ValueError) as cm:
            load_gltf_mesh(gltf_path)
        self.assertIn('Non-finite', str(cm.exception))

    def test_gltf_rejects_nonfinite_node_transform(self):
        verts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        gltf_data = make_synthetic_gltf(verts, node_trs={'translation': [float('inf'), 0.0, 0.0]})
        gltf_path = self.tmp_dir / 'test_node_inf.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')

        with self.assertRaises(ValueError) as cm:
            load_gltf_mesh(gltf_path)
        self.assertIn('translation', str(cm.exception))

    def test_gltf_requires_position_float_vec3_and_unsigned_scalar_indices(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        tris = np.array([[0, 1, 2]], dtype=np.int64)
        cases = (
            (make_synthetic_gltf(verts, tris, component_type=5123), 'POSITION accessor must be FLOAT VEC3'),
            (make_synthetic_gltf(verts, tris, index_component_type=5126), 'Indices accessor must be unsigned SCALAR'),
        )
        for index, (gltf_data, error) in enumerate(cases):
            with self.subTest(error=error):
                gltf_path = self.tmp_dir / f'test_accessor_type_{index}.gltf'
                gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')
                with self.assertRaises(ValueError) as cm:
                    load_gltf_mesh(gltf_path)
                self.assertIn(error, str(cm.exception))

    def test_gltf_rejects_sparse_accessors_and_invalid_integer_fields(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        mutations = (
            (lambda data: data['accessors'][0].update({'sparse': {'count': 1}}), 'Sparse accessor'),
            (lambda data: data['accessors'][0].update({'count': 2.5}), 'count must be a nonnegative integer'),
            (lambda data: data['accessors'][0].update({'byteOffset': -1}), 'byteOffset must be a nonnegative integer'),
            (lambda data: data['bufferViews'][0].update({'byteLength': 10 ** 6}), 'exceeds buffer'),
            (lambda data: data['bufferViews'][0].update({'byteStride': 3}), 'Invalid byteStride'),
        )
        for index, (mutate, error) in enumerate(mutations):
            with self.subTest(error=error):
                gltf_data = make_synthetic_gltf(verts)
                mutate(gltf_data)
                gltf_path = self.tmp_dir / f'test_accessor_bounds_{index}.gltf'
                gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')
                with self.assertRaises(ValueError) as cm:
                    load_gltf_mesh(gltf_path)
                self.assertIn(error, str(cm.exception))

    def test_gltf_rejects_cycles_multiple_parents_and_invalid_roots(self):
        verts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        cases = (
            ([{'children': [0]}], [{'nodes': [0]}], 'Cycle detected'),
            ([{'children': [2]}, {'children': [2]}, {'mesh': 0}], [{'nodes': [0, 1]}], 'multiple parents'),
            ([{'mesh': 0}], [{'nodes': [2]}], 'Invalid scene root'),
        )
        for index, (nodes, scenes, error) in enumerate(cases):
            with self.subTest(error=error):
                gltf_data = make_synthetic_gltf(verts)
                gltf_data['nodes'] = nodes
                gltf_data['scenes'] = scenes
                gltf_path = self.tmp_dir / f'test_node_graph_{index}.gltf'
                gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')
                with self.assertRaises(ValueError) as cm:
                    load_gltf_mesh(gltf_path)
                self.assertIn(error, str(cm.exception))

    def test_grid_centroid_lod_removes_degenerate_and_duplicate_faces(self):
        points = np.array([
            [0.00001, 0.0, 0.0], [0.00009, 0.0, 0.0],
            [0.001, 0.0, 0.0], [0.0, 0.001, 0.0],
        ], dtype=np.float32)
        triangles = np.array([[0, 2, 3], [1, 2, 3], [3, 2, 0], [0, 1, 3]], dtype=np.int64)

        points_lod, triangles_lod, max_displacement = cluster_mesh_for_visualization(points, triangles)
        repeat_points, repeat_triangles, repeat_displacement = cluster_mesh_for_visualization(points, triangles)

        self.assertEqual(triangles_lod.shape, (2, 3))
        np.testing.assert_array_equal(triangles_lod, [[0, 2, 1], [1, 2, 0]])
        np.testing.assert_array_equal(points_lod, repeat_points)
        np.testing.assert_array_equal(triangles_lod, repeat_triangles)
        self.assertAlmostEqual(max_displacement, 0.00004, places=7)
        self.assertEqual(max_displacement, repeat_displacement)

    def test_archive_member_requires_exact_source_relative_path(self):
        expected = 'mainassem/urdf/mainassem.urdf'
        self.assertEqual(_archive_member([expected], expected, 'URDF'), expected)
        with self.assertRaisesRegex(ValueError, 'has 0 matching ZIP entries'):
            _archive_member([f'wrapper/{expected}'], expected, 'URDF')

    def test_j1_cleaner_replaces_source_limit_and_safety_controller(self):
        source = ET.fromstring(
            "<joint name='revolute_2' type='revolute'>"
            "<origin xyz='1 2 3'/><parent link='base'/><child link='j1'/>"
            "<limit lower='-1' upper='1' effort='5' velocity='6'/>"
            "<safety_controller soft_lower_limit='-0.5' soft_upper_limit='0.5'/>"
            "</joint>"
        )
        cleaned = _copy_j1_with_display_limit(source)
        limits = cleaned.findall('limit')
        self.assertEqual(len(limits), 1)
        self.assertAlmostEqual(float(limits[0].get('lower')), -math.pi / 2.0)
        self.assertAlmostEqual(float(limits[0].get('upper')), math.pi / 2.0)
        self.assertEqual(limits[0].get('effort'), '0')
        self.assertEqual(limits[0].get('velocity'), '0')
        self.assertEqual(cleaned.findall('safety_controller'), [])
        self.assertEqual(cleaned.find('origin').get('xyz'), '1 2 3')
        self.assertIsNot(cleaned.find('origin'), source.find('origin'))
        self.assertEqual(len(source.findall('limit')), 1)

    def test_binary_render_packing_removes_collapsed_faces_and_compacts_vertices(self):
        triangles = np.array([
            [[0.0, 0.0, 0.0], [0.0002, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [[5.0, 0.0, 0.0], [6.0, 0.0, 0.0], [5.0, 1.0, 0.0]],
        ])
        positions, indices = _pack_millimeter_render_mesh(triangles)
        self.assertEqual(positions.shape, (3, 3))
        self.assertEqual(indices.shape, (1, 3))
        self.assertEqual(set(indices.reshape(-1).tolist()), {0, 1, 2})

        vanished = triangles[:1]
        with self.assertRaisesRegex(ValueError, 'no triangles after 1-micron vertex rounding'):
            _pack_millimeter_render_mesh(vanished)

    def test_urdf_graph_rejects_duplicate_names_bad_edges_cycles_and_disconnected_roots(self):
        cases = (
            ("<robot><link name='a'/><link name='a'/></robot>", 'Duplicate link name'),
            ("<robot><link name='a'/><joint name='j' type='fixed'><parent link='missing'/><child link='a'/></joint></robot>", 'unknown parent or child'),
            ("<robot><link name='a'/><link name='b'/><joint name='j1' type='fixed'><parent link='a'/><child link='b'/></joint><joint name='j2' type='fixed'><parent link='a'/><child link='b'/></joint></robot>", 'more than one parent'),
            ("<robot><link name='a'/><link name='b'/><joint name='j1' type='fixed'><parent link='a'/><child link='b'/></joint><joint name='j2' type='fixed'><parent link='b'/><child link='a'/></joint></robot>", 'Cycle detected'),
            ("<robot><link name='a'/><link name='b'/></robot>", 'exactly one root'),
            ("<robot><link name='a'/><link name='b'/><joint name='j' type='floating'><parent link='a'/><child link='b'/></joint></robot>", 'Unsupported URDF joint type'),
        )
        for index, (source, error) in enumerate(cases):
            with self.subTest(error=error):
                urdf_path = self.tmp_dir / f'bad_graph_{index}.urdf'
                urdf_path.write_text(source, encoding='utf-8')
                with self.assertRaises(ValueError) as cm:
                    RobotModel(urdf_path)
                self.assertIn(error, str(cm.exception))

    def test_urdf_rejects_zero_joint_axis(self):
        urdf_path = self.tmp_dir / 'zero_axis.urdf'
        urdf_path.write_text(
            "<robot><link name='a'/><link name='b'/><joint name='j' type='revolute'>"
            "<parent link='a'/><child link='b'/><axis xyz='0 0 0'/></joint></robot>",
            encoding='utf-8',
        )
        robot = RobotModel(urdf_path)
        with self.assertRaisesRegex(ValueError, 'finite and nonzero'):
            robot.joint_axis(robot.joints['j'])

    def test_urdf_rejects_unsupported_geometry_scale_and_multiple_visuals(self):
        cases = (
            ("<visual><geometry><mesh filename='package://mainassem/meshes/a.gltf' scale='2 1 1'/></geometry></visual>", 'Non-unity mesh scale'),
            ("<visual><geometry><box size='1 1 1'/></geometry></visual>", 'Only mesh visual geometry'),
            ("<visual><geometry><mesh filename='package://mainassem/meshes/a.gltf'/></geometry></visual><visual><geometry><mesh filename='package://mainassem/meshes/b.gltf'/></geometry></visual>", 'Multiple visual elements'),
        )
        for index, (visuals, error) in enumerate(cases):
            with self.subTest(error=error):
                urdf_path = self.tmp_dir / f'bad_geometry_{index}.urdf'
                urdf_path.write_text(f"<robot><link name='a'>{visuals}</link></robot>", encoding='utf-8')
                with self.assertRaises(ValueError) as cm:
                    RobotModel(urdf_path)
                self.assertIn(error, str(cm.exception))

    def test_gltf_rejects_index_out_of_bounds(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
        tris = np.array([[0, 1, 5]], dtype=np.int64)
        gltf_data = make_synthetic_gltf(verts, tris)
        gltf_path = self.tmp_dir / 'test_oob.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')

        with self.assertRaises(ValueError) as cm:
            load_gltf_mesh(gltf_path)
        self.assertIn('out of bounds', str(cm.exception))

    def test_gltf_rejects_external_buffer_uri(self):
        verts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        gltf_data = make_synthetic_gltf(verts, buffer_uri='http://example.com/buffer.bin')
        gltf_path = self.tmp_dir / 'test_ext.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')

        with self.assertRaises(ValueError) as cm:
            load_gltf_mesh(gltf_path)
        self.assertIn('External buffers and network URIs are rejected', str(cm.exception))

    def test_gltf_rejects_unsupported_primitive_mode(self):
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]], dtype=np.float32)
        gltf_data = make_synthetic_gltf(verts, primitive_mode=1)
        gltf_path = self.tmp_dir / 'test_mode.gltf'
        gltf_path.write_text(json.dumps(gltf_data), encoding='utf-8')

        with self.assertRaises(ValueError) as cm:
            load_gltf_mesh(gltf_path)
        self.assertIn('TRIANGLES (mode 4) required', str(cm.exception))

    def test_reject_nonempty_output(self):
        source_dir = self.tmp_dir / 'src'
        source_dir.mkdir()
        archive_file = self.tmp_dir / 'dummy.zip'
        archive_file.write_bytes(b'PK\x05\x06' + b'\x00' * 18)

        out_dir = self.tmp_dir / 'out'
        out_dir.mkdir()
        (out_dir / 'existing.txt').write_text('occupied', encoding='utf-8')

        with self.assertRaises(ValueError) as cm:
            build_manual_arm(source_dir, archive_file, out_dir)
        self.assertIn('must be empty or not exist', str(cm.exception))

    def test_reject_output_inside_source(self):
        source_dir = self.tmp_dir / 'src'
        source_dir.mkdir()
        out_dir = source_dir / 'nested_out'
        archive_file = self.tmp_dir / 'dummy.zip'
        archive_file.write_bytes(b'PK\x05\x06' + b'\x00' * 18)

        with self.assertRaises(ValueError) as cm:
            build_manual_arm(source_dir, archive_file, out_dir)
        self.assertIn('cannot be inside source tree', str(cm.exception))


class TestGeneratedArmModel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.source_dir = WORKSPACE_ROOT / '_arm_inputs' / 'cad-export-v3'
        cls.archive_path = WORKSPACE_ROOT / '_arm_inputs' / 'mainAssem (3).zip'

        if not cls.source_dir.is_dir() or not cls.archive_path.is_file():
            raise unittest.SkipTest("Grounding inputs (_arm_inputs) not available in working tree")

        cls.out_dir = Path(tempfile.mkdtemp(prefix='m01_arm_build_'))
        cls.model_json = build_manual_arm(
            cls.source_dir,
            cls.archive_path,
            cls.out_dir
        )

        cls.urdf_tree = ET.parse(cls.out_dir / 'arm.urdf')
        cls.urdf_root = cls.urdf_tree.getroot()

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, 'out_dir') and cls.out_dir.is_dir():
            shutil.rmtree(cls.out_dir, ignore_errors=True)

    def test_generated_tree_exactly_3_active_chain(self):
        links = self.urdf_root.findall('link')
        joints = self.urdf_root.findall('joint')
        self.assertEqual(len(links), 76, "Must preserve all 76 links")
        self.assertEqual(len(joints), 75, "Must preserve all 75 joints")

        active_chain = ['revolute_2', 'revolute_1', 'revolute_3']
        joint_map = {j.get('name'): j for j in joints}

        for jn in active_chain:
            self.assertIn(jn, joint_map)

        self.assertEqual(joint_map['revolute_4'].get('type'), 'fixed')
        self.assertIsNone(joint_map['revolute_4'].find('axis'), "Fixed joint revolute_4 must have no axis element")

        for j_name, j_elem in joint_map.items():
            if j_name in active_chain:
                self.assertIn(j_elem.get('type'), ('revolute', 'continuous'))
            else:
                self.assertEqual(j_elem.get('type'), 'fixed', f"Joint {j_name} must be fixed")

        mj_joints = self.model_json['joints']
        self.assertEqual(len(mj_joints), 3)
        self.assertNotIn('ordered_joints', self.model_json)
        expected = [
            ('J1 base', 'base', 'revolute_2', 0),
            ('J2 shoulder', 'shoulder', 'revolute_1', 1),
            ('J3 elbow', 'elbow', 'revolute_3', 2),
        ]
        for item, (exp_name, exp_role, exp_urdf, exp_id) in zip(mj_joints, expected):
            self.assertEqual(item['name'], exp_name)
            self.assertEqual(item['role'], exp_role)
            self.assertEqual(item['urdf'], exp_urdf)
            self.assertEqual(item['motor_id'], exp_id)

    def test_unchanged_passive_origin_and_link_transforms(self):
        src_urdf = ET.parse(self.source_dir / 'mainassem' / 'urdf' / 'mainassem.urdf').getroot()
        src_joints = {j.get('name'): j for j in src_urdf.findall('joint')}
        cleaned_joints = {j.get('name'): j for j in self.urdf_root.findall('joint')}

        for j_name, src_j in src_joints.items():
            clean_j = cleaned_joints[j_name]
            src_orig = src_j.find('origin')
            clean_orig = clean_j.find('origin')
            if src_orig is None:
                self.assertIsNone(clean_orig)
            else:
                self.assertEqual(src_orig.get('xyz'), clean_orig.get('xyz'), f"xyz mismatch on {j_name}")
                self.assertEqual(src_orig.get('rpy'), clean_orig.get('rpy'), f"rpy mismatch on {j_name}")

    def test_j1_180_degree_total_limits(self):
        joint_map = {j.get('name'): j for j in self.urdf_root.findall('joint')}
        j1 = joint_map['revolute_2']
        self.assertEqual(j1.get('type'), 'revolute')
        self.assertEqual(len(j1.findall('limit')), 1)
        self.assertEqual(j1.findall('safety_controller'), [])
        lim = j1.find('limit')
        self.assertIsNotNone(lim)
        self.assertAlmostEqual(float(lim.get('lower')), -math.pi / 2.0, places=9)
        self.assertAlmostEqual(float(lim.get('upper')), math.pi / 2.0, places=9)
        self.assertEqual(lim.get('effort'), '0')
        self.assertEqual(lim.get('velocity'), '0')

        self.assertEqual(joint_map['revolute_1'].get('type'), 'continuous')
        self.assertEqual(joint_map['revolute_3'].get('type'), 'continuous')

        mj = self.model_json['joints']
        self.assertEqual(mj[0]['limits_deg'], [-90.0, 90.0])
        self.assertIn('provisional', mj[0]['limit_provenance'])
        self.assertIn('uncommissioned zero', mj[0]['limit_provenance'])
        self.assertIsNone(mj[1]['limits_deg'])
        self.assertIsNone(mj[2]['limits_deg'])

    def test_base_fixed_vs_moving_duplicate_visual_removed(self):
        links = {l.get('name'): l for l in self.urdf_root.findall('link')}

        self.assertGreater(len(links['none_2'].findall('visual')), 0, "none_2 must retain its visual mesh")

        self.assertEqual(len(links['none_1'].findall('visual')), 0, "none_1 visual must be removed")
        self.assertIsNotNone(links['none_1'].find('inertial'), "none_1 structural inertial must be preserved")

        joints = {j.get('name'): j for j in self.urdf_root.findall('joint')}
        self.assertIn('fastened_2', joints)
        self.assertEqual(joints['fastened_2'].find('child').get('link'), 'none_1')

        prov = self.model_json['cleanup_provenance']
        self.assertEqual(prov['unmapped_material_meshes'], sorted([
            '_8_32_Thin_Nylock_Nut__32_02_.gltf',
            '_8_32_x________2_1_2______Star_Drive_Screw______276_8016_.gltf',
            'Part_1_7.gltf',
            'Part_1_8.gltf',
            'Socket_button_head_screw_M4x0_7_x_16.gltf',
        ]))
        self.assertEqual(prov['dropped_visual']['link'], 'none_1')
        self.assertTrue(prov['dropped_visual']['structural_link_preserved'])
        yaw_diff = prov['dropped_visual']['yaw_difference_deg']
        self.assertAlmostEqual(yaw_diff, 3.35, delta=0.1)
        source_robot = RobotModel(self.source_dir / 'mainassem' / 'urdf' / 'mainassem.urdf')
        self.assertEqual(prov['dropped_visual']['parent_joint'], source_robot.parent_joint['none_1'].get('name'))
        self.assertEqual(prov['dropped_visual']['parent_link'], source_robot.parent_joint['none_1'].find('parent').get('link'))
        self.assertEqual(prov['retained_base_motor']['parent_joint'], source_robot.parent_joint['none_2'].get('name'))

    def test_relative_asset_resolution(self):
        for v in self.urdf_root.iter('visual'):
            m = v.find('geometry/mesh')
            if m is not None:
                filename = m.get('filename', '')
                self.assertTrue(filename.startswith('meshes/'), f"Must be relative meshes/: {filename}")
                self.assertTrue(filename.endswith('.stl'), f"Must be portable STL: {filename}")
                resolved = self.out_dir / filename
                self.assertTrue(resolved.is_file(), f"Mesh file must exist on disk: {resolved}")

        for b_name, b_data in self.model_json['bodies'].items():
            for mat_name, entry in b_data.items():
                rel_file = entry['file']
                self.assertTrue(rel_file.startswith('meshes/'))
                self.assertTrue(rel_file.endswith('.bin'))
                resolved = self.out_dir / rel_file
                self.assertTrue(resolved.is_file(), f"Binary mesh must exist: {resolved}")

        for rel_file, meta in self.model_json['portable_meshes'].items():
            self.assertTrue(rel_file.startswith('meshes/'))
            self.assertTrue(rel_file.endswith('.stl'))
            resolved = self.out_dir / rel_file
            self.assertTrue(resolved.is_file(), f"Portable STL must exist: {resolved}")

    def test_hash_and_provenance(self):
        self.assertEqual(
            self.model_json['source']['archive_sha256'],
            'ebb0777628e318e27eca25686adefed16dabc92daec44229f51913b915d6cd72'
        )
        self.assertEqual(
            self.model_json['source']['urdf_sha256'],
            '8c3b60f2bc8c19b6843143f491b92fe4365a9ce9928f39e3566ae3903b05666c'
        )
        self.assertEqual(self.model_json['units'], 'mm')
        self.assertEqual(self.model_json['cleaned_urdf']['units'], 'm')
        self.assertTrue(all(meta['units'] == 'm' for meta in self.model_json['portable_meshes'].values()))

        urdf_bytes = (self.out_dir / 'arm.urdf').read_bytes()
        self.assertEqual(
            self.model_json['cleaned_urdf']['sha256'],
            hashlib.sha256(urdf_bytes).hexdigest()
        )

        for rel_path, meta in self.model_json['portable_meshes'].items():
            disk_bytes = (self.out_dir / rel_path).read_bytes()
            self.assertEqual(meta['sha256'], hashlib.sha256(disk_bytes).hexdigest())

        for b_name, mats in self.model_json['bodies'].items():
            for mat_name, entry in mats.items():
                disk_bytes = (self.out_dir / entry['file']).read_bytes()
                self.assertEqual(entry['sha256'], hashlib.sha256(disk_bytes).hexdigest())

        self.assertTrue(self.model_json['placeholder_inertials'])
        self.assertFalse(self.model_json['dynamics_validated'])
        self.assertFalse(self.model_json['collision_validated'])

    def test_generated_bin_shape_index_bounds_finite(self):
        bin_files = list((self.out_dir / 'meshes').glob('*.bin'))
        self.assertGreater(len(bin_files), 0, "Must have generated .bin rendering files")

        for bf in bin_files:
            data = bf.read_bytes()
            b_name, mat_name = bf.stem.split('.', 1)
            entry = self.model_json['bodies'][b_name][mat_name]
            num_verts = entry['vertices']
            num_tris = entry['triangles']

            expected_len = num_verts * 12 + num_tris * 3 * 4
            self.assertEqual(len(data), expected_len, f"File length mismatch for {bf.name}")

            verts = np.frombuffer(data[:num_verts * 12], dtype='<f4').reshape(-1, 3)
            self.assertTrue(np.all(np.isfinite(verts)), f"Vertices in {bf.name} must be finite")

            indices = np.frombuffer(data[num_verts * 12:], dtype='<u4').reshape(-1, 3)
            if num_tris > 0:
                self.assertGreaterEqual(indices.min(), 0)
                self.assertLess(indices.max(), num_verts, f"Index out of bounds in {bf.name}")

    def test_independent_model_world_frame_and_origins(self):
        cad_to_world = self.model_json['cad_to_world']
        mat = np.array(cad_to_world['matrix'])
        orig = np.array(cad_to_world['origin'])

        j1 = self.model_json['joints'][0]
        self.assertAlmostEqual(j1['origin'][0], 0.0, places=3)
        self.assertAlmostEqual(j1['origin'][1], 0.0, places=3)

    def test_cad_to_world_floor_uses_full_resolution_source_mesh(self):
        source_robot = RobotModel(self.source_dir / 'mainassem' / 'urdf' / 'mainassem.urdf')
        frames = source_robot.fk()
        actuated = ['revolute_2', 'revolute_1', 'revolute_3']
        base_triangles = []
        for link_name in source_robot.links:
            if source_robot.get_body(link_name, actuated) != 'base':
                continue
            for raw_filename, visual_transform in source_robot.visuals(link_name):
                source_points, source_faces = load_gltf_mesh(
                    self.source_dir / 'mainassem' / 'meshes' / Path(raw_filename).name
                )
                transform = frames[link_name] @ visual_transform
                points_cad_mm = (source_points @ transform[:3, :3].T + transform[:3, 3]) * 1000.0
                base_triangles.append(points_cad_mm[source_faces].reshape(-1, 3))
        up_cad = R_UP.T @ np.array([0.0, 0.0, 1.0])
        source_floor = np.concatenate(base_triangles) @ up_cad
        j1 = source_robot.joints['revolute_2']
        parent = j1.find('parent').get('link')
        j1_origin = (frames[parent] @ source_robot.joint_origin(j1))[:3, 3] * 1000.0
        expected_origin = j1_origin - (j1_origin @ up_cad - source_floor.min()) * up_cad

        np.testing.assert_allclose(self.model_json['cad_to_world']['origin'], expected_origin, atol=1e-9)

    def test_dense_visual_lod_respects_face_cap_and_reports_real_displacement(self):
        dense = [
            (path, meta) for path, meta in self.model_json['portable_meshes'].items()
            if meta['source_triangles'] > 50000
        ]
        self.assertTrue(dense, 'v3 must include a dense source visual mesh')
        for _, meta in dense:
            self.assertGreater(meta['generated_triangles'], 0)
            self.assertLessEqual(meta['generated_triangles'], 50000)
            self.assertGreater(meta['generated_vertices'], 0)
            lod = meta['lod']
            self.assertEqual(lod['method'], 'floor_grid_vertex_centroid')
            self.assertEqual(lod['grid_meters'], 0.0004)
            self.assertLessEqual(lod['max_vertex_displacement_meters'], 0.00046)
            self.assertEqual(lod['purpose'], 'visualization_only')
            self.assertFalse(lod['collision_validated'])
            self.assertFalse(lod['dynamics_validated'])

        none_meta = self.model_json['portable_meshes']['meshes/NONE.stl']
        self.assertEqual(none_meta['source_triangles'], 1226332)
        self.assertEqual(struct.unpack_from('<I', (self.out_dir / 'meshes/NONE.stl').read_bytes(), 80)[0], none_meta['generated_triangles'])
        self.assertAlmostEqual(none_meta['lod']['max_vertex_displacement_meters'], 0.0004566798844, delta=1e-7)


if __name__ == '__main__':
    unittest.main()
