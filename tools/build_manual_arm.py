#!/usr/bin/env python3
"""Adapted from reference-importer.py (upstream code attribution)."""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import struct
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

if __package__:
    from .arm_gltf import load_gltf_mesh
else:
    from arm_gltf import load_gltf_mesh

# Upstream CAD orientation grounding: CAD -Z is up -> world +Z is up
R_UP = np.diag([1.0, -1.0, -1.0])
MAX_VISUAL_TRIANGLES = 50000
VISUAL_CLUSTER_CELL_METERS = 0.0004
MESH_URI_PREFIX = 'package://mainassem/'

MATERIAL_MAP = {
    'NONE.gltf': 'motor',
    'Part_1.gltf': 'link',
    'Part_1_1.gltf': 'link',
    'Part_1_2.gltf': 'tube',
    'Part_1_6.gltf': 'frame',
    'Part_1_9.gltf': 'frame',
    'Part_1_10.gltf': 'frame',
    'Part_1_11.gltf': 'frame',
    'Part_1_3.gltf': 'hub',
    'Part_1_4.gltf': 'hub',
    'Part_1_5.gltf': 'hub',
}


# Adapted from reference-importer.py routines (upstream code attribution)

def rpy(r: float, p: float, y: float) -> np.ndarray:
    """Euler roll-pitch-yaw rotation matrix (XYZ convention)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr               ]
    ], dtype=np.float64)


def pose(xyz, r_mat: np.ndarray) -> np.ndarray:
    M = np.eye(4, dtype=np.float64)
    M[:3, :3] = r_mat
    M[:3, 3] = xyz
    return M


def parse_nums(s: str) -> list[float]:
    values = [float(x) for x in s.split()]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Expected finite numeric values')
    return values


def _mesh_uri_path(raw_filename: str) -> PurePosixPath:
    if not isinstance(raw_filename, str) or not raw_filename.startswith(MESH_URI_PREFIX):
        raise ValueError(f'Only {MESH_URI_PREFIX}<relative path> mesh URIs are supported: {raw_filename!r}')
    relative = raw_filename[len(MESH_URI_PREFIX):]
    parts = relative.split('/')
    if '\\' in relative or any(part in ('', '.', '..') for part in parts):
        raise ValueError(f'Unsafe package://mainassem mesh path: {raw_filename}')
    path = PurePosixPath(relative)
    if path.is_absolute() or len(path.parts) != 2 or path.parts[0] != 'meshes':
        raise ValueError(f'Only package://mainassem/meshes/<file> mesh paths are supported: {raw_filename}')
    return path


def _archive_member(zip_names: list[str], relative_path: str, description: str) -> str:
    expected = PurePosixPath(relative_path).as_posix()
    matches = [name for name in zip_names if name == expected]
    if len(matches) != 1:
        raise ValueError(f'{description} path {expected} has {len(matches)} matching ZIP entries')
    return matches[0]


def _copy_j1_with_display_limit(source_joint: ET.Element) -> ET.Element:
    joint = ET.Element('joint', {'name': source_joint.get('name'), 'type': 'revolute'})
    for element in source_joint:
        if element.tag not in ('limit', 'safety_controller'):
            joint.append(copy.deepcopy(element))
    ET.SubElement(joint, 'limit', {
        'lower': str(-math.pi / 2.0),
        'upper': str(math.pi / 2.0),
        'effort': '0',
        'velocity': '0',
    })
    return joint


def _pack_millimeter_render_mesh(triangles: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    triangles = np.asarray(triangles, dtype=np.float64)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3):
        raise ValueError('Rendering triangles must be an N x 3 x 3 array')
    vertices = triangles.reshape(-1, 3)
    quantized = np.round(vertices * 1000.0).astype(np.int64)
    _, first, inverse = np.unique(quantized, axis=0, return_index=True, return_inverse=True)
    indices = inverse.reshape(-1, 3)
    usable = (
        (indices[:, 0] != indices[:, 1])
        & (indices[:, 1] != indices[:, 2])
        & (indices[:, 2] != indices[:, 0])
    )
    indices = indices[usable]
    if not len(indices):
        raise ValueError('Rendering mesh has no triangles after 1-micron vertex rounding')

    used_vertices, compact_indices = np.unique(indices.reshape(-1), return_inverse=True)
    positions = vertices[first[used_vertices]].astype('<f4')
    return positions, compact_indices.reshape(-1, 3).astype('<u4')


def cluster_mesh_for_visualization(
    points: np.ndarray,
    triangles: np.ndarray,
    cell_meters: float = VISUAL_CLUSTER_CELL_METERS,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Cluster original vertices into grid-cell centroids for display-only LOD."""
    cell_meters = float(cell_meters)
    if not math.isfinite(cell_meters) or cell_meters <= 0.0:
        raise ValueError('Visualization cluster cell must be a positive finite number')
    source_points = np.asarray(points, dtype=np.float64)
    source_triangles = np.asarray(triangles)
    if source_points.ndim != 2 or source_points.shape[1] != 3 or not np.all(np.isfinite(source_points)):
        raise ValueError('Visualization points must be a finite N x 3 array')
    if source_triangles.ndim != 2 or source_triangles.shape[1] != 3:
        raise ValueError('Visualization triangles must be an M x 3 array')
    if source_triangles.size and (
        not np.issubdtype(source_triangles.dtype, np.integer)
        or source_triangles.min() < 0
        or source_triangles.max() >= len(source_points)
    ):
        raise ValueError('Visualization triangle indices must be valid integers')
    if len(source_points) == 0:
        return source_points.astype(np.float32), source_triangles.astype(np.int64), 0.0

    grid = np.floor(source_points / cell_meters)
    limits = np.iinfo(np.int64)
    if not np.all(np.isfinite(grid)) or np.any(grid < limits.min) or np.any(grid > limits.max):
        raise ValueError('Visualization points exceed the supported grid coordinate range')
    _, inverse = np.unique(grid.astype(np.int64), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    centroids = np.column_stack([
        np.bincount(inverse, weights=source_points[:, axis]) / counts
        for axis in range(3)
    ])
    max_displacement = float(np.linalg.norm(source_points - centroids[inverse], axis=1).max())

    remapped = inverse[source_triangles.astype(np.int64)]
    nondegenerate = (
        (remapped[:, 0] != remapped[:, 1])
        & (remapped[:, 1] != remapped[:, 2])
        & (remapped[:, 2] != remapped[:, 0])
    )
    remapped = remapped[nondegenerate]
    if len(remapped):
        sorted_faces = np.sort(remapped, axis=1)
        a, b, c = remapped.T
        odd_permutation = (
            (a > b).astype(np.int8)
            + (a > c).astype(np.int8)
            + (b > c).astype(np.int8)
        ) % 2
        oriented_keys = np.column_stack((sorted_faces, odd_permutation))
        _, first = np.unique(oriented_keys, axis=0, return_index=True)
        remapped = remapped[np.sort(first)]
    return centroids.astype(np.float32), remapped.astype(np.int64), max_displacement


def write_binary_stl(filepath: Path, points: np.ndarray, triangles: np.ndarray):
    """Write standard binary STL in meters."""
    num_triangles = len(triangles)
    header = b'IM6014 manual arm portable STL export' + b'\0' * (80 - len(b'IM6014 manual arm portable STL export'))

    if num_triangles == 0:
        with open(filepath, 'wb') as f:
            f.write(header)
            f.write(struct.pack('<I', 0))
        return

    v0 = points[triangles[:, 0]]
    v1 = points[triangles[:, 1]]
    v2 = points[triangles[:, 2]]

    normals = np.cross(v1 - v0, v2 - v0)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms == 0.0] = 1.0
    normals = normals / norms

    facet_dtype = np.dtype([
        ('normal', '<f4', (3,)),
        ('v0',     '<f4', (3,)),
        ('v1',     '<f4', (3,)),
        ('v2',     '<f4', (3,)),
        ('attr',   '<u2')
    ])
    facets = np.empty(num_triangles, dtype=facet_dtype)
    facets['normal'] = normals
    facets['v0'] = v0
    facets['v1'] = v1
    facets['v2'] = v2
    facets['attr'] = 0

    with open(filepath, 'wb') as f:
        f.write(header)
        f.write(struct.pack('<I', num_triangles))
        f.write(facets.tobytes())


class RobotModel:

    def __init__(self, urdf_path: Path):
        self.urdf_path = urdf_path
        self.tree = ET.parse(urdf_path)
        self.root_elem = self.tree.getroot()
        if self.root_elem.tag != 'robot':
            raise ValueError(f'Expected a URDF robot root in {urdf_path}')

        link_elements = self.root_elem.findall('link')
        joint_elements = self.root_elem.findall('joint')

        def name_map(elements, description):
            result = {}
            for element in elements:
                name = element.get('name')
                if not isinstance(name, str) or not name.strip():
                    raise ValueError(f'{description} names must be non-empty')
                if name in result:
                    raise ValueError(f'Duplicate {description} name: {name}')
                result[name] = element
            return result

        self.links = name_map(link_elements, 'link')
        self.joints = name_map(joint_elements, 'joint')
        if not self.links:
            raise ValueError('URDF must contain at least one link')
        for joint in self.joints.values():
            if joint.get('type') not in ('fixed', 'revolute', 'continuous'):
                raise ValueError(f'Unsupported URDF joint type on {joint.get("name")}: {joint.get("type")}')

        for link_name, link in self.links.items():
            visuals = link.findall('visual')
            if len(visuals) > 1:
                raise ValueError(f'Multiple visual elements on link {link_name} are unsupported')
            for element_kind in ('visual', 'collision'):
                for element in link.findall(element_kind):
                    geometry = element.find('geometry')
                    if geometry is None or len(geometry) != 1 or geometry[0].tag != 'mesh':
                        raise ValueError(f'Only mesh {element_kind} geometry is supported on link {link_name}')
                    mesh = geometry[0]
                    scale = parse_nums(mesh.get('scale', '1 1 1'))
                    if len(scale) != 3 or scale != [1.0, 1.0, 1.0]:
                        raise ValueError(f'Non-unity mesh scale is unsupported on link {link_name}')

        self.parent_joint: dict[str, ET.Element] = {}
        self.kids: dict[str, list[ET.Element]] = {}
        for j in self.joints.values():
            parents = j.findall('parent')
            children = j.findall('child')
            if len(parents) != 1 or len(children) != 1:
                raise ValueError(f'Joint {j.get("name")} must have exactly one parent and child')
            parent = parents[0].get('link')
            child = children[0].get('link')
            if parent not in self.links or child not in self.links:
                raise ValueError(f'Joint {j.get("name")} references an unknown parent or child link')
            if parent == child:
                raise ValueError(f'Joint {j.get("name")} cannot connect a link to itself')
            if child in self.parent_joint:
                raise ValueError(f'Link {child} has more than one parent joint')
            self.parent_joint[child] = j
            self.kids.setdefault(parent, []).append(j)

        states: dict[str, int] = {}

        def visit(link_name: str):
            if states.get(link_name) == 1:
                raise ValueError(f'Cycle detected in URDF at link {link_name}')
            if states.get(link_name) == 2:
                return
            states[link_name] = 1
            for joint in self.kids.get(link_name, []):
                visit(joint.find('child').get('link'))
            states[link_name] = 2

        for link_name in self.links:
            visit(link_name)
        roots = [name for name in self.links if name not in self.parent_joint]
        if len(roots) != 1:
            raise ValueError(f'URDF must have exactly one root link; found {len(roots)}')
        self.root_link = roots[0]
        reachable = set()
        stack = [self.root_link]
        while stack:
            link_name = stack.pop()
            if link_name in reachable:
                continue
            reachable.add(link_name)
            stack.extend(j.find('child').get('link') for j in self.kids.get(link_name, []))
        if reachable != set(self.links):
            raise ValueError('All URDF links must be reachable from the single root')

    def joint_origin(self, joint_elem: ET.Element) -> np.ndarray:
        o = joint_elem.find('origin')
        if o is None:
            return np.eye(4, dtype=np.float64)
        xyz = parse_nums(o.get('xyz', '0 0 0'))
        rpy_vals = parse_nums(o.get('rpy', '0 0 0'))
        if len(xyz) != 3 or len(rpy_vals) != 3:
            raise ValueError(f'Joint {joint_elem.get("name")} origin must have three xyz and rpy values')
        return pose(xyz, rpy(*rpy_vals))

    def joint_axis(self, joint_elem: ET.Element) -> np.ndarray:
        a = joint_elem.find('axis')
        if a is None:
            return np.array([1.0, 0.0, 0.0], dtype=np.float64)
        axis = parse_nums(a.get('xyz', '1 0 0'))
        if len(axis) != 3:
            raise ValueError(f'Joint {joint_elem.get("name")} axis must have three values')
        axis = np.array(axis, dtype=np.float64)
        norm = np.linalg.norm(axis)
        if not np.isfinite(norm) or norm == 0.0:
            raise ValueError(f'Joint {joint_elem.get("name")} axis must be finite and nonzero')
        return axis

    def fk(self) -> dict[str, np.ndarray]:
        """Compute link frames at zero pose relative to root in CAD meters."""
        frames: dict[str, np.ndarray] = {self.root_link: np.eye(4, dtype=np.float64)}
        stack = [self.root_link]
        while stack:
            n = stack.pop()
            for j in self.kids.get(n, []):
                c = j.find('child').get('link')
                frames[c] = frames[n] @ self.joint_origin(j)
                stack.append(c)
        return frames

    def get_body(self, link: str, actuated_joints: list[str]) -> str:
        """Find moving body key ('base', 'j1', 'j2', 'j3') for a link."""
        cur = link
        while cur in self.parent_joint:
            j = self.parent_joint[cur]
            j_name = j.get('name')
            if j_name in actuated_joints:
                idx = actuated_joints.index(j_name)
                return f'j{idx + 1}'
            cur = j.find('parent').get('link')
        return 'base'

    def visuals(self, link: str):
        """Yield (mesh_filename, visual_transform_matrix)."""
        link_elem = self.links[link]
        for v in link_elem.findall('visual'):
            m = v.find('geometry/mesh')
            if m is None:
                continue
            o = v.find('origin')
            v_mat = self.joint_origin(v) if o is not None else np.eye(4, dtype=np.float64)
            filename = m.get('filename', '')
            yield filename, v_mat


def build_manual_arm(source_dir: Path, archive_path: Path, output_dir: Path) -> dict:
    """Generate arm.urdf, model.json, portable STLs, and rendering meshes.

    Returns:
        dict: Generated model.json content.
    """
    source_dir = source_dir.resolve()
    archive_path = archive_path.resolve()
    output_dir = output_dir.resolve()

    if not source_dir.is_dir():
        raise ValueError(f"Source directory does not exist: {source_dir}")
    if not archive_path.is_file():
        raise ValueError(f"Archive file does not exist: {archive_path}")
    if output_dir == source_dir:
        raise ValueError("Output directory cannot be identical to source directory")
    if output_dir.is_relative_to(source_dir):
        raise ValueError(f"Output directory {output_dir} cannot be inside source tree {source_dir}")
    if source_dir.is_relative_to(output_dir):
        raise ValueError(f"Source directory {source_dir} cannot be inside output directory {output_dir}")

    if output_dir.exists() and (not output_dir.is_dir() or any(output_dir.iterdir())):
        raise ValueError(f"Output directory {output_dir} must be empty or not exist")

    urdfs = list(source_dir.rglob('*.urdf'))
    urdf_path = source_dir / 'mainassem' / 'urdf' / 'mainassem.urdf'
    if len(urdfs) != 1 or urdfs[0].resolve() != urdf_path.resolve():
        raise ValueError(f"Expected one unambiguous v3 URDF at {urdf_path}; found {len(urdfs)}")

    package_root = urdf_path.parent.parent
    if package_root.name != 'mainassem':
        raise ValueError(f"Expected URDF package directory 'mainassem', found {package_root.name}")

    archive_bytes = archive_path.read_bytes()
    archive_sha256 = hashlib.sha256(archive_bytes).hexdigest()
    disk_urdf_bytes = urdf_path.read_bytes()
    urdf_sha256 = hashlib.sha256(disk_urdf_bytes).hexdigest()

    with zipfile.ZipFile(archive_path, 'r') as zf:
        zip_names = [item.filename for item in zf.infolist()]
        if len(zip_names) != len(set(zip_names)):
            raise ValueError(f'Duplicate ZIP paths in archive {archive_path}')

        zip_urdf_name = _archive_member(zip_names, urdf_path.relative_to(source_dir).as_posix(), 'URDF')
        if zf.read(zip_urdf_name) != disk_urdf_bytes:
            raise ValueError('URDF file content does not match archive exact bytes')

        source_robot = RobotModel(urdf_path)
        referenced_mesh_files: set[str] = set()
        visual_mesh_paths: set[str] = set()
        verified_meshes: dict[str, Path] = {}

        def verify_mesh(raw_filename: str, description: str) -> tuple[str, Path]:
            mesh_rel = _mesh_uri_path(raw_filename)
            rel_key = mesh_rel.as_posix()
            mesh_path = (package_root / Path(*mesh_rel.parts)).resolve()
            if not mesh_path.is_relative_to(package_root) or not mesh_path.is_relative_to(source_dir):
                raise ValueError(f'Mesh path {mesh_path} escapes the v3 source package')
            if not mesh_path.is_file():
                raise ValueError(f'Referenced mesh file missing from source tree: {mesh_path}')
            if rel_key not in verified_meshes:
                zip_name = _archive_member(zip_names, mesh_path.relative_to(source_dir).as_posix(), description)
                if zf.read(zip_name) != mesh_path.read_bytes():
                    raise ValueError(f'Mesh {rel_key} bytes do not match archive exact bytes')
                verified_meshes[rel_key] = mesh_path
            return rel_key, mesh_path

        stem_paths: dict[str, str] = {}
        for link_name in source_robot.links:
            for raw_filename, _ in source_robot.visuals(link_name):
                rel_key, mesh_path = verify_mesh(raw_filename, f'Visual mesh on {link_name}')
                visual_mesh_paths.add(rel_key)
                stem = mesh_path.stem
                if stem in stem_paths and stem_paths[stem] != rel_key:
                    raise ValueError(f'Ambiguous output mesh name {stem}: {stem_paths[stem]} and {rel_key}')
                stem_paths[stem] = rel_key
                referenced_mesh_files.add(mesh_path.name)

        for link_name, link in source_robot.links.items():
            for collision in link.findall('collision'):
                raw_filename = collision.find('geometry/mesh').get('filename', '')
                rel_key, _ = verify_mesh(raw_filename, f'Collision mesh on {link_name}')
                if rel_key not in visual_mesh_paths:
                    raise ValueError(f'Collision-only mesh {rel_key} is unsupported')

    if len(source_robot.links) != 76:
        raise ValueError(f"Expected source URDF with 76 links, found {len(source_robot.links)}")
    if len(source_robot.joints) != 75:
        raise ValueError(f"Expected source URDF with 75 joints, found {len(source_robot.joints)}")

    actuated_chain = ['revolute_2', 'revolute_1', 'revolute_3']
    passive_joint = 'revolute_4'

    expected_moving = set(actuated_chain + [passive_joint])
    moving = {
        name for name, joint in source_robot.joints.items()
        if joint.get('type') in ('revolute', 'continuous')
    }
    if moving != expected_moving:
        raise ValueError(f'Expected moving joints {sorted(expected_moving)}, found {sorted(moving)}')
    expected_parent_bodies = {'revolute_2': 'base', 'revolute_1': 'j1', 'revolute_3': 'j2'}
    for joint_name, body in expected_parent_bodies.items():
        parent_link = source_robot.joints[joint_name].find('parent').get('link')
        if source_robot.get_body(parent_link, actuated_chain) != body:
            raise ValueError(f'{joint_name} is not attached to the expected {body} body')

    F_cad = source_robot.fk()

    none_1_vis = list(source_robot.visuals('none_1'))
    none_2_vis = list(source_robot.visuals('none_2'))
    if len(none_1_vis) != 1 or len(none_2_vis) != 1:
        raise ValueError('Expected one source visual on each of none_1 and none_2')
    none_1_mesh = _mesh_uri_path(none_1_vis[0][0]).as_posix()
    none_2_mesh = _mesh_uri_path(none_2_vis[0][0]).as_posix()
    if none_1_mesh != none_2_mesh:
        raise ValueError('none_1 is not a duplicate of the retained none_2 source mesh')
    if source_robot.get_body('none_1', actuated_chain) != 'j1':
        raise ValueError('none_1 is not owned by the J1 moving body')
    if source_robot.get_body('none_2', actuated_chain) != 'base':
        raise ValueError('none_2 is not owned by the fixed base body')

    t_cad_none1 = F_cad['none_1'] @ none_1_vis[0][1]
    t_cad_none2 = F_cad['none_2'] @ none_2_vis[0][1]

    pos_diff_mm = np.linalg.norm(t_cad_none1[:3, 3] - t_cad_none2[:3, 3]) * 1000.0
    r_diff = t_cad_none2[:3, :3].T @ t_cad_none1[:3, :3]
    trace_val = np.clip((np.trace(r_diff) - 1.0) / 2.0, -1.0, 1.0)
    yaw_diff_deg = float(np.degrees(np.arccos(trace_val)))

    if pos_diff_mm > 1.0:
        raise ValueError(f"none_1 does not match none_2 nominal position: offset {pos_diff_mm:.4f} mm")

    none_1_parent_joint = source_robot.parent_joint['none_1']
    none_2_parent_joint = source_robot.parent_joint['none_2']
    none_1_parent = none_1_parent_joint.find('parent').get('link')
    none_2_parent = none_2_parent_joint.find('parent').get('link')

    cleanup_provenance = {
        'dropped_visual': {
            'link': 'none_1',
            'source_mesh': none_1_mesh,
            'parent_joint': none_1_parent_joint.get('name'),
            'parent_link': none_1_parent,
            'body': source_robot.get_body('none_1', actuated_chain),
            'transform_cad': t_cad_none1.tolist(),
            'nominal_origin_cad_mm': (t_cad_none1[:3, 3] * 1000.0).tolist(),
            'yaw_difference_deg': yaw_diff_deg,
            'structural_link_preserved': True,
            'structural_joint_preserved': True,
            'reason': 'Duplicate base motor visual on moving J1 link; removed visual while keeping structural link and joint'
        },
        'retained_base_motor': {
            'link': 'none_2',
            'source_mesh': none_2_mesh,
            'parent_joint': none_2_parent_joint.get('name'),
            'parent_link': none_2_parent,
            'body': source_robot.get_body('none_2', actuated_chain),
            'transform_cad': t_cad_none2.tolist(),
            'reason': 'Stationary base motor firmly attached to base link part_1_25'
        },
        'passive_joint': {
            'joint': 'revolute_4',
            'parent': source_robot.joints[passive_joint].find('parent').get('link'),
            'child': source_robot.joints[passive_joint].find('child').get('link'),
            'action': 'converted continuous to fixed; removed axis element; exported transform preserved'
        }
    }
    cleanup_provenance['unmapped_material_meshes'] = sorted(
        referenced_mesh_files.difference(MATERIAL_MAP)
    )

    meshes_dir = package_root / 'meshes'
    source_meshes: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    loaded_meshes: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    mesh_metadata: dict[str, dict] = {}

    for mesh_name in sorted(referenced_mesh_files):
        gltf_file = meshes_dir / mesh_name
        source_points, source_triangles = load_gltf_mesh(gltf_file)
        display_points, display_triangles = source_points, source_triangles
        max_displacement = 0.0
        lod_method = None
        lod_cell_meters = None
        if len(source_triangles) > MAX_VISUAL_TRIANGLES:
            display_points, display_triangles, max_displacement = cluster_mesh_for_visualization(
                source_points, source_triangles
            )
            if len(display_triangles) == 0 or len(display_triangles) > MAX_VISUAL_TRIANGLES:
                raise ValueError(
                    f'Visualization mesh {mesh_name} produced {len(display_triangles)} triangles; '
                    f'required 1..{MAX_VISUAL_TRIANGLES}'
                )
            lod_method = 'floor_grid_vertex_centroid'
            lod_cell_meters = VISUAL_CLUSTER_CELL_METERS

        source_meshes[mesh_name] = (source_points, source_triangles)
        loaded_meshes[mesh_name] = (display_points, display_triangles)
        mesh_metadata[mesh_name] = {
            'source_file': mesh_name,
            'source_vertices': len(source_points),
            'source_triangles': len(source_triangles),
            'generated_vertices': len(display_points),
            'generated_triangles': len(display_triangles),
            'lod_approximation': lod_method is not None,
            'lod': None if lod_method is None else {
                'method': lod_method,
                'grid_meters': lod_cell_meters,
                'max_vertex_displacement_meters': max_displacement,
                'purpose': 'visualization_only',
                'collision_validated': False,
                'dynamics_validated': False,
            },
        }

    output_dir.mkdir(parents=True, exist_ok=True)
    out_meshes_dir = output_dir / 'meshes'
    out_meshes_dir.mkdir(parents=True, exist_ok=True)

    portable_mesh_entries: dict[str, dict] = {}
    for mesh_name, (pts, tris) in loaded_meshes.items():
        stl_name = f"{Path(mesh_name).stem}.stl"
        stl_path = out_meshes_dir / stl_name
        write_binary_stl(stl_path, pts, tris)
        stl_bytes = stl_path.read_bytes()
        stl_sha256 = hashlib.sha256(stl_bytes).hexdigest()

        meta = dict(mesh_metadata[mesh_name])
        meta['file'] = f"meshes/{stl_name}"
        meta['units'] = 'm'
        meta['sha256'] = stl_sha256
        portable_mesh_entries[f"meshes/{stl_name}"] = meta

    cleaned_root = ET.Element('robot', {'name': 'mainassem'})

    for link in source_robot.root_elem.findall('link'):
        link_copy = ET.SubElement(cleaned_root, 'link', {'name': link.get('name')})
        inertial = link.find('inertial')
        if inertial is not None:
            link_copy.append(copy.deepcopy(inertial))

        link_name = link.get('name')
        for v in link.findall('visual'):
            if link_name == 'none_1':
                continue

            v_copy = ET.SubElement(link_copy, 'visual')
            orig = v.find('origin')
            if orig is not None:
                v_copy.append(copy.deepcopy(orig))

            geom = v.find('geometry')
            if geom is not None and geom.find('mesh') is not None:
                orig_mesh_name = geom.find('mesh').get('filename', '')
                clean_stem = Path(orig_mesh_name).stem
                g_elem = ET.SubElement(v_copy, 'geometry')
                scale = geom.find('mesh').get('scale', '1 1 1')
                ET.SubElement(g_elem, 'mesh', {'filename': f"meshes/{clean_stem}.stl", 'scale': scale})

            mat = v.find('material')
            if mat is not None:
                v_copy.append(copy.deepcopy(mat))

        for col in link.findall('collision'):
            col_copy = ET.SubElement(link_copy, 'collision')
            orig = col.find('origin')
            if orig is not None:
                col_copy.append(copy.deepcopy(orig))
            geom = col.find('geometry')
            if geom is not None and geom.find('mesh') is not None:
                clean_stem = Path(geom.find('mesh').get('filename', '')).stem
                g_elem = ET.SubElement(col_copy, 'geometry')
                ET.SubElement(g_elem, 'mesh', {'filename': f"meshes/{clean_stem}.stl"})

    for joint in source_robot.root_elem.findall('joint'):
        j_name = joint.get('name')
        orig_type = joint.get('type')

        if j_name == passive_joint:
            j_copy = ET.SubElement(cleaned_root, 'joint', {'name': j_name, 'type': 'fixed'})
            orig = joint.find('origin')
            if orig is not None:
                j_copy.append(copy.deepcopy(orig))
            parent_elem = joint.find('parent')
            child_elem = joint.find('child')
            if parent_elem is not None:
                j_copy.append(copy.deepcopy(parent_elem))
            if child_elem is not None:
                j_copy.append(copy.deepcopy(child_elem))
        elif j_name == 'revolute_2':
            cleaned_root.append(_copy_j1_with_display_limit(joint))
        else:
            j_copy = ET.SubElement(cleaned_root, 'joint', {'name': j_name, 'type': orig_type})
            for elem in joint:
                j_copy.append(copy.deepcopy(elem))

    cleaned_urdf_path = output_dir / 'arm.urdf'
    cleaned_tree = ET.ElementTree(cleaned_root)
    ET.indent(cleaned_tree, space='    ', level=0)
    cleaned_tree.write(cleaned_urdf_path, encoding='utf-8', xml_declaration=True)
    cleaned_urdf_bytes = cleaned_urdf_path.read_bytes()
    cleaned_urdf_sha256 = hashlib.sha256(cleaned_urdf_bytes).hexdigest()

    link_body_map = {n: source_robot.get_body(n, actuated_chain) for n in source_robot.links}

    link_tris_cad_mm: dict[str, np.ndarray] = {}
    link_materials: dict[str, str] = {}
    base_source_triangles_cad_mm = []

    for n in source_robot.links:
        if n == 'none_1':
            continue
        t_accum = []
        for raw_filename, v_mat in source_robot.visuals(n):
            mesh_name = Path(raw_filename).name
            pts, tris = loaded_meshes[mesh_name]
            source_pts, source_tris = source_meshes[mesh_name]
            M = F_cad[n] @ v_mat
            pts_cad_mm = ((pts @ M[:3, :3].T + M[:3, 3]) * 1000.0)
            t_accum.append(pts_cad_mm[tris])
            if link_body_map[n] == 'base':
                source_pts_cad_mm = ((source_pts @ M[:3, :3].T + M[:3, 3]) * 1000.0)
                base_source_triangles_cad_mm.append(source_pts_cad_mm[source_tris].reshape(-1, 3))
            link_materials[n] = MATERIAL_MAP.get(mesh_name, 'hardware')
        if t_accum:
            link_tris_cad_mm[n] = np.concatenate(t_accum)

    base_pts_cad_mm = np.concatenate(base_source_triangles_cad_mm)

    up_cad = R_UP.T @ np.array([0.0, 0.0, 1.0])
    base_floor_cad = (base_pts_cad_mm @ up_cad).min()

    j1_joint_elem = source_robot.joints['revolute_2']
    m_j1_cad = F_cad[j1_joint_elem.find('parent').get('link')] @ source_robot.joint_origin(j1_joint_elem)
    j1_origin_cad_mm = m_j1_cad[:3, 3] * 1000.0

    origin_cad_mm = j1_origin_cad_mm - (j1_origin_cad_mm @ up_cad - base_floor_cad) * up_cad

    def world(pts_cad):
        return (np.asarray(pts_cad, dtype=np.float64) - origin_cad_mm) @ R_UP.T

    cad_to_world_matrix = [
        [1.0,  0.0,  0.0, -float(origin_cad_mm[0])],
        [0.0, -1.0,  0.0,  float(origin_cad_mm[1])],
        [0.0,  0.0, -1.0,  float(origin_cad_mm[2])],
        [0.0,  0.0,  0.0,  1.0                    ]
    ]

    ordered_joints = []
    joint_specs = [
        ('J1 base', 'base', 'revolute_2', 0, [-90.0, 90.0], 'provisional kinematic display bound centered on uncommissioned zero'),
        ('J2 shoulder', 'shoulder', 'revolute_1', 1, None, 'continuous CAD type does not establish physical clearance; commissioning required'),
        ('J3 elbow', 'elbow', 'revolute_3', 2, None, 'continuous CAD type does not establish physical clearance; commissioning required'),
    ]

    for name, role, jn, motor_id, limits_deg, limit_provenance in joint_specs:
        j_elem = source_robot.joints[jn]
        parent_link = j_elem.find('parent').get('link')
        m_j = F_cad[parent_link] @ source_robot.joint_origin(j_elem)
        origin_cad_val = m_j[:3, 3] * 1000.0
        axis_cad_val = m_j[:3, :3] @ source_robot.joint_axis(j_elem)
        axis_cad_val = axis_cad_val / np.linalg.norm(axis_cad_val)

        origin_world = world(origin_cad_val)
        axis_world = axis_cad_val @ R_UP.T
        axis_world = axis_world / np.linalg.norm(axis_world)

        ordered_joints.append({
            'name': name,
            'role': role,
            'urdf': jn,
            'motor_id': motor_id,
            'origin': [float(x) for x in origin_world],
            'axis': [float(x) for x in axis_world],
            'limits_deg': limits_deg,
            'limit_provenance': limit_provenance,
        })

    bodies_dict: dict[str, dict] = {'base': {}, 'j1': {}, 'j2': {}, 'j3': {}}

    for b in ['base', 'j1', 'j2', 'j3']:
        b_links = [n for n in link_tris_cad_mm if link_body_map[n] == b]
        materials_in_body = sorted({link_materials[n] for n in b_links})

        for mat in materials_in_body:
            tris_in_mat = [world(link_tris_cad_mm[n].reshape(-1, 3)).reshape(-1, 3, 3)
                           for n in b_links if link_materials[n] == mat]
            if not tris_in_mat:
                continue
            T = np.concatenate(tris_in_mat)

            pos, indices = _pack_millimeter_render_mesh(T)
            idx_bytes = indices.tobytes()

            bin_filename = f"{b}.{mat}.bin"
            bin_rel_path = f"meshes/{bin_filename}"
            bin_path = out_meshes_dir / bin_filename
            bin_data = pos.tobytes() + idx_bytes
            bin_path.write_bytes(bin_data)
            bin_sha256 = hashlib.sha256(bin_data).hexdigest()

            bodies_dict[b][mat] = {
                'file': bin_rel_path,
                'vertices': len(pos),
                'triangles': len(indices),
                'sha256': bin_sha256,
            }

    model_json_content = {
        'schema': 1,
        'units': 'mm',
        'source': {
            'archive_file': archive_path.name,
            'archive_sha256': archive_sha256,
            'urdf_file': urdf_path.name,
            'urdf_sha256': urdf_sha256,
        },
        'cleaned_urdf': {
            'file': 'arm.urdf',
            'units': 'm',
            'sha256': cleaned_urdf_sha256,
        },
        'joints': ordered_joints,
        'bodies': bodies_dict,
        'cad_to_world': {
            'matrix': cad_to_world_matrix,
            'origin': [float(x) for x in origin_cad_mm],
            'r_up': R_UP.tolist(),
        },
        'portable_meshes': portable_mesh_entries,
        'cleanup_provenance': cleanup_provenance,
        'placeholder_inertials': True,
        'dynamics_validated': False,
        'collision_validated': False,
    }

    model_json_path = output_dir / 'model.json'
    model_json_path.write_text(json.dumps(model_json_content, indent=2) + '\n', encoding='utf-8')

    return model_json_content


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Generate kinematic URDF, portable STLs, and rendering assets from CAD export.'
    )
    parser.add_argument(
        '--source', required=True, type=Path,
        help='Already-extracted export directory containing mainassem/urdf/mainassem.urdf'
    )
    parser.add_argument(
        '--archive', required=True, type=Path,
        help='Original zip archive file'
    )
    parser.add_argument(
        '--output', required=True, type=Path,
        help='New output directory for generated assets'
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        build_manual_arm(args.source, args.archive, args.output)
        print(f"Successfully generated manual arm model in {args.output.resolve()}", file=sys.stderr)
        return 0
    except Exception as e:
        print(f"Error building manual arm model: {e}", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
