#!/usr/bin/env python3
"""Audit structural URDF and produce a repeatable, STL-only R2025a bundle.

Does not create SolidWorks models, certify CAD properties, or overwrite control
plugin URDF. Import folders are immutable runs; use a new folder on re-export.
"""
import argparse
import copy
import hashlib
import importlib.metadata
import json
import math
import pathlib
import shutil
import xml.etree.ElementTree as ET
from datetime import datetime, timezone


def numbers(text, count):
    try:
        result = [float(value) for value in text.split()]
    except (AttributeError, ValueError) as error:
        raise ValueError('Missing or invalid numeric property') from error
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError('Non-finite value or incorrect vector length')
    return result


def origin(element):
    node = element.find('origin')
    if node is None:
        return {'xyz': [0., 0., 0.], 'rpy': [0., 0., 0.]}
    return {'xyz': numbers(node.get('xyz', '0 0 0'), 3),
            'rpy': numbers(node.get('rpy', '0 0 0'), 3)}


def determinant(a):
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] ** 2)
            - a[0][1] * (a[0][1] * a[2][2] - a[1][2] * a[0][2])
            + a[0][2] * (a[0][1] * a[1][2] - a[1][1] * a[0][2]))


def inertia_tensor(node):
    names = ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')
    if node is None:
        raise ValueError('Moving rigid body requires a full COM inertia tensor')
    values = {name: numbers(node.get(name), 1)[0] for name in names}
    x, y, z, xy, xz, yz = (values[name] for name in names)
    scale = max(x, y, z)
    if scale <= 0:
        raise ValueError('Inertia must be positive definite')
    a = [[x / scale, xy / scale, xz / scale],
         [xy / scale, y / scale, yz / scale],
         [xz / scale, yz / scale, z / scale]]
    if a[0][0] <= 0 or a[0][0] * a[1][1] - a[0][1] ** 2 <= 0 or determinant(a) <= 0:
        raise ValueError('Inertia must be positive definite')
    # A physical inertia also requires C = trace(I)/2 * identity - I >= 0.
    half_trace = sum(a[i][i] for i in range(3)) / 2
    c = [[(half_trace if i == j else 0) - a[i][j] for j in range(3)] for i in range(3)]
    minors = [c[i][i] for i in range(3)]
    minors += [c[i][i] * c[j][j] - c[i][j] ** 2 for i, j in ((0, 1), (0, 2), (1, 2))]
    minors.append(determinant(c))
    if min(minors) < -1e-10:
        raise ValueError('Inertia violates physical principal-moment inequalities')
    return values


def mesh_path(uri, base):
    if not uri:
        raise ValueError('Mesh filename missing')
    if uri.startswith('package://'):
        parts = uri[len('package://'):].split('/', 1)
        if len(parts) != 2:
            raise ValueError('Invalid package mesh URI')
        uri = parts[1]  # --package-root explicitly identifies this exported package.
    elif '://' in uri:
        raise ValueError('Only local exported mesh files are accepted')
    target = (base / uri).resolve()
    if not target.is_relative_to(base.resolve()) or not target.is_file():
        raise ValueError('Mesh missing or outside exported package: ' + uri)
    if target.suffix.lower() != '.stl':
        raise ValueError('Initial adapter supports STL only; textured/DAE exports need a separate compatibility gate')
    return target


def audit(root, package_root):
    base = pathlib.Path(package_root)
    if root.tag != 'robot' or not root.get('name'):
        raise ValueError('Structural URDF robot/name required (not plugin configuration)')
    nodes = root.findall('link')
    links = {node.get('name'): node for node in nodes}
    if not links or None in links or len(links) != len(nodes):
        raise ValueError('Unique named links required')
    joints = root.findall('joint')
    active = [joint for joint in joints if joint.get('type') != 'fixed']
    if len(active) != 1 or active[0].get('type') != 'revolute':
        raise ValueError('Exactly one bounded revolute DOF is required')
    children, graph, names = set(), {name: [] for name in links}, set()
    for joint in joints:
        parent, child = joint.find('parent'), joint.find('child')
        if parent is None or child is None or not joint.get('name') or joint.get('name') in names:
            raise ValueError('Unique named joints with parent/child required')
        names.add(joint.get('name'))
        p, c = parent.get('link'), child.get('link')
        if p not in links or c not in links or p == c or c in children:
            raise ValueError('Invalid or repeated parent/child link')
        children.add(c); graph[p].append(c)
        origin(joint)
    roots = set(links) - children
    if len(roots) != 1:
        raise ValueError('One connected tree root required')
    visited = set()
    def walk(name):
        if name in visited:
            raise ValueError('URDF cycle')
        visited.add(name)
        for child in graph[name]: walk(child)
    root_name = next(iter(roots)); walk(root_name)
    if visited != set(links):
        raise ValueError('Disconnected link or cycle')
    properties = {}
    meshes = []
    for name, node in links.items():
        inertial = node.find('inertial')
        if inertial is None and name != root_name:
            raise ValueError('Non-root rigid body missing inertial: ' + name)
        if inertial is not None:
            mass = inertial.find('mass')
            value = numbers(mass.get('value') if mass is not None else None, 1)[0]
            if value <= 0:
                raise ValueError('Mass must be positive')
            pose = origin(inertial)
            properties[name] = {'mass_kg': value, 'center_of_mass_m': pose['xyz'],
                                'inertia_frame_rpy_rad': pose['rpy'],
                                'inertia_kg_m2': inertia_tensor(inertial.find('inertia'))}
        else:
            properties[name] = {'fixed_base_without_inertial': True}
        for feature in node.findall('visual') + node.findall('collision'):
            origin(feature)
            geometry = feature.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise ValueError('One geometry primitive or mesh per visual/collision required')
            shape = geometry[0]
            if shape.tag == 'mesh':
                mesh_path(shape.get('filename'), base)
                scale = numbers(shape.get('scale', '1 1 1'), 3)
                if min(scale) <= 0: raise ValueError('Mesh scale must be positive')
                meshes.append({'uri': shape.get('filename'), 'scale': scale})
            else:
                dims = (numbers(shape.get('size'), 3) if shape.tag == 'box' else
                        numbers(shape.get('radius'), 1) if shape.tag == 'sphere' else
                        numbers(shape.get('radius'), 1) + numbers(shape.get('length'), 1) if shape.tag == 'cylinder' else [])
                if not dims or min(dims) <= 0 or max(dims) > 2:
                    raise ValueError('Invalid geometry/SI size (this mini joint must be below 2 m)')
    joint = active[0]
    axis_node, limit = joint.find('axis'), joint.find('limit')
    axis = numbers(axis_node.get('xyz') if axis_node is not None else '1 0 0', 3)
    if abs(sum(v * v for v in axis) - 1) > 1e-6:
        raise ValueError('Joint axis must be a nonzero unit vector')
    if limit is None: raise ValueError('Joint limits required')
    bounds = {key: numbers(limit.get(key), 1)[0] for key in ('lower', 'upper', 'effort', 'velocity')}
    if bounds['lower'] >= bounds['upper'] or bounds['effort'] <= 0 or bounds['velocity'] <= 0:
        raise ValueError('Invalid joint limits')
    return {'robot': root.get('name'), 'root_link': root_name, 'links': properties, 'meshes': meshes,
            'joint': {'name': joint.get('name'), 'axis': axis, 'origin': origin(joint), **bounds},
            'manual_checks_pending': ['CAD units and STL bounding dimensions', 'CAD mass/COM/tensor and frame identity',
                                      'horizontal zero and positive lifting direction', 'Webots dynamics and motion',
                                      'control device mapping; structural URDF does not replace plugin configuration']}


def digest(file):
    return hashlib.sha256(pathlib.Path(file).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--package-root', help='Exported package directory containing meshes (default: URDF folder)')
    parser.add_argument('--output', help='New run folder; refuses to overwrite an existing folder')
    parser.add_argument('--audit-only', action='store_true')
    args = parser.parse_args()
    source = pathlib.Path(args.input).resolve()
    base = pathlib.Path(args.package_root).resolve() if args.package_root else source.parent
    tree = ET.parse(source)
    report = audit(tree.getroot(), base)
    report.update(source_urdf=str(source), source_sha256=digest(source),
                  created_utc=datetime.now(timezone.utc).isoformat(), exporter_origin='not_certified_by_this_tool')
    if args.audit_only:
        print(json.dumps(report, ensure_ascii=False, indent=2)); return
    if not args.output: parser.error('--output required for conversion')
    version = importlib.metadata.version('urdf2webots')
    if version != '2025.0.0': raise ValueError('Pinned urdf2webots 2025.0.0 required; got ' + version)
    output = pathlib.Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    adapted = copy.deepcopy(tree)
    copied = {}
    for node in adapted.getroot().iter('mesh'):
        original = mesh_path(node.get('filename'), base)
        if original not in copied:
            relative = pathlib.Path('meshes') / ('mesh_' + str(len(copied)) + '.stl')
            (output / 'meshes').mkdir(exist_ok=True)
            shutil.copyfile(original, output / relative)
            copied[original] = relative.as_posix()
        node.set('filename', copied[original])
    adapted_path = output / 'adapted.urdf'
    adapted.write(adapted_path, encoding='utf-8', xml_declaration=True)
    from urdf2webots.importer import convertUrdfFile
    proto = output / 'JointCad.proto'
    convertUrdfFile(input=str(adapted_path), output=str(proto), linkToDef=True, jointToDef=True, targetVersion='R2025a')
    if not proto.is_file() or proto.stat().st_size == 0:
        raise RuntimeError('Converter did not produce a PROTO; compatibility gate failed')
    report.update(converter_version=version, target='R2025a',
                  files_sha256={p.relative_to(output).as_posix(): digest(p) for p in output.rglob('*') if p.is_file()},
                  input_mesh_sha256={str(p): digest(p) for p in copied},
                  conversion_only=True, solidworks_export_and_webots_run_verified=False)
    (output / 'import_manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Converted and audited:', proto, '\nCAD correspondence and Webots runtime remain unverified.')


if __name__ == '__main__':
    main()
