#!/usr/bin/env python3
"""Generate a nominal test fixture from geometric equations, not final native CAD.

Density=2700 kg/m³ is a declared aluminium hypothesis. T001 geometry was
checked previously, but its unspecified material is NOT certified by this.
"""
import argparse
import csv
import hashlib
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _sum_bodies(parts):
    mass = sum(p[0] for p in parts)
    cx = sum(p[0] * p[1] for p in parts) / mass
    cy = sum(p[0] * p[2] for p in parts) / mass
    cz = sum(p[0] * p[3] for p in parts) / mass
    return dict(mass_kg=mass, com_x_m=cx, com_y_m=cy, com_z_m=cz,
                ixx=sum(p[4] + p[0] * (p[2] ** 2 + p[3] ** 2) for p in parts) - mass * (cy ** 2 + cz ** 2),
                iyy=sum(p[5] + p[0] * (p[1] ** 2 + p[3] ** 2) for p in parts) - mass * (cx ** 2 + cz ** 2),
                izz=sum(p[6] + p[0] * (p[1] ** 2 + p[2] ** 2) for p in parts) - mass * (cx ** 2 + cy ** 2),
                ixy=sum(p[7] - p[0] * p[1] * p[2] for p in parts) + mass * cx * cy,
                ixz=sum(p[8] - p[0] * p[1] * p[3] for p in parts) + mass * cx * cz,
                iyz=sum(p[9] - p[0] * p[2] * p[3] for p in parts) + mass * cy * cz)


def nominal_body(payload_mass_kg, cam_body=None):
    if not math.isfinite(payload_mass_kg) or payload_mass_kg < 0:
        raise ValueError('Payload must be nonnegative and finite')
    density, length, width, thick = 2700.0, .180, .030, .008
    m = density * length * width * thick
    parts = [(m, .075, 0, 0, m * (width ** 2 + thick ** 2) / 12,
              m * (length ** 2 + width ** 2) / 12, m * (length ** 2 + thick ** 2) / 12, 0, 0, 0)]
    for x, radius in ((0, .006), (.150, .003)):
        removed = -density * math.pi * radius ** 2 * thick
        diam = removed * (radius ** 2 / 4 + thick ** 2 / 12)
        parts.append((removed, x, 0, 0, diam, removed * radius ** 2 / 2, diam, 0, 0, 0))
    # A 15mm cube at x=.150m. Empty payload has no mass but remains an outline.
    inertia = payload_mass_kg * .015 ** 2 / 6
    parts.append((payload_mass_kg, .150, 0, 0, inertia, inertia, inertia, 0, 0, 0))
    if cam_body:
        c = cam_body
        parts.append(tuple(c[k] for k in ('mass_kg', 'com_x_m', 'com_y_m', 'com_z_m',
                                         'ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')))
    return _sum_bodies(parts)


def cam_mass_properties(cam, thickness_m=.005, density_kg_m3=2700, samples=2048):
    step = 2 * math.pi / samples
    area = fx = fz = x2 = z2 = xz = 0.0
    for i in range(samples):
        phi = i * step
        r = cam.radius_derivatives(phi)[0]
        c, s = math.cos(phi), math.sin(phi)
        area += .5 * r ** 2 * step
        fx += r ** 3 * c * step / 3
        fz += r ** 3 * s * step / 3
        x2 += r ** 4 * c ** 2 * step / 4
        z2 += r ** 4 * s ** 2 * step / 4
        xz += r ** 4 * c * s * step / 4
    scale = thickness_m * density_kg_m3
    mass = scale * area
    cx, cz = fx / area, fz / area
    return dict(mass_kg=mass, com_x_m=cx, com_y_m=-.012, com_z_m=cz,
                ixx=scale * z2 + mass * thickness_m ** 2 / 12 - mass * cz ** 2,
                iyy=scale * (x2 + z2) - mass * (cx ** 2 + cz ** 2),
                izz=scale * x2 + mass * thickness_m ** 2 / 12 - mass * cx ** 2,
                ixy=0, ixz=-scale * xz + mass * cx * cz, iyz=0)


def write_cam_visual(cam, path, samples=128):
    """Derived Webots geometry, NOT a native CAD source or contact validation."""
    vertices = []
    for y in (-.0025, .0025):
        for i in range(samples):
            angle = 2 * math.pi * i / samples
            radius = cam.radius_derivatives(angle)[0]
            vertices.append((radius * math.cos(angle), y, radius * math.sin(angle)))
    faces = [list(range(samples)), list(range(2 * samples - 1, samples - 1, -1))]
    for i in range(samples):
        n = (i + 1) % samples
        faces.append([i, i + samples, n + samples, n])
    points = '\n'.join('      %.12g %.12g %.12g,' % point for point in vertices)
    indices = '\n'.join('      ' + ', '.join(str(v) for v in face) + ', -1,' for face in faces)
    pathlib.Path(path).write_text(
        '#VRML_SIM R2025a utf8\n# Derived from prepare_joint_study.py. Not SolidWorks CAD.\n'
        'PROTO StudyCamGeometry [] {\n  IndexedFaceSet {\n    coord Coordinate { point [\n' + points +
        '\n    ] }\n    coordIndex [\n' + indices + '\n    ]\n    convex TRUE\n  }\n}\n', encoding='utf-8')


def prepare(output_dir, report_path):
    from research.gravity_compensation.geometry import FourierCam, RollerCompensator
    output_dir = pathlib.Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    cam = FourierCam(.025, cos_m=(-.002, 0, 0, 0))
    write_cam_visual(cam, output_dir / 'StudyCamGeometry.proto')
    moving_cam = cam_mass_properties(cam)
    factory = lambda preload: RollerCompensator(cam, .004, 0, 10000, preload, .035,
                                               angle_limit_rad=math.pi / 3)
    # Adjust once for a known load at horizontal zero. This is not a fitted
    # ideal torque curve and does not optimize the shape over the travel.
    unit_torque = factory(.01).evaluate(0).torque_nm / .01
    if unit_torque <= 0: raise ValueError('Cam has wrong compensation direction')
    body_fields = ['payload_index', 'side', 'mass_kg', 'com_x_m', 'com_y_m', 'com_z_m',
                   'ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']
    rows, cases = [], []
    for index, load in enumerate((0.0, .125, .25)):
        baseline = nominal_body(load)
        candidate = nominal_body(load, moving_cam)
        # Full COM determines gravity including added eccentric cam mass.
        zero_gravity = candidate['mass_kg'] * candidate['com_x_m'] * 9.81
        preload = zero_gravity / unit_torque
        geometry = factory(preload)
        path = output_dir / ('study_energy_' + str(index) + '.csv')
        states = [geometry.evaluate(-math.pi / 3 + i * math.pi / 180 / 20)
                  for i in range(2401)]
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=['angle_rad', 'energy_j', 'compression_m'])
            writer.writeheader()
            writer.writerows(dict(angle_rad=s.q_rad, energy_j=s.energy_j,
                                  compression_m=s.spring_compression_m) for s in states)
        for side, properties in (('baseline', baseline), ('candidate', candidate)):
            rows.append(dict(payload_index=index, side=side, **properties))
        cases.append(dict(payload_mass_kg=load, preload_m=preload, zero_gravity_nm=zero_gravity,
                          compression_min_m=min(s.spring_compression_m for s in states),
                          compression_max_m=max(s.spring_compression_m for s in states),
                          max_pressure_angle_rad=max(abs(s.pressure_angle_rad) for s in states),
                          geometry_nodes=len(states)))
    body_path = output_dir / 'study_bodies.csv'
    with body_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=body_fields)
        writer.writeheader()
        writer.writerows(rows)
    source = ROOT / 'mechanical/cad_native/import_test/T001_test_link_r01.SLDPRT'
    report = dict(status='nominal_geometry_fixture_not_native_CAD_export',
                  density_hypothesis_kg_m3=2700, gravity_m_s2=9.81,
                  native_material_verified=False, native_inertia_verified=False,
                  explicit_contact_validated=False, optimizer_run=False,
                  source_native_sha256=hashlib.sha256(source.read_bytes()).hexdigest() if source.exists() else None,
                  cam=dict(mean_radius_m=.025, cos_m=[-.002, 0, 0, 0], sin_m=[0, 0, 0, 0],
                           thickness_m=.005, roller_radius_m=.004, stiffness_n_per_m=10000,
                           max_compression_m=.035, mass_properties=moving_cam),
                  cases=cases, rigid_bodies=rows,
                  generated_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in output_dir.glob('study_*.csv')})
    report_path = pathlib.Path(report_path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default=str(ROOT / 'ros2_ws/src/ai_robotic_joint_ros2/resource'))
    parser.add_argument('--report', default=str(ROOT / 'data/verification/study_fixture_geometry.json'))
    args = parser.parse_args()
    result = prepare(args.output_dir, args.report)
    print(json.dumps(dict(status=result['status'], cases=result['cases']), ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
