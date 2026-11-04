"""T001 native part adapter for the user's existing AI+CAD executor.

No cloud request, no modification of AI+CAD, no closing existing documents.
All input lengths are mm; SolidWorks API lengths are metres.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
EXECUTOR = Path(os.environ.get('JOINT_MINI_SW_EXECUTOR', str(ROOT / 'local/solidworks_executor.py')))
TARGET = ROOT / 'mechanical/cad_native/import_test/T001_test_link_r01.SLDPRT'
REPORT = ROOT / 'data/verification/T001_native_solidworks_r01.json'
PREVIEW = ROOT / 'mechanical/cad_native/import_test/T001_test_link_r01.bmp'
PARAMETERS = dict(length_mm=180., height_mm=30., thickness_mm=8.,
                  pitch_mm=150., left_margin_mm=15., pivot_diameter_mm=12.,
                  tip_diameter_mm=6.)


def validate_parameters(p):
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
           not math.isfinite(v) or v <= 0 for v in p.values()):
        raise ValueError('All dimensions must be finite and positive.')
    if not math.isclose(p['length_mm'], p['pitch_mm'] + 2 * p['left_margin_mm']):
        raise ValueError('Length must equal pitch plus two end margins.')
    for key in ('pivot_diameter_mm', 'tip_diameter_mm'):
        if p[key] >= min(p['height_mm'], 2 * p['left_margin_mm']):
            raise ValueError('Hole touches or crosses plate boundary.')
    if p['pitch_mm'] <= (p['pivot_diameter_mm'] + p['tip_diameter_mm']) / 2:
        raise ValueError('Holes overlap.')


def expected_volume(p):
    area = p['length_mm'] * p['height_mm']
    area -= math.pi * (p['pivot_diameter_mm'] ** 2 + p['tip_diameter_mm'] ** 2) / 4
    return area * p['thickness_mm'] / 1e9


def read_api(obj, name):
    value = getattr(obj, name)
    # pywin32 dispatch objects are callable but are already the returned
    # object; calling them invokes DISPID_VALUE, not the original method.
    if hasattr(value, '_oleobj_'):
        return value
    return value() if callable(value) else value


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--resume', action='store_true', help='Resume only the owned unsaved document recorded in a failed report.')
    parser.add_argument('--restore-input-dim', choices=('true', 'false'), help='Restore a known original toggle after recovering an interrupted run.')
    args = parser.parse_args()
    validate_parameters(PARAMETERS)
    if args.check_only:
        print(json.dumps({'parameters': PARAMETERS, 'expected_volume_m3': expected_volume(PARAMETERS)}))
        return 0
    if TARGET.exists() or (REPORT.exists() and not args.resume):
        raise FileExistsError('Target/report already exists; refusing to overwrite or create duplicate parts.')
    if not EXECUTOR.exists():
        raise FileNotFoundError(EXECUTOR)

    import pythoncom
    from win32com.client import VARIANT
    pythoncom.CoInitialize()
    record = dict(part_id='T001', source_id='S008', parameters_mm=PARAMETERS,
                  utc_time=datetime.now(timezone.utc).isoformat(),
                  adapter_sha256=sha(__file__), reused_executor=str(EXECUTOR),
                  reused_executor_sha256=sha(EXECUTOR),
                  cloud_calls=0, native_saved=False, material_assigned=False,
                  status='started', stage='load_existing_executor')
    model = None
    app = None
    input_dim_original = None
    try:
        spec = importlib.util.spec_from_file_location('joint_existing_cad_executor', EXECUTOR)
        executor = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(executor)
        record['stage'] = 'connect'
        app = executor.connect_to_solidworks()
        input_dim_original = app.GetUserPreferenceToggle(10)
        if args.restore_input_dim is not None:
            input_dim_original = args.restore_input_dim == 'true'
        record['input_dimension_original'] = input_dim_original
        app.SetUserPreferenceToggle(10, False)
        record['solidworks_revision'] = read_api(app, 'RevisionNumber')
        template = app.GetUserPreferenceStringValue(executor.SW_DEFAULT_TEMPLATE_PART)
        if not template:
            template = app.GetDocumentTemplate(1, '', 0, 0., 0.)
        if not template or not Path(template).exists():
            raise RuntimeError('No available part template.')
        record['template'] = template
        record['stage'] = 'new_single_part'
        if args.resume:
            previous = json.loads(REPORT.read_text(encoding='utf-8'))
            if previous['status'] != 'failed' or previous['native_saved'] or previous['stage'] != 'native_profile':
                raise RuntimeError('Cannot resume beyond initial owned profile stage.')
            model = app.ActiveDoc
            if model is None or read_api(model, 'GetTitle') != previous['new_document_title'] or read_api(model, 'GetPathName'):
                raise RuntimeError('Active document is not the owned unsaved part.')
            record['prior_attempt'] = previous
        else:
            model = app.NewDocument(template, 0, 0., 0.)
        if model is None or read_api(model, 'GetType') != 1:
            raise RuntimeError('New document is not a part.')
        record['new_document_title'] = read_api(model, 'GetTitle')
        if read_api(model, 'IsOpenedReadOnly'):
            raise RuntimeError('Document is read-only.')

        record['stage'] = 'native_profile'
        skm = model.SketchManager
        p = PARAMETERS
        if not args.resume:
            record['plane'] = executor._select_first_reference_plane(model)
            skm.InsertSketch(True)
            rectangle = skm.CreateCornerRectangle(-p['left_margin_mm']/1000, -p['height_mm']/2000, 0.,
                                                  (p['length_mm']-p['left_margin_mm'])/1000,
                                                  p['height_mm']/2000, 0.)
            if rectangle is None or len(rectangle) != 4:
                raise RuntimeError('Rectangle creation failed.')
            pivot = skm.CreateCircleByRadius(0., 0., 0., p['pivot_diameter_mm']/2000)
            tip = skm.CreateCircleByRadius(p['pitch_mm']/1000, 0., 0., p['tip_diameter_mm']/2000)
            if pivot is None or tip is None:
                raise RuntimeError('Hole circles creation failed.')
        else:
            record['plane'] = previous['plane']
        sketch = read_api(model, 'GetActiveSketch2')
        if sketch is None and args.resume:
            current = model.FeatureByName('Sketch_LinkProfile')
            if current is None or not current.Select2(False, 0):
                raise RuntimeError('Owned profile not found for resume.')
            skm.InsertSketch(True)
            sketch = read_api(model, 'GetActiveSketch2')
        if sketch is None or len(read_api(sketch, 'GetSketchSegments')) != 6:
            raise RuntimeError('Expected active 4-line/2-circle sketch.')
        feature = read_api(model, 'FirstFeature')
        sketch_feature = None
        while feature is not None:
            if read_api(feature, 'GetTypeName2') == 'ProfileFeature':
                sketch_feature = feature
            feature = read_api(feature, 'GetNextFeature')
        if sketch_feature is None:
            raise RuntimeError('Active profile feature not found.')
        sketch_feature.Name = 'Sketch_LinkProfile'
        empty = VARIANT(pythoncom.VT_DISPATCH, None)

        record['stage'] = 'native_dimensions'
        dims = []
        def dim(entities, method, location, name, expected):
            print('Dimension: ' + name, flush=True)
            record['dimension_in_progress'] = name
            if args.resume and name == 'OverallLength':
                existing = model.Parameter('D1@Sketch_LinkProfile') or model.Parameter('OverallLength@Sketch_LinkProfile')
                if existing is not None and abs(existing.SystemValue - expected/1000) < 1e-9:
                    existing.Name = name
                    dims.append({'name': name, 'value_m': existing.SystemValue, 'reused': True})
                    return
            model.ClearSelection2(True)
            for i, entity in enumerate(entities):
                if isinstance(entity, tuple):
                    kind, x, y = entity
                    selected = model.Extension.SelectByID2('', kind, x, y, 0., i > 0, 0, empty, 0)
                else:
                    selected = entity.Select4(i > 0, empty)
                if not selected:
                    raise RuntimeError('Dimension selection failed: ' + name)
            display = getattr(model, method)(*location)
            if display is None:
                raise RuntimeError('Dimension failed: ' + name)
            dimension = display.GetDimension2(0)
            dimension.Name = name
            dimension.SystemValue = expected / 1000
            dims.append({'name': name, 'value_m': dimension.SystemValue})
        # Native driving dimensions on line lengths and hole diameters.
        def point_at(x, y):
            points = [v for v in read_api(sketch, 'GetSketchPoints2')
                      if abs(v.X-x) < 1e-9 and abs(v.Y-y) < 1e-9]
            if not points:
                raise RuntimeError('Expected sketch point missing.')
            return points[0]
        horizontal = ('SKETCHSEGMENT', .075, -.015)
        vertical = ('SKETCHSEGMENT', -.015, 0.)
        pivot = ('SKETCHSEGMENT', .006, 0.)
        tip = ('SKETCHSEGMENT', .153, 0.)
        bottom_left = point_at(-.015, -.015)
        dim([horizontal], 'AddDimension2', (.075, -.028, 0.), 'OverallLength', p['length_mm'])
        dim([vertical], 'AddDimension2', (-.026, 0., 0.), 'ProfileHeight', p['height_mm'])
        dim([pivot], 'AddDiameterDimension2', (.025, .025, 0.), 'PivotDiameter', p['pivot_diameter_mm'])
        dim([tip], 'AddDiameterDimension2', (.165, .025, 0.), 'TipDiameter', p['tip_diameter_mm'])
        pivot_center = point_at(0., 0.)
        tip_center = point_at(.150, 0.)
        model.ClearSelection2(True)
        if not pivot_center.Select4(False, empty):
            raise RuntimeError('Pivot center selection failed.')
        model.SketchAddConstraints('sgFIXED')
        dim([bottom_left, pivot_center], 'AddHorizontalDimension2', (-.0075, -.035, 0.), 'LeftMargin', p['left_margin_mm'])
        dim([bottom_left, pivot_center], 'AddVerticalDimension2', (-.035, -.0075, 0.), 'HalfHeight', p['height_mm']/2)
        model.ClearSelection2(True)
        if not pivot_center.Select4(False, empty) or not tip_center.Select4(True, empty):
            raise RuntimeError('Hole centers selection failed.')
        model.SketchAddConstraints('sgHORIZONTALPOINTS')
        dim([pivot_center, tip_center], 'AddHorizontalDimension2', (.075, .038, 0.), 'HolePitch', p['pitch_mm'])
        record['driving_dimensions'] = dims
        model.ClearSelection2(True)
        skm.InsertSketch(True)

        record['stage'] = 'midplane_extrusion'
        if not sketch_feature.Select2(False, 0):
            raise RuntimeError('Profile selection failed.')
        thickness = p['thickness_mm']/1000
        extrusion = model.FeatureManager.FeatureExtrusion2(
            True, False, False, 6, 0, thickness, thickness,
            False, False, False, False, 0., 0., False, False, False, False,
            True, True, True, 0, 0., False)
        if extrusion is None:
            raise RuntimeError('Midplane extrusion failed.')
        extrusion.Name = 'LinkPlateMidplane'
        read_api(model, 'EditRebuild3')

        record['stage'] = 'geometry_validation'
        bodies = model.GetBodies2(0, False)
        if not bodies or len(bodies) != 1:
            raise RuntimeError('Expected one solid body.')
        body = bodies[0]
        box = list(read_api(body, 'GetBodyBox'))
        expected_box = [-.015, -.015, -.004, .165, .015, .004]
        # SolidWorks bounding box is approximate; volume and cylindrical faces
        # provide independent checks rather than treating it as exact geometry.
        if any(abs(actual-expected) > 1e-5 for actual, expected in zip(box, expected_box)):
            raise RuntimeError('Body bounding box differs from requested envelope: ' + str(box))
        properties = body.GetMassProperties(1.)  # Unit density for geometry only.
        volume = float(properties[3])
        if not math.isclose(volume, expected_volume(p), rel_tol=1e-7, abs_tol=1e-12):
            raise RuntimeError('Volume mismatch.')
        cylinders = []
        for face in read_api(body, 'GetFaces'):
            surface = read_api(face, 'GetSurface')
            if read_api(surface, 'IsCylinder'):
                cp = list(read_api(surface, 'CylinderParams'))
                cylinders.append(cp)
        if len(cylinders) != 2:
            raise RuntimeError('Expected exactly two cylindrical through-hole walls.')
        cylinders.sort(key=lambda v: v[0])
        for cp, expected_x, radius in zip(cylinders, (0., .150), (.006, .003)):
            if abs(cp[0]-expected_x) > 1e-8 or abs(cp[1]) > 1e-8 or abs(cp[6]-radius) > 1e-8 or abs(abs(cp[5])-1) > 1e-8:
                raise RuntimeError('Hole radius/axis/center mismatch.')
        record.update(body_count=1, bounding_box_m=box, cylinder_surfaces=cylinders,
                      volume_m3=volume, expected_volume_m3=expected_volume(p),
                      geometric_centroid_m=list(properties[:3]),
                      material_mass_and_inertia_validated=False)
        features = []
        feature = read_api(model, 'FirstFeature')
        while feature is not None:
            features.append({'name': feature.Name, 'type': read_api(feature, 'GetTypeName2')})
            feature = read_api(feature, 'GetNextFeature')
        record['features'] = features
        try:
            record['sketch_constrained_status'] = read_api(sketch, 'GetConstrainedStatus')
        except Exception as error:
            record['sketch_constrained_status_read_error'] = str(error)

        record['stage'] = 'save_native'
        TARGET.parent.mkdir(parents=True, exist_ok=True)
        if TARGET.exists():
            raise FileExistsError(TARGET)
        model.ShowNamedView2('*Trimetric', 8)
        read_api(model, 'ViewZoomtofit2')
        errors = model.SaveAs3(str(TARGET), 0, 1)
        if errors != 0 or not TARGET.is_file():
            raise RuntimeError('Native SaveAs3 failed: ' + str(errors))
        record.update(native_saved=True, native_file=str(TARGET), native_sha256=sha(TARGET))
        record['stage'] = 'native_preview'
        try:
            if model.SaveBMP(str(PREVIEW), 1200, 500) and PREVIEW.exists():
                record.update(preview=str(PREVIEW), preview_sha256=sha(PREVIEW))
            else:
                record['preview_error'] = 'SaveBMP returned false or file absent.'
        except Exception as error:
            record['preview_error'] = str(error)
        record.update(status='geometry_verified_native_saved', stage='complete')
        print(json.dumps(record, ensure_ascii=False, indent=2))
        return 0
    except Exception as error:
        import traceback
        traceback.print_exc()
        record.update(status='failed', error=str(error), error_type=type(error).__name__)
        print(json.dumps(record, ensure_ascii=False, indent=2))
        return 1
    finally:
        if app is not None and input_dim_original is not None:
            try:
                app.SetUserPreferenceToggle(10, input_dim_original)
                record['input_dimension_preference_restored'] = app.GetUserPreferenceToggle(10) == input_dim_original
            except Exception as error:
                record['input_dimension_restore_error'] = str(error)
        REPORT.parent.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')
        pythoncom.CoUninitialize()


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    raise SystemExit(run())
