"""Offline quasi-static geometry sweep, not Webots feedback or measured loss."""

from __future__ import annotations

import math
import hashlib
import json
import argparse
import csv
import io
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.gravity_compensation import FourierCam, GeometryError, RollerCompensator


def _gravity(body, q, g):
    return -body["mass_kg"] * g * (body["com_x_m"] * math.cos(q)
                                   - body["com_z_m"] * math.sin(q))


def _rms(values):
    return math.sqrt(sum(v*v for v in values) / len(values))


def _crossovers(metrics, cases, ratios):
    result = []
    for index, case in enumerate(cases):
        scan, bracket, previous = [], None, None
        for ratio in sorted(ratios):
            group = [row for row in metrics if row["case_index"] == index and row["loss_ratio"] == ratio]
            benefits = [r["torque_rms_benefit_fraction"] for r in group
                        if r["valid"] and r["torque_rms_benefit_fraction"] is not None]
            guaranteed = len(benefits) == len(group)
            worst = min(benefits) if benefits else None
            scan.append(dict(loss_ratio=ratio, all_scenarios_valid=guaranteed,
                             worst_direction_benefit_fraction=worst))
            if bracket is None and guaranteed and worst <= 0:
                bracket = [previous, ratio]
            previous = ratio
        result.append(dict(payload_mass_kg=case["payload_mass_kg"], loss_scan=scan,
                           sampled_loss_ratio_bracket=bracket,
                           bracket_is_not_a_continuous_threshold=True))
    return result


def analyze_fixture(fixture, *, samples=181, stiffness_factors=(0.9, 1.0, 1.1),
                    preload_offsets_m=(-0.0005, 0.0, 0.0005),
                    loss_ratios=(0.0, 0.05, 0.10, 0.20, 0.50, 1.0, 2.0)):
    """Recompute passive compensation from geometry; never read torque tables."""
    if isinstance(samples, bool) or not isinstance(samples, int) or not 3 <= samples <= 4097:
        raise ValueError("samples must be an integer in [3, 4097]")
    for name, values in (("stiffness factors", stiffness_factors),
                         ("preload offsets", preload_offsets_m), ("loss ratios", loss_ratios)):
        if not values or any(isinstance(v, bool) or not isinstance(v, (int, float))
                             or not math.isfinite(v) for v in values):
            raise ValueError(f"{name} must contain finite numbers")
    if any(v <= 0 for v in stiffness_factors) or any(v < 0 for v in loss_ratios):
        raise ValueError("Require positive stiffness factors and nonnegative losses")
    if not fixture["cases"]:
        raise ValueError("At least one payload case is required")
    angles = [-math.pi/4 + (math.pi/2)*i/(samples-1) for i in range(samples)]
    cam_data, gravity = fixture["cam"], fixture["gravity_m_s2"]
    if not math.isfinite(gravity) or gravity <= 0:
        raise ValueError("Gravity must be finite and positive")
    cam = FourierCam(cam_data["mean_radius_m"], tuple(cam_data["cos_m"]),
                     tuple(cam_data["sin_m"]))
    metrics, nominal_curves = [], []
    for index, case in enumerate(fixture["cases"]):
        if not math.isfinite(case["payload_mass_kg"]) or case["payload_mass_kg"] < 0:
            raise ValueError("Declared payload must be finite and nonnegative")
        selected = [b for b in fixture["rigid_bodies"] if b["payload_index"] == index]
        bodies = {b["side"]: b for b in selected}
        if len(selected) != 2 or set(bodies) != {"baseline", "candidate"}:
            raise ValueError("Each case needs its own baseline and candidate body")
        for body in bodies.values():
            if body["mass_kg"] <= 0 or any(not math.isfinite(body[k])
                                           for k in ("mass_kg", "com_x_m", "com_z_m")):
                raise ValueError("Body mass/COM must be finite, with positive mass")
        baseline = [-_gravity(bodies["baseline"], q, gravity) for q in angles]
        candidate_gravity = [_gravity(bodies["candidate"], q, gravity) for q in angles]
        baseline_rms = _rms(baseline)
        # Find nominal gravitational amplitude within the operating interval,
        # rather than allowing a coarse grid to understate the loss scale.
        body = bodies["candidate"]
        critical = math.atan2(-body["com_z_m"], body["com_x_m"])
        peak_angles = [angles[0], angles[-1]] + [critical + i*math.pi for i in (-1, 0, 1)
                                                if angles[0] <= critical+i*math.pi <= angles[-1]]
        gravity_scale = max(abs(_gravity(body, q, gravity)) for q in peak_angles)
        nominal = RollerCompensator(cam, cam_data["roller_radius_m"], 0.0,
                                    cam_data["stiffness_n_per_m"], case["preload_m"],
                                    cam_data["max_compression_m"])
        nominal_states = [nominal.evaluate(q) for q in angles]
        nominal_curves.append(dict(
            payload_mass_kg=case["payload_mass_kg"],
            baseline_gravity_nm=[-v for v in baseline],
            candidate_gravity_nm=candidate_gravity,
            passive_torque_nm=[s.torque_nm for s in nominal_states],
            spring_compression_m=[s.spring_compression_m for s in nominal_states],
            spring_energy_j=[s.energy_j for s in nominal_states],
        ))
        for factor in stiffness_factors:
            for offset in preload_offsets_m:
                failure, candidate, states = "", None, []
                try:
                    module = RollerCompensator(
                        cam, cam_data["roller_radius_m"], 0.0,
                        cam_data["stiffness_n_per_m"] * factor,
                        case["preload_m"] + offset, cam_data["max_compression_m"],
                    )
                    for q in angles:
                        states.append(module.evaluate(q))
                    candidate = [-g-s.torque_nm for g, s in zip(candidate_gravity, states)]
                except GeometryError as exc:
                    failure = f"{exc}; evaluated {len(states)}/{samples} angle nodes"
                for loss in loss_ratios:
                    loss_nm = gravity_scale * loss
                    for direction in ("upward", "downward"):
                        sign = 1 if direction == "upward" else -1
                        required = None if candidate is None else [v + sign*loss_nm for v in candidate]
                        candidate_rms = None if required is None else _rms(required)
                        metrics.append(dict(
                            case_index=index, payload_mass_kg=case["payload_mass_kg"], stiffness_factor=factor,
                            preload_offset_m=offset, loss_ratio=loss, direction=direction,
                            valid=candidate is not None, failure=failure,
                            baseline_motor_rms_nm=baseline_rms,
                            baseline_motor_peak_nm=max(abs(v) for v in baseline),
                            candidate_motor_rms_nm=candidate_rms,
                            candidate_motor_peak_nm=None if required is None else max(abs(v) for v in required),
                            additional_loss_nm=loss_nm,
                            loss_loop_area_j=None if candidate is None else 2*loss_nm*(angles[-1]-angles[0]),
                            torque_rms_benefit_fraction=None if candidate_rms is None or baseline_rms == 0
                                                       else 1-candidate_rms/baseline_rms,
                            compression_min_m=None if candidate is None else min(s.spring_compression_m for s in states),
                            compression_max_m=None if candidate is None else max(s.spring_compression_m for s in states),
                        ))
    config = dict(fixture=fixture, samples=samples, stiffness_factors=stiffness_factors,
                  preload_offsets_m=preload_offsets_m, loss_ratios=loss_ratios,
                  q_min_rad=angles[0], q_max_rad=angles[-1])
    digest = hashlib.sha256(json.dumps(config, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return dict(status="offline_static_synthetic_scan_not_webots_feedback",
                model="R0 ideal baseline versus F-N unoptimized geometry; not five-group study",
                assumptions={"uncertainty": "synthetic sweep; not measured/manufacturer tolerances",
                             "additional_loss": "candidate-only incremental moving-branch Coulomb torque L; no stiction; R0 ideal reference",
                             "loss_scale": "L=ratio*peak nominal candidate gravity in operating range",
                             "loop_area": "conditional model dissipation 2*L*angular_span; not measured hysteresis",
                             "rms": "uniform-angle node RMS, not time-series/measured RMS",
                             "excluded": "dynamics, actual feedback, control accuracy, energy savings, optimization"},
                config=config, config_sha256=digest, angle_grid_rad=angles,
                nominal_curves=nominal_curves, metrics=metrics,
                crossovers=_crossovers(metrics, fixture["cases"], loss_ratios),
                counts=dict(metric_rows=len(metrics), valid_rows=sum(r["valid"] for r in metrics),
                            failed_rows=sum(not r["valid"] for r in metrics)))


def write_report(fixture_path, csv_path, json_path, *, samples=181):
    """Write only this generator's derived outputs, with reproducibility hashes."""
    fixture_path, csv_path, json_path = (Path(p).resolve() for p in (fixture_path, csv_path, json_path))
    if not all(path.resolve().is_relative_to(ROOT) for path in (csv_path, json_path)):
        raise ValueError("Derived outputs must stay inside this project")
    if csv_path.resolve() == json_path.resolve() or fixture_path.resolve() in (csv_path.resolve(), json_path.resolve()):
        raise ValueError("Input, CSV and JSON paths must be distinct")
    generator = "scripts/analyze_study_fixture.py"
    if csv_path.exists() or json_path.exists():
        if not json_path.exists() or json.loads(json_path.read_text(encoding="utf-8")).get("generator") != generator:
            raise ValueError("Refusing to overwrite outputs without this generator's provenance")
    source = fixture_path.read_bytes()
    report = analyze_fixture(json.loads(source), samples=samples)
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=list(report["metrics"][0]))
    writer.writeheader()
    writer.writerows(report["metrics"])
    csv_bytes = stream.getvalue().encode("utf-8")
    report.update(generator=generator, input_file=str(fixture_path.relative_to(ROOT)),
                  input_sha256=hashlib.sha256(source).hexdigest(),
                  analysis_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  geometry_code_sha256=hashlib.sha256((ROOT/"research/gravity_compensation/geometry.py").read_bytes()).hexdigest(),
                  csv_sha256=hashlib.sha256(csv_bytes).hexdigest())
    json_bytes = (json.dumps(report, indent=2, allow_nan=False) + "\n").encode("utf-8")
    for path, content in ((csv_path, csv_bytes), (json_path, json_bytes)):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, default=ROOT/"data/verification/study_fixture_geometry.json")
    parser.add_argument("--csv", type=Path, default=ROOT/"data/processed/study_static_scan.csv")
    parser.add_argument("--json", type=Path, default=ROOT/"data/verification/study_static_scan.json")
    parser.add_argument("--samples", type=int, default=181)
    args = parser.parse_args()
    report = write_report(args.fixture, args.csv, args.json, samples=args.samples)
    print(json.dumps({"status": report["status"], "counts": report["counts"],
                      "crossovers": report["crossovers"], "config_sha256": report["config_sha256"]}, indent=2))


if __name__ == "__main__":
    main()
