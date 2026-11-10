"""Reproduce synthetic software checks; this does not run ROS2 or Webots."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws/src/ai_robotic_joint_ros2"
GENERATOR = "scripts/verify_joint_study.py"
DEFAULT_OUTPUT = ROOT / "data/verification/study_software_verification.json"


def _text(value):
    return value.decode("utf-8", "replace") if isinstance(value, bytes) else (value or "")


def _capture(name, command, timeout, *, env=None):
    started = time.monotonic()
    result = dict(name=name, command=[str(x) for x in command], cwd=str(ROOT),
                  exit_code=None, status="blocked", elapsed_s=0.0, output="")
    try:
        process = subprocess.run(result["command"], cwd=ROOT, env=env, timeout=timeout,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 encoding="utf-8", errors="replace", check=False)
        result.update(exit_code=process.returncode,
                      status="passed" if process.returncode == 0 else "failed",
                      output=process.stdout)
    except subprocess.TimeoutExpired as exc:
        result.update(status="failed", reason="command exceeded timeout",
                      output=_text(exc.stdout))
    except OSError as exc:
        result["reason"] = f"tool could not be started: {exc}"
    result["elapsed_s"] = round(time.monotonic() - started, 6)
    return result


def _count(result, kind):
    """Counts are not combined: assertions, test cases and groups are different."""
    output = result["output"]
    if kind == "python":
        match = re.search(r"Ran (\d+) tests?\b", output)
        count, unit = (int(match[1]) if match else 0), "unittest_test_cases"
    elif kind == "manual_js":
        match = re.search(r"PASS (\d+) frontend boundary groups", output)
        count, unit = (int(match[1]) if match else 0), "reported_boundary_groups"
    elif kind == "study_js":
        match = re.search(r"PASS (\d+) synthetic frontend behavior tests", output)
        count, unit = (int(match[1]) if match else 0), "synthetic_frontend_behavior_tests"
    elif kind == "surrogate":
        rows = re.findall(r"^SURROGATE payload=(\d+) target=([-+\d.eE]+) dt=([-+\d.eE]+)\b", output, re.MULTILINE)
        count, unit = len(rows), "ideal_rigid_body_surrogate_cases"
        reported = {(int(p), float(q), float(dt)) for p, q, dt in rows}
        expected = {(p, q, dt) for p in range(3) for q in (-30.0, 30.0) for dt in (0.016, 0.008)}
        result["reported_cases"] = [dict(payload_index=p, target_deg=q, dt_s=dt) for p, q, dt in sorted(reported)]
        result["evidence_scope"] = "ideal_fixed_axis_surrogate_not_webots_feedback"
        if result["status"] == "passed" and (count != 12 or reported != expected):
            result.update(status="failed", reason="expected all 12 unique payload/target/time-step cases")
    else:
        count, unit = len(re.findall(r"^PASS\b", output, re.MULTILINE)), "reported_cpp_behavior_groups"
    result.update(count=count, count_unit=unit)
    if result["status"] == "passed" and count == 0:
        result.update(status="failed", reason="exit zero but no expected test summary was reported")


def _blocked(name, reason):
    return dict(name=name, command=[], cwd=str(ROOT), exit_code=None, status="blocked",
                reason=reason, elapsed_s=0.0, output="")


def _find_tool(names, fallbacks=()):
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    return next((str(p) for p in map(Path, fallbacks) if p.is_file()), None)


def _node_script_path(node, script):
    # WSL can launch Windows Node, but Node then expects a Windows filename.
    if os.name != "nt" and node.lower().endswith(".exe"):
        converter = shutil.which("wslpath")
        if not converter:
            raise OSError("Windows Node requires wslpath to convert the script path")
        converted = subprocess.run([converter, "-w", str(script)], timeout=10,
                                   capture_output=True, text=True, check=True)
        return converted.stdout.strip()
    return str(script)


def _hash_sources():
    """Manifest of in-scope code/configuration, not installed tool binaries."""
    extensions = {".py", ".ps1", ".js", ".css", ".html", ".cpp", ".hpp", ".msg",
                  ".urdf", ".csv", ".proto", ".wbt", ".xml", ".txt"}
    files = [p for directory in (ROOT / "tests", ROOT / "research", ROOT / "scripts", PACKAGE)
             for p in directory.rglob("*")
             if p.is_file() and p.suffix in extensions and "__pycache__" not in p.parts]
    manifest = {str(p.relative_to(ROOT)).replace("\\", "/"):
                hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    digest = hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest()
    return manifest, digest


def run_verification(*, timeout=120.0):
    """Run each required check, retaining failures and unavailable tool states."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be a positive finite number of seconds")
    started = time.monotonic()
    manifest_before, hash_before = _hash_sources()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONUTF8="1")
    compiler = _find_tool(("g++", "clang++"), ("H:/MSYS2/mingw64/bin/g++.exe",))
    node = _find_tool(("node", "node.exe"),
                      ("C:/Program Files/nodejs/node.exe", "/mnt/c/Program Files/nodejs/node.exe"))
    tool_info = {"python": _capture("python_version", [sys.executable, "--version"], 10, env=env)}
    for name, tool in (("cpp", compiler), ("node", node)):
        tool_info[name] = (_capture(name + "_version", [tool, "--version"], 10, env=env)
                           if tool else _blocked(name + "_version", "tool not found; no install attempted"))
    checks = []
    python = _capture("python_unittest", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"],
                      timeout, env=env)
    _count(python, "python")
    checks.append(python)
    with tempfile.TemporaryDirectory(prefix="joint-study-verify-") as temporary:
        for name, source in (("manual_reference", ROOT / "tests/manual_reference_test.cpp"),
                             ("study_pair", PACKAGE / "tests/study_pair_test.cpp")):
            binary = Path(temporary) / (name + (".exe" if os.name == "nt" else ""))
            if not compiler:
                checks.extend((_blocked(name + "_compile", "C++ compiler not found"),
                               _blocked(name + "_run", "depends on unavailable compilation")))
                if name == "study_pair":
                    checks.append(_blocked("study_pid_surrogate", "depends on unavailable compilation"))
                continue
            cpp_env = dict(env)
            if os.name == "nt":
                cpp_env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
            # Assertions are enabled: deliberately never define NDEBUG.
            compiled = _capture(name + "_compile", [compiler, "-std=c++17", "-O2", "-I",
                                PACKAGE / "include", source, "-o", binary], timeout, env=cpp_env)
            checks.append(compiled)
            if compiled["status"] == "passed":
                executed = _capture(name + "_run", [binary], timeout, env=cpp_env)
                _count(executed, "cpp")
                checks.append(executed)
                if name == "study_pair":
                    # No argv[2]: exercise all 12 cases without writing the old report.
                    surrogate = _capture("study_pid_surrogate", [binary, PACKAGE / "resource"], timeout, env=cpp_env)
                    _count(surrogate, "surrogate")
                    checks.append(surrogate)
            else:
                checks.append(_blocked(name + "_run", "compilation did not pass"))
                if name == "study_pair":
                    checks.append(_blocked("study_pid_surrogate", "compilation did not pass"))
        actuator_source = ROOT / "tests/actuator_torque_test.cpp"
        actuator_binary = Path(temporary) / ("actuator_torque_test" + (".exe" if os.name == "nt" else ""))
        if not compiler:
            checks.extend((_blocked("actuator_torque_compile", "C++ compiler not found"),
                           _blocked("actuator_torque_run", "depends on unavailable compilation")))
        else:
            compiled = _capture(
                "actuator_torque_compile",
                [compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-I", ROOT / "knee_stair_l1/include",
                 actuator_source, "-o", actuator_binary],
                timeout, env=cpp_env)
            checks.append(compiled)
            if compiled["status"] == "passed":
                executed = _capture("actuator_torque_run", [actuator_binary], timeout, env=cpp_env)
                _count(executed, "cpp")
                checks.append(executed)
            else:
                checks.append(_blocked("actuator_torque_run", "compilation did not pass"))
    for name, source, kind in (("manual_frontend", "manual_frontend_test.js", "manual_js"),
                               ("study_frontend", "study_frontend_test.js", "study_js")):
        if not node:
            checks.append(_blocked(name, "Node.js not found; no install attempted"))
            continue
        try:
            result = _capture(name, [node, _node_script_path(node, ROOT / "tests" / source)], timeout, env=env)
        except (OSError, subprocess.SubprocessError) as exc:
            result = _blocked(name, f"cannot convert script path: {exc}")
        _count(result, kind)
        checks.append(result)
    manifest_after, hash_after = _hash_sources()
    changed = sorted(p for p in set(manifest_before) | set(manifest_after)
                     if manifest_before.get(p) != manifest_after.get(p))
    if changed:
        checks.append(_blocked("stable_source_snapshot", "sources changed during verification: " + ", ".join(changed)))
    statuses = {x["status"] for x in checks}
    overall = "failed" if "failed" in statuses else "blocked" if "blocked" in statuses else "passed"
    for record in checks + list(tool_info.values()):
        output = record["output"]
        record["output_characters"] = len(output)
        record["output_truncated"] = len(output) > 6000
        record["output"] = output if len(output) <= 6000 else output[:3000] + "\n...[truncated]...\n" + output[-3000:]
    return dict(generator=GENERATOR, schema_version=1, status=overall,
                created_utc=datetime.now(timezone.utc).isoformat(),
                elapsed_s=round(time.monotonic() - started, 6),
                evidence_scope="synthetic_software_checks_not_actual_webots_motion",
                does_not_verify=["ROS2 plugin build/runtime", "actual Webots feedback or motion",
                                 "native SolidWorks model", "physical friction/hysteresis", "full five-group optimization"],
                tools=tool_info, checks=checks,
                counts_are_not_additive=True, temporary_cpp_binaries_cleaned=True,
                source_sha256_before=hash_before, source_sha256_after=hash_after,
                changed_sources=changed, source_manifest_sha256=manifest_after)


def write_verification(output=DEFAULT_OUTPUT, *, timeout=120.0):
    """Only overwrite this generator's derived JSON inside data/verification."""
    output = Path(output).resolve()
    if not output.is_relative_to((ROOT / "data/verification").resolve()) or output.suffix != ".json":
        raise ValueError("output must be a JSON file inside data/verification")
    if output.exists() and json.loads(output.read_text(encoding="utf-8")).get("generator") != GENERATOR:
        raise ValueError("refusing to overwrite JSON without this generator's provenance")
    report = run_verification(timeout=timeout)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=float, default=120.0, help="per-check timeout in seconds")
    args = parser.parse_args()
    report = write_verification(args.output, timeout=args.timeout)
    summary = dict(status=report["status"], output=str(args.output), elapsed_s=report["elapsed_s"],
                   checks=[{k: x[k] for k in ("name", "status", "exit_code") } | 
                           {k: x[k] for k in ("count", "count_unit") if k in x} for x in report["checks"]])
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return {"passed": 0, "failed": 1, "blocked": 2}[report["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
