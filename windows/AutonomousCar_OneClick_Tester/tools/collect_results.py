#!/usr/bin/env python3
"""Build truthful, machine-readable reports for the standalone Windows runner."""
import argparse
import csv
import datetime as dt
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

STATUSES = ("PASS", "FAIL", "BLOCKED", "TIMEOUT", "SKIPPED", "NOT TESTED")
DEFAULT_TESTS = {
    "environment": "NOT TESTED",
    "offline_tests": "NOT TESTED",
    "lidar_sensors": "NOT TESTED",
    "stationary_slam": "NOT TESTED",
    "map_save": "NOT TESTED",
    "moving_slam_trial_1": "NOT TESTED",
    "moving_slam_trial_2": "NOT TESTED",
    "moving_slam_trial_3": "NOT TESTED",
    "turning_clearance_probe": "NOT TESTED",
    "production_turning": "NOT TESTED",
    "readiness": "NOT TESTED",
    "nav2": "NOT TESTED",
    "additional_fault_injection": "NOT TESTED",
}

def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")

def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, path)

def manifest_path(run_dir):
    return Path(run_dir) / "RUN_MANIFEST.json"

def load_manifest(run_dir):
    path = manifest_path(run_dir)
    if not path.is_file():
        raise FileNotFoundError(f"run manifest is missing: {path}")
    return json.loads(path.read_text(encoding="utf-8"))

def rel_file(run_dir, value):
    if not value:
        return None
    path = Path(value)
    if path.is_absolute():
        try:
            return str(path.resolve().relative_to(Path(run_dir).resolve()))
        except ValueError:
            return str(path)
    return str(path)

def init_run(args):
    root = Path(args.run_dir)
    root.mkdir(parents=True, exist_ok=True)
    path = manifest_path(root)
    if path.exists():
        raise FileExistsError(f"refusing to replace existing run manifest: {path}")
    for item in ("environment", "tests", "ros_logs", "gazebo_logs", "maps",
                 "trajectories", "resource_monitoring"):
        (root / item).mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": 1,
        "run_id": args.run_id,
        "execution_mode": args.mode,
        "scenario": args.scenario,
        "seed": args.seed,
        "started_utc": utc_now(),
        "finished_utc": None,
        "overall_status": "RUNNING",
        "overall_reason": "run in progress",
        "project_path": "/home/ashish/AutonomousNavigationCar",
        "branch": None,
        "commit": None,
        "environment": {},
        "tests": {name: {"status": status, "reason": "not yet executed", "artifacts": []}
                  for name, status in DEFAULT_TESTS.items()},
        "cleanup": {"status": "PENDING", "details": None},
        "notes": [],
    }
    atomic_json(path, manifest)
    return 0

def record_test(args):
    root = Path(args.run_dir)
    data = load_manifest(root)
    status = args.status.upper()
    if status not in STATUSES:
        raise ValueError("invalid test status: " + status)
    item = data["tests"].setdefault(args.test_id, {"artifacts": []})
    item["status"] = status
    item["reason"] = args.reason or ""
    item["exit_code"] = args.exit_code
    item["result_file"] = rel_file(root, args.result_file)
    item["log_file"] = rel_file(root, args.log_file)
    item["trial_id"] = args.trial_id
    item["updated_utc"] = utc_now()
    artifacts = []
    for candidate in (item["result_file"], item["log_file"]):
        if candidate and candidate not in artifacts:
            artifacts.append(candidate)
    item["artifacts"] = artifacts
    atomic_json(manifest_path(root), data)
    return 0

def set_metadata(args):
    root = Path(args.run_dir)
    data = load_manifest(root)
    meta_path = Path(args.metadata_file)
    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8-sig"))
    except Exception as exc:
        raise ValueError(f"cannot parse metadata {meta_path}: {exc}") from exc
    key = args.key
    data[key] = meta
    if isinstance(meta, dict):
        data["branch"] = meta.get("git_branch", data.get("branch"))
        data["commit"] = meta.get("git_commit", data.get("commit"))
    atomic_json(manifest_path(root), data)
    return 0

def read_json_file(root, rel):
    if not rel:
        return None
    path = Path(rel)
    if not path.is_absolute():
        path = root / path
    try:
        return json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError):
        return None

def resource_minimum(run_dir):
    path = Path(run_dir) / "resource_monitoring" / "windows_memory.csv"
    values = []
    try:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                value = row.get("windowsAvailableMiB")
                if value:
                    values.append(int(float(value)))
    except (OSError, ValueError):
        pass
    return min(values) if values else None

def metric_summary(root, data):
    tests = data.get("tests", {})
    sensor = read_json_file(root, tests.get("lidar_sensors", {}).get("result_file"))
    if isinstance(sensor, dict) and isinstance(sensor.get("probe"), dict):
        sensor = sensor["probe"]
    stationary = read_json_file(root, tests.get("stationary_slam", {}).get("result_file"))
    nav2 = read_json_file(root, tests.get("nav2", {}).get("result_file"))
    moving = []
    for name in sorted(tests):
        if not name.startswith("moving_slam_trial_"):
            continue
        item = tests[name]
        result = read_json_file(root, item.get("result_file"))
        if result is None:
            moving.append({"trial_id": item.get("trial_id"), "status": item.get("status"),
                           "reason": item.get("reason")})
            continue
        displacement = result.get("truth_forward_displacement_m",
                                  result.get("total_forward_displacement_m"))
        moving.append({
            "trial_id": item.get("trial_id") or result.get("trial_id"),
            "status": item.get("status"),
            "truth_forward_displacement_m": displacement,
            "slam_relative_displacement_error_m": result.get("slam_relative_displacement_error_m"),
            "slam_relative_yaw_error_rad": result.get("slam_relative_yaw_error_rad"),
            "reason": item.get("reason"),
        })
    scan = sensor.get("scan", {}) if isinstance(sensor, dict) else {}
    camera = sensor.get("camera", {}) if isinstance(sensor, dict) else {}
    return {
        "windows_min_available_mib": resource_minimum(root),
        "lidar_scan_messages": scan.get("messages"),
        "lidar_scan_rate_hz": scan.get("measured_wall_rate_hz"),
        "camera_frames": camera.get("frames"),
        "imu_messages": sensor.get("imu_messages") if isinstance(sensor, dict) else None,
        "filtered_odom_messages": sensor.get("filtered_odom_messages") if isinstance(sensor, dict) else None,
        "lidar_local_map": sensor.get("lidar_local_map") if isinstance(sensor, dict) else None,
        "rgb_fused_map": sensor.get("rgb_fused_map") if isinstance(sensor, dict) else None,
        "stationary_slam": None if not isinstance(stationary, dict) else {
            "scan_messages": stationary.get("scan_messages"),
            "map_messages": stationary.get("map_messages"),
            "slam_pose_messages": stationary.get("slam_pose_messages"),
            "map_to_odom_tf_messages": stationary.get("map_to_odom_tf_messages"),
            "map_snapshot": stationary.get("map_snapshot"),
            "diagnostic_pass": stationary.get("diagnostic_pass"),
            "error": stationary.get("error"),
        },
        "moving_slam_trials": moving,
        "nav2": nav2,
    }

def overall_status(data):
    values = [str(v.get("status", "NOT TESTED")).upper()
              for v in data.get("tests", {}).values()]
    if "FAIL" in values:
        return "FAIL", "one or more tests failed"
    if "TIMEOUT" in values:
        return "TIMEOUT", "one or more bounded tests timed out"
    if "BLOCKED" in values:
        return "BLOCKED", "one or more tests were blocked by resources or readiness"
    if "NOT TESTED" in values:
        return "NOT TESTED", "one or more tests were not executed"
    if "SKIPPED" in values:
        return "PARTIAL", "eligible checks completed, with explicitly skipped tests"
    return "PASS", "all reported tests passed"

def render_summary(root, data):
    status, reason = overall_status(data)
    counts = {key: 0 for key in STATUSES}
    for test in data.get("tests", {}).values():
        state = str(test.get("status", "NOT TESTED")).upper()
        counts[state if state in counts else "NOT TESTED"] += 1
    metrics = metric_summary(root, data)
    lines = [
        "AUTONOMOUS NAVIGATION CAR - STANDALONE TEST RUN",
        "=" * 56,
        f"Overall status: {status}",
        f"Reason: {reason}",
        f"Run ID: {data.get('run_id')}",
        f"Started (UTC): {data.get('started_utc')}",
        f"Finished (UTC): {data.get('finished_utc') or 'not finalized'}",
        f"Execution mode: {data.get('execution_mode')}",
        f"Scenario / seed: {data.get('scenario')} / {data.get('seed')}",
        f"Project branch: {data.get('branch') or 'unknown'}",
        f"Project commit: {data.get('commit') or 'unknown'}",
        f"Project path: {data.get('project_path')}",
        "",
        "Machine and resource summary",
        "-" * 28,
    ]
    win = data.get("environment", {}).get("windows", {})
    wsl = data.get("environment", {}).get("wsl", {})
    lines += [
        f"Windows total / available RAM at final sample: {win.get('windowsTotalMiB')} / {win.get('windowsAvailableMiB')} MiB",
        f"Minimum Windows available RAM during sampling: {metrics.get('windows_min_available_mib')} MiB",
        f"Windows memory load / CPU: {win.get('windowsMemoryLoadPercent')}% / {win.get('windowsSystemCpuPercent')}%",
        f"WSL available RAM / swap used: {wsl.get('wslAvailableMiB', wsl.get('wsl_available_mib'))} / {wsl.get('wslSwapUsedMiB', wsl.get('wsl_swap_used_mib'))} MiB",
        f"GPU: {data.get('environment', {}).get('graphics', {}).get('output', 'not captured')}",
        f"Windows run-volume free space: {win.get('windowsRunVolumeAvailableMiB')} MiB",
        "",
        "Test outcomes",
        "-" * 28,
        "PASS={PASS} FAIL={FAIL} BLOCKED={BLOCKED} TIMEOUT={TIMEOUT} SKIPPED={SKIPPED} NOT TESTED={NOT TESTED}".format(**counts),
    ]
    for name, item in data.get("tests", {}).items():
        lines.append(f"- {name}: {item.get('status')} - {item.get('reason') or 'no additional reason'}")
        if item.get("trial_id"):
            lines.append(f"  trial_id: {item['trial_id']}")
    lines += [
        "",
        "Observed sensor and SLAM measurements",
        "-" * 28,
        f"LiDAR messages / measured wall rate: {metrics.get('lidar_scan_messages')} / {metrics.get('lidar_scan_rate_hz')} Hz",
        f"Camera frames: {metrics.get('camera_frames')}",
        f"Filtered odometry messages: {metrics.get('filtered_odom_messages')}",
        f"Stationary SLAM: {json.dumps(metrics.get('stationary_slam'), sort_keys=True)}",
        "Bounded moving-SLAM trials:",
    ]
    if metrics["moving_slam_trials"]:
        for trial in metrics["moving_slam_trials"]:
            lines.append("  " + json.dumps(trial, sort_keys=True))
    else:
        lines.append("  no trial result was recorded")
    nav_status = data.get("tests", {}).get("nav2", {}).get("status", "NOT TESTED")
    lines += [
        f"Nav2 result: {nav_status}",
        f"Nav2 measured detail: {json.dumps(metrics.get('nav2'), sort_keys=True)}",
        "",
        "Warnings and cleanup",
        "-" * 28,
        f"Cleanup: {data.get('cleanup', {}).get('status')} - {data.get('cleanup', {}).get('details')}",
        f"Notes: {'; '.join(data.get('notes', [])) or 'none'}",
        "",
        "Evidence files",
        "-" * 28,
        f"Run directory: {data.get('windows_run_path') or root}",
        f"WSL run path: {root}",
    ]
    for name, item in data.get("tests", {}).items():
        for artifact in item.get("artifacts", []):
            lines.append(f"- {artifact}")
    lines += [
        "",
        "Interpretation",
        "-" * 28,
        "A command exit code or successful node launch is not proof of movement, SLAM quality, or autonomous navigation.",
        "Moving SLAM is reported only from the bounded forward trials and their sensor/pose evidence.",
        "Phase 6 predefined-map navigation is not part of this run.",
    ]
    if status in ("BLOCKED", "NOT TESTED", "TIMEOUT"):
        lines.append("Recommended next action: resolve the exact resource/readiness blocker above, then run the BAT again.")
    elif status in ("FAIL", "PARTIAL"):
        lines.append("Recommended next action: inspect the listed failing or skipped test logs before changing navigation code.")
    else:
        lines.append("Recommended next action: review the saved evidence before accepting any capability as validated.")
    return "\n".join(lines) + "\n"

def render_analyze(root, data):
    tests = data.get("tests", {})
    blockers = [f"{name}: {item.get('reason')}"
                for name, item in tests.items()
                if item.get("status") in ("BLOCKED", "FAIL", "TIMEOUT", "NOT TESTED")]
    strongest = []
    for name in ("lidar_sensors", "stationary_slam", "moving_slam_trial_1",
                 "moving_slam_trial_2", "moving_slam_trial_3", "turning_clearance_probe",
                 "readiness", "nav2"):
        item = tests.get(name, {})
        strongest.extend(item.get("artifacts", []))
    lines = [
        "ANALYZE THIS STANDALONE TEST RUN FIRST",
        f"Run folder: {data.get('windows_run_path') or root}",
        f"Run ID: {data.get('run_id')}",
        f"Overall: {data.get('overall_status')} - {data.get('overall_reason')}",
        f"Project version: {data.get('branch')} @ {data.get('commit')}",
        "",
        "Executed:",
    ]
    for name, item in tests.items():
        lines.append(f"- {name}: {item.get('status')} - {item.get('reason') or 'no additional reason'}")
    lines += ["", "Strong evidence files:"]
    lines.extend(f"- {item}" for item in (strongest or ["See RUN_SUMMARY.txt and the tests/ subfolders."]))
    lines += [
        "",
        "Engineering blockers:",
        *([f"- {item}" for item in blockers] or ["- none recorded"]),
        "",
        f"Production turning passed: {'yes' if tests.get('production_turning', {}).get('status') == 'PASS' else 'no (not validated or skipped)'}",
        f"Moving SLAM passed: {'yes' if all(tests.get(f'moving_slam_trial_{i}', {}).get('status') == 'PASS' for i in range(1,4)) else 'no'}",
        f"Realistic Nav2 attempted: {'yes' if tests.get('nav2', {}).get('status') not in ('SKIPPED','BLOCKED','NOT TESTED') else 'no'}",
        "",
        "Future Codex request:",
        "\"I have run the standalone Autonomous Navigation Car tests while Codex was closed. Review the latest run folder, analyze the recorded Gazebo, LiDAR, SLAM, localization, turning, safety and navigation results, identify the exact blockers, and implement the next justified corrections on feature/phase8c-lidar.\"",
    ]
    return "\n".join(lines) + "\n"

def finalize(args):
    root = Path(args.run_dir)
    data = load_manifest(root)
    data["finished_utc"] = utc_now()
    status, reason = overall_status(data)
    data["overall_status"] = status
    data["overall_reason"] = reason
    atomic_json(manifest_path(root), data)
    summary_json = {
        "schema_version": 1,
        "run": data,
        "metrics": metric_summary(root, data),
        "test_status_counts": {key: sum(1 for v in data["tests"].values()
                                        if v.get("status") == key) for key in STATUSES},
    }
    (root / "RUN_SUMMARY.json").write_text(json.dumps(summary_json, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (root / "RUN_SUMMARY.txt").write_text(render_summary(root, data), encoding="utf-8")
    (root / "ANALYZE_ME_FIRST.txt").write_text(render_analyze(root, data), encoding="utf-8")
    if args.latest_dir:
        latest = Path(args.latest_dir)
        latest.mkdir(parents=True, exist_ok=True)
        for filename in ("RUN_SUMMARY.txt", "RUN_SUMMARY.json", "ANALYZE_ME_FIRST.txt"):
            src = root / filename
            tmp = latest / (filename + ".tmp")
            shutil.copyfile(src, tmp)
            os.replace(tmp, latest / filename)
        pointer = latest / "LATEST_RUN.txt"
        tmp = latest / "LATEST_RUN.txt.tmp"
        tmp.write_text((data.get("windows_run_path") or str(root)) + "\n" +
                       data.get("run_id", "") + "\n", encoding="utf-8")
        os.replace(tmp, pointer)
    return 0


def run_bounded(args):
    command = list(args.argv)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        raise ValueError("run-bounded requires a command after --")
    started = __import__("time").monotonic()
    timed_out = False
    cleanup_confirmed = True
    output = ""
    exit_code = None
    proc = __import__("subprocess").Popen(
        command, cwd=args.cwd or None, stdout=__import__("subprocess").PIPE,
        stderr=__import__("subprocess").STDOUT, text=True, bufsize=1)
    try:
        output, _ = proc.communicate(timeout=args.timeout)
        exit_code = proc.returncode
    except __import__("subprocess").TimeoutExpired:
        timed_out = True
        try:
            proc.send_signal(__import__("signal").SIGINT)
            output, _ = proc.communicate(timeout=args.grace)
            exit_code = proc.returncode
        except __import__("subprocess").TimeoutExpired as exc:
            output = (exc.output or "") if isinstance(exc.output, str) else ""
            cleanup_confirmed = False
            exit_code = None
    except KeyboardInterrupt:
        timed_out = True
        try:
            proc.send_signal(__import__("signal").SIGINT)
            output, _ = proc.communicate(timeout=args.grace)
            exit_code = proc.returncode
        except __import__("subprocess").TimeoutExpired as exc:
            output = (exc.output or "") if isinstance(exc.output, str) else ""
            cleanup_confirmed = False
            exit_code = None
    elapsed = __import__("time").monotonic() - started
    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(output or "", encoding="utf-8", errors="replace")
    status = "TIMEOUT" if timed_out else ("PASS" if exit_code == 0 else "FAIL")
    result = {
        "classification": "BOUNDED_COMMAND_EXECUTION",
        "status": status,
        "command": command,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "cleanup_confirmed": cleanup_confirmed,
        "elapsed_wall_s": round(elapsed, 3),
        "log_file": str(log_path),
    }
    result_path = Path(args.result)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 124 if timed_out else (0 if exit_code == 0 else int(exit_code or 1))

def classify_sensor(args):
    payload = json.loads(Path(args.result).read_text(encoding="utf-8-sig"))
    scan = payload.get("scan") or {}
    camera = payload.get("camera") or {}
    failures = []
    if int(scan.get("messages") or 0) < max(2, int(args.duration * 0.5)):
        failures.append("too_few_lidar_scan_messages")
    if not (scan.get("measured_wall_rate_hz") or 0) > 0:
        failures.append("lidar_scan_rate_unavailable")
    if int(camera.get("frames") or 0) < 1:
        failures.append("camera_frames_missing")
    if int(camera.get("camera_info_messages") or 0) < 1:
        failures.append("camera_info_missing")
    if int(payload.get("imu_messages") or 0) < 1:
        failures.append("imu_messages_missing")
    if int(payload.get("filtered_odom_messages") or 0) < 1:
        failures.append("filtered_odometry_missing")
    if not isinstance(payload.get("lidar_local_map"), dict):
        failures.append("lidar_local_map_missing")
    if not isinstance(payload.get("rgb_fused_map"), dict):
        failures.append("rgb_fused_map_missing")
    if not isinstance(payload.get("lidar_static_tf"), dict):
        failures.append("lidar_static_tf_missing")
    safety = payload.get("safety_state") or {}
    if safety.get("valid") is not True:
        failures.append("safety_state_not_valid")
    guard = payload.get("command_guard_state") or {}
    status = "PASS" if not failures else "FAIL"
    print(json.dumps({"status": status, "failures": failures,
                      "scan_messages": scan.get("messages"),
                      "scan_rate_hz": scan.get("measured_wall_rate_hz"),
                      "camera_frames": camera.get("frames"),
                      "camera_info_messages": camera.get("camera_info_messages"),
                      "filtered_odom_messages": payload.get("filtered_odom_messages"),
                      "safety_authorized": guard.get("authorized"),
                      "safety_valid": safety.get("valid")}, indent=2, sort_keys=True))
    return 0 if status == "PASS" else 1

def annotate_result(args):
    path = Path(args.result)
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    payload["run_id"] = args.run_id
    payload["trial_id"] = args.trial_id
    payload["test_id"] = args.test_id
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, path)
    return 0

def set_cleanup(args):
    root = Path(args.run_dir)
    data = load_manifest(root)
    data["cleanup"] = {"status": args.status.upper(), "details": args.details}
    atomic_json(manifest_path(root), data)
    return 0

def mark_interrupted(args):
    root = Path(args.run_dir)
    data = load_manifest(root)
    data.setdefault("notes", []).append(args.reason)
    data["execution_interrupted"] = True
    for item in data.get("tests", {}).values():
        if item.get("status") == "NOT TESTED":
            item["status"] = "BLOCKED"
            item["reason"] = "run interrupted before this test started: " + args.reason
    data["cleanup"] = {
        "status": "BLOCKED",
        "details": "Windows controller finalized the run after the WSL worker exited before recording cleanup.",
    }
    atomic_json(manifest_path(root), data)
    return 0

def self_test():
    with tempfile.TemporaryDirectory(prefix="anc-oneclick-selftest-") as temp:
        root = Path(temp)
        data = {
            "run_id": "selftest-only",
            "execution_mode": "SELF_TEST",
            "started_utc": utc_now(),
            "finished_utc": utc_now(),
            "scenario": "S00",
            "seed": 1,
            "branch": "feature/phase8c-lidar",
            "commit": "synthetic-test-only",
            "project_path": "/not-a-real-run",
            "environment": {},
            "cleanup": {"status": "PASS", "details": "synthetic report-unit-test only"},
            "notes": ["synthetic statuses exist only in a temporary report-unit-test directory"],
            "tests": {
                "environment": {"status": "PASS", "reason": "synthetic self-test"},
                "lidar_sensors": {"status": "FAIL", "reason": "synthetic fail-report test", "artifacts": []},
                "nav2": {"status": "BLOCKED", "reason": "synthetic readiness blocker test", "artifacts": []},
                "production_turning": {"status": "SKIPPED", "reason": "synthetic skipped test", "artifacts": []},
            },
        }
        status, _ = overall_status(data)
        report = render_summary(root, data)
        analyze = render_analyze(root, data)
        assert status == "FAIL"
        assert "lidar_sensors: FAIL" in report
        assert "nav2: BLOCKED" in report
        assert "synthetic fail-report test" in report
        assert "synthetic readiness blocker test" in analyze
        data["tests"]["lidar_sensors"]["status"] = "PASS"
        status, _ = overall_status(data)
        assert status == "BLOCKED", "a BLOCKED test must not be reported as PASS"
    print("collect_results.py self-test: PASS (synthetic FAIL/BLOCKED fixtures kept in temporary storage)")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    init_p = sub.add_parser("init")
    init_p.add_argument("--run-dir", required=True)
    init_p.add_argument("--run-id", required=True)
    init_p.add_argument("--mode", required=True)
    init_p.add_argument("--scenario", default="S00")
    init_p.add_argument("--seed", type=int, default=101)
    init_p.set_defaults(func=init_run)
    rec_p = sub.add_parser("record")
    rec_p.add_argument("--run-dir", required=True)
    rec_p.add_argument("--test-id", required=True)
    rec_p.add_argument("--status", required=True)
    rec_p.add_argument("--reason", default="")
    rec_p.add_argument("--result-file")
    rec_p.add_argument("--log-file")
    rec_p.add_argument("--exit-code", type=int)
    rec_p.add_argument("--trial-id")
    rec_p.set_defaults(func=record_test)
    meta_p = sub.add_parser("metadata")
    meta_p.add_argument("--run-dir", required=True)
    meta_p.add_argument("--key", required=True)
    meta_p.add_argument("--metadata-file", required=True)
    meta_p.set_defaults(func=set_metadata)
    fin_p = sub.add_parser("finalize")
    fin_p.add_argument("--run-dir", required=True)
    fin_p.add_argument("--latest-dir")
    fin_p.set_defaults(func=finalize)

    bounded_p = sub.add_parser("run-bounded")
    bounded_p.add_argument("--timeout", type=float, required=True)
    bounded_p.add_argument("--grace", type=float, default=8.0)
    bounded_p.add_argument("--cwd")
    bounded_p.add_argument("--log", required=True)
    bounded_p.add_argument("--result", required=True)
    bounded_p.add_argument("argv", nargs=argparse.REMAINDER)
    bounded_p.set_defaults(func=run_bounded)
    classify_p = sub.add_parser("classify-sensor")
    classify_p.add_argument("--result", required=True)
    classify_p.add_argument("--duration", type=float, default=12.0)
    classify_p.set_defaults(func=classify_sensor)
    annotate_p = sub.add_parser("annotate")
    annotate_p.add_argument("--result", required=True)
    annotate_p.add_argument("--run-id", required=True)
    annotate_p.add_argument("--trial-id", required=True)
    annotate_p.add_argument("--test-id", required=True)
    annotate_p.set_defaults(func=annotate_result)
    cleanup_p = sub.add_parser("cleanup")
    cleanup_p.add_argument("--run-dir", required=True)
    cleanup_p.add_argument("--status", required=True)
    cleanup_p.add_argument("--details", default="")
    cleanup_p.set_defaults(func=set_cleanup)
    interrupted_p = sub.add_parser("mark-interrupted")
    interrupted_p.add_argument("--run-dir", required=True)
    interrupted_p.add_argument("--reason", required=True)
    interrupted_p.set_defaults(func=mark_interrupted)

    self_p = sub.add_parser("self-test")
    self_p.set_defaults(func=lambda _args: self_test() or 0)
    args = parser.parse_args()
    try:
        return args.func(args)
    except Exception as exc:
        print(f"collect_results.py ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

if __name__ == "__main__":
    raise SystemExit(main())
