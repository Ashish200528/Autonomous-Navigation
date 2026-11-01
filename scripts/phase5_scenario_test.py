#!/usr/bin/env python3
"""Static determinism, geometry, world-SDF, and registry checks for Phase 5."""
import hashlib
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from scenario_generator import (
    ROOT, WORLD_NAMES, build_world_sdf, corridor_is_clear,
    find_scenario, generate_layout, registry_data,
)

OUTPUT = ROOT / "docs" / "evidence" / "phase5" / "static-validation.json"


def check(checks, name, passed, detail):
    checks.append({
        "name": name,
        "status": "PASS" if passed else "FAIL",
        "detail": detail,
    })


def main():
    data = registry_data()
    checks = []
    results = []
    active = [s for s in data["scenarios"] if s.get("status") == "active_phase5"]
    active_ids = [s["id"] for s in active]
    check(checks, "phase5_scenario_ids",
          set(active_ids) == {"S00", "S01", "S02", "S03", "S04", "S09"},
          f"active IDs={active_ids}")

    layouts = {}
    for scenario_id in ("S00", "S01", "S02", "S03", "S04", "S09"):
        layout = generate_layout(scenario_id, 101, data)
        layouts[scenario_id] = layout
        failed = [c["name"] for c in layout["validation"]["checks"]
                  if not c["passed"]]
        check(checks, f"{scenario_id}_geometry",
              layout["validation"]["passed"],
              f"{len(layout['validation']['checks'])} checks; failed={failed}")
        check(checks, f"{scenario_id}_route_expectation",
              layout["route_validation"]["found"] == layout["route_expected"],
              f"found={layout['route_validation']['found']}, "
              f"expected={layout['route_expected']}")
        if scenario_id == "S01":
            direct = corridor_is_clear(
                [layout["start_xyyaw"][:2], layout["goal_xyyaw"][:2]], layout)
            check(checks, "S01_direct_path_blocked", not direct,
                  f"straight centerline clear={direct}")
        if scenario_id == "S09":
            check(checks, "S09_is_no_route",
                  not layout["route_validation"]["found"] and
                  layout["validity"] == "no_route",
                  f"grid route={layout['route_validation']['found']}")
        names = [o["name"] for o in layout["obstacles"]]
        check(checks, f"{scenario_id}_generic_entity_names",
              all(name.startswith("obstacle_") for name in names),
              f"entity names={names}")
        check(checks, f"{scenario_id}_static_flags",
              all(isinstance(o.get("static"), bool) for o in layout["obstacles"]),
              f"flags={[o.get('static') for o in layout['obstacles']]}")
        results.append({
            "scenario_id": scenario_id,
            "seed": 101,
            "layout_sha256": layout["layout_sha256"],
            "world_sha256": hashlib.sha256(build_world_sdf(layout)).hexdigest(),
            "route_found": layout["route_validation"]["found"],
            "obstacle_count": len(layout["obstacles"]),
            "shape_types": sorted({o["shape"] for o in layout["obstacles"]}),
            "start_xyyaw": layout["start_xyyaw"],
        })

    for scenario_id in ("S02", "S04"):
        first = generate_layout(scenario_id, 101, data)
        repeated = generate_layout(scenario_id, 101, data)
        other = generate_layout(scenario_id, 202, data)
        first_world = build_world_sdf(first)
        repeated_world = build_world_sdf(repeated)
        check(checks, f"{scenario_id}_same_seed_exact_replay",
              first["layout_sha256"] == repeated["layout_sha256"] and
              first_world == repeated_world,
              f"layout hash={first['layout_sha256']}; "
              f"world hash={hashlib.sha256(first_world).hexdigest()}")
        check(checks, f"{scenario_id}_different_seed_changes_layout",
              first["layout_sha256"] != other["layout_sha256"],
              f"seed101={first['layout_sha256']}; seed202={other['layout_sha256']}")
        check(checks, f"{scenario_id}_different_seed_remains_valid",
              other["validation"]["passed"] and
              other["route_validation"]["found"] == other["route_expected"],
              f"seed202 route={other['route_validation']['found']}; "
              f"failed={[c['name'] for c in other['validation']['checks'] if not c['passed']]}")

    active_profiles = {s["sensor_profile"] for s in active}
    check(checks, "profiles_remain_separate",
          active_profiles == {"REFERENCE", "REALISTIC"},
          f"profile labels={sorted(active_profiles)}")
    check(checks, "truth_not_a_robot_input",
          all(not layouts[sid]["navigation_attempted"] for sid in layouts),
          "Scenario generator marks navigation_attempted=false; it has no ROS truth publisher.")

    sdf_results = []
    canonical_dir = ROOT / "simulation" / "scenarios" / "worlds"
    for scenario_id, filename in WORLD_NAMES.items():
        path = canonical_dir / filename
        generated = build_world_sdf(generate_layout(scenario_id, 101, data))
        matches = path.exists() and path.read_bytes() == generated
        check(checks, f"{scenario_id}_canonical_world_matches_generator",
              matches, f"path={path.relative_to(ROOT)} exists={path.exists()}")
        parsed = False
        sdf_stdout = ""
        if path.exists():
            try:
                ET.parse(path)
                proc = subprocess.run(
                    ["gz", "sdf", "-k", str(path)],
                    capture_output=True, text=True, timeout=20,
                )
                parsed = proc.returncode == 0
                sdf_stdout = (proc.stdout + proc.stderr).strip()
            except Exception as exc:
                sdf_stdout = f"{type(exc).__name__}: {exc}"
        check(checks, f"{scenario_id}_gz_sdf_validate", parsed, sdf_stdout)
        sdf_results.append({
            "scenario_id": scenario_id,
            "file": str(path.relative_to(ROOT)),
            "gz_sdf_valid": parsed,
            "output": sdf_stdout,
        })

    report = {
        "schema_version": 1,
        "scope": "Phase 5 static scenario validation only",
        "navigation_attempted": False,
        "checks": checks,
        "scenarios": results,
        "world_sdf": sdf_results,
        "overall_status": "PASS" if all(c["status"] == "PASS" for c in checks) else "FAIL",
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    log = OUTPUT.with_suffix(".log")
    log.write_text("\n".join(
        f"{c['status']} {c['name']}: {c['detail']}" for c in checks) +
        f"\nOVERALL {report['overall_status']}\n")
    print(json.dumps({
        "status": report["overall_status"],
        "checks_passed": sum(c["status"] == "PASS" for c in checks),
        "checks_total": len(checks),
        "failed": [c["name"] for c in checks if c["status"] != "PASS"],
        "json": str(OUTPUT),
        "log": str(log),
    }, indent=2))
    return 0 if report["overall_status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
