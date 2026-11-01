# Changelog

## v0.1.0 — Phase 0 planning baseline

- Created the independent simulation repository and planned package layout.
- Recorded requirements, decisions, explicit configuration placeholders, scenarios, metrics, and prioritized issues.
- Captured the initial WSL/Ubuntu/GPU/storage environment inventory.
- No ROS packages, robot model, simulation code, or hardware application files were changed.

## Phase 6 reference Nav2 baseline - 2026-11-02

- Added the anc_navigation Nav2 reference package, guarded command output, and bounded headless launch.
- Added per-run geometry-derived ORACLE_REFERENCE maps, a navigation mode in the scenario runner, metrics, MCAP evidence, TF authority checks, and static acceptance checks.
- Validated S00-S04 navigation and S09 bounded no-route abort with recorded limits; see docs/evidence/phase6/.
- Kept the Phase 6 navigation status separate from historical Phase 5 sensor-profile metadata.
