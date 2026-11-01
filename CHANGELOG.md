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

## Phase 7 RGB-only perception - 2026-11-02

- Added the separate anc_perception REALISTIC_RGB_ONLY profile with conservative FREE/OCCUPIED/UNKNOWN masks, confidence, overlay, and health.
- Added offline-only geometry evaluation, fixed-seed RGB dataset capture, and live missing/stale RGB fault checks.
- Validated six packages, 13 static checks, and 12 captured views; documented measured limits in docs/PHASE_REPORT.md and docs/evidence/README.md.
- Kept ORACLE_REFERENCE navigation separate.

## Phase 8 RGB-constrained stop-only baseline - 2026-11-02

- Added the separate anc_realistic_navigation package, base_link-relative RGB FREE/UNKNOWN grid, freshness and localization invalidation, and a hard-locked single /cmd_vel safe-stop supervisor.
- Disabled known-start map-to-odom TF in the Phase 8 launch while preserving the shared launch default for Phase 6.
- Added projection, live stop, camera fault/recovery, localization dropout, command watchdog, scenario-repeat, and Phase 6 reference-regression evidence.
- Classified Phase 8 PASS WITH LIMITS for stop-only behavior; autonomous motion remains BLOCKED by a disconnected free patch and unmeasured braking distance.
