# Native turning validation — 2026-11-03

## Result and authority

Stage A remains BLOCKED. All 13 case categories were exercised as **preproduction calibration probes**, not as an authorized production-turning matrix. Nine motion categories passed; four fault categories failed the unchanged timing contract or evaluator capture. Runtime angular authority remained calibration-only at <=0.10 rad/s; linear output was zero. No production flags, controller stopping constants, safety timeouts or readiness flags were changed. Stages B–D (commissioning, realistic autonomous S00, S01 avoidance) were NOT RUN because turning prerequisites failed. Parking/exploration remain unvalidated. Previously accepted straight moving-SLAM and Phase 6 evidence remains preserved.

## Implementation and measurement

Added scripts/run_turn_matrix.py and tests/phase8c_turn_matrix_probe.py. Each case runs in a fresh ROS domain and headless Gazebo instance; only project-owned process groups are stopped. The probe requests /cmd_vel_nav, observes the guard's output, samples independent truth approximately every 0.1 s, and requests zero on an evaluator clearance/drift/contact limit or early gate revocation. Truth and scenario geometry are evaluator-only. No geometry-derived map is sent to realistic runtime nodes.

Clearance is a conservative robot circumdisk-to-static-box distance, including rotated boxes. Four independent geometry regression tests cover walls, rotated corners, overlap and nearest obstacles; the full Phase 8C offline suite passes 49 tests. No package source changed, so the previously verified workspace build remains applicable; new Python files pass compile checks. The runner now returns a failing exit status for unsuccessful cases.

Completed trial outputs were audited: absolute yaw <=0.10 rad/s, linear output zero, no recorded non-ground contacts. Low-friction and fault probes audited exactly one /cmd_vel publisher: anc_safety_command_guard. Two external CLI audits missed their short-lived domains and recorded Unknown topic; they are not positive evidence. The probe's in-process publisher audits provide the positive evidence.

## Observed results

| Category | Calibration result | Scope or blocker |
|---|---|---|
| Clockwise | PASS | -0.10 rad/s, 2 s |
| Anticlockwise | PASS | +0.10 rad/s, 2 s |
| Multi-speed | PASS | +0.02, -0.05, +0.10 rad/s, 2 s each |
| Repeated turns | PASS | Four alternating +/-0.10 rad/s pulses |
| Extended rotation | PASS | +0.10 rad/s for 8 s |
| Near wall | PASS | S00 y=2.05 m, +0.05 rad/s |
| Near box | PASS | S01 x=-1.2 m, +0.05 rad/s; no traversal |
| Narrow passage | PASS | Corrected S03 x=0,y=0 between constriction walls |
| Low friction | PASS WITH LIMITS | Ground mu/mu2=0.5; +/-0.10 rad/s; one friction setting only |
| LiDAR loss | FAIL | Zero output 0.304 s; confirmed settled 0.873 s |
| Localization loss | FAIL | Zero output 0.254 s; confirmed settled 0.938 s |
| Command timeout | FAIL | Initial evaluator truth-service timeout; retry zero output 0.338 s, confirmed settled 0.906 s |
| Unknown swept region | FAIL | Initial gate revocation before injection; actual injected retry zero output 0.144 s, confirmed settled 0.820 s |

The unchanged bound is 0.75 s for zero-request/fault-to-confirmed-settled observation. Pose settling requires <=0.005 m/s and <=0.01 rad/s maintained for 0.5 s, as in the existing calibration evaluator. Sensor/command detection plus that confirmation window exceeds the contract in fault tests. Zero output alone is insufficient to pass. No criterion was relaxed or relabelled to promote these results. Physical deceleration versus the 0.5 s observation hold needs separate instrumentation before defining a defensible production stopping envelope. Residual yaw sampled after zero was zero in completed cases; sampling delay means this does not establish a zero stopping-angle bound.

LiDAR/localization faults pause the corresponding project process and resume it in finally cleanup. Unknown-space fault pauses the local scan mapper and publishes an explicitly all-UNKNOWN test grid on its output topic; this is adversarial fault injection, not a sensor-derived map or localization input used to claim navigation success. A fresh simulator/domain prevents contamination of subsequent trials. The initial unknown-space run failed before its planned injection; its fault-time label is superseded by the retry's explicit fault_injected field. Future probes require injection to have occurred before a fault case can pass.

The original S03 case sampled the passage approach at x=-2.5. The separate corrected constriction run sampled x=0,y=0; use that result for the narrow-passage row. Initial baseline probes were incrementally instrumented; absent publisher-audit fields must not be treated as sampled audits. The original command-timeout capture lost full metrics to a truth-service timeout; collision and stopping claims do not extend to that incomplete run.

## Resources and limits

During the sampled sweep/retry period, 0.5-second samples showed peak summed project RSS 1,546,236 KiB and minimum system MemAvailable 1,503,584 KiB. RSS can double-count shared pages and these are sampled peaks, not exact maxima. The resource monitor began after the earliest baseline trials and includes idle/retry periods until explicitly stopped. Raw resources.jsonl and its monitor source are preserved. Existing desktop applications were left alone; swap usage increased during the sweep. EGL, startup timestamp/TF warnings remain in bounded launch logs. No ROS/Gazebo/NVIDIA or unrelated system configuration changed.

Odometry-versus-truth heading-delta errors and measured angular response are recorded as slip indicators. They do not isolate tire slip from controller tracking or estimator lag. This single lower-friction world is not a complete friction-envelope validation. Static box clearance does not validate dynamic obstacles or swept motion with simultaneous translation. All production cases require actual production authority and a reviewed angular stopping model; this sweep intentionally supplies neither.

## Reproduction

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
python3 scripts/run_turn_matrix.py docs/evidence/turn-matrix-new-run
# Optional individual case, always with a NEW output directory:
python3 scripts/run_turn_matrix.py docs/evidence/turn-timeout-new-run command_timeout
python3 -m unittest discover -s tests -p 'phase8c*test.py'
```

A failing experiment exits nonzero and still retains evidence. Each fresh simulator is sequential; do not run multiple invocations together on this 8 GB machine. Primary compact results: docs/evidence/native-turn-matrix-20261103/sweep-summary.json. Corrected/retried evidence: native-turn-constriction-20261103, native-turn-timeout-retry-20261103, native-turn-unknown-retry-20261103 under docs/evidence. Full per-message result.json captures and original terminal logs remain local and ignored; summaries, bounded logs, SDF fixtures and specifications are versioned.

## Visual demonstrations of currently verified capabilities

Phase 6 ORACLE_REFERENCE navigation (uses a predefined reference map; not realistic LiDAR autonomy):

```bash
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
python3 scripts/phase6_navigation_runner.py --scenario S00 --seed 101 \
  --run-id "phase6-visual-$(date +%Y%m%dT%H%M%S)" --gui --rviz --hold-window
```

Realistic RGB + LiDAR + SLAM graphical sensing, with no goal and production yaw disabled:

```bash
./scripts/run_lidar_demo.sh --scenario S00 --gui --rviz --duration 90
```

Run only one demonstration at a time. The LiDAR launcher retains memory and readiness checks. No autonomous realistic graphical demonstration has been validated. No readiness bypass or first-goal commissioning authority was introduced: production turning remains the prerequisite. These commands describe existing interfaces; no GUI run was performed in this headless session.
