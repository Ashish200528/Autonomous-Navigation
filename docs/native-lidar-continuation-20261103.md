# Native LiDAR continuation — 2026-11-03

## Verified status

- Migration milestone committed as f954f69 and pushed to origin/feature/phase8c-lidar; remote SHA verified.
- Moving SLAM: PASS WITH LIMITS, three fresh S00 straight-motion trials, unchanged acceptance criteria. No long trajectory, loop-closure, turn-localization or obstacle navigation claim.
- Production turning: NOT AUTHORIZED. Production angular authority and readiness flags remain unchanged.
- Realistic Nav2 S00 / S01: NOT RUN, blocked by production-turning evidence. Parking and exploration: NOT VALIDATED. Hardware: PENDING.

## Diagnosis and corrections

The evaluator subscribed to the correct /pose type but started after SLAM initialization. SLAM Toolbox 2.8.5 publishes /pose with volatile reliable QoS, so the initial stationary scan-matched pose was unavailable to a late subscriber. Pose publication occurs only for accepted processed scans, not at the map-to-odom broadcast frequency. The previous 0.10 m translation threshold supplied too few accepted scans in a 0.215 m commissioning movement.

Source inspected: https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp (setROSInterfaces, shouldProcessScan, addScan/publishPose). No replacement or synthetic localization poses were added.

The new runner starts the evaluator before the simulator. The probe waits for actual SLAM pose and map-to-odom baseline before starting motion. The readiness wait budget increases to 40 s for startup only; motion and acceptance limits are unchanged. Translation processing threshold is 0.02 m, below the 0.05 m map grid spacing; the 0.25 s time limit still caps processing at 4 Hz. Heading processing threshold remains 0.10 rad.

First batch: 2/3 overall PASS, all trials received nine SLAM pose updates and passed pose-error checks. One failed the existing two-map-update requirement because the 3 s map interval was too sparse for a short capture. That failure is preserved in native-slam-correction-20261103. Map interval was then set to 1 s for timely commissioning feedback. Final batch: 3/3 PASS.

## Final measured results

| Trial | Travel m | SLAM pose/map/scan/TF counts | Position error m | Heading error rad | Timeout response s | RTF |
|---|---:|---|---:|---:|---:|---:|
| trial0 | 0.214989 | 9/5/25/101 | 0.024052 | 0.000100 | 0.316754 | 0.9999 |
| trial1 | 0.214989 | 10/6/29/113 | 0.057473 | 0.000874 | 0.324361 | 1.0003 |
| trial2 | 0.214989 | 9/5/28/112 | 0.015983 | 0.000203 | 0.324717 | 1.0001 |

All three trials: 0.05 m/s bounded requests, maximum 4 s / 0.50 m limit, no non-ground contacts, settled after command timeout. Scan counts cover the complete drive/stop measurement interval and indicate approximately 5 Hz. RTF uses simulation clock versus monotonic time over that interval. Position/heading errors are relative SLAM displacement errors against evaluator-only truth, not absolute global accuracy.

Half-second resource sampling recorded peak aggregate project RSS around 1.55 GiB and minimum host available RAM 1.73 GiB. RSS sums may include shared pages; peaks between samples are not measured. Raw samples and compact per-trial metrics are saved. No other desktop applications were terminated.

ROS dependency initialization succeeded via normal Ubuntu administrative authentication. rosdep update completed. The audit identified seven manifests using the unresolvable ament_python dependency key. They now declare python3-setuptools as build dependency while retaining ament_python build_type. rosdep check now passes without skip keys or custom rules. Full sequential rebuild passed all nine packages. Phase 8C offline suite passed 45/45 after the SLAM changes.

Rendering EGL/dri2 warnings remain despite usable RGB and real-time runs; no ROS/Gazebo/NVIDIA reinstall was justified. Startup timestamp/TF-cache drops are preserved; corrected runs completed with actual pose updates. Timestamp validity checks remain active. SIGINT at process-group shutdown can produce Python exit-code -2 entries; these are distinct from mid-trial failures.

## Reproduction and evidence

Source /opt/ros/jazzy/setup.bash and simulation/ros2_ws/install/setup.bash, then run:

```bash
python3 scripts/run_native_slam_trials.py docs/evidence/native-slam-new-run
```

The output directory must be new. Runs are sequential and headless, Nav2 off, angular cap zero, truth evaluator-only, and the safety guard owns /cmd_vel. Full per-message JSON remains local; compact summary JSON, resource samples and bounded logs are versioned. Original migration report is historical; its rosdep blocker is superseded here.

## Nav2 commissioning boundary

The existing global readiness check requires a previous successful guarded Nav2 trial, creating a first-trial circular dependency. A separate S00-only commissioning authorization must eventually require validated production turning, three moving-SLAM passes and a live single-guard publisher audit, while allowing the first bounded Nav2 result to be collected. The normal demonstration gate should still require that successful result afterward. This is a proposed workflow only: no commissioning bypass or runtime authority was added in this session. Turning's 13-case production matrix is still a prerequisite; changing a boolean cannot establish it. No predefined reference map was used in any realistic run.

## Turning calibration continuation

The existing bounded pure-yaw trial ran six two-second S00 pulses at +/-0.02, +/-0.05 and +/-0.10 rad/s, all PASS. No non-ground contacts; zero-to-settled times 0.6741–0.7164 s. Production angular authority remained false. See docs/evidence/native-turning-20261103/rotation-summary.json and reproduce.py (source ROS/workspace; use ROS_DOMAIN_ID=140 and single-thread BLAS/OMP).

These extend the earlier 1.2-second pulses, but do not satisfy the 13-case production-turning validator. Low-friction, extended/repeated operation, near-obstacle motion and fault-stop rotation still need measured validation and a reviewed production stopping envelope. Realistic Nav2 S00 and S01 were not attempted. No parking or exploration claim is made.

## Production-turning follow-up

See [native-turning-matrix-20261103.md](native-turning-matrix-20261103.md). All 13 categories were probed under existing calibration authority. Nine motion categories passed; all four fault categories remain failed under the 0.75 s confirmed-settling contract. Production authority, realistic autonomous S00 and S01 remain BLOCKED/NOT RUN. The 49-test offline suite passed; completed measured trials recorded no non-ground contacts. No production readiness flags were enabled.
