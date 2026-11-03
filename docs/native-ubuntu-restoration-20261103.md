# Native Ubuntu restoration — 2026-11-03

Branch: feature/phase8c-lidar. Existing history preserved; no commit, merge or push performed.

## Environment and dependencies
Ubuntu 24.04.5, ROS Jazzy under /opt/ros/jazzy. GTX 1650 driver responds to nvidia-smi (4096 MiB VRAM). Host: 7.1 GiB usable RAM, 4 GiB swap, 74 GiB free on 96 GiB root partition at inspection.

All external package.xml dependencies resolve through ament or Python imports; ament_python is verified through the colcon build extension rather than the ament runtime package index. Supporting matplotlib and YAML are installed. No dependency installation was necessary for the successful build and executed simulations. rosdep is installed but uninitialized; sudo authentication prevented rosdep init. Full rosdep index audit remains incomplete. To finish it interactively: sudo rosdep init; rosdep update; source /opt/ros/jazzy/setup.bash; rosdep check --from-paths simulation/ros2_ws/src --ignore-src. Do not replace the installed ROS/Gazebo distribution.

## Repairs and build
- Native Linux memory preflight uses /proc/meminfo; Windows telemetry remains required under WSL. Existing 2 GiB headless / 2.5 GiB GUI host thresholds and 1.5 GiB Linux threshold remain.
- Declared missing gui launch argument in realistic_rgb_lidar.launch.py. Without it the original launch failed before simulation.
- Fixed undefined trial_id in the existing bounded moving-SLAM probe using a fresh UUID. Acceptance criteria and motion authority unchanged.

Fresh sequential symlink build: all 9 packages passed, 14.73 seconds wall time, maximum timed-command RSS 44,660 KiB, no swaps. MAKEFLAGS=-j1, CMAKE_BUILD_PARALLEL_LEVEL=1, OPENBLAS_NUM_THREADS=1; colcon build --executor sequential --parallel-workers 1 --symlink-install. The anc_lidar rebuild passed after the launch fix. Existing Phase 8C unittest discovery passed 45 tests, including final rerun; launcher bash syntax, launch argument inspection and git diff --check passed. Build RSS is not a whole-machine/process-tree peak.

## Actual simulation evidence
Phase 6 headless Gazebo S00 seed101: PASS, action SUCCEEDED, final truth goal error 0.1438 m, 1.8562 m truth path, zero non-ground contacts, final robot stop confirmed. Five Nav2 lifecycle nodes active. Reference profile uses its geometry-derived static map; that map was never supplied to realistic LiDAR nodes. Graph audit had a /tf discovery failure despite evaluator TF observations; preserved in result.json. No GUI/RViz demonstration was performed.

Realistic sensor retry: 54 RGB frames (320x240) at measured 4.91 wall Hz; 49 scans with 360 beams (last 355 hits), sensor health ~5 Hz. Probe wall-rate estimate was 5.33 Hz. Local LiDAR snapshot 4190 FREE / 258 OCCUPIED / 27952 UNKNOWN. All 200 observed commands zero. Command guard remained stopped with no request; safety gate independently reported forward eligibility.

Stationary online SLAM: diagnostic PASS over 25 wall seconds, 128 scans, 9 maps, 514 map-to-odom updates, 520 zero commands, one authoritative publisher, no non-ground contacts and zero truth drift. Map: 155x115 at 0.05 m, 1441 FREE / 4 OCCUPIED / 16380 UNKNOWN. Zero SLAM pose messages: stationary mapping does not validate moving localization.

Original failed launches/probes and reporting failure are retained. The first moving probe performed bounded motion but failed to serialize its full measurements because trial_id was undefined; no pass is claimed. A later attempt was denied by scan-health safety checks while an interrupted-session Gazebo process still existed. The orphan was terminated before the isolated retry.

## Runtime resources and warnings
One sensor/SLAM runtime sample: 2.3 GiB RAM available, 4.8 GiB used, swap 60 KiB; Gazebo RSS 376344 KiB and ps lifetime CPU 43%, RGB perception RSS 182672 KiB and CPU 34.1%. These are samples, not peaks or real-time-factor benchmarks. Desktop Firefox processes consume substantial RAM. Runtime logs contain EGL/dri2 NVIDIA warnings despite successful RGB rendering, Ogre visibility-mask and SDF gz_frame_id warnings, startup RGB timestamp rejection and SLAM TF-cache drops. Their absence over longer runs is not established. Interrupted background launch required explicit process cleanup; no unrelated desktop processes were stopped.

## Remaining limits
Production turning and combined motion remain disabled, readiness configuration unchanged. Realistic Nav2 goals, obstacle traversal/replanning, autonomous parking and hardware operation remain blocked/unvalidated. No predefined map or simulator truth entered realistic navigation; truth was used only by evaluators. Further experiments need independent evidence under the existing gates.

Evidence: docs/evidence/native-ubuntu-20261103/ and docs/evidence/phase6/runs/native-ubuntu-s00-20261103/.

## Isolated bounded forward result
Fresh isolated S00 run (moving-isolated.json): overall FAIL under unchanged acceptance criteria. The safety-controlled request was 0.05 m/s for at most 4 seconds, with a 0.50 m travel limit. Measured travel 0.214889 m, maximum output 0.05 m/s, 86 nonzero output samples; stale-command zero response 0.319753 seconds, pose-derived stop settled, sampled post-zero travel 0 m, zero non-ground contact pairs. The Gazebo twist still reported 0.0107996 m/s while pose-derived speed was zero, so stopping evidence must be interpreted with that simulator discrepancy. Odometry relative displacement error was 0.0000135 m.

Moving capture received 28 scans, 2 map updates, 112 map-to-odom updates and 10 evaluator truth samples, but only 1 SLAM pose update. Initial SLAM pose was missing, so relative SLAM displacement/yaw errors could not be computed. Moving SLAM therefore FAILS; this run establishes neither accurate moving localization nor navigation readiness. The isolated run was stopped and simulation process cleanup checked. No safety thresholds, readiness flags or production angular cap were changed.
