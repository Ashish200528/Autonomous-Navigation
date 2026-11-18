# Graphical S00, S01 coverage and route-distance follow-up — 2026-11-04

## Verified capability at this milestone

| Trial | Actual result | Scope |
|---|---|---|
| S00 Gazebo + RViz, fresh observed run | PASS autonomous goal and final stop | .75 m destination; .6349 m truth path, .1304 m final error, .00552 m final localization error, zero non-ground contacts |
| S00 2 m destination, headless | PASS goal and final stop | 1.9345 m truth path, .1125 m final error, .03536 m final localization error, zero non-ground contacts |
| Original S01, -3 m start to +3 m goal | FAIL / ABORTED, no movement | Destination outside initial observed SLAM map; obstacle avoidance is NOT demonstrated |
| S00 3 m first attempt | INCOMPLETE | Interrupted before final result; launch log includes progress failure, but no final acceptance result exists |

Earlier accepted Phase 6 and two realistic S00 trials remain preserved. Mixed/reverse movement is still denied. No runtime simulator truth, predefined reference occupancy map, safety-limit change or unrelated software shutdown was used.

## Graphical validation and resource evidence

Fresh accepted evidence is in docs/evidence/native-gui-s00-safe-domain-20261104 and the sibling -observer directory. The observer captures only Gazebo/RViz windows whose X11 PIDs belong to its commissioning descendant tree; it does not capture unrelated desktop/browser windows. Twelve screenshots, CPU/RAM/swap samples, actual navigation-result.json, truth and command CSVs are saved. Screenshot timestamps can be correlated with trajectory timestamps through monotonic_s. Observed autonomous goal success is independent of GUI-window creation.

Minimum sampled host available RAM: 1687 MiB. Peak summed descendant RSS: 2726 MiB (includes shared-page double counting). Runtime RTF .982. The navigation evaluator's matching-process sample reported peak RSS 2365 MiB and mean CPU 455% (approximately 4.55 logical cores); this narrower process selection excludes some GUI descendants. No memory abort/OOM was observed. Swap was already in use; observer.json records its evolution. These are sampled resource measurements, not exact whole-system peaks.

Two attempts were blocked by the unchanged 2560 MiB GUI preflight; no simulator started. One rendered Gazebo/RViz but failed lifecycle activation before any goal. These are preserved under native-gui-s00-20261104-observer, native-gui-s00-final-20261104-observer, and native-gui-s00-retry-20261104{,-observer}. They are not autonomous successes. A prior user-created s00-visual-20261103T235519 result also reports PASS and is preserved separately, but has no equivalent resource/screenshot observer.

RViz still logs the indexed-image GLSL sampler warning, and its default dock layout leaves a narrow viewport. Screenshots show a rendered map/scan view and Gazebo robot, but this milestone does not claim a clean renderer log, polished presentation layout, or desktop video. No driver or installed RViz shader was replaced. The launch delay is 12 s for GUI commissioning (6 s headless) to separate initialization bursts. The observer loads screenshot libraries after windows appear to reduce preflight overhead and propagates the commissioning child's exit code.

Selected screenshots: native-gui-s00-safe-domain-20261104-observer/window-09.png (RViz) and window-11.png (Gazebo). View them alongside navigation-result.json; a screenshot alone is not goal-completion evidence.

## Native DDS domain correction

Previous project experiments used domains around 180; DDS-derived ports in that range overlap this host's 32768–60999 ephemeral range. Official guidance: https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Domain-ID.rst. New commissioning accepts 0..100, defaults to 42, and matrix/SLAM test defaults now use lower ranges. No sysctl, middleware installation, ROS distribution or security setting changed. This removes a known port-conflict risk; it does not prove every historical lost lifecycle response had that cause. The new low-domain GUI and 2 m runs both completed successfully.

## S01 observations and additional requirements

The existing S01 generator is used unchanged. The launch now preserves its -3 m spawn (rather than accidentally using the S00 origin inside the obstacle), and independent truth verifies the spawn before a goal is sent. Initial SLAM map: 136x115 at .05 m; 1134 FREE, 50 OCCUPIED, 14456 UNKNOWN cells; goal outside map bounds. Nav2 aborted, measured movement was zero, and no non-ground contacts occurred. Unknown occupancy remains blocked.

S01 needs sensor-derived incremental coverage before planning to an occluded/out-of-bounds goal. The controller also needs enough known-free space for its full .5236 m swept turn disk at intermediate stops; a geometrically short path can be infeasible for sequential turn/drive execution. Planner padding/inflation and observation-goal selection must respect that existing runtime restriction. A successful empty-arena route does not establish these requirements. No hardcoded obstacle detour or relaxed angular/unknown-space gate was introduced at this checkpoint.

## Longer-route measurement

The commissioning CLI supports S00 .75–3 m goals and the original S01 goal. Longer routes have explicit travel/timeout bounds and still use the same motion authority. Initial/final SLAM map snapshots are exported as evidence only and never loaded into navigation. Continuous localization error samples include TF age, so timing offsets can be distinguished from estimator drift.

The accepted 2 m run had maximum sampled XY localization error .03537 m and maximum absolute TF age .066 s; final error .03536 m. RTF .994, sampled process peak RSS 1719.7 MiB. This establishes one bounded multi-metre success, not a loop-closure or long-duration localization study. The first 2 m attempt failed lifecycle startup and is retained; the 3 m attempt has no final result and must be repeated before classification.

## Reproduction

From the repository root, source ROS/workspace and run only one simulation at a time:

```bash
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/observe_gui_commission.py \
  "docs/evidence/gui-new-$(date +%Y%m%dT%H%M%S)" --domain-id 45
```

This is the successfully tested graphical .75 m S00 profile, including RAM checks and screenshots. A blocked/failing child is not an observer success. For the accepted 2 m headless profile:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/commission_lidar_s00.py \
  docs/evidence/native-turn-production-matrix-20261103/envelope.json \
  "docs/evidence/s00-2m-new-$(date +%Y%m%dT%H%M%S)" \
  --distance-m 2 --timeout-s 180 --domain-id 44
```

To reproduce S01's diagnostic failure, use the same command with a new directory and --scenario S01 --timeout-s 120 --domain-id 47. This is not a successful visual obstacle-avoidance demonstration. Phase 6's existing reference launcher remains available, with its mode explicitly distinguished in native-nav2-s00-20261103.md.
