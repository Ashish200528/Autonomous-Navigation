# Sensor coverage and localization follow-up — 2026-11-04

The graphical S00 demonstration and direct 2 m S00 success remain accepted in [the graphical report](native-gui-s01-distance-20261104.md), committed as `5fefc6be20f6b74e99fe249e5dafd402493130fb`. This follow-up tests sensor-selected observation goals and investigates longer-route/S01 localization failures. A partial observation goal is not completion of the original S01 mission.

## Actual Gazebo results

| Profile / destination | Mission result | Truth path | Final goal error | Final localization error |
|---|---|---:|---:|---:|
| Baseline S00, 3 m repeat | FAIL, Nav2 TIMEOUT | 3.008 m | .046 m | .372 m |
| Baseline S00, 2 m via one sensor-selected observation goal | PASS, original goal SUCCEEDED and stopped | 1.938 m | .126 m | .018 m |
| Baseline S01, first observation leg | FAIL localization; original goal never attempted | 1.662 m | .104 m to intermediate goal | .220 m |
| Baseline S01 repeat with stationary diagnostic | FAIL localization; original goal never attempted | 1.588 m | .072 m to intermediate goal | .200 m |
| Experimental odometry-prior S00, observation leg | FAIL, intermediate goal TIMEOUT | 1.209 m | .151 m to intermediate goal | .015 m |
| Experimental odometry-prior S00, direct 2 m | PASS, goal SUCCEEDED and stopped | 2.022 m | .128 m | .037 m |
| Experimental odometry-prior S01, first observation leg | FAIL localization; original goal never attempted | 1.701 m | .138 m to intermediate goal | .276 m |
| Same S01 profile, loop closing disabled | FAIL localization; original goal never attempted | 1.584 m | .089 m to intermediate goal | .245 m |

All eight completed trials confirmed the final stop, recorded zero non-ground contacts and zero combined translation/rotation guard outputs. S01 partial-leg minimum evaluator clearance was .586 m. None traversed the obstacle or attempted the original destination. The experimental S00 observation timeout is retained as FAIL despite small localization error. The 3 m robot physically approached the destination but did not satisfy autonomous-goal/localization acceptance. Earlier interrupted 3 m evidence remains INCOMPLETE, separately from this completed failing repeat.

Evidence directories are named in [summary.json](evidence/native-lidar-followup-20261104/summary.json). Reproduce those measurements using `python3 docs/evidence/native-lidar-followup-20261104/analyze.py`. Navigation result JSON, bounded launch logs, actual command/trajectory CSVs, initial/final sensor maps and individual observation results are retained. Map exports are diagnostic artifacts; none is loaded into runtime navigation.

## Observation-goal policy and motion limits

`scripts/lidar_observation_goals.py` chooses incremental goals using only the live SLAM grid, the local LiDAR grid and the current SLAM pose. Candidates must belong to the connected global FREE component and have a locally observed FREE disk of at least .95 m, rounded conservatively to grid cells. Unknown and occupied cells are excluded. Steps are .35–2 m, require progress toward the original destination, and avoid recently selected goals. The policy can fail to find a goal; it is not a complete exploration planner.

The original known-start map frame is fixed across legs. Reanchoring each goal at the robot's updated position would move the destination and hide localization drift; rotated-frame regression tests cover that failure. Every leg must pass the existing stop, localization and goal checks before continuation. Original-goal results and intermediate results are saved separately.

S01 uses a separate conservative global planner footprint: .38 m padding gives a .5325 m inscribed radius, exceeding the validated .5236 m swept turn disk. Inflation is .70 m. This planning preference does not replace live swept-disk checks, establish near-obstacle turning success, or guarantee that the resulting plan is executable. Unknown remains non-plannable. S00 retains its original Nav2 parameters.

The authoritative guard, angular/linear caps, stopping model, command timeouts, sensor freshness, primitive-switch confirmation, RGB gate and production-turn evidence are unchanged. Mixed or reverse requests still stop. The runner additionally cancels on independently evaluated clearance below .10 m. Evaluator truth is never an input to the observation policy, SLAM, planner, costmaps or command guard.

## Localization diagnosis

After the repeated S01 partial leg, a 45 s zero-command hold collected 444 independent samples. Truth displacement during the hold was zero. At hold start, raw wheel-odometry XY error was .00640 m and fused-odometry error .00155 m relative to the known start. SLAM localization error was .19964 m; `map -> odom` translation was (-.20019, -.02137) m. That transform remained identical throughout the hold, while wheel odometry and truth remained stationary. Localization error ended at .19965 m. All recorded guard outputs were zero.

This isolates a substantial SLAM map correction rather than physical motion or continuing wheel drift during the hold. It does not yet distinguish scan-match ambiguity, earlier scan timing, graph constraints or loop-closure effects. Fresh TF alone cannot prove localization accuracy: the baseline 3 m run first exceeded .10 m error at about world x=2.192 m with only .006 s TF age.

The optional `slam_toolbox_odom_prior.yaml` keeps scan matching and online mapping enabled while testing a stronger distance prior (`distance_variance_penalty=.0025`, minimum penalty .05). The [installed-version Karto implementation](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.8.5/lib/karto_sdk/src/Mapper.cpp) penalizes corrections as squared distance divided by this parameter. These values are an experimental simulator hypothesis informed by measured odometry, not hardware covariance calibration. The baseline configuration remains the default.

The first experimental observation leg had maximum sampled localization error .02471 m but timed out. The direct 2 m experimental trial passed with maximum error .04694 m and maximum absolute TF age .068 s. Its live parameter query confirmed .0025. A late query in the earlier observation trial found no node after shutdown; that file is a failed audit, not confirmation. Improved localization alone does not qualify a failed mission or justify promoting the experimental profile. The follow-up S01 trial still failed at .27578 m error. A separate diagnostic with the same prior and `do_loop_closing=false` also failed at .24531 m (maximum sampled .25595 m). This demonstrates that disabling loop closing alone does not resolve the bias; it leaves scan matching / sequential scan constraints and sensor-model/timing effects to investigate. Both S01 intermediate actions reported SUCCEEDED, but mission acceptance failed and original-goal continuation was prevented. The no-loop YAML is retained as an experimental input in the follow-up evidence, not a new runtime default.

New runs snapshot and hash the exact SLAM/Nav2 YAML bytes before launch, and launch those snapshots. Earlier runs predate this provenance addition and retain their run settings and launch logs; no retroactive runtime-parameter claim is made.

## Resources, rates and remaining requirements

The eight completed headless trials had measured scan rates 5.00–5.01 Hz, RGB counts consistent with about 5 Hz, map counts consistent with about 1 Hz, and RTF .993–.997. Sampled matching-process peak RSS was 1715–2005 MiB and mean CPU 324–442% (roughly 3.2–4.4 logical cores). Resource measurements are sampled, command-pattern based and may double-count shared pages. Multi-leg final resource summaries cover the final leg; per-leg files retain the preceding leg's samples. Legacy `wsl_*` JSON key names do not indicate WSL execution. A separate exact TF/SLAM-pose publication-rate measurement is not provided by this evaluator; TF age/error traces are retained.

S01 obstacle traversal and its original six-metre destination remain unvalidated. Repeated stopping and mixed-request suppression consume substantial route time; this does not justify accepting combined/reverse motion or weakening the interlock. Baseline 3 m localization and the RGB `floor_support_area_too_small` stop near the far wall are additional longer-route blockers. The next requirements are reproducible localization on S01 partial legs, measured turn clearance along an actual avoidance route, and a bounded original-goal success. Parking, general exploration and hardware autonomy remain unvalidated.

Full sequential rebuild: nine packages passed in 12.0 s. All 98 Phase 8C offline tests passed; rosdep reports all system dependencies satisfied. These logs are saved beside the summary. No new dependency installation was required.

## Reproduction of accepted capabilities

The accepted graphical S00 profile remains:

```bash
cd ~/AutonomousNavigationCar
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/observe_gui_commission.py \
  "docs/evidence/gui-new-$(date +%Y%m%dT%H%M%S)" --domain-id 45
```

This launches Gazebo/RViz, records resources/screenshots and independently checks the .75 m autonomous goal. GUI windows alone do not establish goal success. Existing RViz shader warnings/layout limitations remain documented in the graphical report. Run only one project simulation at a time; the unchanged available-RAM preflight can block launch.

The accepted baseline two-metre observation workflow is headless:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/commission_lidar_s00.py \
  docs/evidence/native-turn-production-matrix-20261103/envelope.json \
  "docs/evidence/s00-observation-new-$(date +%Y%m%dT%H%M%S)" \
  --distance-m 2 --timeout-s 180 --domain-id 49 --observation-steps 1
```

No successful S01 graphical command is claimed. Phase 6 reference navigation is unchanged and remains a separate predefined-map demonstration; its command is in [the S00 commissioning guide](native-nav2-s00-20261103.md).
