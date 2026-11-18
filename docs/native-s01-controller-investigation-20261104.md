# S01 execution follow-up — 2026-11-04

The scan-endpoint localization correction is committed and directly pushed as `a6c74d54754dbab7cd0c457b103223d98445bc6e`; remote SHA verification succeeded. Its accepted direct S00 2 m and 3 m results remain in [the source-fix report](native-slam-source-fix-20261104.md). Baseline configurations, graphical S00, reference navigation and the earlier accepted observation workflow remain available.

## Completed experiments

| Experiment | Actual result | Final localization error |
|---|---|---:|
| Longer-lookahead profile, S00 2 m | PASS / SUCCEEDED and stopped | .01936 m |
| Lower angular-acceleration profile, S00 2 m | PASS / SUCCEEDED and stopped | .00355 m |
| Stricter observation policy, S00 2 m | PASS: intermediate and original goals | .00932 m at original goal |
| PoseProgressChecker, S01 | First observation PASS; second TIMEOUT | .02616 m at failed second leg |
| Longer lookahead, S01 | First observation PASS; second TIMEOUT | .01855 m at failed second leg |
| Lower angular acceleration, first S01 attempt | FAIL before motion: parameter-audit array serialization | not evaluated |
| Lower angular acceleration, S01 retry | First observation PASS; second TIMEOUT | .00939 m at failed second leg |
| Stricter observation policy, S01 | Three observation goals PASS; original goal TIMEOUT | .02876 m at original timeout |
| Planned continuation diagnostic, S01 | First observation PASS; second TIMEOUT; no continuation executed | .00496 m at failed second leg |

The stricter S01 run travelled 4.510 m cumulatively, but stopped 2.708 m from the original destination. Its three intermediate localization errors were .02128, .01926 and .02995 m, with confirmed stops. The second strict-policy attempt did not reproduce full intermediate completion: its second leg timed out .415 m from the intermediate target. No original S01 goal was reached, and no S01 graphical success is claimed.

The first stricter S00 workflow travelled 1.936 m to its original 2 m destination, with .13558 m goal error and a confirmed final stop. Its mission elapsed time was 110.4 s. The direct longer-lookahead S00 trial took 40.3 s, versus 54.8 s for the earlier corrected default direct 2 m trial; these are single trials with different live scans and are not a controlled performance benchmark.

All completed motion trials in this table recorded zero non-ground contacts, zero mixed guard outputs and confirmed final stops. The startup serialization failure has no accepted movement/stop result. The exact result files, per-leg measurements, sampled resources and rates are indexed in [summary.json](evidence/native-s01-controller-investigation-20261104/summary.json).

## Controlled changes and limits

All three Nav2 profiles are optional. The default S00/Nav2/SLAM configurations are unchanged.

* `nav2_s01_pose_progress.yaml` uses the installed PoseProgressChecker with .5 rad angular progress. The .10 m translation criterion and 12 s watchdog remain unchanged. Earlier independent failure windows showed real rotations of .52–1.08 rad despite only .03–.04 m translation; the translation-only checker misclassified those windows. Other windows had genuinely limited movement. [Nav2's implementation](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_controller/plugins/pose_progress_checker.cpp) accepts either sufficient translation or rotation.
* `nav2_s01_sequential_lookahead.yaml` changes only the lookahead/minimum lookahead from .35 to .60 m relative to that profile. It passed S00 but did not qualify S01.
* `nav2_s01_measured_slew.yaml` changes only RPP's angular acceleration to .12 rad/s² relative to the pose-progress profile, matching the existing smoother's commanded angular slew limit. [RPP uses that parameter for slew and stopping prediction](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp); its default was 3.2. This is not a new physical braking certification. S01 did not improve, so the profile is not promoted. The target/command trace recorded 144 pure-turn samples with path-heading error above .2 rad, only four requesting <=.02 rad/s; an initial low-command deadband alone does not explain the failure.

Repeated mixed requests were denied, and primitive changes incurred the existing confirmed-stop wait. These are intended safety responses, not grounds to remove the guard or shorten the wait. The experiments establish an execution-throughput problem after the localization fix; they do not establish that one parameter alone causes every timeout. Observation goal selection varies with live sensor-built maps, which also changes route/turn duration.

The optional conservative observation policy caps steps at 1.5 m, chooses arrival-bearing intermediate headings, and requires a wholly observed FREE .60 m radius capsule before the original goal. It checks intersecting cell corners and map bounds; occupied/unknown cells deny readiness. This is stricter than connected-goal visibility, uses no scenario obstacle oracle, and still delegates planning to Nav2 and motor authority to the existing guard. Default observation behavior remains unchanged. Original goal position/yaw, the 300 s mission bound, localization .10 m limits, collision/clearance limits and stop confirmation are unchanged.

An optional post-timeout diagnostic implementation keeps the original mission FAIL, requires every existing safety observation plus confirmed cancellation before a separate bounded action, and labels that action non-qualifying. Its eligibility checks are offline tested. The attempted run failed an intermediate leg before this path could execute; no live continuation or original-goal success is claimed. The default is disabled.

## Instrumentation, preservation and validation

Passive diagnostics now record the controller lookahead target and query actual runtime parameters after lifecycle activation. The recorded S01 retry confirms scan matching and loop closing enabled, normal distance variance .5, `/realistic/scan/slam_hits`, RPP angular acceleration .12, smoother cap .10 rad/s, and the original goal tolerances. ROS array-valued parameter serialization was corrected after the startup failure; a regression test covers it. Late external CLI queries which found no node after shutdown remain failed audits, not positive parameter evidence.

Full scan-range traces for controller-parameter probes are archived under `~/.local/state/anc-evidence-raw/20261104-controller/`. Each affected run includes `raw-source-local.json` with the full archive hash/path, the bounded `controller-source.jsonl.gz`, full derived source analysis and scan-correction CSV. The bounded stream retains truth, TF, SLAM poses, odometry, graph, IMU and controller targets; raw range arrays remain local. Regeneration from bounded traces produced identical per-scan pose-attribution CSVs. `scripts/analyze_slam_source.py <directory>` handles the bounded trace; `--source <full-local-archive>` restores range-level analysis. The complete causal baseline/corrected source traces in the earlier localization milestone remain in Git.

All nine packages rebuilt sequentially in 12.1 s. The full discoverable offline suite passed 126 tests; separate Phase 6 reference, Phase 7 perception and Phase 8 safety static checks passed. Existing fault-stop contracts, angular authority/envelope, freshness gates, unknown-space denials, reverse/mixed denials and the sole authoritative `/cmd_vel` guard were not changed.

S00 remains demonstrated. S01 goal execution, reproducible avoidance, parking, exploration and hardware autonomy remain unvalidated. The next substantive requirement is a controller/planning protocol designed for the already validated sequential motion primitives, followed by independent bounded turn/stop qualification and S00 regression before S01. More passing parameter-file tests alone would not resolve that requirement.
