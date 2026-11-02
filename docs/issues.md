# Open issues after Phase 8C Branch 1 assessment

## Resolved through Phase 3

- WSL2 Ubuntu 24.04 and the ROS 2 Jazzy/Gazebo Harmonic toolchain were verified.
- E: storage, WSL memory/swap, WSLg, and GPU visibility were checked.
- ROS talker/listener, headless Gazebo, RViz2, packaged Nav2 sample, and a REFERENCE map-frame goal were verified.
- The custom differential-drive model builds and passes forward, reverse, rotation, wall-stop, odometry, and TF checks within the limits recorded in the phase report.
- Phase 3 RGB camera and CameraInfo, IMU, clock, odometry, wheel joint states, and the full robot TF chain publish at verified rates.
- The Phase 2 contact gap was diagnosed. Gazebo's Contact system was loaded after UserCommands, so sensors on the runtime-spawned robot were not registered. Loading UserCommands before Contact and bridging the actual scoped contact topic restored wall-contact telemetry.
- RViz2 displayed the camera image and robot frames while headless Gazebo ran.
- Phase 4 added known-start EKF localization using wheel vx/vy and sanitized gyro z, with truth isolated to the evaluator. Aggregate motion RMSE and wall-stall x error improved in the accepted single run.
- The corrected model passed the Phase 3 regression: 20/20 sensor smoke checks.

## Resolved through Phase 5

- Phase 5 added six bounded worlds, deterministic layout generation, geometry/no-route validation, evaluator-only truth capture, known-start spawn/TF, and cleanly terminated headless trials. It did not test navigation or safe stopping.

## Resolved through Phase 6 reference navigation

- Nav2 completed the selected S00-S04 reference goals and aborted the static S09 no-route goal within the declared bound. The S09 run recorded nonzero travel before the abort and a truth-twist/odometry discrepancy; see the Phase 6 report and per-run result.

## Resolved through Phase 7 RGB-only perception

- Added a separate REALISTIC_RGB_ONLY profile and class-agnostic image mask, confidence, overlay, and health outputs with a verified camera-only input contract.
- Added offline-only geometry truth evaluation and saved RGB/mask/confidence/overlay examples for S00-S04 plus the P7C1 appearance/occlusion challenge.
- Verified missing/stale RGB fails closed to UNKNOWN and confidence zero; static checks passed 13/13 and the final six-case dataset completed with clean shutdowns.
- Accepted only as PASS WITH LIMITS: zero false-FREE on known occupied pixels and useful floor coverage were measured, but occupied recall is 26.67%, total UNKNOWN coverage is 80.98%, and P7C1 returned all UNKNOWN when floor support was unobservable.

## Resolved through Phase 8 stop-only baseline

- Added a camera-only local traversability snapshot that preserves unsupported space as UNKNOWN, plus a separate 20 Hz safe-stop supervisor with sole cmd_vel authority.
- Static projection checks passed 7/7; nine bounded S00/S01/S02/S03/S09 headless stop trials passed with 1,052/1,052 zero command samples and no movement beyond the 0.02 m stop tolerance.
- Camera absence, missing filtered localization, command timeout, stale RGB, fresh-input recovery, sole command authority, no Nav2, and no known-start map-to-odom were checked.
- Phase 8 is accepted only as PASS WITH LIMITS for stop-only. Autonomous motion remains blocked: observed FREE cells do not connect to the footprint, and emergency-braking distance is unmeasured.

## Resolved through Phase 8B feasibility assessment

- Added configurable camera mount height, downward pitch, projection band, and floor-seed candidates; dataset evidence records the live camera TF used for each view.
- Compared C0-C4 candidates. C2 achieved the longest full-footprint corridor (0.48 m), but had 0% occupied recall across 3,390 evaluator-known occupied pixels.
- Measured 24 isolated simulator stopping trials at four speeds, two friction settings, and three repeats. Maximum stop distance was 0.00298 m at 0.18 m/s and mu=0.5.
- Calculated a 0.656429 m forward-corridor requirement including measured camera age/latency, stop distance, Phase 4 slip-derived localization allowance, and explicit safety margin. The 0.48 m best corridor fails the gate; Phase 8 remains locked to zero velocity.
- Final seven-package build and Phase 6/7/8 regression evidence passed. No autonomous movement or Phase 9 work was performed.

## Resolved through Phase 8C Branch 1

- Added an opt-in planar LiDAR/ROS bridge, scan validation/fault handling, one-scan FREE/OCCUPIED/UNKNOWN map, conservative optional RGB fusion, and separate REALISTIC_RGB_LIDAR profile.
- Added a measured forward-only gate and sole authoritative command guard. Invalid/stale scan, map, localization, RGB perception and command inputs fail closed; bounded S00 motion and fault stops passed.
- Phase 8 remains stop-only and shared launch keeps LiDAR disabled by default in older profiles.
- Fresh Phase 3/4 regressions and Phase 5/6/7/8 static tests, Phase 8 live stop-only check, and one Phase 8B brake repeat are indexed in docs/evidence/phase8c/lidar/README.md.
- SLAM did not demonstrate drift reduction; Nav2 was not enabled. Stage D remains untested.

## P0 before hardware-faithful claims or transfer

- Measure chassis footprint, height, mass, wheel radius, wheel separation, and center of mass. Current simulation values are placeholders.
- Measure camera mount pose and calibration, including resolution, FOV, intrinsics, distortion, noise, and image timing.
- Measure IMU noise/bias and derive estimator covariance from hardware data.
- Define hardware watchdog and independent electrical emergency stop before any motor integration.

## P1 before hardware-faithful navigation/perception experiments

- Reduce wheel-odometry divergence under slip. Phase 4 low-friction trial measured +0.4705 m wheel error with zero chassis displacement; EKF output followed the wheel estimate. Do not replace odometry with simulator pose.
- Improve rectangle closure and raw rotation calibration. The accepted Phase 4 final7 truth ended 0.134 m / 0.980 rad from start; filtered endpoint error exceeded raw odometry in that run. This supersedes the earlier 0.127 m / 0.882 rad summary.
- Validate sensor behavior over multiple seeds, longer runs, and camera viewpoints.
- Measure or tune contact report semantics before treating repeated contact points as collision episodes.
- Test Gazebo's GUI and RViz2 running together; Phase 3 used headless Gazebo with RViz2.
- Decide whether to add calibrated camera noise/distortion only after choosing hardware and measuring it.

## P2 for later navigation experiments

- Add unknown-pose acquisition separately from the known-initial-pose baseline.
- Phase 5 completed seeded geometry worlds for S00-S04 and S09 plus a headless scenario harness. Phase 6 added an ORACLE_REFERENCE Nav2 baseline and one bounded static no-route abort. Dynamic obstacles, RGB perception, unknown-pose localization, general sensor-loss safe stopping, and parking remain later-phase work.
- Improve Phase 7 RGB perception recall and test camera/floor calibration under varied lighting and real hardware; preserve UNKNOWN as distinct from FREE.
- Improve camera-derived free-space coverage and occupied recall, and characterize localization uncertainty. Phase 8B measured simulator stopping distance, but it is not a hardware or all-surface bound; require a continuous corridor longer than the Phase 8B gate before enabling motion or replanning.
- Define parking tolerance and dwell using measured footprint, then test stationary pose error.
- Plan UNO Q adapters only after simulation interfaces and regression checks are stable.

Earlier phases remain complete only within their stated limits. Phase 8 remains an independent RGB-only stop-only baseline. Phase 8C Branch 1 permits bounded forward motion in clear S00, but angular motion, obstacle avoidance and goal navigation remain blocked or unrun. Drift correction, low-obstacle coverage, broader scenarios, unknown-pose acquisition, parking and hardware transfer remain unresolved.
