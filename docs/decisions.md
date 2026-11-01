# Decisions and assumptions

Status: Phase 0 baseline and verified Phase 1 through Phase 8 stop-only results with stated limits. Phase 8 autonomous motion remains blocked by the feasibility gate. Changes to the implementation track should be evidence-backed and recorded here.

| ID | Decision | Rationale / status |
|---|---|---|
| D-01 | Use Windows 11 + WSL2 Ubuntu 24.04 LTS (noble). | The installed Ubuntu-24.04 distro is verified as WSL 2. |
| D-02 | Use ROS 2 Jazzy + Gazebo Harmonic + ros_gz + RViz2 + Nav2. | Installed versions were verified: Gazebo Sim 8.15.0, Nav2 1.3.13, ros_gz 1.0.24. |
| D-03 | Keep this simulation repo at /home/ashish/AutonomousNavigationCar, independent of the existing UNO Q application. | Prevents simulation work from overwriting hardware work. |
| D-04 | Use a differential-drive robot with configurable, initially unmeasured mechanics. | Phase 2 uses marked design placeholders. Measure the real robot before sim-to-real claims. |
| D-05 | First navigation baseline starts with a known map-frame transform and accepts a fixed or operator-selected map-frame goal. | Phase 1 reference goal passed after an initial pose was provided; unknown-pose acquisition is a separate experiment. |
| D-06 | The configured project baseline goal is map (2.0 m, 0.0 m, 0.0 rad) from nominal start (0,0,0). | Provisional open-arena coordinate, not a hardware waypoint. The packaged reference trial used a separate map and goal at (2.0,-0.5). |
| D-07 | Initial parking acceptance target is position error <=0.20 m, yaw error <=10 degrees, held stationary for 5 s. | Provisional target, not tested; refine after chassis/footprint measurement. |
| D-08 | Name sensor modes REFERENCE and REALISTIC; report them separately. | The successful Nav2 test used REFERENCE map and laser data, not camera-only perception. |
| D-09 | Realistic perception represents FREE, OCCUPIED, and UNKNOWN; unknown is not clear. | Conservative behavior and honest reporting under uncertain RGB observations. |
| D-10 | Keep simulator truth and collision evaluation in evaluator-only outputs. | Phase 2 truth was used only by the motion evaluator; controller and navigation code did not subscribe to it. |
| D-11 | Stop on stale/unavailable required perception or pose inputs. | Fail-safe simulation behavior; hardware still requires independent electrical watchdog/E-stop. |
| D-12 | Start with a low-memory/headless-first environment. | WSL has about 3.5 GiB RAM. RViz2 was checked separately; concurrent Gazebo GUI/RViz use remains untested. |
| D-13 | Use a known-start identity map -> odom transform for the initial localization baseline. | Unknown-pose acquisition remains separate. |
| D-14 | Fuse wheel vx/vy from /odometry/wheel and sanitized gyro z from /imu/data only. | IMU orientation is unavailable; wheel pose, raw Gazebo pose, and evaluator truth are excluded from the EKF. |
| D-15 | Assign odom -> base_footprint to the EKF. | Gazebo DiffDrive TF bridge is removed; launch config assigns the EKF the dynamic edge. Per-edge publisher GID cannot be correlated from current RMW MessageInfo. |
| D-16 | Keep ground truth inside test evaluators. | Phase 4 evaluator calls GetEntityState and writes local comparison evidence; estimator nodes do not receive that data. |
| D-17 | Make Phase 7 RGB perception class-agnostic and fail closed to UNKNOWN. | Runtime uses camera RGB/CameraInfo plus clock age only; simulator truth stays in a separate offline evaluator. |
| D-18 | Limit FREE ground projection to the calibrated 0.50-0.60 m forward band. | Camera height/pitch, pinhole, and planar-floor assumptions are provisional; ambiguous or unsupported pixels remain UNKNOWN. This is not metric depth. |
| D-19 | Keep REALISTIC_RGB_ONLY perception separate from ORACLE_REFERENCE navigation. | Phase 6 reference maps/costmaps remain intact; Phase 7 does not start navigation. |
| D-20 | Preserve unsupported local grid cells as UNKNOWN and project only high-confidence FREE pixels. | RGB obstacle appearance has no metric range; treating it as a metric occupied cell would overstate what the sensor provides. |
| D-21 | Keep Phase 8 motion locked behind a feasibility gate and publish unconditional zero velocity in the accepted stop-only profile. | Current FREE patch is disconnected/too short and braking distance is unmeasured; no autonomous motion is justified. |
| D-22 | Keep Phase 8 separate from ORACLE_REFERENCE and omit the known-start map-to-odom transform. | The realistic profile has no reference map or privileged initial localization; Phase 6 remains a separate comparison. |

## Unresolved decisions

- Real chassis dimensions, mass, footprint, center of mass, wheel radius/separation, friction, encoder model, and acceleration limits.
- Camera mount XYZ/RPY, resolution, frame rate, horizontal FOV, intrinsics, and distortion.
- IMU noise/bias and odometry covariance.
- Gazebo contact-event semantics; repeated contact records are telemetry, not unique episodes.
- Wheel slip remains uncorrected by the available sensors; Phase 4 measured about 0.468 m false displacement with the chassis stationary.
- Covariance calibration across hardware measurements, multiple seeds, and missing/stale wheel input.
- Scenario arena set, unknown-pose acquisition, improved RGB perception recall/calibration, evidence sufficient to pass the motion gate, RGB-only navigation, and parking validation.
- Whether WSLg performance is sufficient for Gazebo GUI and RViz concurrently.
