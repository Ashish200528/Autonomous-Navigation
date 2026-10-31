# Decisions and assumptions

Status: Phase 0 baseline; decisions below are frozen for the first implementation track unless an evidence-backed change is recorded here.

| ID | Decision | Rationale / status |
|---|---|---|
| D-01 | Use Windows 11 + WSL2 Ubuntu 24.04 LTS (`noble`). | Installed distro verified in the environment report. |
| D-02 | Target ROS 2 Jazzy + Gazebo Harmonic + `ros_gz` + RViz2 + Nav2. | Compatible baseline specified by the project roadmap; verify package availability before installation. |
| D-03 | Keep this simulation repo at `/home/ashish/AutonomousNavigationCar`, independent of the existing UNO Q application. | Prevents simulation work from overwriting hardware work. |
| D-04 | Use a differential-drive robot with configurable, initially unmeasured mechanics. | Wheel geometry, chassis dimensions and mass must be measured or explicitly selected and documented before modeling. |
| D-05 | First navigation baseline starts with a known map-frame transform and accepts a fixed or operator-selected map-frame goal. | Unknown initial-pose acquisition is a separate experiment, not hidden simulator truth. |
| D-06 | Initial nominal goal is `map: (x=2.0 m, y=0.0 m, yaw=0.0 rad)` from nominal start `(0,0,0)`. | Provisional open-arena smoke-test coordinate; must be checked against the first world bounds. It is not a hardware waypoint. |
| D-07 | Initial parking acceptance target is position error ≤0.20 m, yaw error ≤10°, held stationary for 5 s. | Provisional starting threshold based on the roadmap's suggested values; refine after chassis/footprint measurement. Not a measured result or safety certification. |
| D-08 | Name sensor modes `REFERENCE` and `REALISTIC`; report them separately. | Reference sensing can validate navigation plumbing but must not imply camera-only capability. |
| D-09 | Realistic perception represents `FREE`, `OCCUPIED`, and `UNKNOWN`; unknown is not clear. | Conservative behavior and honest reporting under uncertain RGB observations. |
| D-10 | Keep simulator truth and collision evaluation in evaluator-only outputs. | Navigation/perception must not subscribe to truth, perfect depth/raycast, or hidden obstacle positions. |
| D-11 | Stop on stale/unavailable required perception or pose inputs. | Fail-safe simulation behavior; hardware still requires independent electrical watchdog/E-stop. |
| D-12 | Start with a low-memory/headless-first environment. | Current WSL limit is about 3.5 GiB RAM; GUI/rendering and concurrent Gazebo/RViz capability remain unverified. |

## Unresolved decisions

- Chassis length, width, height, mass, footprint and collision/inertial properties.
- Wheel radius, wheel separation, encoder model, friction and maximum velocity/acceleration.
- Camera mounting XYZ/RPY, resolution, frame rate, horizontal FOV, intrinsics and distortion.
- IMU noise/bias and odometry noise/covariance.
- Arena dimensions, first world limits and exact test start/goal relative to that world.
- Whether the provisional parking dwell/tolerance fits the physical chassis and demonstration.
- Whether WSLg graphics performance can sustain Gazebo and RViz; headless-only gate if not.
- Exact `ros_gz`, Nav2 and support package versions after checking official installation guidance and APT candidate versions.
