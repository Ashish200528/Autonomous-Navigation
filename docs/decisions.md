# Decisions and assumptions

Status: Phase 0 baseline with verified Phase 1 and Phase 2 results. Changes to the implementation track should be evidence-backed and recorded here.

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

## Unresolved decisions

- Real chassis dimensions, mass, footprint, center of mass, wheel radius/separation, friction, encoder model, and acceleration limits.
- Camera mount XYZ/RPY, resolution, frame rate, horizontal FOV, intrinsics, and distortion.
- IMU noise/bias and odometry covariance.
- Gazebo contact-sensor message path; no contact messages were observed in the Phase 2 trial.
- Wheel slip and odometry behavior under a hard obstacle stop.
- Scenario arena set, unknown-pose acquisition, realistic perception, safe-stop policy, and parking validation.
- Whether WSLg performance is sufficient for Gazebo GUI and RViz concurrently.
