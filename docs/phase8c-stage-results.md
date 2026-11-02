# Phase 8C remaining LiDAR development results

Date: 2026-11-02
Branch: feature/phase8c-lidar
Project: /home/ashish/AutonomousNavigationCar

This additive continuation preserves the earlier report and failed attempts.

## Stage status

| Stage | Status | Verified scope |
|---|---|---|
| 1. Turning | PASS WITH LIMITS | Six short pure-yaw calibration pulses and swept-disk denial probes. Production yaw disabled; no combined motion or wall/box movement trial passed. |
| 2. SLAM/localization | PASS WITH LIMITS | 25-second stationary S00 map and map-to-odom publication. No moving pose estimate observed; drift reduction unproven. |
| 3. Realistic goal navigation | BLOCKED / NOT RUN | Production turning and moving localization unvalidated; Nav2 stayed off. |
| 4. Avoidance/replanning | BLOCKED / NOT RUN | No obstacle traversal, moving obstacle, Nav2 path or replanning test. |
| 5. Parking | BLOCKED LIVE | Sequential rotate/drive proposal passed 7 unit tests; not runtime-wired and no parking goal issued. |
| 6. Comparison/resources | PARTIAL WITH LIMITS | Saved phase evidence aggregated; not a controlled head-to-head navigation study. |
| 7. UNO Q transfer | PARTIAL DESIGN ONLY | Architecture, interface, sensor comparison and checklist documented; no physical board tested. |
| 8. Demonstration | PASS WITH LIMITS | Headless sensor composite, safe run guide and RViz config; no global plan or desktop capture claimed. |

## Stage 1: calibration-only rotation

Six 1.2-second S00 pulses at +/-0.02, +/-0.05 and +/-0.10 rad/s passed. Command-to-zero-output latency was 0.00386-0.04985 s. First slow pose-derived rate was observed 0.019-0.172 s after zero; settling took 0.579-0.722 s. Absolute yaw changes before zero were 0.0216-0.1085 rad. Maximum center drift was 0.004337 m, residual yaw was zero in saved samples, and no non-ground contact was recorded.

These characterize short pulses in clear S00 only. They do not establish production angular stopping distance, combined-motion envelope or rotation beside a wall while moving. Default angular cap is 0.0 rad/s. Calibration mode rejects translation and combined requests.

Swept disk radius is 0.523593 m: footprint circumradius plus center-drift allowance, sensor-mount uncertainty and margin. The S00 near-wall no-motion probe denied rotation with 18 occupied and 4 unknown swept cells. Near-box/constriction probes also found occupied/unknown cells, but both additionally failed RGB floor-support validation and are not isolated clearance-only tests.

## Stage 2: stationary mapping

SLAM settings: 0.25 s minimum scan interval (4 Hz processing cap), queue size 3, 0.05 s transform publishing and 3 s map updates. Bounded S00 stationary run: 25.001 wall seconds / 24.53 simulation seconds, 128 scans, 8 map messages, 508 map-to-odom TF messages. It produced a 155 x 115 map at 0.05 m resolution: 1,447 free, 6 occupied, 16,372 unknown. Map was exported to PGM/YAML. All 520 /cmd_vel samples were zero; no non-ground contacts.

No SLAM pose messages were observed. Stationary truth drift cannot measure localization accuracy or improvement. The tuned log contained no queue-full or EKF missed-update-rate warning in this 25-second window, but did include a startup TF-cache message-filter drop and diagnostic-subscriber QoS warnings. This brief check does not prove earlier issues solved. The failed monitor attempt remains preserved.

## Stages 3-5: navigation, avoidance and parking

Nav2 goals, obstacle traversal, moving-obstacle response, replanning and live parking were not attempted. They require a production yaw stop model and moving localization evidence that are not established. The parking proposal passed 7/7 unit tests, remains non-authoritative and explicitly blocks when the gate is calibration-only. It is not wired to runtime velocity.

## Stage 6: profile-specific comparisons and resources

Generated JSON, CSV and plots aggregate saved evidence:

- Phase 6 ORACLE_REFERENCE: S00-S04 goal success 5/5, mean final XY error 0.14382 m and zero contact events; S09 aborted for expected no-route. This is not realistic LiDAR navigation.
- Phase 7 RGB: 12 views, false-FREE rate 0, useful known-floor FREE coverage 88.01%, occupied precision 100%, recall 26.67%, UNKNOWN 80.98%.
- Phase 8 RGB-only stop profile: 9/9 runs passed with 1,052/1,052 zero commands and maximum truth displacement 0.00001055 m; motion was not attempted.
- Phase 8C LiDAR: 358/360 beams usable at 4.94 Hz. RGB added zero mapped cells over LiDAR-only in the accepted clear S00 snapshot.
- Phase 8C stationary SLAM: 25 seconds, 128 scans, 8 map updates and no SLAM pose messages.

These profiles are not head-to-head. Phase 6 used an oracle reference map; Phase 7 evaluated image labels; Phase 8 stayed stopped; Phase 8C evaluated LiDAR and bounded straight motion.

WSL is assigned about 3.5 GiB and host has 8 GiB. Phase 8 recorded max sampled process RSS 852,672 KiB and minimum available memory 1,956,248 KiB. One summarized process sample during tuned SLAM reported about 2.2 GiB available and Gazebo around 508,616 KiB RSS / 112% instantaneous process CPU; this is not a whole-system peak or time series. After simulation shutdown, idle available memory was about 3.0 GiB. These are heterogeneous capacity observations, not a benchmark ranking.

## Stage 7: hardware boundary

The hardware plan documents a Linux MPU / STM32 split, compact Bridge contract, watchdog/E-stop requirements, LiDAR A1/C1 tradeoffs and calibration checklist. UNO Q Debian Trixie compatibility with ROS Jazzy remains unverified; Jazzy Tier 1 is Ubuntu Noble arm64. No hardware/software/wiring changed. Readiness PARTIAL; physical validation PENDING HARDWARE.

## Stage 8: saved demonstration

Headless video is 640 x 360 at 5 fps for 4:21.8 and composites RGB, LiDAR, gate state and /cmd_vel. It is not a Gazebo desktop recording and shows no Nav2 planning. MP4 stays local under existing *.mp4 ignore rule. Use docs/phase8c-lidar-run-guide.md. RViz was not opened in this pass.

## Capability matrix

| Capability | Result |
|---|---|
| LiDAR sensing | PASS WITH LIMITS |
| Local mapping | PASS WITH LIMITS |
| Safe rotation | PASS WITH LIMITS - calibration only; production yaw zero |
| SLAM and moving localization | BLOCKED |
| Autonomous goal navigation | BLOCKED |
| Obstacle avoidance | BLOCKED |
| Replanning | BLOCKED |
| Autonomous parking | BLOCKED live; 7 unit tests passed |
| Fault handling | PASS WITH LIMITS for tested forward-motion and stop-only inputs |
| Full simulation evaluation | BLOCKED / INCOMPLETE |
| Hardware deployment readiness | PARTIAL |
| Physical hardware validation | PENDING HARDWARE |

An initial build wrapper failure was due to set -u before colcon; the bounded retry completed all 8 packages in 38 seconds. The first partial log and wrapper error remain preserved. Run aggregate reporting with python3 scripts/phase8c_stage6_report.py from repository root. It does not start a simulator or publish commands. GitHub issue listing returned no existing issues; none was created.
