# Phase 8C continuation: guarded Nav2 integration and live-demo gate

Date: 2026-11-02
Branch: feature/phase8c-lidar
Parent reviewed commit: 6736a5c46d303bbd33c986175d431c06d00cdcbf

## Result

The Nav2, GUI, evaluator, and launcher integration is now present in source and passes offline build/static checks. **No Gazebo or Nav2 simulation was started in this continuation.** The Windows host had only 619 MiB available RAM at the final preflight check; the new launcher correctly refuses to start a GUI run below its 2.5 GiB threshold. Production turning and moving SLAM also remain unvalidated, so the Nav2 readiness gate stays false.

## Implemented

- Added a low-memory Nav2 launch that routes controller and behavior requests through velocity smoothing, collision monitoring, then the existing sole safety command guard. The final /cmd_vel publisher remains the guard.
- Changed the global costmap to consume the live SLAM /map and block unknown cells. Local/global costmaps use the robot polygon footprint. Nav2 request speeds remain bounded; the guard's default angular cap remains 0 rad/s.
- Added an opt-in Gazebo GUI path and an RViz view for the map, scan, plan, TF, robot and footprint. Headless remains the default.
- Added a known-start online-SLAM goal evaluator. Gazebo entity state is consumed only by that evaluator. Its result requires fresh RGB, scan, map and odometry, a successful goal or bounded no-route abort, no contact events, and exactly one /cmd_vel publisher named anc_safety_command_guard. It saves JSON and truth-trajectory CSV plus best-effort WSL CPU/RSS/swap samples.
- Added scripts/run_lidar_demo.sh with scenario, seed, duration, GUI and goal options. It checks Windows host RAM through WSL interop and WSL available RAM before scenario generation or launch. It requires 2 GiB Windows RAM for headless, 2.5 GiB for GUI, and 1.5 GiB available inside WSL.
- Added an explicit false readiness file. A requested goal cannot start Nav2 until production turning, moving SLAM and the live guarded command chain each have verified evidence paths. With the present false gate, --goal prints BLOCKED and the launcher would run sensor/SLAM only after memory passes.

## Verification and preserved evidence

- The Nav2/GUI package build passed for 8 packages before a final duplicate-YAML-key cleanup; colcon was not rerun after that formatting-only edit. The final YAML passed parsing and all six Nav2 static tests. See stage9-nav2-gui-build-2026-11-02.log.
- Final offline tests passed: Nav2 config 6/6, publisher audit 2/2, common infrastructure 9/9, LiDAR model 15/15, parking proposal 7/7. See stage9-offline-validation-acceptance2-2026-11-02.log; earlier runs are retained.
- Launch argument inspection passed for the RGB + LiDAR profile and custom Nav2 launch.
- Launcher syntax/help checks passed. The memory preflight returned BLOCKED before scenario generation or Gazebo startup: Windows 619 MiB available against the 2,560 MiB GUI minimum; WSL 3,147 MiB available against 1,536 MiB. See stage9-launch-preflight-verified-2026-11-02.log. No demo run directory was created.
- Earlier launcher attempts exposed strict-mode and display-unit issues. Both were corrected; the final evidence log reflects the corrected behavior. The earlier logs are retained for audit.
- One intermediate offline run caught a malformed new default-profile test assertion; it was corrected and all six Nav2 config tests passed in the final acceptance2 log. The failed intermediate log is retained.
- git diff --check passed before this report was written.

These checks validate source/configuration behavior only. There is no new moving-car result, Nav2 goal result, obstacle-avoidance result, parking run, Gazebo/RViz screenshot, or desktop video from this continuation. The previously saved sensor composite is not an autonomous Gazebo demonstration.

## Capability matrix

| Capability | Classification | Current evidence or blocker |
|---|---|---|
| LiDAR sensing | PASS WITH LIMITS | Prior verified 358/360 usable beams at 4.94 Hz; planar/low-obstacle limits remain. |
| RGB + LiDAR fusion | PASS WITH LIMITS | Existing fuser tests pass; RGB added no cells over LiDAR in the accepted clear S00 comparison. |
| Safe forward motion | PASS WITH LIMITS | Prior bounded clear-S00 and selected fault-stop trials; this continuation did not repeat them. |
| Safe production turning | BLOCKED | Production angular authority remains zero; short calibration pulses do not establish a production stopping envelope. |
| Combined motion | NOT TESTED | No simultaneous linear/angular trial passed. |
| Moving SLAM/localization | BLOCKED | Stationary mapping only; moving drift against truth remains unmeasured. |
| Nav2 goal navigation | BLOCKED | Integration is built, but no live goal was issued; readiness prerequisites are false. |
| Obstacle avoidance | BLOCKED | No LiDAR Nav2 obstacle course was run. |
| Dynamic obstacle handling | NOT TESTED | No moving obstacle trial. |
| Replanning | NOT TESTED | No blocked-route replanning trial. |
| Automatic parking | BLOCKED | 7/7 proposal unit tests only; no live parking movement or hold. |
| Fault recovery | PASS WITH LIMITS | Previously tested stop behavior for selected sensor/command faults; recovery while autonomously moving is unverified. |
| Resource performance | NOT TESTED | No Nav2 runtime sample; only current memory preflight and earlier bounded phase measurements. |
| Gazebo/RViz visual demonstration | PASS WITH LIMITS | Existing headless sensor composite only. GUI launch path is built but not opened or recorded. |
| UNO Q readiness | PARTIAL / PENDING HARDWARE | Transfer design exists; no board/application/hardware testing was done. |

## Reproduction

From WSL:

~~~bash
cd /home/ashish/AutonomousNavigationCar
./scripts/run_lidar_demo.sh --scenario S01 --seed 101 --gui --goal
~~~

At the recorded memory level it exits before starting the simulator. After sufficient RAM is available, the still-false readiness gate prevents this command from claiming or attempting autonomous navigation; it runs the sensor/online-SLAM profile and records that the goal is blocked. Production turning and moving-SLAM evidence are prerequisites before a Nav2 trial can be enabled.
