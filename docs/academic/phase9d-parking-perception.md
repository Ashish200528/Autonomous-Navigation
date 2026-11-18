# Phase 9D — ArUco parking perception foundation

IMPLEMENTED / TESTED OFFLINE / TESTED IN GAZEBO AND ROS. No autonomous parking controller or motor authority is implemented by this milestone.

## Camera and isolated bay

The generated parking scene reuses the existing 320×240 robot RGB sensor with horizontal FOV 1.0471975512 rad, zero distortion, camera centre (0.10,0,0.24) m in base_footprint and downward pitch .8 rad. Successful navigation camera defaults are unchanged. The existing downward view can observe a floor marker over a bounded range; no alternate camera or depth camera was installed.

The bay has cyan borders at y=±.25 m, length .8 m. ArUco DICT_4X4_50 marker ID **23** lies in the world XY plane at (.35,0,.004) m. The black marker square is **.18 m** wide; its white quiet border extends the textured panel to .225 m. An explicit OBJ vertex/UV mapping ties marker XY corners to world XY for independent evaluation. The staging pose is (-.32,0,0), and the intended final robot pose is (-.05,0,0). These coordinates define the test and evaluator only; runtime detection receives pixels and CameraInfo, not world poses. Captions on saved real camera images label the intended target as evaluator-only.

The distant staging view does not detect the tag. Thus a future handoff would need separately validated navigation to a visible observation pose (for example the tested -.08 m position), followed by a new bounded controller qualification. This report does not authorize that motion or claim parking readiness.

## Detection and pose mathematics

`aruco_pose.py` detects ID 23 and extracts corners. Missing, wrong or multiple/duplicate IDs invalidate the observation. Adaptive threshold detection is tried first. The initial actual Gazebo image failed because the printed quiet border was in shadow; a standard global Otsu fallback recovers the same marker from those saved pixels. The failed pilot and its original result remain in the evidence. No simulator coordinates enter the detector.

For known side length s=.18 m, the marker object corners are (-s/2,+s/2,0), (+s/2,+s/2,0), (+s/2,-s/2,0), (-s/2,-s/2,0). OpenCV IPPE_SQUARE solves `pixel ~ K (R X + t)` with calibrated distortion. The module requires positive optical depth, finite results, adequate pixel edge size (≥12 px), and ≤2 px reprojection RMS. Near-equal planar solutions differing by more than 10° are rejected. Input timestamps must be within -.05 to +.35 s of the consumer clock. Incorrect/missing calibration and stale images fail closed.

`t` is marker-centre translation in the camera optical frame (x right, y down, z forward); Euclidean distance is ||t||. Rotation is marker-to-optical. The node publishes the complete pose with the source image header. An image-up orientation metric is available; it is explicitly not robot yaw. Robot heading in the marker plane is computed by transforming the robot's forward unit vector through the documented camera mount and inverse estimated marker rotation, then taking atan2(y,x). The mount pitch is an optional node parameter (`camera_pitch_down_rad`, default .8); physical transfer requires a measured mount rather than relying on this simulation value.

## Actual stationary results

Nine fixed robot poses produced 36 actual Gazebo frames. Seven views / 28 frames yielded accepted poses (**77.8%**); all failed frames are included. Accepted camera-to-marker distances span .34380–.49051 m. Across accepted observations:

- Maximum absolute distance error: **.01446 m**.
- Maximum translation-vector error: **.01455 m**.
- Maximum robot heading error: **.002745 rad** (about .157°).
- Repeated frames at each static pose were deterministic; this is not evidence of robustness under real sensor noise or moving-camera blur.

At .30298 m, detections were rejected by the existing reprojection limit. At .61692 m, the marker was missing. Those limits were retained. Full per-frame estimated poses, independently computed evaluator truths, source timestamps and rejection reasons are in `phase9d-results/parking/result-with-robot-heading.json`. The baseline result and real RGB/detection images are also retained.

The evaluator computes ground truth from the known static robot placement and camera/marker transforms after detection, independently of the detector. It never publishes truth as runtime perception. No robot was placed at a parking target and counted as autonomous parking; all views are stationary perception tests.

Peak sampled Gazebo server RSS was 950 MiB; minimum sampled host available RAM was 3660 MiB. A separate offline loop on an actual camera image measured mean/p95 detector latency 1.182/1.330 ms, process RSS 141.5 MiB and aggregate CPU about 607% during the saturated loop (OpenCV's default threading, not continuous live duty). This benchmark excludes simulator CPU. Exact scope/measurements are in detector-benchmark.json.

## ROS observation and faults

`aruco_node.py` subscribes `/camera/image_raw` and `/camera/camera_info`, checks their dimensions, optical frame, timestamps and freshness, and publishes only:

- `/parking/marker_pose` — PoseStamped in the source optical frame.
- `/parking/perception_health` — valid/invalid state, reason, measured distance/quality/latency and `motor_authority=false`.
- `/parking/marker_overlay` — actual camera pixels with detected corners and status.

The final sensing-only Gazebo/ROS probe passed five real observations and five pose messages, preserving source timestamps. It froze only its own static simulator, injected a blank image as an explicitly synthetic **fault**, and checked marker-loss rejection, stale-camera health and absence of new poses after loss. It audited that the observer has no `/cmd_vel` publisher. No synthetic image is counted as positive Gazebo detection evidence. Final results/logs are in `parking/live-final/`.

Eighteen targeted Phase 9 tests and all 210 existing pytest cases pass. Tests cover semantic-label preservation, scene and image split integrity, frozen checkpoint hash, tensor/checkpoint/metric checks, ArUco ID/duplicate handling, known-size pose fixtures, extrinsic heading convention, ROS encoding/timestamps and fail-closed calibration/staleness, plus rejection before simulator startup when RAM is insufficient. No expensive S01 navigation run was performed for these isolated perception changes.

## Reproduction

```bash
cd ~/AutonomousNavigationCar
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
GZ_IP=127.0.0.1 GZ_PARTITION=parking-matrix OPENBLAS_NUM_THREADS=1 python3 scripts/phase9/parking_capture.py /tmp/new-parking-matrix --views 9
ROS_DOMAIN_ID=91 GZ_IP=127.0.0.1 GZ_PARTITION=parking-live OPENBLAS_NUM_THREADS=1 python3 scripts/phase9/aruco_live_probe.py /tmp/new-parking-matrix/view-00.sdf /tmp/new-parking-live
OPENBLAS_NUM_THREADS=1 .venv/phase9/bin/python -m unittest discover -s scripts/phase9 -p 'test_*.py'
```

Run simulations sequentially. The generated scenes have no drive plugin or commanded motion and use the existing 2 GiB preflight / 768 MiB runtime memory floors. To run the observer independently with a proper live camera/clock: `python3 scripts/phase9/aruco_node.py --ros-args -p use_sim_time:=true`. Consumers must check both health and pose age; absence of new poses is not permission to reuse a stale pose.

## Transfer and next gate

UNO Q / Fire Bird V integration remains future work. It requires real camera intrinsics/distortion and extrinsics, a physically measured .18 m marker, timing/latency checks on the target computer, explicit serial/controller interfaces and separate braking/clearance trials. The ATmega controller must receive only independently guarded bounded primitives. No physical firmware, reverse motion, combined motion or new movement authorization was added.

The next milestone can design a parking controller using the measured visible range and fail-closed states, but must separately validate fresh-pose acquisition, moving-camera accuracy, marker-loss stopping, guard integration, clearance and final parking tolerances before motor trials. This foundation is **not authorization for autonomous parking**.
