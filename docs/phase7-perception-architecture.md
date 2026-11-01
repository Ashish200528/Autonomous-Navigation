# Phase 7 RGB-only perception architecture

## Runtime boundary

anc_perception implements the separate REALISTIC_RGB_ONLY profile. The realistic_perception.launch.py includes the RGB camera, sanitized IMU and existing EKF odometry for the system profile, but anc_rgb_perception subscribes to exactly:

- /camera/image_raw
- /camera/camera_info
- /clock (timestamp-age validation only)

Its outputs are /perception/image_mask, /perception/confidence, /perception/overlay, and /perception/health. The mask is mono8: FREE=0, OCCUPIED=100, UNKNOWN=255. UNKNOWN confidence is zero. Missing/stale image data, unusable CameraInfo, unsupported distortion, or insufficient floor support invalidates the frame and publishes all UNKNOWN rather than retaining earlier FREE observations.

Runtime code does not subscribe to /odom, /odometry/filtered, /imu/data, /map, /scan, depth, Gazebo entity-state services, or scenario manifests. It does not use obstacle identities, exact poses, or hidden geometry. The Phase 6 ORACLE_REFERENCE map and costmap remain isolated in their reference launch and are not silently switched to this profile.

## Classification and projection

The CPU-only OpenCV classifier uses broad image appearance and gradient cues, not named object classes. A fixed Lab reference [171,128,128] was measured from the Phase 3 camera frame floor ROI [96,216,224,235]. Runtime floor appearance does not adapt from scene geometry or current output labels. A calibrated-looking low-gradient seed of at least 24 pixels and a connected floor component at least 4,000 pixels are required to validate a frame.

FREE is limited to pinhole ground-plane projection on a planar floor in the camera-centered forward band [0.50,0.60] m. Assumptions are a 320x240 image, CameraInfo fx/fy about 277.13 px and principal point (160,120), zero lens distortion, level camera pitch, and camera height about 0.18 m above the floor. Nominal simulated horizontal FOV is 60 degrees. Invalid/nonzero distortion is rejected; unsupported or occluded ground regions remain UNKNOWN. Occupied candidates are generic visual cues, not semantic classes. No metric depth or arbitrary obstacle distance is inferred from one RGB image.

These values are specific to simulated floor material, illumination, rendering, and provisional camera mounting. They have not been calibrated against the physical camera or real floor. Projection range, floor appearance, and image thresholds need new validation if these conditions change.

## Offline evaluation boundary

scripts/phase7_ground_truth.py is used by the dataset/evaluator only. It ray-casts the known test world to label visible floor, occupied geometry, and unobservable pixels for comparison. It is not a dependency of anc_perception and does not publish labels or geometry to ROS. The dataset runner records raw RGB, predicted mask/confidence/overlay, evaluator-only truth, health, timing, and the live ROS graph contract per view.

The accepted run and exact seeds are recorded in docs/evidence/phase7/dataset/accepted-phase7-final-verified/dataset-results.json and simulation/scenarios/scenario_registry.yaml. Aggregation distinguishes evaluator-known pixels from unknown/out-of-view pixels; P7C1's all-UNKNOWN result is visible in per-case evidence and is not reported as an accuracy win.

## Interpretation and safety limits

The final 12-view run recorded zero false-FREE pixels among 465,242 known occupied pixels and 88.01% useful coverage on known floor, but occupied recall was only 26.67% and UNKNOWN covered 80.98% of all predicted pixels. The P7C1 floor-like object/shadow challenge invalidated both views because floor support was unobservable. Weak-contrast obstacles in S03/S04 were frequently UNKNOWN. These outcomes support only a conservative, narrowly calibrated visual prototype. The mask is not a costmap, metric depth map, verified safety boundary, autonomous navigation input, collision-avoidance result, or sim-to-real claim. Phase 8 is outside this implementation.
