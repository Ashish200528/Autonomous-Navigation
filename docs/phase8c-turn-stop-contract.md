# Phase 8C angular stopping contract V2

This contract replaces the *test evaluator's* combined 0.75 s fault-to-confirmed-settled criterion. It does not change runtime command/sensor timeouts, speed caps, perception validity or swept-space checks. Historical V1 failures remain failures under V1.

## Why separate physical stop and observation?

The command watchdog is 0.30 s. Even instantaneous physical deceleration, followed by the existing mandatory 0.50 s stationary hold, needs >=0.80 s in the worst case before timer/sampling delays. V1's 0.75 s combined criterion therefore conflicts with that watchdog/observation design. Recorded fault traces confirmed this: first verified slow samples arrived in 0.220–0.474 s, then approximately 0.54 s of observation followed. The correct fix is to specify physical response and independent confirmation, not reduce the hold or shorten sensor validity to make scores pass.

## Explicit acceptance conditions

- Physical stopping **upper bound <=0.75 s** from fault injection or zero request. An observed slow sample time plus source/query uncertainty is used; it is not a claim of exact instantaneous stopping.
- **Cumulative absolute angular travel upper bound <=0.05 rad**, measured from the last pose bracketing the trigger through confirmation, including reverse movement. This is stricter than checking endpoint yaw difference or only yaw after the first zero sample.
- Separately record cumulative post-zero angular travel upper bound; it must also fit 0.05 rad before an evidence envelope can load.
- Stationary pose-derived rates <=0.005 m/s and <=0.01 rad/s for **at least 0.50 s**, evaluated using a conservative source-time bracket. Missing/nonfinite rates, frozen simulation timestamps and rebound cannot establish a hold. The collector extends the observation; it does not truncate the hold.
- Maximum relevant intersample gap <=0.15 s. Physical-stop uncertainty includes the worst relevant query/source bracket. Angular uncertainty adds angular cap times (maximum gap + source/query bracket).
- Confirmation deadline **1.40 s = 0.75 physical + 0.50 observation + 0.15 sampling**. This explicitly increases the *combined observation deadline* from V1; it does not increase the physical stopping limit or shorten observation. Both physical and confirmation tests must pass independently.
- Clearance >=0.10 m, center drift <=0.10 m, no non-ground collisions, correct direction and actual bounded motion. Fault tests require a real recorded injection. A capture failure is a failed trial.
- Exactly one /cmd_vel publisher, anc_safety_command_guard. No combined translation/rotation authorization follows from pure-yaw tests.

Relevant query uncertainty is computed around the fault/stop segment. A slow startup query before movement is not uncertainty in a later measurement. Source gaps during the actual stop do count and can fail the test. The measurements are sampled simulator bounds, not formal continuous-time guarantees or hardware certification. Between-sample excursions cannot be excluded without assumptions about the measured angular envelope; production/hardware generalization requires further validation.

## Independent tests and implementation

`tests/phase8c_turn_stop_contract.py` is independent of ROS and the runtime controller. Regression tests reject late stopping, excessive angular travel including rebound cancellation, incomplete holds, missing zero output, excessive sample gaps, source uncertainty, frozen simulation time, and nonfinite evidence. A separate loader validates a complete hashed thirteen-case measurement matrix; cached PASS booleans alone are insufficient.

ROS subscriber delivery order is not controller ordering. The first fault batch recorded one -0.0004 s apparent suppression delay because output and guard-state callbacks arrived in different order. Later telemetry adds source monotonic decision timestamps to safety and guard states. Reports distinguish observed delivery times from source decision times; neither modifies runtime control decisions.

## Truth isolation

The service evaluator uses a separate ROS node for /gzserver/get_entity_state requests, but capture timeouts still occurred. The optional native stream evaluator subscribes to Gazebo transport /world/anc_phase2_test/dynamic_pose/info, selects the named anc_robot model, and preserves the Pose_V simulator timestamp. /clock bounds source age and frozen/stale data fails closed. It publishes no truth topic or runtime TF.

The ROS TF bridge was rejected for this stream because the conversion uses individual pose headers/child-frame metadata, whereas the observed Gazebo stream carries its timestamp at Pose_V level and model identity in pose.name. Source inspected: https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/tf2_msgs.cpp and https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/geometry_msgs.cpp. Native gz.transport13 and gz.msgs10 Python bindings were already installed; no dependency or driver replacement was needed.

## Production boundary

Passing this contract in calibration does not authorize normal production turning. The evidence loader permits only a bounded production-controller validation profile after all thirteen calibration categories independently pass. That profile uses a measured envelope and blocks translation. A new live thirteen-case matrix under that profile is required before the same loader permits normal sequential forward/turn authority. Defaults remain disabled; malformed/missing/modified evidence or incomplete measurements produce zero authority. The generic guard explicitly rejects mixed commands when combined_motion_authorized is false. No Nav2 commissioning goal may start solely because these offline mechanisms exist.
