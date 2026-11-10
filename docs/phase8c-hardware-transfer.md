# Phase 8C hardware transfer plan: Arduino UNO Q

Status: design only. No UNO Q hardware was connected, flashed or modified. No purchase is recommended.

## Compute and operating system

Arduino documents UNO Q as a QRB2210 Linux MPU with four Cortex-A53 cores up to 2.0 GHz plus an STM32U585 Cortex-M33 MCU up to 160 MHz. Variants have 2 GB RAM / 16 GB storage or 4 GB RAM / 32 GB storage; Arduino recommends 4 GB for a full single-board-computer workflow. The UNO Q runs Debian Linux. Current Arduino material reports a UNO Q 4 GB workflow on Debian GNU/Linux 13 (Trixie), which does not establish ROS compatibility.

ROS 2 Jazzy lists Ubuntu 24.04 Noble on arm64 as Tier 1. REP-2000 lists Debian Bookworm on arm64 as Tier 3 source-build support; that does not establish Jazzy binaries or a validated install on UNO Q Debian Trixie. Before deployment, inspect actual OS and architecture and prove ROS package support, ros_gz needs, DDS discovery, device access and resource headroom. A Noble container is only a candidate after serial/USB passthrough, network/DDS, permissions, timing and performance are tested. Do not replace the existing UNO Q application.

Prefer the 4 GB variant if the Linux MPU must host ROS perception/navigation, but begin with the MCU wheel controller and minimal sensor bridge. The existing host simulation already uses a WSL cap near 3.5 GiB. Onboard Nav2 + SLAM + RGB is not benchmarked; the recommendation is an estimate.

## Proposed responsibilities

Linux MPU:
- Run ROS drivers and validation for physical LiDAR, camera, sanitized IMU and encoder feedback.
- Run mapping, localization, costmaps and planners only after simulation and physical gates pass.
- Own the high-level velocity gate and observe MCU acknowledgements, sensor freshness and localization health.
- Convert bounded geometry_msgs/Twist requests to differential wheel setpoints from measured geometry.

STM32U585 MCU:
- Run wheel PID, encoder counting and motor PWM/current/thermal monitoring on deterministic timers.
- Reject malformed, out-of-range, stale-sequence or expired commands.
- Enforce a local command watchdog and report encoders, watchdog state and driver faults.
- Control a motor-driver enable line physically overridden by a latching emergency stop.
- Never depend on Linux executor scheduling or Bridge latency for immediate wheel control.

## Proposed MPU-to-MCU contract

Design only; no device firmware implements this yet:
- Command fields: sequence number, left/right target wheel speed, enable state and expiry. Proposed update 20-50 Hz and maximum 200 ms watchdog before ramp-to-zero and driver disable. Tune on real motors.
- Feedback fields: sequence acknowledgement, encoder counts or speeds, MCU monotonic timestamp, watchdog state, driver faults and E-stop state.
- At boot, link loss, invalid command, missed sequence or fault: MCU output disabled and wheel targets zero. Linux must see healthy feedback before enable.
- Send compact setpoints/telemetry through Arduino Bridge. Keep camera and LiDAR on Linux USB/camera interfaces.

Official Bridge uses MessagePack RPC, Linux /dev/ttyHS1, MCU Serial1 and 115200 baud. Arduino reserves these endpoints for arduino-router; application code must not open them directly. The 256-byte per-call limit suits compact motor/status RPC, not scans or images.

## Physical LiDAR comparison

| Model | Official characteristics | Integration notes |
|---|---|---|
| RPLIDAR A1 | 360 deg, 0.15-12 m, 5.5 Hz, <=1 deg, 170 g, 96.8 x 70.3 x 55 mm, 5 V / 100 mA / 0.5 W, UART at 3.3 V | Simpler lower-bandwidth UART and lower listed power; larger body and slower rate. Measure min range and scan height on chassis. |
| RPLIDAR C1 | 0.05-12 m at 70% reflectivity, 0.05-6 m at 10%; 8-12 Hz, 0.72 deg; TTL UART 460800; about 110 g. Datasheet lists 5 V, 230-260 mA at 10 Hz and 800 mA startup. | Better listed near range/resolution and smaller package, with higher bandwidth and power transient. Use separately budgeted 5 V. TX may reach 3.5 V; do not assume direct MCU compatibility. Use level-safe USB-UART or a suitable translator. |

No current price comparison was made. Cost, exact kit wiring, availability and mechanical mounting remain unresolved. Neither model was tested on the chassis.

## Electrical and mechanical requirements

- Fuse the motor supply to measured startup/stall current; isolate it from compute/sensor rails as required by the driver.
- Give LiDAR a regulated 5 V rail sized for startup current and cable drop. Do not power motors from logic headers.
- Confirm every signal voltage before wiring. UNO Q maker MCU headers are 3.3 V; MPU JCTL/JMISC/JMEDIA pins are 1.8 V.
- Add connector strain relief, sensor vibration control and a rigid sensor-to-base transform.
- Install a physical latching E-stop that interrupts motor-driver enable/power independently of Linux and Bridge.
- Guard wheels, lift chassis for initial direction checks, then use a secured test area and person-held physical stop for first floor tests.

## Calibration and bring-up checklist

1. Record UNO Q SKU, RAM/storage, Debian version, kernel, architecture, available RAM and devices. Prove ROS support without changing the existing app.
2. Identify motor driver, voltage/current limits, encoder, battery protection and E-stop.
3. Measure wheel radius, track width, caster location, footprint, sensor transforms and camera calibration.
4. Measure wheel deadband, speed, acceleration, coast/brake deceleration and emergency stop distance under multiple loads, surfaces and battery levels.
5. Calibrate encoder scale/direction and compare odometry with independent pose on straight, spin and loop trials.
6. Measure IMU bias/noise/timestamp offset; verify covariance and frame signs.
7. Validate LiDAR rate/error/min range, surfaces, vibration, lighting and dropped scans. Verify voltage, baud and startup power margin.
8. With wheels lifted, test MCU watchdog after Linux/Bridge loss; then verify physical E-stop before floor movement.
9. Start with wheels lifted and low speed in a secured area. Record stopping distance before autonomous commands.
10. Repeat fault, localization, mapping, navigation and parking tests on hardware.

## ROS abstraction boundary

Keep anc_navigation_common sensor-neutral. Hardware drivers should expose validated sensor_msgs/LaserScan, sensor_msgs/Image + CameraInfo, sensor_msgs/Imu, encoder odometry, TF and health status. One ROS adapter owns /cmd_vel; internal MCU RPC may not bypass the guard. Keep Gazebo plugins and physical drivers as separate launch profiles with matching topic/safety contracts.

## Sources

- UNO Q architecture: https://docs.arduino.cc/hardware/uno-q
- UNO Q datasheet: https://docs.arduino.cc/resources/datasheets/ABX00162-ABX00173-datasheet.pdf
- UNO Q manual: https://docs.arduino.cc/tutorials/uno-q/user-manual/
- Bridge API: https://docs.arduino.cc/software/app-lab/bridge/bridge-api
- UNO Q power/voltage: https://docs.arduino.cc/tutorials/uno-q/power-specification
- UNO Q Trixie workflow note: https://docs.arduino.cc/pt/tutorials/uno-q/ai-coding-agents/
- ROS REP-2000: https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst
- ROS Jazzy Ubuntu installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- SLAMTEC A1 specs: https://www.slamtec.com/en/lidar/a1spec
- SLAMTEC C1 specs: https://www.slamtec.com/en/c1/spec
- SLAMTEC C1 datasheet: https://wiki.slamtec.com/download/attachments/83066883/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf?api=v2&modificationDate=1700531479533&version=1
