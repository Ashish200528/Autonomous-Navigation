# Prioritized issue list

## P0 — resolve before dependent implementation

- [ ] Measure robot chassis footprint, height, mass, wheel radius and wheel separation.
- [ ] Select/measure camera mount pose and record calibration, resolution and FOV.
- [ ] Verify free disk and WSL memory budget before large package installs; preserve space for logs.
- [ ] Confirm WSLg rendering path; if unreliable, document headless launch path and limits.
- [ ] Verify current official ROS Jazzy/Gazebo Harmonic installation steps and APT candidate versions before installing.

## P1 — resolve before first robot model or navigation gate

- [ ] Choose arena dimensions and confirm nominal start/goal fit with clearance.
- [ ] Refine parking tolerance and dwell after the footprint is known.
- [ ] Define wheel/IMU noise and covariance values from measurements or labeled assumptions.
- [ ] Choose first camera profile and resource budget for image topic/visualization.

## P2 — later experiments

- [ ] Define unknown-pose acquisition separately from known-initial-pose baseline.
- [ ] Add dynamic obstacle and lighting/occlusion variants to seeded scenarios.
- [ ] Decide when the optional three-ultrasonic profile becomes available.
- [ ] Plan UNO Q adapters only after simulation interface and regression gate are stable.
