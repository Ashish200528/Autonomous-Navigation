# Phase 8C live demonstration recording report

- **Date:** 2026-11-02
- **Repository:** `/home/ashish/AutonomousNavigationCar`
- **Branch / HEAD:** `feature/phase8c-lidar` / `c1a41d9`
- **Mode:** Headless Gazebo; no desktop screen capture or RViz window, to reduce load.
- **Simulation launch profile:** `ros2 launch anc_lidar realistic_rgb_lidar.launch.py world_file:=<world> enable_slam:=false enable_nav2:=false`

## Saved video

- **File:** `docs/evidence/live-demo-2026-11-02/phase8c-live-demo.mp4`
- **Format:** MP4, 640×360, 5 fps, 1,309 frames, 261.8 seconds (4:21.8), 6,534,691 bytes.
- **Content:** Composite of the robot RGB camera, live 2D LiDAR scan, safety-gate state, scene label, and actual `/cmd_vel`. It is not a recording of the full Gazebo 3D desktop.
- **Verification:** Opened with OpenCV FFMPEG; first, middle, and last frames all decoded. The file was finalized by cleanly interrupting the recorder. An earlier probe while it was still being written reported `moov atom not found`; that was an incomplete-file check and was resolved on finalization.

## Scene results

| Scene | Observed result |
| --- | --- |
| S00 Empty Arena | Safety gate authorized the single guarded 1-second, 0.05 m/s straight command. The robot moved about 4.9 cm; output returned to zero after the command expired. |
| S01 Single Box | No motion. Gate reported stale scan/health data (`scan_health_stale`). |
| S02 Random Obstacles, seed 101 | Generator reported `route_found:true`, `validation_passed:true`; gate authorized; stationary. Layout SHA-256: `bf4f06eaf451774aa9155aa163f805b43e993d69f770bab94c3aeefa8b8504b5`. |
| S02 Random Obstacles, seed 202 | Generator reported `route_found:true`, `validation_passed:true`; gate authorized; stationary. Layout SHA-256: `6ec2ec7135ec52dbfdbc8bbbf004af9d3c7c8e8ec0360455fb69332fa5547add`. |
| S03 Narrow Passage | Gate reported 0.8 m corridor; no motion or turning attempted. |
| S09 No Route | Scene loaded and left running headless. Gate snapshot: authorized/pass, measured corridor 3.60 m, scan rate 4.91 Hz, usable beams 99.17%. Actual `/cmd_vel` was all zeros. |

**Scope limitation:** Nav2 and SLAM were disabled throughout, as requested for this low-load demonstration. Therefore this recording does not demonstrate Nav2 planning or prove global no-route detection in S09. No turns or obstacle traversal were attempted.

## Errors and environment notes

- Initial desktop browser launch from Ubuntu had failed earlier because no Linux browser opener was installed; this recording avoided that dependency.
- Some prior world shutdowns showed Python teardown exceptions in the safety command guard / wheel-odometry covariance path; S01 and S02 seed 202 also needed Gazebo server escalation after launch-manager timeouts. The final S09 Gazebo server is still running; the recorder has exited.
- During final verification WSL showed 3.5 GiB total memory and 2.1 GiB available.
- No project source files or packages were changed/installed for this recording. The temporary recorder was `/tmp/anc_live_demo_recorder.py`. No commit was made. Existing untracked evidence files were left untouched; this report and the MP4 are new evidence artifacts.


- Git status: the MP4 matches the existing .gitignore rule *.mp4; this report is untracked. No ignore rules were changed and nothing was committed.
