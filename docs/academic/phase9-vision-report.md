# Phase 9 execution report

Dataset generation, validation and real CNN training/evaluation are implemented. The authoritative LiDAR/SLAM/Nav2 configurations and guard sources were not edited. Existing uncommitted Phase 8C changes were preserved and excluded from Phase 9 commits.

Evidence: phase9-data/ contains images, masks, generated worlds, camera provenance, exact timestamps, scene splits and review images. phase9-results/ contains the best trained checkpoint, metrics, held-out comparison images, losses and test logs. The first live probe processed eight real Gazebo camera frames with matching source/output timestamps and no CNN motor publishers. It passed inference but exposed a shutdown-only double-rcl_shutdown exception; live-first/ retains that failure. A guarded rclpy.ok() cleanup correction is included; the follow-up probe is recorded separately. No GUI S01 success is inferred from this vision work.

## Reproduce

```bash
cd ~/AutonomousNavigationCar
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
GZ_IP=127.0.0.1 GZ_PARTITION=phase9-collection OPENBLAS_NUM_THREADS=1 python3 scripts/phase9/collect.py /tmp/new-phase9-data --scenes 36
python3 scripts/phase9/dataset.py /tmp/new-phase9-data
OPENBLAS_NUM_THREADS=1 .venv/phase9/bin/python scripts/phase9/train.py /tmp/new-phase9-data /tmp/new-phase9-training --epochs 25
ROS_DOMAIN_ID=87 GZ_IP=127.0.0.1 GZ_PARTITION=phase9-live .venv/phase9/bin/python scripts/phase9/live_probe.py docs/academic/phase9-results/best.pt /tmp/new-phase9-live
```

Run collection, training and the live test sequentially. Collection refuses existing output directories and requires 2 GiB available RAM, with a 768 MiB runtime floor. Large future datasets should use a local path outside Git; retain a small reviewed subset plus hashes and commands. Current dataset is small enough to commit in full. Node command alone: `.venv/phase9/bin/python scripts/phase9/shadow_node.py docs/academic/phase9-results/best.pt --ros-args -p use_sim_time:=true`. It subscribes /camera/image_raw and publishes /perception/cnn/{mask,confidence,overlay,health}. Health contains actual latency and source age; stale or invalid input invalidates health. Low-confidence pixels become 255; there is no motor publisher. Missing models and checkpoint mismatches fail closed.

Next: strengthen independent scene/texture-held-out evaluation and fault tests before any optional hazard-stop integration. ArUco, parking and IoT are outside this assignment and remain NOT YET IMPLEMENTED.

## Verified outcome

- Dataset: 108 RGB images and 108 semantic masks at 320×240; 87 distinct images admitted to splits (71 train, 8 validation, 8 test). The remaining duplicate captures stay archived but never enter training/validation/test. All class counts and duplicate exclusions are in metadata/validation.json.
- Dataset size at validation: about 2.2 MB including generated worlds and provenance. Minimum sampled host available RAM during capture: 4497.1 MiB. Initial per-process capture RSS instrumentation excluded the server root process and recorded zero, so that value is unavailable, not a measured zero. The collector is corrected for future runs. No capture memory floor was bypassed.
- Six targeted checks pass: exact cloned sensor parameters, scene/pixel split isolation, known confusion-matrix arithmetic, tensor/checkpoint checks, ROS encoding/header conversion, missing-model/stale-input rejection. Existing full offline regression: 210 pytest tests pass. No full S01 run was needed or performed for this optional perception work.
- Final live probe: SIMULATION PASS, 8 valid predictions, 8 masks, all input/output timestamps matched; mean measured inference 2.035 ms. live-final/live-rgb-and-cnn.png is an actual camera/overlay capture. Final CNN node shutdown is clean. Existing simulator cleanup warnings remain in process-0.log.
- GPU training was not used. The GTX 1650 was detected with 4 GiB VRAM; the CPU-only PyTorch environment is deliberate and measured. Peak training RSS was 584.7 MiB. No performance claim on CUDA is made.

The classical comparison maps UNKNOWN abstentions to semantic other; that is an evaluation convention with a documented limitation. Current data is highly colour-consistent and closely related across scenes. High held-out scores do not demonstrate robustness to new worlds, new lighting or real cameras. Additional materials and independent worlds are the next useful dataset extension. No change to runtime navigation authority or demonstrated navigation capability is claimed. ArUco parking and IoT were not started.
