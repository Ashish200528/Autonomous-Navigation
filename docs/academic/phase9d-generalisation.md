# Phase 9D — frozen CNN generalisation and one bounded retrain

TESTED OFFLINE / DATA CAPTURED IN GAZEBO. The original 108-pair dataset and checkpoint remain unchanged. The original checkpoint SHA256 is `fa59636f29a1c7f048361839f64811416a5bdfba4fdfce99f3943f361e24a4fe`.

The separate `phase9d-data/` contains 90 RGB/mask pairs from 30 new scene configurations, using the same independently labelled semantic sensors and exact timestamp pairing. Floor and obstacle ambient/diffuse reflectance, wall/box dimensions and lighting direction/intensity vary deterministically with seeds 17000 + scene + 1000. Camera viewpoints remain the existing stationary collection grid. These are colour/reflectance and geometry shifts, not a test of real textures or physical-camera noise. The original navigation camera, SLAM, Nav2 and guard configuration were not edited.

The scene split is 72 train / 9 validation / 9 test images. Original images are not mixed into this training set. Test scenes are scene IDs 9, 19 and 29; their views are never used for gradient updates or checkpoint selection. The experiment recipe was recorded before evaluation in `phase9d-results/experiment-plan.json`: original 4,899-parameter architecture, seed 91, Adam .003, batch four, at most 25 epochs, validation patience five. No architecture, threshold or training parameter was tuned against this test set.

| Model / evaluation domain | Mean IoU | Floor IoU | Obstacle IoU | Other IoU | Obstacle precision | Obstacle recall | Missed obstacle pixels |
|---|---:|---:|---:|---:|---:|---:|---:|
| Original CNN / original held-out set | .99818 | .99948 | .99577 | .99930 | .99922 | .99654 | 40 |
| Original CNN / shifted held-out set | .48536 | .47425 | .000077 | .98175 | .000080 | .001983 | 3020 / 3026 |
| Diverse retrain / shifted held-out set | .64880 | .97539 | 0 | .97101 | 0 | 0 | 3026 / 3026 |

All nine shifted test frames are included. Increased diversity improved average floor/other segmentation but **did not produce a useful obstacle detector**. The retrained candidate is not promoted over the original shadow checkpoint. No CNN is authorized to grant movement or override LiDAR. The architecture's local appearance features and the small colour-consistent training domains are material limitations. A future experiment needs stronger spatial/context cues, broader materials and a fresh held-out scene set; further tuning on these nine test images is not justified.

Classical OpenCV rejected all nine shifted test frames with `floor_seed_unobservable`, preserving its conservative behaviour. The evaluation initially aborted on this expected exception. Failed evaluation logs are retained; the corrected evaluator counts every invalid frame as an abstention, records the reason and keeps the frame. Under the previous explicit UNKNOWN→other comparison convention its mIoU is .04389, obstacle recall 0, false negatives 3026. This is **not** evidence that it produced valid semantic segmentation on these scenes. Full confusion matrices, per-class precision/recall, image-wise confidence and failure comparisons are saved in `phase9d-results/generalisation/`.

One fixed-recipe retrain ran to early stopping at epoch 16; its best validation checkpoint is saved separately under `generalisation/training/best.pt`. Training plus held-out evaluation took 6.22 s, with sampled peak RSS about 586 MiB, on two CPU threads. Checkpoint size is about 23.5 kB. Original/retrained shifted-test inference means were 1.87/1.63 ms (p95 2.78/2.68 ms). Mean confidence on wrong pixels, averaged over frames with errors, was .726/.860: the retrain can be confidently wrong. Classical latency and its invalid-frame reasons are retained in both evaluation JSON files. Confidence is not a safety certificate.

The first training process saved a candidate but then failed in its classical comparison before serializing metrics. After fixing exception handling, the identical fixed recipe was rerun for a complete reproducible report. This was an instrumentation correction, not test-driven model tuning. The initial failed logs are retained.

## Reproduce sequentially

```bash
cd ~/AutonomousNavigationCar
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
GZ_IP=127.0.0.1 GZ_PARTITION=phase9d-data OPENBLAS_NUM_THREADS=1 python3 scripts/phase9/collect.py /tmp/new-diverse --scenes 30 --appearance diverse --scene-offset 1000
python3 scripts/phase9/dataset.py /tmp/new-diverse
OPENBLAS_NUM_THREADS=1 .venv/phase9/bin/python scripts/phase9/evaluate.py docs/academic/phase9-results/best.pt /tmp/new-diverse /tmp/new-baseline-ood
OPENBLAS_NUM_THREADS=1 .venv/phase9/bin/python scripts/phase9/train.py /tmp/new-diverse /tmp/new-diverse-model --epochs 25
OPENBLAS_NUM_THREADS=1 .venv/phase9/bin/python scripts/phase9/evaluate.py /tmp/new-diverse-model/best.pt /tmp/new-diverse /tmp/new-retrained-ood
```

All outputs must be new directories. Train only after Gazebo collection exits. The original isolated CPU PyTorch environment is reused; no additional model weights or system software were installed. See [the original training report](deep-learning.md) for environment setup. Existing navigation regression tests remain the appropriate check for optional vision changes; no full S01 mission was run.
