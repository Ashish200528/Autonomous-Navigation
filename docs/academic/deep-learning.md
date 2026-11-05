# Deep learning — measured initial experiment

TRAINED / TESTED OFFLINE. A compact convolutional encoder/decoder has 4899 parameters: 3→8 convolution, 8→16 stride-2 convolution, 16→16 convolution, bilinear upsampling, 16→8 convolution, 8→3 pointwise classifier. ReLU activations; no pretrained weights. Input is 160×120 RGB. Seed 91, Adam .003, batch 4, square-root inverse-frequency weighted cross entropy with ignored label 255. Best validation checkpoint is saved; patience 5, bounded at 25 epochs. Forward/backpropagation, finite loss and changed weights are asserted. Checkpoint schema/classes are checked before loading.

108 captured pairs; deduplicated splits: {'train': 71, 'validation': 8, 'test': 8}. Validation selects the best checkpoint; test images are used only for final evaluation. Training and held-out evaluation took 9.48 s on CPU (two PyTorch threads); peak sampled process RSS 584.7 MiB. CPU inference mean/p95 2.620/2.686 ms. Checkpoint 23509 bytes, SHA256 fa59636f29a1c7f048361839f64811416a5bdfba4fdfce99f3943f361e24a4fe. Full confusion matrices, per-class precision/recall/IoU, losses, model examples and loss graph are in phase9-results/.

Held-out CNN mean IoU 0.998182; per-class IoU [0.9994757732344791, 0.9957685664939551, 0.9993005595523581]. Classical mean IoU 0.596761, per-class IoU [0.7891910791438775, 0.6004666839512575, 0.40062447460069656]. Both methods use the same held-out originals and labels. Classical UNKNOWN is mapped to other for this comparison; UNKNOWN is an abstention and does not establish semantic other recognition. Classical processing is at 320×240 before nearest-neighbour reduction to evaluation size; CNN processing is at 160×120. Mean classical latency 15.196 ms therefore is not a comparison at equal compute resolution. Existing classical configuration receives the actual .24 m/.8 rad dataset geometry and the previously used near-floor band.

High CNN scores describe this narrow, colour-consistent synthetic domain. They do not establish robust vision, improved navigation, physical-robot accuracy or generalization to new textures/lighting. A richer scene/material-held-out dataset is required next. The trained CNN currently contributes independent shadow observations only.

The isolated .venv/phase9 environment uses system-site-packages for ROS/OpenCV and installs PyTorch 2.15.0+cpu locally. System Python lacks ensurepip; an isolated pip was bootstrapped using https://bootstrap.pypa.io/get-pip.py. CPU wheels came from https://download.pytorch.org/whl/cpu. No GPU driver, CUDA toolkit or system ROS package changed. GTX 1650 has 4 GiB VRAM; CPU training avoided the much larger CUDA dependency download. CUDA execution has NOT BEEN TESTED.

## Isolated environment setup

```bash
python3 -m venv --without-pip --system-site-packages .venv/phase9
curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/phase9-get-pip.py
.venv/phase9/bin/python /tmp/phase9-get-pip.py
.venv/phase9/bin/python -m pip install 'torch==2.15.0+cpu' --index-url https://download.pytorch.org/whl/cpu
```

Use the ROS setup scripts before training/comparison or live inference so the existing anc_perception module is available. Only the dedicated virtual environment receives Python packages. Local installed versions are saved in phase9-results/environment-local.txt. No model download occurs; best.pt is trained from random initialization on the committed dataset.
