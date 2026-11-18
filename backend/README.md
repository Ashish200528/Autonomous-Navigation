# Fire Bird autodrive — Raspberry Pi backend

Node.js on the Pi 4B: camera + YOLO object distance, brain, safety, dashboard. It drives
the Fire Bird V through the ATmega2560 bridge firmware in [`../atmega/`](../atmega/).
Design and phases: [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md).

## Run

```sh
# on the PC, once: make the small models (plan §10.1)
python tools/export_yolo.py --model yolo26n.pt        # -> models/yolo26n_256x320.onnx + .json (9.8 MB)
python tools/export_depth.py                          # -> models/midas_small_256_int8.onnx (17 MB)
# copy backend/ to the Pi, then on the Pi:
sh scripts/install-pi.sh
sudo systemctl start anc-backend                       # or: npm start
npm test                                               # unit tests, no hardware needed
```

Open `http://<pi-address>:8080/`. The robot starts in **STOP**. Paste the token from
`config/secret` into the dashboard to use Manual / Auto / GO. Without a model in
`models/`, the backend runs without YOLO (manual driving works; auto stays stopped).

Every tuning value is in `config/default.json`; dashboard changes go to `config/local.json`.

Without a depth model, cue C and the dense 3D cloud are off; everything else still works.

## Tools

| Command | Phase | What |
|---|---|---|
| `node tools/benchmark.js` | 0 | every model × 1–4 threads → ms, Hz, RAM, °C (stop the backend first) |
| `python tools/calibrate_camera.py --live` | 2 | checkerboard → `camera.intrinsics` in `config/local.json` |
| `node tools/compare_python.mjs <frames_dir> yolo26n.pt` | 3 | Pi YOLO pipeline vs Ultralytics on the same frames |
| `node src/record/replay.js runs/<dir>` | any | re-run the current brain on a recorded drive (record from the dashboard) |

## Status

All components in the plan are written and tested on a PC (40 unit tests, `npm test`):
firmware, serial link, wheel control, odometry, camera, YOLO, your distance method + floor
contact + dense depth + Sharp self-calibration, ArUco, 3D voxel map + virtual scan, brain,
safety, dashboard with 3D view, recording and replay.

Checked on the PC with real models: YOLO boxes match Ultralytics within 1 px; YOLO, ArUco
and the depth process all run together through the full backend.

Not yet done: the firmware has not been compiled (no avr-gcc on the dev PC), and nothing has
run on the robot or the Pi. That is Phases 0–9 of the plan.
