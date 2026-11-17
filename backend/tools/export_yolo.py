"""Run on the PC, not the Pi. Makes the small YOLO model the Pi runs (plan §10.1).

    pip install ultralytics onnx onnxruntime onnxsim
    python export_yolo.py --model yolo26n.pt                       # FP32, 256x320
    python export_yolo.py --model yolo26n.pt --int8 calib_frames/  # + INT8 from your own webcam frames

Copies to backend/models/: <name>_256x320.onnx (+ _int8.onnx) and <name>_256x320.json (class names).
Use the same .pt as object_detection_new_babu.py (yolov8n.pt / yolo26n.pt) so classes match.
"""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np

H, W = 256, 320  # 4:3 webcam -> 320x240 content + 8 px letterbox top/bottom


def letterbox(img_bgr):
    import cv2
    h, w = img_bgr.shape[:2]
    s = min(W / w, H / h)
    nw, nh = round(w * s), round(h * s)
    out = np.full((H, W, 3), 114, np.uint8)
    y0, x0 = (H - nh) // 2, (W - nw) // 2
    out[y0:y0 + nh, x0:x0 + nw] = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return out[..., ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0  # RGB CHW, same as preprocess.js


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="yolo26n.pt")
    ap.add_argument("--int8", metavar="FRAMES_DIR", help="~200 JPG/PNG frames from your webcam on your track")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "models"))
    a = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(a.model)
    onnx_path = Path(model.export(format="onnx", imgsz=[H, W], simplify=True, opset=17, dynamic=False, batch=1))
    stem = f"{Path(a.model).stem}_{H}x{W}"
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dst = out / f"{stem}.onnx"
    shutil.copy(onnx_path, dst)
    names = model.names
    (out / f"{stem}.json").write_text(json.dumps([names[i] for i in range(len(names))]))
    print(f"FP32 -> {dst} ({dst.stat().st_size / 1e6:.1f} MB)")

    if a.int8:
        import cv2
        from onnxruntime.quantization import CalibrationDataReader, QuantFormat, QuantType, quantize_static
        from onnxruntime.quantization.shape_inference import quant_pre_process

        files = sorted(p for p in Path(a.int8).iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        if len(files) < 50:
            raise SystemExit(f"need ~200 calibration frames, found {len(files)}")
        input_name = "images"

        class Reader(CalibrationDataReader):
            def __init__(self):
                self.it = iter(files)

            def get_next(self):
                p = next(self.it, None)
                return None if p is None else {input_name: letterbox(cv2.imread(str(p)))}

        pre = out / f"{stem}_pre.onnx"
        quant_pre_process(str(dst), str(pre))
        q = out / f"{stem}_int8.onnx"
        quantize_static(str(pre), str(q), Reader(), quant_format=QuantFormat.QDQ,
                        activation_type=QuantType.QUInt8, weight_type=QuantType.QInt8, per_channel=True)
        pre.unlink()
        print(f"INT8 -> {q} ({q.stat().st_size / 1e6:.1f} MB). Keep it only if Phase 0 shows it is faster "
              f"and boxes stay within 4 px of FP32.")


if __name__ == "__main__":
    main()
