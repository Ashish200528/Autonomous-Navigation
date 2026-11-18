"""Run on the PC (or the Pi). Gets the dense depth model the Pi runs (plan §7.3, §10.1).

    pip install onnx onnxruntime
    python export_depth.py            # -> backend/models/midas_small_256_int8.onnx (~17 MB)

MiDaS v2.1 small, official ONNX release (MIT licence), 256x256 input, relative inverse depth out.
Weights are quantized to INT8 (dynamic quantization: no calibration images needed for this model).
"""
import argparse
import urllib.request
from pathlib import Path

URL = "https://github.com/isl-org/MiDaS/releases/download/v2_1/model-small.onnx"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "models"))
    ap.add_argument("--fp32", action="store_true", help="keep the FP32 model too")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    fp32 = out / "midas_small_256.onnx"
    if not fp32.exists():
        print(f"downloading {URL}")
        urllib.request.urlretrieve(URL, fp32)

    from onnxruntime.quantization import QuantType, quantize_dynamic
    q = out / "midas_small_256_int8.onnx"
    quantize_dynamic(str(fp32), str(q), weight_type=QuantType.QUInt8)
    if not a.fp32:
        fp32.unlink()
    print(f"-> {q} ({q.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
