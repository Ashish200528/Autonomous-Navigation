"""Phase 2: checkerboard calibration of the webcam (plan §13). Writes camera.intrinsics to
config/local.json, which the backend then uses instead of the HFOV guess.

    pip install opencv-python numpy
    python calibrate_camera.py --live            # webcam 0 at 640x480: SPACE grabs a view, Q finishes
    python calibrate_camera.py --dir shots/      # or calibrate from saved photos

Print a 9x6 inner-corner checkerboard (default 25 mm squares), tape it flat, take 15-25 views
at different angles and distances covering the whole image. Target: RMS reprojection < 0.5 px.
Stop the backend first if it has the camera open.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.025, help="square size in metres")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--dir")
    ap.add_argument("--device", default=0)
    a = ap.parse_args()

    pattern = (a.cols, a.rows)
    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    obj, img, size = [], [], None

    def add(gray):
        ok, c = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            obj.append(objp)
            img.append(cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), crit))
        return ok, c

    if a.dir:
        for p in sorted(Path(a.dir).iterdir()):
            g = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
            if g is not None:
                size = g.shape[::-1]
                print(p.name, "ok" if add(g)[0] else "no board")
    else:
        cap = cv2.VideoCapture(int(a.device) if str(a.device).isdigit() else a.device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            size = gray.shape[::-1]
            found, c = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_FAST_CHECK)
            view = frame.copy()
            if found:
                cv2.drawChessboardCorners(view, pattern, c, found)
            cv2.putText(view, f"views: {len(obj)}  SPACE=grab  Q=done", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("calibrate", view)
            k = cv2.waitKey(1) & 0xFF
            if k == ord(" ") and found:
                add(gray)
            elif k == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()

    if len(obj) < 10:
        raise SystemExit(f"need >= 10 good views, have {len(obj)}")
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None)
    print(f"RMS reprojection error: {rms:.3f} px ({'OK' if rms < 0.5 else 'too high: take more/better views'})")
    intr = {"fx": K[0, 0], "fy": K[1, 1], "cx": K[0, 2], "cy": K[1, 2], "dist": dist.ravel().tolist(),
            "rms_px": rms, "width": size[0], "height": size[1]}
    if size != (640, 480):
        print("WARNING: calibrated at", size, "but the backend runs at 640x480; recalibrate at 640x480.")

    local = Path(__file__).resolve().parents[1] / "config" / "local.json"
    data = json.loads(local.read_text()) if local.exists() else {}
    data.setdefault("camera", {})["intrinsics"] = {k: (float(v) if not isinstance(v, list) else v) for k, v in intr.items()}
    local.write_text(json.dumps(data, indent=2))
    print(f"wrote camera.intrinsics to {local}")


if __name__ == "__main__":
    main()
