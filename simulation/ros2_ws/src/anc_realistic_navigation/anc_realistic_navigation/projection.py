"""Pure, conservative image-to-local-grid projection helpers."""
import math
import numpy as np

FREE = 0
OCCUPIED = 100
UNKNOWN = -1

def quaternion_matrix(quaternion):
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid_camera_quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)

def validate_camera_info(info, width, height, expected_frame):
    if int(info.get("width", 0)) != int(width) or int(info.get("height", 0)) != int(height):
        raise ValueError("camera_info_dimensions_mismatch")
    if info.get("frame_id") != expected_frame:
        raise ValueError("camera_info_frame_mismatch")
    if info.get("distortion_model", "plumb_bob") not in ("plumb_bob", ""):
        raise ValueError("camera_info_unsupported_distortion_model")
    distortion = [float(v) for v in info.get("d", [])]
    if any(not math.isfinite(v) or abs(v) > 1e-7 for v in distortion):
        raise ValueError("camera_info_nonzero_distortion")
    k = [float(v) for v in info.get("k", [])]
    if len(k) != 9:
        raise ValueError("camera_info_invalid_K")
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError("camera_info_invalid_intrinsics")
    if not (-width <= cx <= 2*width and -height <= cy <= 2*height):
        raise ValueError("camera_info_invalid_principal_point")
    return fx, fy, cx, cy

def project_free_pixels(mask, confidence, camera_info, translation, quaternion,
                        *, expected_frame="camera_optical_frame",
                        grid_width=75, grid_height=75, resolution=0.02,
                        origin_x=0.0, origin_y=-0.75, floor_z=-0.08,
                        min_confidence=200):
    """Project only high-confidence FREE pixels; all other cells stay UNKNOWN.

    OCCUPIED image pixels are not projected because RGB classification provides
    no defensible metric obstacle range. Their count is returned as a stop cue.
    """
    mask = np.asarray(mask, dtype=np.uint8)
    confidence = np.asarray(confidence, dtype=np.uint8)
    if mask.ndim != 2 or confidence.shape != mask.shape:
        raise ValueError("mask_confidence_shape_mismatch")
    height, width = mask.shape
    fx, fy, cx, cy = validate_camera_info(camera_info, width, height, expected_frame)
    rotation = quaternion_matrix(quaternion)
    tx, ty, tz = (float(v) for v in translation)
    if not all(math.isfinite(v) for v in (tx, ty, tz)):
        raise ValueError("invalid_camera_translation")
    if resolution <= 0 or grid_width <= 0 or grid_height <= 0:
        raise ValueError("invalid_grid_geometry")
    data = np.full(grid_width*grid_height, UNKNOWN, dtype=np.int8)
    ys, xs = np.where((mask == FREE) & (confidence >= int(min_confidence)))
    projected = 0
    if len(xs):
        rays_optical = np.column_stack(((xs-cx)/fx, (ys-cy)/fy,
                                        np.ones(len(xs), dtype=np.float64)))
        rays_base = rays_optical @ rotation.T
        dz = rays_base[:, 2]
        valid = np.isfinite(dz) & (dz < -1e-6)
        ranges = np.full(len(xs), np.nan, dtype=np.float64)
        ranges[valid] = (floor_z-tz)/dz[valid]
        valid &= np.isfinite(ranges) & (ranges > 0.0)
        bx = tx + ranges*rays_base[:, 0]
        by = ty + ranges*rays_base[:, 1]
        finite_xy = valid & np.isfinite(bx) & np.isfinite(by)
        ix = np.zeros(len(xs), dtype=np.int64)
        iy = np.zeros(len(xs), dtype=np.int64)
        ix[finite_xy] = np.floor((bx[finite_xy]-origin_x)/resolution).astype(np.int64)
        iy[finite_xy] = np.floor((by[finite_xy]-origin_y)/resolution).astype(np.int64)
        inside = (finite_xy & (ix >= 0) & (ix < grid_width) &
                  (iy >= 0) & (iy < grid_height))
        data[iy[inside]*grid_width+ix[inside]] = FREE
        projected = int(np.count_nonzero(inside))
    counts = {
        "free_image_pixels": int(np.count_nonzero(mask == FREE)),
        "high_confidence_free_pixels": int(len(xs)),
        "projected_free_pixels": projected,
        "free_grid_cells": int(np.count_nonzero(data == FREE)),
        "occupied_image_pixels_unlocalized": int(np.count_nonzero(mask == OCCUPIED)),
        "unknown_image_pixels": int(np.count_nonzero(mask == 255)),
        "grid_unknown_cells": int(np.count_nonzero(data == UNKNOWN)),
        "grid_occupied_cells": 0,
    }
    return data.tolist(), counts
