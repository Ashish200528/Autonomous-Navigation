"""Sensor-agnostic occupancy and input contracts shared across profiles."""
from dataclasses import dataclass
import math
from typing import Iterable, Optional, Tuple

FREE, OCCUPIED, UNKNOWN = 0, 100, -1


@dataclass(frozen=True)
class GridGeometry:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    frame_id: str

    def validate(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid_dimensions_must_be_positive")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("grid_resolution_must_be_positive_finite")
        if not all(math.isfinite(v) for v in (self.origin_x_m, self.origin_y_m)):
            raise ValueError("grid_origin_must_be_finite")
        if not self.frame_id:
            raise ValueError("grid_frame_must_not_be_empty")


def validate_grid(data: Iterable[int], geometry: GridGeometry):
    geometry.validate()
    values = [int(v) for v in data]
    if len(values) != geometry.width * geometry.height:
        raise ValueError("occupancy_grid_length_mismatch")
    if any(v != UNKNOWN and (v < 0 or v > OCCUPIED) for v in values):
        raise ValueError("occupancy_value_outside_ros_range")
    return values


def classify_range(value: float, range_min_m: float, range_max_m: float) -> Tuple[str, Optional[float]]:
    """Only finite in-range values are valid; others remain UNKNOWN."""
    if (not math.isfinite(range_min_m) or not math.isfinite(range_max_m)
            or range_min_m < 0 or range_max_m <= range_min_m):
        raise ValueError("invalid_scan_range_limits")
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return "UNKNOWN", None
    if not math.isfinite(distance) or distance < range_min_m or distance > range_max_m:
        return "UNKNOWN", None
    return "VALID", distance


def fuse_occupancy(lidar_values: Iterable[int], rgb_values: Iterable[int],
                   *, allow_projected_rgb_free: bool = False):
    """RGB cannot erase a LiDAR obstacle or assign range to visual OCCUPIED."""
    lidar, rgb = [int(v) for v in lidar_values], [int(v) for v in rgb_values]
    if len(lidar) != len(rgb):
        raise ValueError("fusion_grid_length_mismatch")
    result = []
    for laser, visual in zip(lidar, rgb):
        if laser == OCCUPIED or laser == FREE:
            result.append(laser)
        elif laser == UNKNOWN and allow_projected_rgb_free and visual == FREE:
            result.append(FREE)
        else:
            result.append(UNKNOWN)
    return result


def input_age_valid(stamp_s: float, now_s: float, max_age_s: float):
    if (not all(math.isfinite(float(v)) for v in (stamp_s, now_s, max_age_s))
            or max_age_s < 0):
        return False, "timestamp_invalid"
    age = now_s - stamp_s
    if age < -0.05:
        return False, "timestamp_in_future"
    if age > max_age_s:
        return False, "input_stale"
    return True, "fresh"
