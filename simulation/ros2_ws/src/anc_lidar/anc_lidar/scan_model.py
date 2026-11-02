"""Pure 2D LaserScan conditioning, conservative ray tracing and grid utilities."""
from dataclasses import dataclass
import math

from anc_navigation_common.contracts import FREE, OCCUPIED, UNKNOWN, classify_range


@dataclass(frozen=True)
class Grid:
    width: int = 180
    height: int = 180
    resolution_m: float = 0.10
    origin_x_m: float = -9.0
    origin_y_m: float = -9.0
    frame_id: str = "base_link"

    def validate(self):
        if self.width <= 0 or self.height <= 0 or self.resolution_m <= 0:
            raise ValueError("invalid_local_grid_geometry")
        if not all(math.isfinite(v) for v in
                   (self.resolution_m, self.origin_x_m, self.origin_y_m)):
            raise ValueError("nonfinite_local_grid_geometry")


def latest_matching_map_health_pair(map_records, health_records,
                                      timestamp_tolerance_s=0.001):
    """Return the newest grid/health records for the same scan timestamp.

    The safety gate subscribes to OccupancyGrid and health on separate ROS
    topics. Their callbacks can be interleaved, so it must not combine a new
    grid with health from the previous scan (or vice versa).
    Records have ``stamp_s``, ``wall_s`` and ``payload`` keys.
    """
    if not math.isfinite(timestamp_tolerance_s) or timestamp_tolerance_s < 0:
        raise ValueError("map_health_pair_tolerance_invalid")
    matches = []
    for map_record in map_records:
        try:
            map_stamp = float(map_record["stamp_s"])
            map_wall = float(map_record["wall_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(map_stamp) or not math.isfinite(map_wall):
            continue
        for health_record in health_records:
            try:
                health_stamp = float(health_record["stamp_s"])
                health_wall = float(health_record["wall_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if (not math.isfinite(health_stamp) or not math.isfinite(health_wall)
                    or abs(map_stamp-health_stamp) > timestamp_tolerance_s):
                continue
            matches.append((map_stamp, max(map_wall, health_wall),
                            map_record, health_record))
    if not matches:
        return None
    _, _, map_record, health_record = max(matches, key=lambda item: (item[0], item[1]))
    return map_record, health_record


def validate_scan(scan, *, frame_id, samples, angle_min_rad, angle_max_rad,
                  range_min_m, range_max_m, tolerance_rad=0.02):
    """Check scan contract; return a reason string or None when valid."""
    if scan.header.frame_id.lstrip("/") != frame_id.lstrip("/"):
        return "scan_frame_mismatch"
    if len(scan.ranges) != samples:
        return "scan_sample_count_mismatch"
    fields = (scan.angle_min, scan.angle_max, scan.angle_increment,
              scan.range_min, scan.range_max)
    if not all(math.isfinite(float(v)) for v in fields):
        return "scan_geometry_nonfinite"
    if scan.angle_max <= scan.angle_min or scan.angle_increment <= 0:
        return "scan_geometry_invalid"
    if (abs(scan.angle_min-angle_min_rad) > tolerance_rad
            or abs(scan.angle_max-angle_max_rad) > tolerance_rad):
        return "scan_field_of_view_mismatch"
    if abs(scan.range_min-range_min_m) > 0.02 or abs(scan.range_max-range_max_m) > 0.05:
        return "scan_range_limits_mismatch"
    if abs((scan.angle_max-scan.angle_min) -
           scan.angle_increment*(samples-1)) > tolerance_rad:
        return "scan_angle_increment_mismatch"
    return None


def condition_ranges(ranges, *, range_min_m, range_max_m, rng,
                     missed_probability, invalid_probability):
    """Apply deterministic dropout/invalid faults while retaining max-range no-hit rays."""
    if (not 0.0 <= missed_probability < 1.0
            or not 0.0 <= invalid_probability < 1.0
            or missed_probability + invalid_probability >= 1.0):
        raise ValueError("scan_error_probabilities_must_be_in_[0,1)")
    output = []
    valid_count = missed_count = invalid_count = no_return_count = 0
    for value in ranges:
        status, distance = classify_range(value, range_min_m, range_max_m)
        if status != "VALID":
            # Gazebo Harmonic gpu_lidar reports unobstructed rays as +Inf.
            # This sensor-specific no-return encoding means the ray observed
            # clear space through range_max; normalize it to a finite endpoint.
            try:
                numeric_value = float(value)
            except (TypeError, ValueError, OverflowError):
                numeric_value = math.nan
            if math.isinf(numeric_value) and numeric_value > 0.0:
                status, distance = classify_range(range_max_m, range_min_m,
                                                  range_max_m)
        if status != "VALID":
            output.append(float("nan"))
            invalid_count += 1
            continue
        if distance >= range_max_m - 1e-6:
            output.append(float(range_max_m))
            no_return_count += 1
            continue
        draw = rng.random()
        if draw < missed_probability:
            output.append(float("nan"))
            missed_count += 1
        elif draw < missed_probability + invalid_probability:
            output.append(float(range_max_m) + 1.0)
            invalid_count += 1
        else:
            output.append(distance)
            valid_count += 1
    return output, {
        "valid_hit_beams": valid_count,
        "no_return_beams": no_return_count,
        "missed_beams": missed_count,
        "invalid_beams": invalid_count,
        "usable_beams": valid_count + no_return_count,
        "total_beams": len(output),
    }


def _bresenham(x0, y0, x1, y1):
    """Integer grid cells from origin through endpoint, inclusive."""
    dx, dy = abs(x1-x0), abs(y1-y0)
    sx, sy = (1 if x0 < x1 else -1), (-1 if y0 > y1 else 1)
    err = dx - dy
    x, y, cells = x0, y0, []
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2*err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


def raytrace_scan(scan, grid: Grid, *, scanner_x_m=0.0, scanner_y_m=0.0,
                  scanner_yaw_rad=0.0):
    """Build a single-scan map. Invalid beams reveal nothing; max-range rays clear only."""
    grid.validate()
    if not all(math.isfinite(v) for v in (scanner_x_m, scanner_y_m, scanner_yaw_rad)):
        raise ValueError("scanner_transform_nonfinite")
    data = [UNKNOWN] * (grid.width * grid.height)
    occupied = set()
    origin_ix = math.floor((scanner_x_m-grid.origin_x_m)/grid.resolution_m)
    origin_iy = math.floor((scanner_y_m-grid.origin_y_m)/grid.resolution_m)
    if not (0 <= origin_ix < grid.width and 0 <= origin_iy < grid.height):
        return data
    for index, raw in enumerate(scan.ranges):
        status, distance = classify_range(raw, scan.range_min, scan.range_max)
        if status != "VALID":
            continue
        angle = scanner_yaw_rad + scan.angle_min + index*scan.angle_increment
        ex = scanner_x_m + distance*math.cos(angle)
        ey = scanner_y_m + distance*math.sin(angle)
        end_ix = math.floor((ex-grid.origin_x_m)/grid.resolution_m)
        end_iy = math.floor((ey-grid.origin_y_m)/grid.resolution_m)
        ray = _bresenham(origin_ix, origin_iy, end_ix, end_iy)
        is_no_return = distance >= scan.range_max - 1e-6
        for ix, iy in ray[:-1]:
            if 0 <= ix < grid.width and 0 <= iy < grid.height:
                cell = iy*grid.width+ix
                if cell not in occupied:
                    data[cell] = FREE
        if is_no_return:
            if 0 <= end_ix < grid.width and 0 <= end_iy < grid.height:
                cell = end_iy*grid.width+end_ix
                if cell not in occupied and data[cell] == UNKNOWN:
                    data[cell] = FREE
        elif 0 <= end_ix < grid.width and 0 <= end_iy < grid.height:
            occupied.add(end_iy*grid.width+end_ix)
    for cell in occupied:
        data[cell] = OCCUPIED
    return data


def inflate_obstacles(data, grid: Grid, radius_m):
    """Dilate confirmed obstacles into observed FREE cells; UNKNOWN stays UNKNOWN."""
    grid.validate()
    if not math.isfinite(radius_m) or radius_m < 0:
        raise ValueError("inflation_radius_invalid")
    values = list(data)
    if len(values) != grid.width*grid.height:
        raise ValueError("occupancy_grid_length_mismatch")
    cells = int(math.ceil(radius_m/grid.resolution_m))
    radius_sq = (radius_m/grid.resolution_m)**2
    occupied = [i for i, v in enumerate(values) if v == OCCUPIED]
    for index in occupied:
        cx, cy = index % grid.width, index // grid.width
        for dy in range(-cells, cells+1):
            for dx in range(-cells, cells+1):
                if dx*dx+dy*dy > radius_sq:
                    continue
                x, y = cx+dx, cy+dy
                if 0 <= x < grid.width and 0 <= y < grid.height:
                    target = y*grid.width+x
                    if values[target] == FREE:
                        values[target] = OCCUPIED
    return values


def forward_corridor_m(data, grid: Grid, *, robot_front_m=0.215,
                       robot_half_width_m=0.1525, lateral_margin_m=0.02):
    """Length of contiguous fully-known FREE cells across the entire footprint width."""
    if len(data) != grid.width*grid.height:
        raise ValueError("occupancy_grid_length_mismatch")
    y_min = -robot_half_width_m-lateral_margin_m
    y_max = robot_half_width_m+lateral_margin_m
    iy0 = math.floor((y_min-grid.origin_y_m)/grid.resolution_m)
    iy1 = math.floor((y_max-grid.origin_y_m)/grid.resolution_m)
    ix0 = math.floor((robot_front_m-grid.origin_x_m)/grid.resolution_m)
    if ix0 < 0 or iy0 < 0 or iy1 >= grid.height:
        return 0.0
    length = 0
    for ix in range(ix0, grid.width):
        if any(data[iy*grid.width+ix] != FREE for iy in range(iy0, iy1+1)):
            break
        length += 1
    return length*grid.resolution_m


def fuse_lidar_with_rgb_free(lidar_data, lidar_grid: Grid, rgb_data, rgb_geometry):
    """Only project metric RGB FREE cells into LiDAR UNKNOWN; never use RGB OCCUPIED."""
    lidar_grid.validate()
    width, height, resolution, origin_x, origin_y, frame = rgb_geometry
    if frame.lstrip("/") != lidar_grid.frame_id.lstrip("/"):
        raise ValueError("fusion_frame_mismatch")
    if (width <= 0 or height <= 0 or not math.isfinite(resolution)
            or resolution <= 0 or not math.isfinite(origin_x)
            or not math.isfinite(origin_y)):
        raise ValueError("fusion_rgb_geometry_invalid")
    if len(lidar_data) != lidar_grid.width*lidar_grid.height:
        raise ValueError("fusion_lidar_grid_length_mismatch")
    if len(rgb_data) != width*height:
        raise ValueError("fusion_rgb_grid_length_mismatch")
    result = list(lidar_data)
    for iy in range(lidar_grid.height):
        y = lidar_grid.origin_y_m+(iy+0.5)*lidar_grid.resolution_m
        ry = math.floor((y-origin_y)/resolution)
        if not 0 <= ry < height:
            continue
        for ix in range(lidar_grid.width):
            index = iy*lidar_grid.width+ix
            if result[index] != UNKNOWN:
                continue
            x = lidar_grid.origin_x_m+(ix+0.5)*lidar_grid.resolution_m
            rx = math.floor((x-origin_x)/resolution)
            if 0 <= rx < width and rgb_data[ry*width+rx] == FREE:
                result[index] = FREE
    return result
