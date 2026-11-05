"""Pure 2D LaserScan conditioning, conservative ray tracing and grid utilities."""
from dataclasses import dataclass
import math

from anc_navigation_common.contracts import FREE, OCCUPIED, UNKNOWN, classify_range


def slam_hit_ranges(ranges,range_min_m,range_max_m):
    """Encode endpoints for Karto: only actual hits can be scan-match features.

    The local safety mapper separately interprets finite range_max as a clear
    ray. Karto's unfiltered matching points include that finite endpoint, so
    its input must mask no-return and invalid endpoints without changing beam
    indices, angles, timestamps, or the safety map's representation.
    """
    return [float(v) if math.isfinite(v) and range_min_m<v<range_max_m else float('nan') for v in ranges]


def nav2_clearing_ranges(ranges,range_min_m,range_max_m):
    """Preserve hits; encode normalized no-return as +Inf for inf_is_valid.

    LaserProjection drops finite values equal to range_max. Invalid, missed,
    or out-of-range beams remain NaN and never become free-space rays.
    """
    return [float(v) if math.isfinite(v) and range_min_m<v<range_max_m else
            float('inf') if math.isfinite(v) and v==range_max_m else float('nan') for v in ranges]


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



def swept_rotation_clearance(data, grid: Grid, footprint_vertices_xy_m, *,
                             center_drift_allowance_m,
                             sensor_mount_uncertainty_m,
                             safety_margin_m,
                             center_x_m=0.0, center_y_m=0.0):
    """Require measured FREE cells throughout a conservative in-place turn disk.

    The disk encloses every orientation of the configured polygon. It is
    expanded for possible wheel-pivot drift, sensor mounting uncertainty and
    an explicit clearance margin. UNKNOWN and OCCUPIED cells both deny yaw.
    A cell is checked if any part of its square intersects the required disk.
    """
    grid.validate()
    if len(data) != grid.width*grid.height:
        raise ValueError("rotation_grid_length_mismatch")
    scalars = (center_drift_allowance_m, sensor_mount_uncertainty_m,
               safety_margin_m, center_x_m, center_y_m)
    if not all(math.isfinite(float(value)) for value in scalars):
        raise ValueError("rotation_clearance_input_nonfinite")
    if any(float(value) < 0 for value in scalars[:3]):
        raise ValueError("rotation_clearance_margin_negative")
    vertices = []
    try:
        for x, y in footprint_vertices_xy_m:
            x, y = float(x), float(y)
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError
            vertices.append((x, y))
    except (TypeError, ValueError, OverflowError):
        raise ValueError("rotation_footprint_invalid")
    if len(vertices) < 3:
        raise ValueError("rotation_footprint_invalid")

    footprint_radius = max(math.hypot(x, y) for x, y in vertices)
    required_radius = (footprint_radius + float(center_drift_allowance_m)
                       + float(sensor_mount_uncertainty_m)
                       + float(safety_margin_m))
    grid_min_x, grid_min_y = grid.origin_x_m, grid.origin_y_m
    grid_max_x = grid_min_x + grid.width*grid.resolution_m
    grid_max_y = grid_min_y + grid.height*grid.resolution_m
    inside_grid = (
        center_x_m-required_radius >= grid_min_x
        and center_x_m+required_radius <= grid_max_x
        and center_y_m-required_radius >= grid_min_y
        and center_y_m+required_radius <= grid_max_y
    )
    checked = occupied = unknown = 0
    resolution = grid.resolution_m
    ix0 = max(0, math.floor((center_x_m-required_radius-grid_min_x)/resolution)-1)
    ix1 = min(grid.width-1,
              math.ceil((center_x_m+required_radius-grid_min_x)/resolution)+1)
    iy0 = max(0, math.floor((center_y_m-required_radius-grid_min_y)/resolution)-1)
    iy1 = min(grid.height-1,
              math.ceil((center_y_m+required_radius-grid_min_y)/resolution)+1)
    radius_sq = required_radius*required_radius
    for iy in range(iy0, iy1+1):
        cell_y = grid_min_y+(iy+0.5)*resolution
        dy = max(abs(cell_y-center_y_m)-resolution/2.0, 0.0)
        for ix in range(ix0, ix1+1):
            cell_x = grid_min_x+(ix+0.5)*resolution
            dx = max(abs(cell_x-center_x_m)-resolution/2.0, 0.0)
            if dx*dx+dy*dy > radius_sq:
                continue
            checked += 1
            value = data[iy*grid.width+ix]
            if value == OCCUPIED:
                occupied += 1
            elif value != FREE:
                unknown += 1
    authorized = inside_grid and checked > 0 and occupied == 0 and unknown == 0
    if not inside_grid:
        reason = "swept_disk_outside_grid"
    elif occupied:
        reason = "swept_disk_occupied"
    elif unknown or checked == 0:
        reason = "swept_disk_unknown"
    else:
        reason = "swept_disk_clear"
    return {
        "authorized": authorized,
        "reason": reason,
        "footprint_circumradius_m": footprint_radius,
        "required_radius_m": required_radius,
        "center_drift_allowance_m": float(center_drift_allowance_m),
        "sensor_mount_uncertainty_m": float(sensor_mount_uncertainty_m),
        "safety_margin_m": float(safety_margin_m),
        "checked_cells": checked,
        "occupied_cells": occupied,
        "unknown_cells": unknown,
        "inside_grid": inside_grid,
    }


def evaluate_moving_slam_metrics(*, scan_messages, map_updates, truth_pose_samples,
                                 slam_pose_updates, map_to_odom_updates,
                                 map_free_cells, map_occupied_cells,
                                 truth_forward_displacement_m,
                                 slam_relative_displacement_error_m,
                                 slam_relative_yaw_error_rad,
                                 non_ground_contact_events,
                                 min_scan_messages=8, min_map_updates=2,
                                 min_pose_updates=2, min_tf_updates=2,
                                 min_motion_m=0.10, max_motion_m=0.50,
                                 max_position_error_m=0.10,
                                 max_yaw_error_rad=0.10):
    # Require fresh sensor/map/pose activity and finite estimator-to-truth errors.
    # This measurement contract does not grant motion authority.
    integer_measurements = {
        "scan_messages": scan_messages,
        "map_updates": map_updates,
        "truth_pose_samples": truth_pose_samples,
        "slam_pose_updates": slam_pose_updates,
        "map_to_odom_updates": map_to_odom_updates,
        "map_free_cells": map_free_cells,
        "map_occupied_cells": map_occupied_cells,
        "non_ground_contact_events": non_ground_contact_events,
    }
    failures = []
    normalized = {}
    for name, value in integer_measurements.items():
        try:
            converted = int(value)
            if isinstance(value, bool) or converted != value or converted < 0:
                raise ValueError
        except (TypeError, ValueError, OverflowError):
            failures.append(name + "_invalid")
            continue
        normalized[name] = converted

    numeric_measurements = {
        "truth_forward_displacement_m": truth_forward_displacement_m,
        "slam_relative_displacement_error_m": slam_relative_displacement_error_m,
        "slam_relative_yaw_error_rad": slam_relative_yaw_error_rad,
    }
    numeric = {}
    for name, value in numeric_measurements.items():
        try:
            converted = float(value)
            if not math.isfinite(converted):
                raise ValueError
        except (TypeError, ValueError, OverflowError):
            failures.append(name + "_missing_or_nonfinite")
            continue
        numeric[name] = converted

    checks = {}
    count_requirements = (
        ("scan_messages", min_scan_messages),
        ("map_updates", min_map_updates),
        ("truth_pose_samples", 3),
        ("slam_pose_updates", min_pose_updates),
        ("map_to_odom_updates", min_tf_updates),
        ("map_free_cells", 1),
        ("map_occupied_cells", 1),
    )
    for name, minimum in count_requirements:
        checks[name] = normalized.get(name, -1) >= minimum
        if name in normalized and not checks[name]:
            failures.append(name + "_below_minimum")

    displacement = numeric.get("truth_forward_displacement_m")
    checks["bounded_forward_motion"] = (
        displacement is not None and min_motion_m <= displacement <= max_motion_m)
    if displacement is not None and not checks["bounded_forward_motion"]:
        failures.append("truth_forward_displacement_out_of_bounds")

    position_error = numeric.get("slam_relative_displacement_error_m")
    checks["slam_position_error"] = (
        position_error is not None and 0.0 <= position_error <= max_position_error_m)
    if position_error is not None and not checks["slam_position_error"]:
        failures.append("slam_position_error_exceeds_limit")

    yaw_error = numeric.get("slam_relative_yaw_error_rad")
    checks["slam_yaw_error"] = (
        yaw_error is not None and 0.0 <= yaw_error <= max_yaw_error_rad)
    if yaw_error is not None and not checks["slam_yaw_error"]:
        failures.append("slam_yaw_error_exceeds_limit")

    checks["no_non_ground_contacts"] = normalized.get(
        "non_ground_contact_events", -1) == 0
    if "non_ground_contact_events" in normalized and not checks["no_non_ground_contacts"]:
        failures.append("non_ground_contact_observed")

    return {
        "status": "PASS" if not failures else "FAIL",
        "checks": checks,
        "failures": failures,
        "measurements": {**normalized, **numeric},
        "limits": {
            "minimum_scan_messages": int(min_scan_messages),
            "minimum_map_updates": int(min_map_updates),
            "minimum_truth_pose_samples": 3,
            "minimum_slam_pose_updates": int(min_pose_updates),
            "minimum_map_to_odom_updates": int(min_tf_updates),
            "truth_forward_displacement_m": [float(min_motion_m), float(max_motion_m)],
            "maximum_slam_position_error_m": float(max_position_error_m),
            "maximum_slam_yaw_error_rad": float(max_yaw_error_rad),
        },
    }

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
