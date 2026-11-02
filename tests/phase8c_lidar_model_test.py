#!/usr/bin/env python3
"""Pure scan model tests; no Gazebo geometry or truth is used."""
import math
import random
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"simulation/ros2_ws/src/anc_navigation_common"))
sys.path.insert(0, str(ROOT/"simulation/ros2_ws/src/anc_lidar"))
from anc_lidar.scan_model import (Grid, condition_ranges, forward_corridor_m,
                                  fuse_lidar_with_rgb_free, inflate_obstacles,
                                  raytrace_scan, validate_scan, latest_matching_map_health_pair,
                                  swept_rotation_clearance)
from anc_navigation_common.contracts import FREE, OCCUPIED, UNKNOWN


def scan(ranges):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="lidar_link"),
        ranges=ranges, angle_min=-math.pi, angle_max=math.pi,
        angle_increment=2*math.pi/(len(ranges)-1),
        range_min=0.12, range_max=8.0,
    )


class LidarModelTests(unittest.TestCase):
    def test_map_health_pairing_selects_latest_matching_scan(self):
        maps = [
            {"stamp_s": 10.0, "wall_s": 1.0, "payload": "map-10"},
            {"stamp_s": 10.2, "wall_s": 1.2, "payload": "map-10.2"},
        ]
        health = [
            {"stamp_s": 10.0, "wall_s": 1.01, "payload": "health-10"},
            {"stamp_s": 10.2, "wall_s": 1.21, "payload": "health-10.2"},
        ]
        pair = latest_matching_map_health_pair(maps, health)
        self.assertEqual(pair[0]["payload"], "map-10.2")
        self.assertEqual(pair[1]["payload"], "health-10.2")

    def test_map_health_pairing_does_not_mix_interleaved_scans(self):
        maps = [{"stamp_s": 11.2, "wall_s": 2.2, "payload": "new-map"}]
        health = [{"stamp_s": 11.0, "wall_s": 2.0, "payload": "old-health"}]
        self.assertIsNone(latest_matching_map_health_pair(maps, health))

    def test_scan_contract_checks_frame_count_fov_and_limits(self):
        msg = scan([1.0]*5)
        common = dict(frame_id="lidar_link", samples=5,
                      angle_min_rad=-math.pi, angle_max_rad=math.pi,
                      range_min_m=0.12, range_max_m=8.0)
        self.assertIsNone(validate_scan(msg, **common))
        self.assertEqual(validate_scan(msg, **(common | {"frame_id": "wrong"})),
                         "scan_frame_mismatch")
        self.assertEqual(validate_scan(msg, **(common | {"samples": 6})),
                         "scan_sample_count_mismatch")

    def test_invalid_beam_remains_unknown_and_not_free(self):
        msg = scan([float("nan")]*5)
        grid = Grid(width=80, height=80, resolution_m=0.05,
                    origin_x_m=-2, origin_y_m=-2)
        data = raytrace_scan(msg, grid)
        self.assertFalse(any(v == FREE for v in data))
        self.assertTrue(all(v == UNKNOWN for v in data))

    def test_valid_return_marks_ray_free_and_endpoint_occupied(self):
        msg = scan([1.0]*5)
        grid = Grid(width=80, height=80, resolution_m=0.05,
                    origin_x_m=-2, origin_y_m=-2)
        data = raytrace_scan(msg, grid)
        ix = math.floor((1.0-grid.origin_x_m)/grid.resolution_m)
        iy = math.floor((0.0-grid.origin_y_m)/grid.resolution_m)
        self.assertEqual(data[iy*grid.width+ix], OCCUPIED)
        origin = (math.floor((0-grid.origin_y_m)/grid.resolution_m)*grid.width
                  + math.floor((0-grid.origin_x_m)/grid.resolution_m))
        self.assertEqual(data[origin], FREE)

    def test_space_behind_first_return_stays_unknown(self):
        msg = scan([0.5]*5)
        grid = Grid(width=120, height=80, resolution_m=0.05,
                    origin_x_m=-2, origin_y_m=-2)
        data = raytrace_scan(msg, grid)
        behind = math.floor((1.0-grid.origin_x_m)/grid.resolution_m)
        iy = math.floor((0-grid.origin_y_m)/grid.resolution_m)
        self.assertEqual(data[iy*grid.width+behind], UNKNOWN)

    def test_max_range_is_clear_without_inventing_obstacle_endpoint(self):
        msg = scan([8.0]*5)
        grid = Grid(width=220, height=220, resolution_m=0.05,
                    origin_x_m=-5.5, origin_y_m=-5.5)
        data = raytrace_scan(msg, grid)
        endpoint = math.floor((8.0-grid.origin_x_m)/grid.resolution_m)
        iy = math.floor((0-grid.origin_y_m)/grid.resolution_m)
        self.assertNotEqual(data[iy*grid.width+endpoint], OCCUPIED)
        self.assertTrue(any(v == FREE for v in data))
        self.assertTrue(any(v == UNKNOWN for v in data))

    def test_fault_model_records_drop_invalid_and_no_return(self):
        values, counts = condition_ranges([1, 1, 1, 0.1, float("inf"), 8.0],
            range_min_m=0.12, range_max_m=8.0, rng=random.Random(3),
            missed_probability=0.2, invalid_probability=0.2)
        self.assertEqual(len(values), 6)
        self.assertGreaterEqual(counts["invalid_beams"], 2)
        self.assertEqual(counts["no_return_beams"], 2)
        self.assertEqual(counts["total_beams"], 6)
        self.assertTrue(math.isnan(values[3]))
        self.assertEqual(values[4], 8.0)
        self.assertEqual(values[-1], 8.0)

    def test_gazebo_positive_infinity_is_a_max_range_no_return(self):
        values, counts = condition_ranges([float("inf")], range_min_m=0.12,
            range_max_m=6.0, rng=random.Random(1), missed_probability=0.0,
            invalid_probability=0.0)
        self.assertEqual(values, [6.0])
        self.assertEqual(counts["no_return_beams"], 1)
        self.assertEqual(counts["usable_beams"], 1)
        self.assertEqual(counts["invalid_beams"], 0)

    def test_nan_and_negative_infinity_remain_unknown(self):
        values, counts = condition_ranges([float("nan"), float("-inf")],
            range_min_m=0.12, range_max_m=6.0, rng=random.Random(1),
            missed_probability=0.0, invalid_probability=0.0)
        self.assertTrue(all(math.isnan(value) for value in values))
        self.assertEqual(counts["usable_beams"], 0)
        self.assertEqual(counts["invalid_beams"], 2)

    def test_inflation_preserves_unknown_cells(self):
        grid = Grid(width=100, height=100, resolution_m=0.05,
                    origin_x_m=-1, origin_y_m=-2.5)
        data = [FREE]*(grid.width*grid.height)
        obstacle = 50*grid.width+70
        unknown_near = 50*grid.width+71
        data[obstacle] = OCCUPIED
        data[unknown_near] = UNKNOWN
        inflated = inflate_obstacles(data, grid, 0.10)
        self.assertEqual(inflated[obstacle], OCCUPIED)
        self.assertEqual(inflated[50*grid.width+68], OCCUPIED)
        self.assertEqual(inflated[unknown_near], UNKNOWN)
        self.assertGreaterEqual(forward_corridor_m(data, grid), 0.0)


    def test_rotation_clearance_requires_known_free_full_swept_disk(self):
        grid = Grid(width=100, height=100, resolution_m=0.05,
                    origin_x_m=-2.5, origin_y_m=-2.5)
        footprint = [(-0.2, -0.1), (-0.2, 0.1), (0.2, 0.1), (0.2, -0.1)]
        margins = dict(center_drift_allowance_m=0.10,
                       sensor_mount_uncertainty_m=0.02,
                       safety_margin_m=0.05)
        result = swept_rotation_clearance([FREE]*10000, grid, footprint, **margins)
        self.assertTrue(result["authorized"])
        self.assertEqual(result["reason"], "swept_disk_clear")
        self.assertGreater(result["required_radius_m"], result["footprint_circumradius_m"])

        unknown = [FREE]*10000
        unknown[50*grid.width+54] = UNKNOWN
        denied_unknown = swept_rotation_clearance(unknown, grid, footprint, **margins)
        self.assertFalse(denied_unknown["authorized"])
        self.assertEqual(denied_unknown["reason"], "swept_disk_unknown")

        occupied = [FREE]*10000
        occupied[50*grid.width+57] = OCCUPIED
        denied_occupied = swept_rotation_clearance(occupied, grid, footprint, **margins)
        self.assertFalse(denied_occupied["authorized"])
        self.assertEqual(denied_occupied["reason"], "swept_disk_occupied")

    def test_rotation_clearance_fails_closed_at_grid_edge_and_bad_geometry(self):
        small = Grid(width=10, height=10, resolution_m=0.05,
                     origin_x_m=-0.25, origin_y_m=-0.25)
        footprint = [(-0.2, -0.1), (-0.2, 0.1), (0.2, 0.1), (0.2, -0.1)]
        margins = dict(center_drift_allowance_m=0.10,
                       sensor_mount_uncertainty_m=0.02,
                       safety_margin_m=0.05)
        edge = swept_rotation_clearance([FREE]*100, small, footprint, **margins)
        self.assertFalse(edge["authorized"])
        self.assertEqual(edge["reason"], "swept_disk_outside_grid")
        with self.assertRaisesRegex(ValueError, "rotation_grid_length_mismatch"):
            swept_rotation_clearance([FREE], small, footprint, **margins)
        with self.assertRaisesRegex(ValueError, "rotation_footprint_invalid"):
            swept_rotation_clearance([FREE]*100, small, [(0, 0)]*2, **margins)

    def test_rgb_only_fills_unknown_and_cannot_erase_lidar_occupied(self):
        lidar_grid = Grid(width=2, height=1, resolution_m=1.0,
                          origin_x_m=0.0, origin_y_m=0.0)
        lidar = [OCCUPIED, UNKNOWN]
        rgb = [FREE]*4
        fused = fuse_lidar_with_rgb_free(
            lidar, lidar_grid, rgb, (2, 2, 1.0, 0.0, 0.0, "base_link"))
        self.assertEqual(fused, [OCCUPIED, FREE])
        rgb[1] = OCCUPIED
        fused2 = fuse_lidar_with_rgb_free(
            [UNKNOWN, UNKNOWN], lidar_grid, rgb,
            (2, 2, 1.0, 0.0, 0.0, "base_link"))
        self.assertEqual(fused2, [FREE, UNKNOWN])

    def test_rgb_frame_mismatch_fails_closed(self):
        lidar_grid = Grid(width=1, height=1, resolution_m=1.0,
                          origin_x_m=0.0, origin_y_m=0.0)
        with self.assertRaisesRegex(ValueError, "fusion_frame_mismatch"):
            fuse_lidar_with_rgb_free([UNKNOWN], lidar_grid, [FREE],
                                     (1, 1, 1.0, 0.0, 0.0, "map"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
