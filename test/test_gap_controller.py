"""Behavior tests for the planner; no ROS installation is needed."""

import math
from types import SimpleNamespace
import unittest

from autodrive_roboracer.gap_controller import (
    ControllerConfig, calculate_steering, calculate_throttle, compute_command,
    inflate_obstacles, sanitize_ranges, sharp_turn_strength,
)


def make_scan(ranges=None, count=1081):
    """Make scans with the simulator's angular coverage, including 1081 beams."""
    return SimpleNamespace(
        ranges=[math.inf] * count if ranges is None else ranges,
        angle_min=math.radians(-135), angle_increment=math.radians(270 / (count - 1)),
        range_min=0.06, range_max=10.0,
    )


def corridor_scan(left=0.6, right=0.6, end=math.inf):
    """Raycast two parallel walls and an optional wall across the corridor."""
    scan = make_scan()
    for i in range(len(scan.ranges)):
        angle = scan.angle_min + i * scan.angle_increment
        sine, cosine = math.sin(angle), math.cos(angle)
        side = left / sine if sine > 1e-8 else right / -sine if sine < -1e-8 else math.inf
        front = end / cosine if cosine > 1e-8 else math.inf
        distance = min(side, front)
        scan.ranges[i] = distance if distance <= scan.range_max else math.inf
    return scan


class GapControllerTests(unittest.TestCase):
    """Check steering direction, obstacle margins, speed, and bad-data stops."""

    def setUp(self):
        self.config = ControllerConfig()

    def test_open_straight_and_actual_beam_counts(self):
        for count in (1080, 1081, 721):
            command = compute_command(make_scan(count=count), self.config)
            self.assertGreater(command.throttle, 0.0)
            self.assertAlmostEqual(command.steering, 0.0, delta=0.02)
            self.assertLessEqual(command.throttle, self.config.max_throttle)

    def test_parallel_walls_do_not_cause_false_frontal_stop(self):
        command = compute_command(corridor_scan(), self.config)
        self.assertGreater(command.throttle, 0.0)
        self.assertAlmostEqual(command.steering, 0.0, delta=0.02)

    def test_steers_away_from_nearby_wall(self):
        for left, right, sign in ((0.22, 1.0, -1), (1.0, 0.22, 1)):
            command = compute_command(corridor_scan(left, right), self.config)
            self.assertGreater(sign * command.steering, 0.0)
            self.assertGreater(command.throttle, 0.0)

    def test_corner_direction_and_turn_slowdown(self):
        straight = compute_command(make_scan(), self.config)
        for turn in (-1, 1):
            scan = corridor_scan(end=1.5)
            # Remove one side wall to model the opening at a corner.
            for i in range(len(scan.ranges)):
                angle = scan.angle_min + i * scan.angle_increment
                if turn * angle > math.radians(20):
                    scan.ranges[i] = 6.0
            command = compute_command(scan, self.config)
            self.assertGreater(turn * command.steering, 0.0)
            self.assertGreater(command.throttle, 0.0)
            self.assertLess(command.throttle, straight.throttle)

    def test_speed_increases_with_available_distance(self):
        near = compute_command(corridor_scan(end=0.8), self.config)
        far = compute_command(corridor_scan(end=2.0), self.config)
        self.assertGreater(far.throttle, near.throttle)

    def test_stop_before_front_obstacle(self):
        command = compute_command(corridor_scan(end=0.3), self.config)
        self.assertEqual(command.throttle, 0.0)

    def test_stop_with_dangerously_close_side_wall(self):
        command = compute_command(corridor_scan(left=0.1), self.config)
        self.assertEqual(command.throttle, 0.0)

    def test_unknown_front_and_all_invalid_scans_stop(self):
        for value in (math.nan, -math.inf, 0.0, -1.0):
            scan = make_scan()
            scan.ranges[540] = value
            self.assertEqual(compute_command(scan, self.config).throttle, 0.0)
            scan.ranges = [value] * 1081
            self.assertEqual(compute_command(scan, self.config).throttle, 0.0)

    def test_positive_infinity_is_open_and_large_returns_are_capped(self):
        self.assertEqual(sanitize_ranges([math.inf, 11.0, math.nan, 0.02], 0.06, 10),
                         [10, 10, 0, 0])

    def test_obstacles_shrink_neighboring_paths(self):
        ranges = [10.0] * 181
        ranges[90] = 1.0
        inflated = inflate_obstacles(ranges, math.radians(1), self.config.clearance)
        self.assertEqual(inflated[75], 1.0)
        self.assertEqual(inflated[105], 1.0)
        self.assertEqual(inflated[60], 10.0)

    def test_single_long_ray_cannot_win_in_a_blocked_wall(self):
        scan = make_scan([0.6] * 1081)
        scan.ranges[650] = 10.0
        command = compute_command(scan, self.config)
        self.assertLessEqual(command.target_distance, 0.6)

    def test_malformed_geometry_stops(self):
        for field, value in [('angle_increment', 0.0), ('angle_min', math.nan),
                             ('range_max', 0.0), ('ranges', [])]:
            scan = make_scan()
            setattr(scan, field, value)
            self.assertEqual(compute_command(scan, self.config).throttle, 0.0)

    def test_incomplete_field_of_view_stops(self):
        scan = make_scan()
        scan.angle_min = 0.0
        self.assertEqual(compute_command(scan, self.config).throttle, 0.0)

    def test_clearance_parameter_rejects_narrow_opening(self):
        command = compute_command(corridor_scan(left=0.3, right=0.3), self.config)
        self.assertEqual(command.throttle, 0.0)

    def test_steering_is_bounded_during_extreme_turns(self):
        scan = corridor_scan(left=1.0, right=0.16, end=0.6)
        command = compute_command(scan, self.config)
        self.assertTrue(-1.0 <= command.steering <= 1.0)
        self.assertTrue(0.0 <= command.throttle <= self.config.max_throttle)

    def test_invalid_configuration_rejected(self):
        for kwargs in ({'max_throttle': 2.0}, {'clearance': -1.0},
                       {'throttle_gain': math.nan}, {'forward_angle': 100.0},
                       {'turn_slowdown': 1.0}, {'sharp_turn_throttle': 1.1},
                       {'min_lookahead': 2.0}, {'sharp_turn_full_angle': 30.0},
                       {'sharp_turn_steering_gain': 0.8},
                       {'sharp_turn_lookahead_scale': 1.1}):
            with self.assertRaises(ValueError):
                ControllerConfig(**kwargs)

    def test_sharp_turn_detection_is_smooth_and_symmetric(self):
        for angle, expected in ((20, 0.0), (35, 0.0), (52.5, 0.5), (70, 1.0), (80, 1.0)):
            for sign in (-1, 1):
                self.assertAlmostEqual(
                    sharp_turn_strength(math.radians(angle * sign), self.config), expected,
                )

    def test_sharp_turn_steers_harder_without_affecting_straights(self):
        unboosted = ControllerConfig(sharp_turn_steering_gain=1.0,
                                     sharp_turn_lookahead_scale=1.0)
        for sign in (-1, 1):
            angle = math.radians(50 * sign)
            normal, _ = calculate_steering(angle, 6.0, unboosted)
            boosted, _ = calculate_steering(angle, 6.0, self.config)
            self.assertGreater(abs(boosted), abs(normal))
        self.assertEqual(calculate_steering(0.0, 6.0, self.config)[0], 0.0)

    def test_new_tuning_is_faster_on_straights_and_corners(self):
        old = ControllerConfig(max_throttle=0.12, throttle_gain=0.04,
                               turn_slowdown=0.75)
        for steering, strength in ((0.0, 0.0), (1.0, 1.0)):
            faster = calculate_throttle(6.0, 3.0, steering, strength, self.config)
            slower = calculate_throttle(6.0, 3.0, steering, strength, old)
            self.assertGreater(faster, slower)

    def test_corner_cap_does_not_force_acceleration_near_obstacles(self):
        limited = ControllerConfig(sharp_turn_throttle=0.05)
        self.assertLessEqual(calculate_throttle(10, 10, 1.0, 1.0, limited), 0.05)
        near = calculate_throttle(10, limited.stop_distance + 0.01, 1.0, 1.0, limited)
        self.assertLess(near, 0.001)
        self.assertEqual(calculate_throttle(10, 0.1, 1.0, 1.0, limited), 0.0)


if __name__ == '__main__':
    unittest.main()
