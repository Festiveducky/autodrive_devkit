"""Synthetic health/recording tests; runnable without ROS or a simulator."""

import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from autodrive_roboracer.sensor_diagnostics import (
    Diagnostics, JsonlRecorder, StreamHealth, summarize,
)


def header(seconds=10, frame='sensor'):
    """Create a minimal stamped-message header."""
    return NS(stamp=NS(sec=int(seconds), nanosec=round((seconds % 1) * 1e9)), frame_id=frame)


def encoder(position=0.0, seconds=10):
    """Use raw encoder positions without assigning a physical conversion."""
    return NS(position=[position], name=['rear_left'], header=header(seconds))


def scan(ranges=None):
    """Create a consistent five-beam scan."""
    return NS(ranges=[1.0] * 5 if ranges is None else ranges,
              angle_min=-1.0, angle_max=1.0, angle_increment=0.5,
              range_min=0.06, range_max=10.0, scan_time=0.025,
              time_increment=0.005, header=header())


def imu():
    """Create an ordinary stationary IMU message."""
    return NS(orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
              angular_velocity=NS(x=0.0, y=0.0, z=0.2),
              linear_acceleration=NS(x=0.0, y=0.0, z=9.81),
              orientation_covariance=[0.0] * 9,
              angular_velocity_covariance=[0.0] * 9,
              linear_acceleration_covariance=[0.0] * 9, header=header())


class StreamHealthTests(unittest.TestCase):
    """Check clock independence, recovery, and raw encoder derivatives."""

    def test_frequency_age_and_missing_stale_recovery(self):
        health = StreamHealth(0.5)
        self.assertEqual(health.snapshot(5)['status'], 'missing')
        for index in range(5):
            health.observe(5 + index * 0.1, {}, [], (10 + index) * 10**9, 'lidar')
        report = health.snapshot(5.5, 15 * 10**9)
        self.assertAlmostEqual(report['observed_publication_hz'], 10)
        self.assertAlmostEqual(report['receipt_age_s'], 0.1)
        self.assertEqual(report['frame_id'], 'lidar')
        self.assertEqual(report['header_age_s'], 1)
        self.assertEqual(health.snapshot(6)['status'], 'stale')
        self.assertIn('receipt_gap', health.observe(6, {}, [], 15 * 10**9, 'lidar'))
        self.assertEqual(health.snapshot(6)['status'], 'fresh')
        self.assertIsNone(health.snapshot(6)['observed_publication_hz'])

    def test_rolling_frequency_uses_recent_window(self):
        health = StreamHealth(10, window_size=3)
        for now in (0, 1, 2, 2.1, 2.2):
            health.observe(now, {}, [])
        self.assertAlmostEqual(health.snapshot(2.2)['observed_publication_hz'], 10)
        self.assertEqual(health.total_messages, 5)

    def test_zero_or_repeated_intervals_do_not_divide_by_zero(self):
        health = StreamHealth(0.5)
        health.observe(1, {}, [], 123, 'imu')
        health.observe(1, {}, [], 123, 'imu')
        self.assertIsNone(health.snapshot(1)['observed_publication_hz'])
        self.assertEqual(health.repeated_stamps, 1)

    def test_reversal_does_not_break_monotonic_frequency(self):
        diagnostics = Diagnostics()
        diagnostics.observe('left_encoder', encoder(3, 20), now=1)
        diagnostics.observe('left_encoder', encoder(4, 21), now=1.1)
        diagnostics.observe('left_encoder', encoder(0, 1), now=1.2)
        report = diagnostics.snapshot(1.3)
        health = report['streams']['left_encoder']
        self.assertEqual(health['header_timestamp_reversals'], 1)
        self.assertLess(health['header_dt_s'], 0)
        self.assertAlmostEqual(health['observed_publication_hz'], 10)
        self.assertIsNone(health['measurements']['delta_raw'])
        self.assertEqual(report['events'][0]['event'], 'header_timestamp_reversal')
        self.assertEqual(diagnostics.snapshot(1.3)['events'], [])

    def test_encoder_delta_and_no_conversion(self):
        diagnostics = Diagnostics()
        diagnostics.observe('left_encoder', encoder(1), now=1)
        diagnostics.observe('left_encoder', encoder(1.25), now=1.1)
        measured = diagnostics.snapshot(1.1)['streams']['left_encoder']['measurements']
        self.assertAlmostEqual(measured['delta_raw'], 0.25)
        self.assertAlmostEqual(measured['rate_raw_per_s'], 2.5)
        self.assertNotIn('velocity_m_s', measured)

    def test_gap_and_invalid_encoder_break_derivative_baseline(self):
        diagnostics = Diagnostics()
        for position, now in ((1, 1), (3, 2), (math.nan, 2.1), (4, 2.2)):
            diagnostics.observe('right_encoder', encoder(position), now=now)
            measured = diagnostics.snapshot(now)['streams']['right_encoder']['measurements']
            self.assertIsNone(measured['delta_raw'])

    def test_counter_reset_clears_encoder_baselines(self):
        diagnostics = Diagnostics()
        diagnostics.observe('lap_count', NS(data=2), now=1)
        diagnostics.observe('left_encoder', encoder(5), now=1.1)
        diagnostics.observe('lap_count', NS(data=0), now=1.2)
        diagnostics.observe('left_encoder', encoder(0), now=1.3)
        report = diagnostics.snapshot(1.3)
        self.assertEqual(report['reset_candidates'], 1)
        self.assertEqual(report['events'][0]['event'], 'simulator_reset_candidate')
        self.assertIsNone(report['streams']['left_encoder']['measurements']['delta_raw'])

    def test_lap_time_rollover_is_not_a_reset(self):
        diagnostics = Diagnostics()
        for index, value in enumerate((12.0, 0.01, 0.2)):
            diagnostics.observe('lap_time', NS(data=value), now=index)
        self.assertEqual(diagnostics.reset_candidates, 0)

    def test_last_lap_cleared_is_reset_evidence(self):
        diagnostics = Diagnostics()
        diagnostics.observe('last_lap_time', NS(data=12.0), now=1)
        diagnostics.observe('last_lap_time', NS(data=11.0), now=1.1)
        self.assertEqual(diagnostics.reset_candidates, 0)
        diagnostics.observe('last_lap_time', NS(data=0.0), now=1.2)
        self.assertEqual(diagnostics.reset_candidates, 1)

    def test_invalid_status_and_configuration(self):
        diagnostics = Diagnostics()
        diagnostics.observe('throttle', NS(data=2.0), now=1)
        self.assertEqual(diagnostics.snapshot(1)['streams']['throttle']['status'], 'invalid')
        diagnostics.observe('throttle', NS(data=0.0), now=1.1)
        self.assertEqual(diagnostics.snapshot(1.1)['streams']['throttle']['invalid_messages'], 1)
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                Diagnostics(stale_timeout=value)


class MeasurementTests(unittest.TestCase):
    """Separate no-return data, invalid inputs, and reference-only odometry."""

    def test_range_categories_and_strict_json(self):
        measured, issues = summarize('scan', scan([1, math.inf, math.nan, -math.inf, 0]))
        self.assertEqual(measured['finite_returns'], 1)
        self.assertEqual(measured['no_return'], 1)
        self.assertEqual(measured['invalid_ranges'], 3)
        self.assertEqual(measured['nearest_m'], 1)
        self.assertTrue(issues)
        json.dumps(measured, allow_nan=False)

    def test_all_no_returns_is_not_invalid(self):
        measured, issues = summarize('scan', scan([math.inf] * 5))
        self.assertEqual(issues, [])
        self.assertEqual(measured['no_return'], 5)
        self.assertIsNone(measured['nearest_m'])

    def test_malformed_scan_geometry_and_empty_scan(self):
        for attribute, value in (('angle_increment', 0), ('range_max', math.nan),
                                 ('range_min', 11), ('angle_min', math.inf)):
            message = scan()
            setattr(message, attribute, value)
            _, issues = summarize('scan', message)
            self.assertIn('invalid scan geometry', issues)
        self.assertIn('empty scan', summarize('scan', scan([]))[1])
        self.assertIn('beam count inconsistent with angular bounds',
                      summarize('scan', scan([1] * 20))[1])

    def test_actual_1080_and_1081_geometry(self):
        for count in (1080, 1081):
            message = scan([1] * count)
            message.angle_min = -2.35619
            message.angle_max = 2.35619
            message.angle_increment = 0.004363323
            self.assertEqual(summarize('scan', message)[1], [])

    def test_imu_validity(self):
        message = imu()
        measured, issues = summarize('imu', message)
        self.assertEqual(issues, [])
        self.assertEqual(measured['angular_velocity_rad_s'][2], 0.2)
        message.orientation.w = 0
        message.angular_velocity.x = math.nan
        message.orientation_covariance[0] = -1
        self.assertEqual(len(summarize('imu', message)[1]), 3)

    def test_zero_header_stamp_and_empty_frame_reported(self):
        diagnostics = Diagnostics()
        message = encoder()
        message.header = header(0, '')
        diagnostics.observe('left_encoder', message, now=1)
        self.assertEqual(diagnostics.snapshot(1)['streams']['left_encoder']['status'], 'invalid')

    def test_uncompleted_lap_sentinel_and_invalid_durations(self):
        diagnostics = Diagnostics()
        for key in ('last_lap_time', 'best_lap_time'):
            diagnostics.observe(key, NS(data=math.inf), now=1)
            health = diagnostics.snapshot(1)['streams'][key]
            self.assertEqual(health['status'], 'fresh')
            self.assertFalse(health['measurements']['available'])
            self.assertEqual(health['measurements']['unavailable_reason'], 'no_completed_lap')
            diagnostics.observe(key, NS(data=math.nan), now=1.1)
            self.assertEqual(diagnostics.snapshot(1.1)['streams'][key]['status'], 'invalid')
            diagnostics.observe(key, NS(data=-1.0), now=1.2)
            self.assertEqual(diagnostics.snapshot(1.2)['streams'][key]['status'], 'invalid')
            diagnostics.observe(key, NS(data=12.0), now=1.3)
            self.assertTrue(diagnostics.snapshot(1.3)['streams'][key]['measurements']['available'])

    def test_ground_truth_is_opt_in_and_labeled(self):
        self.assertNotIn('odom', Diagnostics().kinds)
        diagnostics = Diagnostics(include_ground_truth=True)
        point = NS(x=1.0, y=2.0, z=0.0)
        pose = NS(position=point, orientation=NS(x=0., y=0., z=0., w=1.))
        message = NS(header=header(), child_frame_id='roboracer_1',
                     pose=NS(pose=pose), twist=NS(twist=NS(linear=point)))
        diagnostics.observe('odom', message, now=1)
        measured = diagnostics.snapshot(1)['streams']['odom']['measurements']
        self.assertEqual(measured['role'], 'simulator_ground_truth_evaluation_only')


class RecorderTests(unittest.TestCase):
    """Check flush, strict JSON, unique names, and clean finalization."""

    def test_readable_before_and_after_close(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = JsonlRecorder(Path(directory) / 'nested', {'mode': 'passive'})
            recorder.write('diagnostic_report', Diagnostics().snapshot(1))
            # The writer is still open: each completed report must already be visible.
            rows = [json.loads(line) for line in
                    recorder.path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(len(rows), 2)
            recorder.close()
            recorder.close()
            rows = [json.loads(line) for line in
                    recorder.path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual([row['record_type'] for row in rows],
                             ['session_start', 'diagnostic_report', 'session_end'])
            other = JsonlRecorder(directory, {})
            other.close()
            self.assertNotEqual(recorder.path.name, other.path.name)

    def test_nonfinite_rejected_without_corrupting_completed_lines(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = JsonlRecorder(directory, {})
            try:
                with self.assertRaises(ValueError):
                    recorder.write('bad', {'value': math.inf})
            finally:
                recorder.close()
            rows = recorder.path.read_text(encoding='utf-8').splitlines()
            self.assertEqual(len(rows), 2)
            for line in rows:
                json.loads(line)


if __name__ == '__main__':
    unittest.main()
