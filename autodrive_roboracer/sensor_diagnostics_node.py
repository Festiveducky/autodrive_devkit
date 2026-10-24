"""Subscribe to RoboRacer sensors and record diagnostics without commanding motion."""

import math
from pathlib import Path
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32, Int32

from autodrive_roboracer.sensor_diagnostics import Diagnostics, JsonlRecorder


class SensorDiagnosticsNode(Node):
    """Keep subscriber wiring separate from testable measurement mathematics."""

    def __init__(self):
        super().__init__('sensor_diagnostics_node')
        self.declare_parameter('topic_prefix', '/autodrive/roboracer_1')
        self.declare_parameter('report_period', 1.0)
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('include_ground_truth', False)
        self.declare_parameter('output_dir', str(Path.home() / 'roboracer_diagnostics'))
        period = float(self.get_parameter('report_period').value)
        if not math.isfinite(period) or period <= 0:
            raise ValueError('report_period must be positive and finite')
        prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
        if not prefix.startswith('/') or not prefix:
            raise ValueError('topic_prefix must be an absolute nonempty ROS namespace')
        self.diagnostics = Diagnostics(
            float(self.get_parameter('stale_timeout').value),
            bool(self.get_parameter('include_ground_truth').value),
        )
        self.recorder = None
        self.subscriptions_by_stream = []
        self.previous_status = {}
        types = {'scan': LaserScan, 'imu': Imu, 'encoder': JointState,
                 'float': Float32, 'count': Int32, 'odom': Odometry}
        topics = {key: f'{prefix}/{key}' for key in self.diagnostics.kinds}
        for key, kind in self.diagnostics.kinds.items():
            self.subscriptions_by_stream.append(self.create_subscription(
                types[kind], topics[key],
                lambda msg, stream=key: self.diagnostics.observe(stream, msg),
                qos_profile_sensor_data,
            ))
        self.recorder = JsonlRecorder(str(self.get_parameter('output_dir').value), {
            'topics': topics, 'report_period_s': period,
            'stale_timeout_s': self.diagnostics.streams['lidar'].stale_timeout,
            'ground_truth_role': 'evaluation_only',
            'encoder_units': 'raw_unverified', 'mode': 'passive_no_actuator_publishers',
            'rate_window_messages': 120,
        })
        self.report_timer = self.create_timer(period, self.report)
        self.get_logger().info(f'Passive diagnostics; JSONL: {self.recorder.path}')

    def report(self):
        """Record full health once per interval and print bounded readable output."""
        report = self.diagnostics.snapshot(ros_now_ns=self.get_clock().now().nanoseconds)
        self.recorder.write('diagnostic_report', report)
        rows = []
        for key, health in report['streams'].items():
            rate = health['observed_publication_hz']
            hz = f'{rate:.1f}Hz' if rate is not None else 'rate=?'
            age = health['receipt_age_s']
            age_text = f'{age:.2f}s' if age is not None else '?'
            rows.append(f"{key}={health['status']}({hz},age={age_text})")
            if self.previous_status.get(key) != health['status']:
                if health['status'] in ('missing', 'stale', 'invalid'):
                    self.get_logger().warning(
                        f"{key}: {health['status']}; issues={health['issues']}")
                self.previous_status[key] = health['status']
        self.get_logger().info(' | '.join(rows))
        scan = report['streams']['lidar']['measurements']
        imu = report['streams']['imu']['measurements']
        left = report['streams']['left_encoder']['measurements']
        right = report['streams']['right_encoder']['measurements']
        self.get_logger().info(
            f"lidar beams={scan.get('beams')} finite={scan.get('finite_returns')} "
            f"no_return={scan.get('no_return')} invalid={scan.get('invalid_ranges')} "
            f"nearest_m={scan.get('nearest_m')} | "
            f"encoder raw delta L={left.get('delta_raw')} R={right.get('delta_raw')} | "
            f"IMU angular_rad_s={imu.get('angular_velocity_rad_s')} "
            f"accel_m_s2={imu.get('linear_acceleration_m_s2')} | "
            f"feedback throttle={report['streams']['throttle']['measurements'].get('value')} "
            f"steering={report['streams']['steering']['measurements'].get('value')}"
        )
        for event in report['events']:
            self.get_logger().warning(f'Diagnostic event: {event}')

    def close_recording(self):
        """Persist final health and release the log even during shutdown."""
        if self.recorder is not None and not self.recorder.file.closed:
            try:
                self.recorder.write('diagnostic_report', self.diagnostics.snapshot())
            finally:
                self.recorder.close()


def main(args=None):
    """Run passive inspection until interrupted; log failures remain visible."""
    # Detached shells may inherit SIGINT ignored; install explicit handlers so
    # ros2 launch, Ctrl+C, and a normal SIGTERM all finalize the recording.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = SensorDiagnosticsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.close_recording()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
