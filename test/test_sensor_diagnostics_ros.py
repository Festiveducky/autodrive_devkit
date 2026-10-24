"""ROS transport check; skipped on hosts without rclpy."""

import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'requires ROS 2 rclpy')
class RosDiagnosticsTests(unittest.TestCase):
    """Verify actual subscriptions, recording, and absence of actuator publishers."""

    def test_transport_recording_and_staleness(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, JointState, LaserScan
        from std_msgs.msg import Float32, Int32
        from autodrive_roboracer.sensor_diagnostics_node import SensorDiagnosticsNode

        with tempfile.TemporaryDirectory() as directory:
            rclpy.init(args=[
                '--ros-args', '-p', 'topic_prefix:=/feature1_transport_test',
                '-p', f'output_dir:={directory}', '-p', 'report_period:=0.1',
                '-p', 'stale_timeout:=0.2',
                '--log-level', 'error',
            ])
            diagnostics = publisher_node = executor = None
            try:
                diagnostics = SensorDiagnosticsNode()
                publisher_node = rclpy.create_node('feature1_test_publisher')
                executor = SingleThreadedExecutor()
                executor.add_node(diagnostics)
                executor.add_node(publisher_node)
                types = {'scan': LaserScan, 'imu': Imu, 'encoder': JointState,
                         'float': Float32, 'count': Int32}
                publishers = {
                    key: publisher_node.create_publisher(
                        types[kind], f'/feature1_transport_test/{key}', qos_profile_sensor_data)
                    for key, kind in diagnostics.diagnostics.kinds.items()
                }
                deadline = time.monotonic() + 5
                received = False
                while time.monotonic() < deadline:
                    for key, pub in publishers.items():
                        message = types[diagnostics.diagnostics.kinds[key]]()
                        if hasattr(message, 'header'):
                            message.header.stamp = publisher_node.get_clock().now().to_msg()
                            message.header.frame_id = 'test_sensor'
                        if key == 'lidar':
                            message.angle_min = -1.0
                            message.angle_max = 1.0
                            message.angle_increment = 0.5
                            message.range_min = 0.06
                            message.range_max = 10.0
                            message.ranges = [1.0] * 5
                        elif key == 'imu':
                            message.orientation.w = 1.0
                        elif key.endswith('encoder'):
                            message.name = ['wheel']
                            message.position = [1.0]
                        pub.publish(message)
                    for _ in range(len(publishers) + 2):
                        executor.spin_once(timeout_sec=0.01)
                    health = diagnostics.diagnostics.snapshot()['streams']
                    if all(row['status'] == 'fresh' for row in health.values()):
                        received = True
                        break
                self.assertTrue(received, health)
                # rclpy creates rosout/parameter event publishers automatically.
                owned = diagnostics.get_publisher_names_and_types_by_node(
                    diagnostics.get_name(), diagnostics.get_namespace())
                self.assertFalse(any(name.startswith('/feature1_transport_test/')
                                     for name, _ in owned), owned)
                self.assertFalse(any(name.endswith('_command') for name, _ in owned), owned)
                diagnostics.report()
                deadline = time.monotonic() + 0.4
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                health = diagnostics.diagnostics.snapshot()['streams']
                self.assertTrue(all(row['status'] == 'stale' for row in health.values()))
                diagnostics.close_recording()
                rows = [json.loads(line) for line in Path(diagnostics.recorder.path).read_text(
                    encoding='utf-8').splitlines()]
                self.assertEqual(rows[-1]['record_type'], 'session_end')
                self.assertTrue(any(
                    row['record_type'] == 'diagnostic_report'
                    and row['streams']['lidar']['status'] == 'fresh' for row in rows))
            finally:
                if executor is not None:
                    executor.shutdown()
                if diagnostics is not None:
                    diagnostics.close_recording()
                    diagnostics.destroy_node()
                if publisher_node is not None:
                    publisher_node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
