"""Check real DDS identity/discovery on isolated, non-actuating topics."""

import importlib.util
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'requires ROS 2 rclpy')
class CommandSafetyRosTests(unittest.TestCase):
    """Verify accepted source metadata, timeouts, and competing writers."""

    def test_transport_ownership_and_recovery(self):
        import rclpy
        from std_msgs.msg import Float32
        from autodrive_roboracer.command_safety_ros import (
            CommandSafetyAdapter, CommandSafetyExecutor, CommandSample,
        )

        rclpy.init()
        bridge = controller = competitor = executor = None
        try:
            bridge = rclpy.create_node('feature2_test_bridge', cli_args=[
                '--ros-args', '--log-level', 'error',
                '-r', '/autodrive/roboracer_1/throttle_command:=/feature2_test/throttle',
                '-r', '/autodrive/roboracer_1/steering_command:=/feature2_test/steering',
                '-r', '/autodrive/roboracer_1/command_status:=/feature2_test/status',
            ])
            controller = rclpy.create_node('feature2_test_controller')
            adapter = CommandSafetyAdapter(bridge)
            subs = [bridge.create_subscription(
                Float32, adapter.topics[key], getattr(adapter, key), 1)
                for key in ('throttle', 'steering')]
            pubs = [controller.create_publisher(Float32, topic, 1)
                    for topic in adapter.topics.values()]
            executor = CommandSafetyExecutor(adapter, num_threads=1)
            executor.add_node(bridge)
            executor.add_node(controller)

            def spin_for(duration):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.01)

            def send_until_enabled():
                deadline = time.monotonic() + 4
                while time.monotonic() < deadline:
                    for pub in pubs:
                        pub.publish(Float32(data=0.1))
                    spin_for(0.03)
                    if adapter.snapshot()['enabled']:
                        return
                self.fail(str(adapter.snapshot()))

            def wait_for_throttle_publishers(count):
                deadline = time.monotonic() + 4
                while time.monotonic() < deadline:
                    spin_for(0.02)
                    if adapter.snapshot()['publisher_counts']['throttle'] == count:
                        return
                self.fail(str(adapter.snapshot()))

            send_until_enabled()
            self.assertEqual(adapter.snapshot()['owner'], '/feature2_test_controller')
            old_stamp = time.time_ns()
            adapter.invalidate('test reconnect')
            adapter.throttle(CommandSample(Float32(data=0.1), {'source_timestamp': old_stamp}))
            self.assertFalse(adapter.snapshot()['enabled'])
            send_until_enabled()
            spin_for(0.35)
            self.assertFalse(adapter.snapshot()['enabled'])
            send_until_enabled()
            competitor = rclpy.create_node('feature2_test_competitor')
            extra = competitor.create_publisher(Float32, '/feature2_test/throttle', 1)
            executor.add_node(competitor)
            wait_for_throttle_publishers(2)
            self.assertFalse(adapter.snapshot()['enabled'])
            self.assertEqual(adapter.snapshot()['publisher_counts']['throttle'], 2)
            competitor.destroy_publisher(extra)
            wait_for_throttle_publishers(1)
            self.assertFalse(adapter.snapshot()['enabled'])
            send_until_enabled()
            # Hold references until the subscription callbacks have been exercised.
            self.assertEqual(len(subs), 2)
        finally:
            if executor is not None:
                executor.shutdown()
            for node in (competitor, controller, bridge):
                if node is not None:
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
