"""Drive toward open LiDAR space with clearance and terminal diagnostics."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from autodrive_roboracer.gap_controller import ControllerConfig, DriveCommand, compute_command


class WallFollowNode(Node):
    """Keep ROS communication separate from the small gap-following algorithm."""

    def __init__(self):
        super().__init__('wall_follow_node')
        self.declare_parameter('debug_period', 1.0)
        self.declare_parameter('print_ranges', False)
        self.declare_parameter('enabled', True)
        self.declare_parameter('scan_timeout', 0.5)
        defaults = ControllerConfig()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.config = ControllerConfig(**{
            name: float(self.get_parameter(name).value) for name in vars(defaults)
        })
        self.enabled = bool(self.get_parameter('enabled').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        if not math.isfinite(self.scan_timeout) or self.scan_timeout <= 0.0:
            raise ValueError('scan_timeout must be finite and greater than zero')
        debug_period = float(self.get_parameter('debug_period').value)
        if not math.isfinite(debug_period) or debug_period <= 0.0:
            raise ValueError('debug_period must be a finite number greater than zero')

        self.latest_scan = None
        self.scan_count = 0
        self.last_reported_count = 0
        self.last_controlled_count = 0
        self.last_scan_time = None
        self.command = DriveCommand()
        self.previous_angle = 0.0
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/autodrive/roboracer_1/lidar',
            self.lidar_callback, qos_profile_sensor_data,
        )
        # Commands are Float32 in [-1, 1]; positive steering turns left.
        self.throttle_publisher = self.create_publisher(
            Float32, '/autodrive/roboracer_1/throttle_command', 1,
        )
        self.steering_publisher = self.create_publisher(
            Float32, '/autodrive/roboracer_1/steering_command', 1,
        )
        self.debug_timer = self.create_timer(debug_period, self.log_scan)
        self.control_timer = self.create_timer(0.05, self.control_step)
        self.get_logger().info(
            'Listening on /autodrive/roboracer_1/lidar. '
            f'Controller enabled={self.enabled}, max throttle={self.config.max_throttle:.2f}, '
            f'wall clearance={self.config.clearance:.2f} m.'
        )

    def lidar_callback(self, scan):
        """Keep the latest complete message for the next control update."""
        self.latest_scan = scan
        self.scan_count += 1
        self.last_scan_time = time.monotonic()

    def publish_command(self):
        """Publish both normalized actuator commands together."""
        self.steering_publisher.publish(Float32(data=float(self.command.steering)))
        self.throttle_publisher.publish(Float32(data=float(self.command.throttle)))

    def control_step(self):
        """Run at 20 Hz; stale scans always override the last driving command."""
        if not self.enabled:
            self.command = DriveCommand(reason='controller disabled')
        elif self.last_scan_time is None:
            self.command = DriveCommand(reason='waiting for scan')
        elif time.monotonic() - self.last_scan_time > self.scan_timeout:
            self.command = DriveCommand(reason='scan timeout: stopped')
        elif self.scan_count != self.last_controlled_count:
            self.command = compute_command(self.latest_scan, self.config, self.previous_angle)
            self.last_controlled_count = self.scan_count
        self.previous_angle = self.command.target_angle
        self.publish_command()

    def stop(self):
        """Send zero throttle and steering before releasing ROS publishers."""
        self.enabled = False
        self.command = DriveCommand(reason='shutdown')
        self.publish_command()

    @staticmethod
    def distance_at_angle(scan, degrees):
        """Format the nearest beam using the incoming scan geometry."""
        if (not scan.ranges or not math.isfinite(scan.angle_min)
                or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0.0):
            return 'unavailable'
        index = round((math.radians(degrees) - scan.angle_min) / scan.angle_increment)
        if not 0 <= index < len(scan.ranges):
            return 'outside scan'
        distance = scan.ranges[index]
        if distance == math.inf:
            return f'no return (>{scan.range_max:.2f} m)'
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            return f'invalid({distance!r})'
        return f'{distance:.3f} m'

    def log_scan(self):
        """Print bounded debug output and detect an absent or stalled stream."""
        scan = self.latest_scan
        if scan is None:
            self.get_logger().info('Waiting for LiDAR: start the bridge and simulator connection.')
            return
        if self.scan_count == self.last_reported_count:
            self.get_logger().warning('No new LiDAR scans since the previous debug report.')
            return
        self.last_reported_count = self.scan_count
        valid = [r for r in scan.ranges
                 if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        nearest = f'{min(valid):.3f} m' if valid else 'unavailable'
        self.get_logger().info(
            f'scan={self.scan_count} frame={scan.header.frame_id!r} '
            f'beams={len(scan.ranges)} valid={len(valid)} '
            f'angles=[{math.degrees(scan.angle_min):.2f}, '
            f'{math.degrees(scan.angle_max):.2f}] deg '
            f'step={math.degrees(scan.angle_increment):.3f} deg '
            f'limits=[{scan.range_min:.2f}, {scan.range_max:.2f}] m | '
            f'right(-90)={self.distance_at_angle(scan, -90)} '
            f'front(0)={self.distance_at_angle(scan, 0)} '
            f'left(+90)={self.distance_at_angle(scan, 90)} nearest={nearest} | '
            f'{self.command.reason} target={math.degrees(self.command.target_angle):+.1f} deg '
            f'path={self.command.target_distance:.2f} m '
            f'turn_boost={self.command.turn_strength:.0%} '
            f'throttle={self.command.throttle:.3f} steering={self.command.steering:+.3f}'
        )
        if self.get_parameter('print_ranges').value:
            self.get_logger().info(f'ranges (m): {list(scan.ranges)}')


def main(args=None):
    """Initialize ROS, spin until interrupted, and release the node/context."""
    # Let Python raise KeyboardInterrupt before ROS destroys its context, so
    # finally can still publish a stop command on Ctrl+C.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = WallFollowNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# "AutoDRIVE Simulator.exe" -force-d3d11

# docker run --platform linux/amd64 --name penn-challenge --rm -it -p 4567:4567 -v "${PWD}/autodrive_devkit:/home/autodrive_devkit/src/autodrive_devkit" --entrypoint /bin/bash penn-challenge
# source install/setup.bash
# ros2 launch autodrive_roboracer bringup_headless.launch.py

# docker exec -it penn-challenge bash
# source /opt/ros/humble/setup.bash
# source /home/autodrive_devkit/install/setup.bash
# ros2 run autodrive_roboracer wall_follow_node
