"""ROS publisher discovery and DDS message timing for the bridge command gate."""

import json
import time
from dataclasses import dataclass

from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

from autodrive_roboracer.command_safety import CommandPublisher, CommandSafety


@dataclass
class CommandSample:
    """Carry the ROS message and metadata through older Humble executors."""

    message: object
    metadata: dict


class CommandSafetyExecutor(MultiThreadedExecutor):
    """Preserve take_message metadata which this pinned Humble otherwise discards."""

    def __init__(self, adapter, **kwargs):
        super().__init__(**kwargs)
        self.adapter = adapter

    def _take_subscription(self, sub):
        if sub.callback not in (self.adapter.throttle, self.adapter.steering):
            return super()._take_subscription(sub)
        with sub.handle:
            packet = sub.handle.take_message(sub.msg_type, sub.raw)
        return CommandSample(*packet) if packet is not None else None


class CommandSafetyAdapter:
    """Adapt the existing Float32 interfaces without changing controller algorithms."""

    def __init__(self, node):
        self.node = node
        node.declare_parameter('command_timeout', 0.25)
        node.declare_parameter('command_owner', '')
        self.gate = CommandSafety(
            float(node.get_parameter('command_timeout').value),
            str(node.get_parameter('command_owner').value),
        )
        self.topics = {
            key: node.resolve_topic_name(f'/autodrive/roboracer_1/{key}_command')
            for key in self.gate.channels
        }
        self.status_publisher = node.create_publisher(
            String, '/autodrive/roboracer_1/command_status', 1)
        self.previous_status = None
        self.accept_after_ns = time.time_ns()
        self.timer = node.create_timer(0.05, self.report)

    def refresh_ownership(self):
        """Reject multiple writers, even before a second writer sends a sample."""
        publishers = {}
        for key, topic in self.topics.items():
            publishers[key] = [
                CommandPublisher(
                    info.node_namespace.rstrip('/') + '/' + info.node_name,
                    bytes(info.endpoint_gid),
                ) for info in self.node.get_publishers_info_by_topic(topic)
            ]
        before = self.gate.topology
        self.gate.set_publishers(publishers['throttle'], publishers['steering'])
        if before != self.gate.topology:
            self.accept_after_ns = time.time_ns()

    def receive(self, key, sample):
        """Reject pre-ownership and delayed samples using DDS source timestamps."""
        self.refresh_ownership()
        stamp = sample.metadata['source_timestamp']
        if stamp < self.accept_after_ns:
            self.gate.invalidate('discarding command sent before ownership transition')
            return
        age = (time.time_ns() - stamp) / 1e9 if stamp > 0 else float('nan')
        # This pinned Humble exposes timestamps but no publisher GID per sample.
        # Enforce sole ownership through the graph; never fabricate a message GID.
        self.gate.receive(key, sample.message.data, source_age=age)

    def invalidate(self, reason):
        """Require commands sent after reconnect/shutdown, not queued old values."""
        self.accept_after_ns = time.time_ns()
        self.gate.invalidate(reason)

    def throttle(self, sample):
        """Receive a normalized throttle command with DDS metadata."""
        self.receive('throttle', sample)

    def steering(self, sample):
        """Receive a normalized steering command with DDS metadata."""
        self.receive('steering', sample)

    def snapshot(self):
        """Check topology and timeout again immediately before socket emission."""
        self.refresh_ownership()
        return self.gate.snapshot()

    def report(self):
        """Publish inspectable watchdog state and log only state transitions."""
        state = self.snapshot()
        self.status_publisher.publish(String(data=json.dumps(state, allow_nan=False)))
        status = (state['enabled'], state['owner'], state['reason'])
        if status != self.previous_status:
            self.node.get_logger().info(
                f"Command gate: enabled={state['enabled']} owner={state['owner']} "
                f"reason={state['reason']}")
            self.previous_status = status
