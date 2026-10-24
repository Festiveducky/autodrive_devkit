"""Thread-safe, ROS-independent actuator freshness and ownership gate."""

from dataclasses import dataclass
import math
from threading import RLock
import time


@dataclass(frozen=True)
class CommandPublisher:
    """Identify a ROS node and the DDS writer for one command channel."""

    node: str
    gid: bytes


class CommandSafety:
    """Require one common controller and a fresh valid command on both channels."""

    channels = ('throttle', 'steering')

    def __init__(self, timeout=0.25, required_owner=''):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('command_timeout must be positive and finite')
        if required_owner and not required_owner.startswith('/'):
            raise ValueError('command_owner must be an absolute ROS node name or empty')
        self.timeout = timeout
        self.required_owner = required_owner
        self.lock = RLock()
        self.owner = None
        self.writers = {}
        self.topology = None
        self.topology_reason = 'waiting for publisher discovery'
        self.reason = self.topology_reason
        self.values = dict.fromkeys(self.channels, 0.0)
        self.times = dict.fromkeys(self.channels)
        self.rejected_messages = 0

    def invalidate(self, reason):
        """Discard both channels, including any previously accepted nonzero value."""
        with self.lock:
            self.values = dict.fromkeys(self.channels, 0.0)
            self.times = dict.fromkeys(self.channels)
            self.reason = reason

    def set_publishers(self, throttle, steering):
        """Refresh graph ownership; changes require a new pair of commands."""
        with self.lock:
            topology = (tuple(throttle), tuple(steering))
            if topology == self.topology:
                return
            self.topology = topology
            self.owner = None
            self.writers = {}
            if len(throttle) != 1 or len(steering) != 1:
                self.topology_reason = 'need exactly one publisher per command topic'
            elif throttle[0].node != steering[0].node:
                self.topology_reason = 'throttle and steering publishers have different owners'
            elif self.required_owner and throttle[0].node != self.required_owner:
                self.topology_reason = 'publisher does not match configured command_owner'
            elif not throttle[0].gid or not steering[0].gid:
                self.topology_reason = 'publisher identity unavailable'
            else:
                self.owner = throttle[0].node
                self.writers = {'throttle': throttle[0].gid, 'steering': steering[0].gid}
                self.topology_reason = ''
            self.invalidate(self.topology_reason or 'waiting for fresh command pair')

    def _expire(self, now):
        if any(stamp is not None and now - stamp > self.timeout
               for stamp in self.times.values()):
            self.invalidate('command timeout')

    def receive(self, channel, value, gid=None, now=None, source_age=0.0):
        """Validate value/age; also check writer identity when supplied by middleware."""
        if channel not in self.channels:
            raise ValueError('unknown command channel')
        now = time.monotonic() if now is None else now
        with self.lock:
            self._expire(now)
            reason = None
            if self.owner is None:
                reason = self.topology_reason
            elif gid is not None and bytes(gid) != self.writers[channel]:
                reason = 'message writer does not match discovered owner'
            elif not math.isfinite(value) or abs(value) > 1.0:
                reason = 'command must be finite and inside [-1, 1]'
            elif not math.isfinite(source_age) or source_age < -0.05:
                reason = 'command source clock is invalid or ahead of bridge clock'
            elif source_age > self.timeout:
                reason = 'command already expired in transport'
            if reason:
                self.rejected_messages += 1
                self.invalidate(reason)
                return False
            self.values[channel] = round(float(value), 3)
            # Project source age onto the monotonic timeline. Transport delay
            # consumes the timeout; receiving a delayed packet does not renew it.
            self.times[channel] = now - max(0.0, source_age)
            return True

    def snapshot(self, now=None):
        """Return the only actuator pair allowed to leave the bridge right now."""
        now = time.monotonic() if now is None else now
        with self.lock:
            self._expire(now)
            enabled = self.owner is not None and all(t is not None for t in self.times.values())
            return {
                'enabled': enabled, 'owner': self.owner,
                'reason': 'fresh exclusive command pair' if enabled else self.reason,
                'throttle': self.values['throttle'] if enabled else 0.0,
                'steering': self.values['steering'] if enabled else 0.0,
                'command_age_s': {key: None if stamp is None else max(0.0, now - stamp)
                                  for key, stamp in self.times.items()},
                'timeout_s': self.timeout, 'rejected_messages': self.rejected_messages,
                'publisher_counts': (None if self.topology is None else {
                    'throttle': len(self.topology[0]), 'steering': len(self.topology[1])}),
            }
