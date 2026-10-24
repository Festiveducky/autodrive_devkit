"""ROS-independent health measurements for passive RoboRacer diagnostics."""

from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid


STREAMS = {
    'lidar': 'scan',
    'imu': 'imu',
    'left_encoder': 'encoder',
    'right_encoder': 'encoder',
    'throttle': 'float',
    'steering': 'float',
    'lap_count': 'count',
    'lap_time': 'float',
    'last_lap_time': 'float',
    'best_lap_time': 'float',
    'collision_count': 'count',
}


def finite(value):
    """Return a JSON-safe number, preserving missing/nonfinite values as null."""
    return float(value) if value is not None and math.isfinite(value) else None


def vector(value):
    """Extract a geometry message's three components without importing ROS."""
    return [finite(value.x), finite(value.y), finite(value.z)]


def summarize(kind, msg):
    """Return (measurements, issues); do not infer unverified physical units."""
    issues = []
    if kind == 'scan':
        geometry = {
            name: finite(getattr(msg, name)) for name in
            ('angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max')
        }
        good_geometry = (
            all(v is not None for v in geometry.values())
            and msg.angle_increment > 0.0
            and 0.0 <= msg.range_min < msg.range_max
            and msg.angle_max > msg.angle_min
        )
        if not good_geometry:
            issues.append('invalid scan geometry')
        count = len(msg.ranges)
        if not count:
            issues.append('empty scan')
        elif good_geometry:
            last_angle = msg.angle_min + (count - 1) * msg.angle_increment
            if abs(last_angle - msg.angle_max) > 1.1 * msg.angle_increment:
                issues.append('beam count inconsistent with angular bounds')
        returns = [float(r) for r in msg.ranges if good_geometry
                   and math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        no_return = sum(r == math.inf for r in msg.ranges)
        invalid = count - len(returns) - no_return
        if invalid:
            issues.append('invalid range readings')
        return dict(
            geometry, beams=count, finite_returns=len(returns), no_return=no_return,
            invalid_ranges=invalid, nearest_m=min(returns) if returns else None,
            farthest_m=max(returns) if returns else None,
            advertised_scan_time_s=finite(msg.scan_time),
            advertised_time_increment_s=finite(msg.time_increment),
        ), issues
    if kind == 'encoder':
        positions = [finite(p) for p in msg.position]
        if len(positions) != 1 or any(p is None for p in positions):
            issues.append('expected one finite encoder position')
        return {'joint_names': list(msg.name), 'positions_raw': positions}, issues
    if kind == 'imu':
        q = msg.orientation
        quaternion = [finite(q.x), finite(q.y), finite(q.z), finite(q.w)]
        norm = finite(math.hypot(*quaternion)) if None not in quaternion else None
        result = {
            'orientation_xyzw': quaternion, 'quaternion_norm': norm,
            'angular_velocity_rad_s': vector(msg.angular_velocity),
            'linear_acceleration_m_s2': vector(msg.linear_acceleration),
        }
        if norm is None or abs(norm - 1.0) > 0.05:
            issues.append('orientation quaternion is not normalized')
        if any(v is None for field in ('angular_velocity_rad_s', 'linear_acceleration_m_s2')
               for v in result[field]):
            issues.append('nonfinite IMU measurement')
        if msg.orientation_covariance[0] == -1:
            issues.append('IMU orientation unavailable')
        if msg.angular_velocity_covariance[0] == -1:
            issues.append('IMU angular velocity unavailable')
        if msg.linear_acceleration_covariance[0] == -1:
            issues.append('IMU acceleration unavailable')
        return result, issues
    if kind == 'odom':
        # This is deliberately never an input to any estimate or command.
        pose = msg.pose.pose
        q = pose.orientation
        result = {
            'role': 'simulator_ground_truth_evaluation_only',
            'child_frame_id': msg.child_frame_id,
            'position_m': vector(pose.position),
            'orientation_xyzw': [finite(q.x), finite(q.y), finite(q.z), finite(q.w)],
            'linear_velocity_raw': vector(msg.twist.twist.linear),
        }
        if any(v is None for key in ('position_m', 'orientation_xyzw', 'linear_velocity_raw')
               for v in result[key]):
            issues.append('nonfinite ground truth')
        return result, issues
    value = finite(msg.data)
    if value is None:
        issues.append('nonfinite scalar')
    elif kind == 'count' and (value < 0 or value != int(value)):
        issues.append('invalid nonnegative counter')
    return {'value': value}, issues


class StreamHealth:
    """Measure arrival timing independently of ROS/header clock changes."""

    def __init__(self, stale_timeout, window_size=120):
        self.stale_timeout = stale_timeout
        self.arrivals = deque(maxlen=window_size)
        self.total_messages = 0
        self.invalid_messages = 0
        self.stamp_reversals = 0
        self.repeated_stamps = 0
        self.last_stamp_ns = None
        self.header_dt_s = None
        self.frame_id = None
        self.last_receipt = None
        self.measurements = {}
        self.issues = []
        self.encoder_baseline = None

    def observe(self, now, measurements, issues, stamp_ns=None, frame_id=None):
        """Update a stream and return timestamp/gap events."""
        events = []
        gap = self.last_receipt is not None and now - self.last_receipt > self.stale_timeout
        if gap:
            self.arrivals.clear()
            self.encoder_baseline = None
            events.append('receipt_gap')
        if stamp_ns is not None and self.last_stamp_ns is not None:
            self.header_dt_s = (stamp_ns - self.last_stamp_ns) / 1e9
            if stamp_ns < self.last_stamp_ns:
                self.stamp_reversals += 1
                self.encoder_baseline = None
                events.append('header_timestamp_reversal')
            elif stamp_ns == self.last_stamp_ns:
                self.repeated_stamps += 1
        else:
            self.header_dt_s = None
        self.arrivals.append(now)
        self.last_receipt = now
        self.last_stamp_ns = stamp_ns
        self.frame_id = frame_id
        self.total_messages += 1
        self.invalid_messages += bool(issues)
        self.measurements = dict(measurements)
        self.issues = list(issues)
        if 'positions_raw' in measurements:
            positions = measurements['positions_raw']
            position = positions[0] if len(positions) == 1 and not issues else None
            delta = rate = None
            if position is not None and self.encoder_baseline is not None:
                previous, previous_time = self.encoder_baseline
                dt = now - previous_time
                if dt > 0:
                    delta = position - previous
                    rate = delta / dt
            self.measurements.update(delta_raw=delta, rate_raw_per_s=rate)
            self.encoder_baseline = (position, now) if position is not None else None
        return events

    def snapshot(self, now, ros_now_ns=None):
        """Return health; retained measurements are not evidence of freshness."""
        age = None if self.last_receipt is None else max(0.0, now - self.last_receipt)
        status = 'missing' if age is None else (
            'stale' if age > self.stale_timeout else ('invalid' if self.issues else 'fresh'))
        span = self.arrivals[-1] - self.arrivals[0] if len(self.arrivals) > 1 else 0
        header_age = None
        if ros_now_ns is not None and self.last_stamp_ns is not None:
            header_age = (ros_now_ns - self.last_stamp_ns) / 1e9
        return {
            'status': status, 'receipt_age_s': age,
            'observed_publication_hz': (
                (len(self.arrivals) - 1) / span if span > 0 else None),
            'rate_window_messages': len(self.arrivals), 'total_messages': self.total_messages,
            'invalid_messages': self.invalid_messages,
            'header_stamp_ns': self.last_stamp_ns, 'header_dt_s': self.header_dt_s,
            'header_age_s': header_age,
            'header_timestamp_reversals': self.stamp_reversals,
            'repeated_header_stamps': self.repeated_stamps,
            'frame_id': self.frame_id, 'issues': self.issues,
            'measurements': dict(self.measurements),
        }


class Diagnostics:
    """Collect stream health and distinguish reset evidence from lap rollover."""

    def __init__(self, stale_timeout=0.5, include_ground_truth=False):
        if not math.isfinite(stale_timeout) or stale_timeout <= 0:
            raise ValueError('stale_timeout must be positive and finite')
        self.kinds = dict(STREAMS)
        if include_ground_truth:
            self.kinds['odom'] = 'odom'
        self.streams = {key: StreamHealth(stale_timeout) for key in self.kinds}
        self.events = []
        self.reset_candidates = 0
        self.previous_scalars = {}

    def observe(self, key, msg, now=None):
        """Inspect a message using monotonic receipt time, not its source clock."""
        now = time.monotonic() if now is None else now
        measurements, issues = summarize(self.kinds[key], msg)
        if key in ('last_lap_time', 'best_lap_time'):
            # Verified live: the simulator sends +inf until a lap is available.
            unavailable = msg.data == math.inf
            measurements['available'] = not unavailable and measurements['value'] is not None
            if unavailable:
                measurements['unavailable_reason'] = 'no_completed_lap'
                issues = []
        if key in ('lap_time', 'last_lap_time', 'best_lap_time'):
            if measurements['value'] is not None and measurements['value'] < 0:
                issues.append('negative lap duration')
        # Steering feedback's units are not assumed to match the normalized command.
        if key == 'throttle' and measurements['value'] is not None:
            if abs(measurements['value']) > 1.001:
                issues.append('normalized throttle feedback outside [-1, 1]')
        header = getattr(msg, 'header', None)
        stamp_ns = (int(header.stamp.sec) * 1000000000 + int(header.stamp.nanosec)
                    if header is not None else None)
        frame = header.frame_id if header is not None else None
        if header is not None and (not frame or stamp_ns == 0):
            issues.append('empty frame or zero header stamp')
        # Lap time naturally decreases every lap; it is not reset evidence alone.
        if key in ('lap_count', 'collision_count', 'last_lap_time') and not issues:
            value = measurements['value']
            previous = self.previous_scalars.get(key)
            reset = previous is not None and (
                value < previous if key != 'last_lap_time' else previous > 0 and value == 0)
            if reset:
                self.reset_candidates += 1
                self.events.append({'stream': key, 'event': 'simulator_reset_candidate',
                                    'previous': previous, 'current': value,
                                    'receipt_monotonic_s': now})
                for stream in self.streams.values():
                    stream.encoder_baseline = None
                # Prevent successive counters from counting the same reset repeatedly.
                self.previous_scalars.clear()
            self.previous_scalars[key] = value
        events = self.streams[key].observe(now, measurements, issues, stamp_ns, frame)
        self.events.extend({'stream': key, 'event': event, 'receipt_monotonic_s': now}
                           for event in events)

    def snapshot(self, now=None, ros_now_ns=None):
        """Take one report and drain only its pending events."""
        now = time.monotonic() if now is None else now
        events, self.events = self.events, []
        return {
            'receipt_monotonic_s': now, 'reset_candidates': self.reset_candidates,
            'events': events,
            'streams': {key: stream.snapshot(now, ros_now_ns)
                        for key, stream in self.streams.items()},
        }


class JsonlRecorder:
    """Flush each diagnostic report to a uniquely named strict JSONL file."""

    def __init__(self, directory, metadata):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.path = directory / f'sensor_diagnostics_{stamp}_{uuid.uuid4().hex[:8]}.jsonl'
        self.file = self.path.open('x', encoding='utf-8')
        self.write('session_start', metadata)

    def write(self, record_type, payload):
        """Write one complete line; fail visibly on bad data or disk errors."""
        record = {'schema_version': 1, 'record_type': record_type,
                  'utc': datetime.now(timezone.utc).isoformat(), **payload}
        self.file.write(json.dumps(record, allow_nan=False) + '\n')
        self.file.flush()

    def close(self):
        """Close idempotently; complete prior records survive an interrupted run."""
        if not self.file.closed:
            try:
                self.write('session_end', {})
            finally:
                self.file.close()
