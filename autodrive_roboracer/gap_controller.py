"""Small, ROS-independent longest-clear-path controller (distances in meters)."""

from dataclasses import dataclass
import math


@dataclass
class ControllerConfig:
    """Simulator tuning; clearance includes the car half-width."""

    clearance: float = 0.35
    stop_distance: float = 0.45
    max_throttle: float = 0.20
    throttle_gain: float = 0.08
    turn_slowdown: float = 0.40
    sharp_turn_throttle: float = 0.14
    steering_gain: float = 1.0
    min_lookahead: float = 0.6
    max_lookahead: float = 1.5
    lookahead_gain: float = 0.5
    sharp_turn_angle: float = 35.0
    sharp_turn_full_angle: float = 70.0
    sharp_turn_steering_gain: float = 1.35
    sharp_turn_lookahead_scale: float = 0.60
    forward_angle: float = 80.0
    wheelbase: float = 0.33
    max_steering_angle: float = 0.52

    def __post_init__(self):
        for name, value in vars(self).items():
            if (not math.isfinite(value) or value < 0.0
                    or (value == 0.0 and name != 'turn_slowdown')):
                raise ValueError(f'{name} must be finite and positive (turn_slowdown may be zero)')
        if self.max_throttle > 1.0 or self.forward_angle > 90.0:
            raise ValueError('max_throttle must be <= 1 and forward_angle <= 90')
        if self.turn_slowdown >= 1.0 or self.sharp_turn_throttle > 1.0:
            raise ValueError('turn_slowdown must be < 1 and sharp_turn_throttle <= 1')
        if self.min_lookahead > self.max_lookahead:
            raise ValueError('min_lookahead must be <= max_lookahead')
        if not self.sharp_turn_angle < self.sharp_turn_full_angle <= 90.0:
            raise ValueError('sharp turn angles must satisfy 0 < start < full <= 90')
        if self.sharp_turn_steering_gain < 1.0 or self.sharp_turn_lookahead_scale > 1.0:
            raise ValueError('sharp turn steering gain must be >= 1 and lookahead scale <= 1')


@dataclass
class DriveCommand:
    """Normalized actuator commands plus information for terminal debugging."""

    throttle: float = 0.0
    steering: float = 0.0
    target_angle: float = 0.0
    target_distance: float = 0.0
    front_clearance: float = 0.0
    reason: str = 'waiting for scan'
    turn_strength: float = 0.0


def sharp_turn_strength(angle, config):
    """Blend smoothly from normal to aggressive steering as the target turns."""
    degrees = abs(math.degrees(angle))
    span = config.sharp_turn_full_angle - config.sharp_turn_angle
    return max(0.0, min(1.0, (degrees - config.sharp_turn_angle) / span))


def calculate_steering(angle, distance, config):
    """Use shorter lookahead and more steering gain for a sharp corner."""
    strength = sharp_turn_strength(angle, config)
    lookahead = max(config.min_lookahead,
                    min(config.max_lookahead, distance * config.lookahead_gain))
    lookahead *= 1.0 - strength * (1.0 - config.sharp_turn_lookahead_scale)
    gain = config.steering_gain * (1.0 + strength * (config.sharp_turn_steering_gain - 1.0))
    steering = gain * math.atan2(2 * config.wheelbase * math.sin(angle), lookahead)
    return steering / config.max_steering_angle, strength


def calculate_throttle(distance, front, steering, strength, config):
    """Keep more corner speed while preserving distance-based slowing/stops."""
    throttle = min(config.max_throttle,
                   config.throttle_gain * max(0.0, min(distance, front) - config.stop_distance))
    throttle *= 1.0 - config.turn_slowdown * abs(steering)
    # A separate corner cap can be tuned without reducing straight-line speed.
    corner_cap = min(config.max_throttle, config.sharp_turn_throttle)
    cap = config.max_throttle + strength * (corner_cap - config.max_throttle)
    return min(throttle, cap)


def sanitize_ranges(ranges, range_min, range_max):
    """Positive infinity means no hit; unknown/too-close returns stay blocked."""
    return [range_max if r == math.inf else
            min(r, range_max) if math.isfinite(r) and r >= range_min else 0.0
            for r in ranges]


def inflate_obstacles(ranges, angle_increment, clearance):
    """Spread each return sideways so a single long ray cannot skim a wall."""
    inflated = list(ranges)
    for index, distance in enumerate(ranges):
        # Unknown beams block a small neighborhood, not the entire scan.
        radius = (math.asin(min(1.0, clearance / distance))
                  if distance > 0.0 else math.radians(3.0))
        width = math.ceil(radius / angle_increment)
        for neighbor in range(max(0, index - width), min(len(ranges), index + width + 1)):
            inflated[neighbor] = min(inflated[neighbor], distance)
    return inflated


def choose_target(angles, ranges, previous_angle):
    """Choose a broad, deep opening, preferring forward travel and continuity."""
    scores = [distance * (0.75 + 0.25 * math.cos(angle))
              for angle, distance in zip(angles, ranges)]
    best = max(scores)
    # Group near-equal rays; aim at the center of a plateau rather than its edge.
    groups = []
    start = None
    for index in range(len(scores) + 1):
        good = index < len(scores) and scores[index] >= 0.95 * best
        if good and start is None:
            start = index
        elif not good and start is not None:
            groups.append((start, index - 1))
            start = None
    centers = [(first + last) // 2 for first, last in groups]
    return min(centers, key=lambda i: abs(angles[i] - previous_angle))


def compute_command(scan, config, previous_angle=0.0):
    """Turn a LaserScan-like object into a command without ROS dependencies."""
    geometry = (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
    if (not scan.ranges or not all(math.isfinite(v) for v in geometry)
            or scan.angle_increment <= 0.0 or scan.range_min < 0.0
            or scan.range_max <= scan.range_min):
        return DriveCommand(reason='invalid scan geometry')

    angles = [scan.angle_min + i * scan.angle_increment for i in range(len(scan.ranges))]
    if angles[0] > -math.pi / 2 or angles[-1] < math.pi / 2:
        return DriveCommand(reason='scan must cover right, front, and left')
    ranges = sanitize_ranges(scan.ranges, scan.range_min, scan.range_max)
    # A narrow sector stops for frontal obstacles; side walls do not count as
    # frontal obstacles just because their radial distance is small.
    front = min((r for a, r in zip(angles, ranges) if abs(a) <= math.radians(10)), default=0.0)
    if front <= config.stop_distance:
        return DriveCommand(front_clearance=front, reason='front blocked or unknown')

    # Use the full scan for inflation so obstacles just outside the steering
    # field still shrink unsafe paths at its edges.
    inflated = inflate_obstacles(ranges, scan.angle_increment, config.clearance)
    indices = [i for i, a in enumerate(angles) if abs(a) <= math.radians(config.forward_angle)]
    target = choose_target([angles[i] for i in indices],
                           [inflated[i] for i in indices], previous_angle)
    index = indices[target]
    angle, distance = angles[index], inflated[index]
    if distance <= config.stop_distance:
        return DriveCommand(front_clearance=front, reason='no clear path')

    steering, strength = calculate_steering(angle, distance, config)
    # Push away from a nearby side wall. The rear half of the car also needs
    # room when steering toward that wall.
    left = min((r for a, r in zip(angles, ranges)
                if math.radians(60) <= a <= math.pi / 2), default=0.0)
    right = min((r for a, r in zip(angles, ranges)
                 if -math.pi / 2 <= a <= math.radians(-60)), default=0.0)
    steering += (max(0.0, config.clearance - right)
                 - max(0.0, config.clearance - left)) / config.clearance
    steering = max(-1.0, min(1.0, steering))
    if min(left, right) < 0.15:
        return DriveCommand(front_clearance=front, reason='side dangerously close or unknown')

    throttle = calculate_throttle(distance, front, steering, strength, config)
    return DriveCommand(throttle, steering, angle, distance, front, 'driving', strength)
