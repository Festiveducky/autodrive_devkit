# Racing implementation progress

Last updated: 2026-10-24. Read this file and `SIMULATOR_RUNBOOK.md` before continuing.

## Working agreement and architecture

Implement one testable feature per iteration. After implementation, automated checks,
and a conceptual/math explanation, stop for review of simulator results. Do not advance
just because code builds. Preserve unrelated working-tree changes and the existing
controller while its replacement is being developed.

Selected architecture: **sensor-based LiDAR SLAM -> minimum-curvature geometric path ->
learned feasible speed profile -> dynamic nonlinear MPC**. Simulator world pose is for
evaluation, not localization. Keep corridor, geometric path, and timed trajectory
interfaces separate so a later late-apex/minimum-time planner can replace the geometry
stage. Minimum curvature is a geometric objective, not proof of minimum lap time.

| # | Feature | Implemented | Automated checks | Simulator validated | Gate / next evidence |
|---|---|---|---|---|---|
| 1 | Passive sensor diagnostics and recording | Yes | 20 numerical tests + 1 ROS transport test; package build; focused lint | Yes, with user's Windows simulator/manual driving | Review evidence below; do not start Feature 2 until review |
| 2 | Bridge command watchdog and exclusive command ownership | No | Pending | Pending | Expired commands cannot retain throttle |
| 3 | Encoder/LiDAR/IMU local odometry and TF ownership | No | Pending | Pending | Correct scale/signs, scan degeneracy handling, one TF parent |
| 4 | Slow measured-speed mapping controller | No | Pending | Pending | Low-speed tracking and stopping, initial acceleration/coasting data |
| 5 | SLAM Toolbox mapping, saving, localization | No | Pending | Pending | Closed consistent map and relocalization |
| 6 | Ordered track corridor and footprint clearance | No | Pending | Pending | Correct topology and boundaries on map overlay |
| 7 | Iterative-QP minimum-curvature path | No | Pending | Pending | Better curvature objective; valid continuous footprint and seam |
| 8 | Actuator and dynamic bicycle/tire identification | No | Pending | Pending | Held-out prediction improvement; established braking semantics |
| 9 | Periodic velocity planning | No | Pending | Pending | Curvature/grip limits and braking before corners |
| 10 | Dynamic NMPC with CasADi/IPOPT | No | Pending | Pending | Low-speed tracking, timing, failure handling, then clean faster laps |
| 11 | Automatic stage gates and progressive learning | No | Pending | Pending | Evidence-based transitions and rollback to validated pace |

## Feature 1: what is implemented

`sensor_diagnostics_node` is a passive subscriber with no actuator publishers. It observes
LiDAR, IMU, left/right encoders, throttle/steering feedback, lap count, current/last/best
lap time, and collision count. Optional `include_ground_truth:=true` adds simulator
odometry explicitly labeled `simulator_ground_truth_evaluation_only`.

The ROS-independent measurement core reports per-stream publication frequency, receipt
age, header stamp/delta/age, frame ID, cumulative invalid-message count, repeated stamps,
and timestamp reversals. It also reports scan geometry and range categories, raw encoder
position/change/rate, IMU quaternion/angular velocity/acceleration, and scalar telemetry.

Every report is flushed as strict JSONL; nonfinite measurements become `null`, not
nonstandard JSON `NaN`/`Infinity`. LiDAR no-returns have their own count. Logs have unique
UTC filenames, schema version, a session-start record, periodic diagnostic reports, and a
session-end record on clean shutdown. Startup/file-write failures are visible errors.
The launch default writes into the host-mounted source tree under `diagnostics/`, which
is ignored by Git. Raw scans are recorded separately with rosbag2; JSONL is a diagnostic
summary, not a lossless replay format.

Live simulator discovery: `last_lap_time` and `best_lap_time` publish positive infinity
before a completed lap exists. These expected sentinels now appear as `value: null`,
`available: false`, `unavailable_reason: no_completed_lap`, with fresh stream health.
NaN, negative durations, and other unexpected nonfinite scalars remain invalid.

### Concepts and mathematics

**Why inspect sensors first?** A mapping error can come from wrong units, missing messages,
bad frame IDs, clock behavior, or geometry. A more advanced planner cannot correct those
input errors. This feature measures the inputs before trusting them as motion estimates.

**Observed rate.** For N callback receipts at monotonic times t_1 through t_N:

```text
f_observed = (N - 1) / (t_N - t_1)
```

There are N-1 intervals between N messages. For receipts at 0.0, 0.1, ..., 0.4 seconds,
the result is 4 / 0.4 = 10 Hz. Use the most recent 120 arrivals, with fewer during startup.
One receipt or a zero-duration window gives `null`, not an invented rate. After a gap
longer than the stale timeout, discard the old rate window so recovery does not mix old
and new timing. The rate of a stale stream remains a historical rate, not current flow.

The bridge publishes sensor messages when it processes simulator packets. Its publication
frequency can differ from LiDAR's advertised `1 / scan_time`; repeated publication does
not prove that physical sensor data was independently refreshed. Stationary equal values
also do not prove a sensor is frozen.

**Freshness uses a monotonic clock:**

```text
receipt_age = monotonic_now - last_receipt
```

A monotonic clock measures elapsed time without wall-clock corrections. Status is
`missing` before any receipt, `stale` after the timeout (default 0.5 s), `invalid` for a
fresh message with validity issues, otherwise `fresh`. Receipt-based freshness does
not establish measurement acquisition time or physical correctness.

**Header timing uses the ROS clock separately:**

```text
header_dt  = (current_header_ns - previous_header_ns) / 1e9
header_age = (ros_now_ns - header_ns) / 1e9
```

Do not subtract a ROS stamp from a monotonic receipt timestamp: they have different
origins. A negative header delta is a reversal event; equal stamps are counted. Header
age assumes a common ROS time domain and can be negative when clocks disagree. In the
current bridge stamps are created on receipt using the ROS clock, not an original
simulator acquisition timestamp. Scalar `Float32`/`Int32` messages have no header, so
their header fields are `null`. Feature 1 does not change bridge timing or TF behavior.

**LiDAR validation.** Beam i should have angle `angle_min + i * angle_increment`.
The last beam should agree with `angle_max` to within about one beam; this tolerates
the bridge's rounded angular metadata and both 1080/1081 beam layouts. Finite readings
inside `[range_min, range_max]` count as returns. Positive infinity means no return
inside range. NaN, negative infinity, and out-of-range finite values are invalid evidence.
An all-infinity scan is not automatically malformed, and infinity is not a wall at 10 m.

**Encoder differences.** This feature computes only:

```text
delta_raw = current_position_raw - previous_position_raw
rate_raw_per_s = delta_raw / receipt_dt
```

Once wheel angle units and any gearing are verified, a later odometry feature can use:

```text
v ~= R * (delta_theta_left + delta_theta_right) / (2 * delta_t)
```

Each wheel travels arc length `R * delta_theta`; the axle's forward displacement is
approximately the average of the left/right distances. Multiplying encoder angle by
the wrong gear ratio directly scales the estimated distance. This node deliberately
does not convert raw values to m/s, unwrap angles, infer gearing, or silently assume a
wheel radius. Invalid samples, stale gaps, header reversals, and reset evidence break
the derivative baseline; the next delta is `null` rather than a false speed spike.

**IMU validity.** A normalized orientation quaternion has
`sqrt(qx^2 + qy^2 + qz^2 + qw^2) ~= 1`; deviations greater than 0.05 are flagged.
The node reports finite angular velocity and acceleration and honors the ROS IMU
unavailable-field covariance marker `-1`. Acceleration is raw: gravity, sensor axes,
and mounting offsets need investigation before interpreting it as tire acceleration.

**Reset evidence is not a reset oracle.** Decreasing lap/collision counters or a
previously nonzero last-lap time becoming zero generate `simulator_reset_candidate`.
Normal current-lap-time rollover does not. A header reversal is recorded independently.
The bridge uses wall-clock stamps, so a simulator reset need not reverse those stamps.
A reset at an already-zero stationary state may be unobservable from these topics.
Candidates can also represent out-of-order packets; they are evidence to inspect, not
confirmed reset counts. Several signals belonging to one reset are coalesced where possible.

### Automated acceptance and evidence

- [x] 20 ROS-independent tests pass on host/container, including live-discovered lap sentinels.
- [x] Existing 20 gap-controller tests pass on host Python.
- [x] Real ROS synthetic transport test passes: all 11 streams received, fresh-to-stale
  transition, JSONL finalization, and no actuator-topic publishers owned by this node.
- [x] Package builds in an isolated ROS 2 Humble workspace; new console entry is registered.
- [x] rosbag2 recording command options inspected in the installed Humble runtime.
- [x] Full simulator acceptance below. Runtime experiments and their precise outcome are
  recorded in `SIMULATOR_RUNBOOK.md`; process startup alone is not a pass.
- [x] Final code builds in an isolated Humble workspace; focused flake8 checks pass.

### Simulator acceptance checklist

Follow the runbook; collect one JSONL and preferably one bag covering the sequence.

- [x] Stationary: all expected streams arrive from the Windows simulator after manual
  Connect. Initial measurements: about 17 Hz publication, 1081 LiDAR beams, 1035 finite
  returns, 46 no-returns, zero invalid ranges, zero actuator feedback. Rates are observed
  transport rates, not assumed sensor refresh rates. Lap-time infinity sentinels handled.
- [x] Manual straight: both encoder positions increase during forward motion. Raw scale
  remains unverified; the manual run also includes collisions and faster driving.
- [x] Manual left/right turns: IMU yaw rates of both signs were recorded; examples
  -1.3522 and +0.9467 rad/s. These are sensor checks, not calibrated dynamics data.
- [x] Stationary disconnect: agent restarted its own bridge; reports became stale.
  User pressed Connect again, and streams recovered. No actuation commands were sent.
- [x] User reset after motion/history: after reconnect, lap count changed from 5 to 0;
  collision count and encoders returned to zero. One reset candidate was recorded.
  Encoder derivative baselines were reset rather than reporting a discontinuity as speed.
- [x] SIGINT finalizes readable JSONL with `session_end`; live bag metadata is readable.
- [x] Persistence verified from Windows after removing the isolated test container:
  synthetic JSONL and 195-message bag survived. The live 70,642-message bag was also
  verified from Windows; its bridge container is deliberately left running.

### Feature 1 completion evidence

Live files: `diagnostics/windows_live_20261024T205316Z/`. The raw bag contains **70,642
messages across 13 topics**, including 5,434 messages on each sensor/telemetry stream
and `/tf`, over about 488 seconds. `/tf_static` was requested but had no publisher in
this bridge; sensor mounting transforms are currently broadcast on `/tf`.

The initial JSONL has 490 records and includes stationary, manual motion, disconnect,
recovery, and reset evidence. It predates the lap-sentinel fix. The separate
`corrected_diagnostics/` log has 7 records and verifies every stream fresh after reset,
with lap-time results explicitly unavailable. Both logs end cleanly. The raw bag retains
the original Float32 infinity values for inspection. `bag_info.txt` and
`evidence_summary.json` are stored alongside it.

Observed incoming gaps exceeded 0.5 seconds at times, causing legitimate receipt-gap
events and occasionally stale health. Publication frequency is therefore not guaranteed
40 Hz even though LiDAR advertises a 0.025 s scan time. Investigate timing/focus/transport
before choosing future control watchdogs; do not hide this by labeling all scans fresh.

Recording has stopped and files are finalized. The agent's bridge in
`roboracer-feature1-live` remains running on host port 4567 so the user's simulator can
stay connected. No new watchdog, localization, or driving behavior was added.

### Remaining uncertainties / exact next step

Encoder scale, wrapping/gearing, actual sensor refresh rate, IMU acceleration convention,
and model-specific reset visibility are unverified. No localization, mapping, vehicle
model, speed control, or MPC behavior is implemented by this feature.

**Next action: review the completed Feature 1 simulator evidence above.** Only after
its acceptance review, implement Feature 2 (bridge command
watchdog). For each subsequent feature add its own derivation, failure modes, test
commands, evidence, and review gate here.
