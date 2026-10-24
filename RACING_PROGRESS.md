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
| 2 | Bridge command watchdog and exclusive command ownership | Yes | 16 core tests + ROS transport test; build/lint; 58 regressions total | Yes | Review live evidence; Feature 3 remains unstarted |
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

At the end of Feature 1, recording stopped and files were finalized. The agent's bridge in
`roboracer-feature1-live` remains running on host port 4567 so the user's simulator can
stay connected. No new watchdog, localization, or driving behavior was added in Feature 1.

### Remaining uncertainties / exact next step

Encoder scale, wrapping/gearing, actual sensor refresh rate, IMU acceleration convention,
and model-specific reset visibility are unverified. No localization, mapping, vehicle
model, speed control, or MPC behavior is implemented by this feature.

Feature 1 was accepted for the next iteration by the user's explicit request to implement
Feature 2 and reuse the existing gap follower for live tests.

## Feature 2: bridge command watchdog and exclusive controller

### Implemented behavior

The bridge now passes every outgoing throttle/steering pair through `CommandSafety`.
Both values default to zero. Nonzero commands require all of the following:

1. Exactly one ROS publisher exists on each driving-command topic.
2. Both publishers belong to the same fully qualified ROS node name.
3. That node matches `command_owner` if an owner was configured.
4. Both commands are finite, inside [-1, 1], and younger than `command_timeout`.
5. Both commands were sent after the latest publisher-ownership transition.

The default timeout is **0.25 seconds**. Discovering another publisher, losing a
publisher, changing endpoint identities, receiving invalid/old data, timing out, or
disconnecting Unity invalidates both channels. Recovery requires a fresh valid pair;
old steering cannot be combined with a new throttle after a fault.

`/autodrive/roboracer_1/command_status` publishes JSON in `std_msgs/String` at a nominal
20 Hz. It reports the resolved owner, publisher counts, reason, command ages, and the
currently allowed normalized output pair. State changes are logged in the bridge.
The check runs again immediately before each Socket.IO response to Unity, not only in
the ROS timer. Existing Float32 command topic names remain supported.

Parameters on `autodrive_bridge`:

- `command_timeout`: positive finite seconds, default 0.25.
- `command_owner`: empty for any unique common controller, or a fully qualified name
  such as `/wall_follow_node` to restrict the controller.

### Reused temporary driving algorithm

`feature2_gap_test.launch.py` starts one bridge and the existing `WallFollowNode` with
its original `gap_controller.compute_command`. It loads the retained YAML and overrides
only `max_throttle` to **0.12** by default. Normal controller defaults remain unchanged.
The bridge is configured for `/wall_follow_node`. Do not start another bridge with it.
The test launch shuts its group down if either controller or bridge exits.

The retained algorithm inflates LiDAR obstacles for clearance, chooses a broad/deep
opening, and uses a pure-pursuit steering geometry. For a target angle alpha at lookahead
distance ell and wheelbase L, its nominal steering angle is:

```text
delta = atan2(2 * L * sin(alpha), ell)
```

This follows from the curvature of a circle passing through the target,
`kappa = 2 * sin(alpha) / ell`, and the bicycle relation `tan(delta) = L * kappa`.
Existing sharp-turn gains and throttle reduction are reused. The 0.12 test cap is a
normalized actuator input, not a speed in m/s. This is temporary mapless test driving;
Feature 4's measured-speed mapping controller is not implemented here.

Only signal handling was adjusted in the wall-follow ROS wrapper: Ctrl+C/SIGTERM now
works from detached test processes and reaches its existing zero-command finalizer.
The numerical driving algorithm and retained tuning were not changed.

### Watchdog mathematics and failure behavior

Let T be the timeout, t the monotonic evaluation time, and t_c the effective source time
of the newest accepted command on channel c. Each channel's age is:

```text
age_c(t) = t - t_c
safe_pair = exclusive_owner AND age_throttle <= T AND age_steering <= T
output = latest_pair if safe_pair else (0, 0)
```

At receipt, DDS reports when the publisher sent its message. Compute transport age with
the system clock, then project that duration onto the monotonic clock:

```text
transport_age = system_now - DDS_source_timestamp
t_c = monotonic_receipt_time - max(0, transport_age)
```

Thus a message that spent 0.20 seconds queued has only 0.05 seconds left when T=0.25;
its arrival does not renew it for another full timeout. Nonfinite timestamps, commands
already older than T, or source clocks more than 0.05 seconds ahead are rejected. This
local Windows/Docker setup shares a system clock; distributed machines require synchronized
clocks before using this source-age policy. ROS simulated time is not used for expiry.

The follower normally publishes both axes every 0.05 seconds (20 Hz), so T permits about
five missed command periods. This is separate from its existing 0.5-second LiDAR freshness
check. A live follower with stale LiDAR publishes a fresh zero pair; a dead follower
publishes nothing, so the bridge independently expires its last pair.

Publisher graph changes are sampled by the timer, command callbacks, and final socket
check. Ownership enforcement is subject to ROS/DDS discovery latency. A discovered
competitor is rejected even if it has not sent a value. Endpoint GIDs distinguish a
restarted publisher from the previous instance even when its node name is unchanged.
Any ownership transition discards queued commands sent before that transition.

This pinned Humble rclpy discards metadata in normal callbacks. `CommandSafetyExecutor`
preserves the DDS timestamps from `take_message` only for the two command subscriptions;
all other subscriptions keep the normal executor behavior. The runtime does **not** expose
a per-message publisher GID, so source ownership is enforced through the graph rather
than claiming a per-message identity check. The ROS integration test exercises this
compatibility path on isolated, non-actuating topics. Recheck this small executor adapter
when upgrading rclpy.

Both the ROS executor and socket callback can access the command state. A reentrant lock
makes invalidation, acceptance, and snapshot evaluation consistent across those threads.
This provides one validated output pair; the legacy two Float32 messages are still not
an atomic combined-command protocol.

Zero throttle removes driving input; it does **not** guarantee zero velocity or active
braking. Unity applies the checked pair on the next bridge exchange. If incoming simulator
packets stop, the last checked input cannot be delivered until transport resumes. A dead
bridge or disconnected simulator is outside an in-bridge watchdog's ability to command.
Physical braking capability and stopping-distance guarantees remain later work.

### Feature 2 acceptance and evidence

- [x] 16 core tests: both-channel expiry, transport age, invalid input, owner mismatch,
  competing publishers, endpoint restart, disconnect invalidation, and fresh-pair recovery.
- [x] Real ROS test: DDS timing through the pinned Humble executor, exclusive publisher
  discovery, timeout, competing publisher without a value, removal, and fresh recovery.
- [x] Final package builds in an isolated Humble workspace; installed test launch imports
  and creates its six actions. Focused style checks pass. All **58 tests** pass in ROS:
  20 retained gap-controller tests, 20 diagnostic-core tests, one diagnostic ROS test,
  16 command-safety core tests, and one command-safety ROS test.
- [x] Reused gap follower moves and steers under the 0.12 normalized throttle cap.
- [x] SIGSTOP and SIGKILL tests cause zero output without controller shutdown callbacks.
- [x] Two competing command publishers block both outputs; fresh operation recovers
  after removing the competitor.
- [x] Telemetry, physical coasting, and feedback checked; logs and bag finalized.

### Feature 2 live evidence

Artifacts: `diagnostics/feature2_20261024T211445Z/`, including `phases.jsonl`,
`state_samples.jsonl`, `live_test_summary.json`, `bag_info.txt`, bridge/controller logs,
diagnostic JSONL, and the raw bag. The bag contains **52,228 messages** over approximately
462 seconds; the finalized diagnostic JSONL contains 453 records. All artifacts remain
in the Windows bind mount. These counts include setup and post-test stationary time.

The user reset the car and selected Autonomous. The agent then drove the retained
controller and injected process/publisher faults. Maximum observed throttle was 0.12,
maximum speed about 2.98 m/s, and collision count stayed **0** throughout the test.

| Case | Gate zero observed | Zero actuator feedback | Physical result |
|---|---|---|---|
| SIGSTOP controller | 0.241 s after pause | 0.291 s | Still coasting when resumed after 2 s |
| Competing publisher | Both outputs zero for all 61 detected-conflict samples | Observed zero during conflict | Fresh control recovered on removal |
| SIGKILL controller | 0.267 s after kill | 0.367 s | Speed below 0.05 m/s after 1.17 s |

Samples are collected at nominal 20 Hz. Displayed delays include sample/scheduler and
transport latency; they are measurements, not hard-real-time guarantees. The gate's
0.25-second rule remains independently enforced at actual socket emission. Two delayed
command samples after resuming were rejected rather than treated as new live commands.
This run validates command expiry and exclusive publisher behavior, not calibrated
braking, optimal speed, or whole-lap navigation performance.

After testing, diagnostics and rosbag were cleanly stopped and the controller was removed.
The final bridge code was restarted and reconnected in `roboracer-feature1-live` with
`command_timeout:=0.25` and `command_owner:=/wall_follow_node`. It remains connected on
port 4567 with no command publishers and zero permitted output. `gap_controller.py`
remains unchanged. See the runbook for the combined test launch and separate-process
crash-test procedure.

**Next action:** review completed Feature 2 evidence with the user. Feature 3 (local
odometry) remains unstarted.
