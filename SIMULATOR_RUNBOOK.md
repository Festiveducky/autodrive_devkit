# Simulator and diagnostics runbook

Last updated: 2026-10-24. This is the reproducible testing reference for future sessions.
Read `RACING_PROGRESS.md` for the current feature and acceptance gate. Features 1 and 2
are implemented and live-tested. Later features remain unstarted. The existing numerical
gap-controller behavior is retained.

## Main workflow: user's Windows simulator and penn-challenge container

The user supplied the commands in this section. Their normal Windows simulator workflow
remains the primary manual acceptance route. The exact Windows executable location is
was verified from the running process as:
`C:\Users\alexr\Downloads\autodrive_simulator_practice_windows\autodrive_simulator\AutoDRIVE Simulator.exe`.
The checked-in executable is Linux-only.

### 1. Start Docker Desktop and the Windows simulator

From PowerShell in the directory containing the Windows executable:

```powershell
# User's command: "AutoDRIVE Simulator.exe" -force-d3d11
& '.\AutoDRIVE Simulator.exe' -force-d3d11
```

Keep the simulator in Manual mode for Feature 1. Connect to the bridge at port 4567 once
the bridge starts: **press the simulator's Connect button manually**. Starting the bridge
or selecting Autonomous mode does not perform this step. On 2026-10-24 the user confirmed
the status changed from Disconnected to Connected only after pressing Connect.
Use IP `127.0.0.1` and port `4567` for this local Windows/Docker port-forwarded setup.
Use the simulator's manual controls for the slow straight/turn tests.
Do not run `wall_follow_node` for these passive/manual acceptance checks.

Status: Windows process and live bridge telemetry verified after the user's manual connection.

### 2. Create the devkit container, build, and start the bridge

From PowerShell at the repository root, use the user's command:

```powershell
docker run --platform linux/amd64 --name penn-challenge --rm -it -p 4567:4567 -v "${PWD}/autodrive_devkit:/home/autodrive_devkit/src/autodrive_devkit" --entrypoint /bin/bash penn-challenge
```

Inside that container:

```bash
cd /home/autodrive_devkit
source /opt/ros/humble/setup.bash
colcon build --packages-select autodrive_roboracer
source install/setup.bash
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

Build after source edits: an old install can otherwise omit the new executable/launch.
`bringup_headless.launch.py` starts the ROS bridge, not the simulator executable.

Status: package build and bridge startup verified in an isolated container workspace.
Port-forwarded Windows-simulator transport remains part of manual acceptance.

### 3. Run the new passive diagnostics in a second terminal

```powershell
docker exec -it penn-challenge bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 launch autodrive_roboracer sensor_diagnostics.launch.py include_ground_truth:=true
```

The launch writes to `/home/autodrive_devkit/src/autodrive_devkit/diagnostics`, visible on
Windows as `autodrive_devkit/diagnostics`. `include_ground_truth` is optional and defaults
to false. All optional odometry records are labeled evaluation-only.

Other supported launch arguments:

```bash
ros2 launch autodrive_roboracer sensor_diagnostics.launch.py \
  output_dir:=/home/autodrive_devkit/src/autodrive_devkit/diagnostics/manual_run \
  report_period:=1.0 stale_timeout:=0.5 \
  topic_prefix:=/autodrive/roboracer_1
```

Direct execution also works, but its default is `~/roboracer_diagnostics`, which is not
inside the Windows bind mount. Always specify a persistent directory when running directly:

```bash
ros2 run autodrive_roboracer sensor_diagnostics_node --ros-args \
  -p output_dir:=/home/autodrive_devkit/src/autodrive_devkit/diagnostics
```

Expect initially `missing`, then `fresh` for valid arriving streams, or `invalid` with
issues. The next line summarizes LiDAR, raw encoder changes, IMU readings, and feedback.
JSONL includes full frame/header timing information. `None`/JSON `null` means not yet
available or deliberately not inferred. Diagnostics use best-effort sensor-data QoS,
compatible with the current reliable bridge and best-effort publishers.

Status: actual ROS synthetic message transport, staleness, recording, and no actuator
publishers verified. See the runtime results section for simulator-generated data.

### 4. Optionally record raw messages in a third terminal

Open another `docker exec -it penn-challenge bash`, then:

```bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
bag_path=/home/autodrive_devkit/src/autodrive_devkit/diagnostics/bag_$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p /home/autodrive_devkit/src/autodrive_devkit/diagnostics
ros2 bag record -o "$bag_path" \
  /autodrive/roboracer_1/lidar \
  /autodrive/roboracer_1/imu \
  /autodrive/roboracer_1/left_encoder \
  /autodrive/roboracer_1/right_encoder \
  /autodrive/roboracer_1/throttle \
  /autodrive/roboracer_1/steering \
  /autodrive/roboracer_1/lap_count \
  /autodrive/roboracer_1/lap_time \
  /autodrive/roboracer_1/last_lap_time \
  /autodrive/roboracer_1/best_lap_time \
  /autodrive/roboracer_1/collision_count \
  /autodrive/roboracer_1/odom /tf /tf_static
```

Do not use `--use-sim-time`: this bridge does not publish `/clock`. The bag preserves raw
messages and their timing for later alignment/replay; recording does not magically
synchronize acquisition times. JSONL is a periodic summary and cannot replace the bag.

Press Ctrl+C in the recorder, then inspect:

```bash
ros2 bag info "$bag_path"
```

Status: installed Humble rosbag2 CLI/options verified; raw recording still needs an
actual recording result before marking its end-to-end acceptance complete.

### 5. Manual test sequence and expected outcomes

1. Record 10 seconds stationary: all expected streams have nonzero message counts;
   note measured rates, frame IDs, and any validity issues.
2. Drive slowly straight, then make slow left and right turns. Compare left/right raw
   encoder changes and IMU Z angular velocity; annotate which maneuver occurred when.
3. Stop the car, then stop the bridge. Within approximately `stale_timeout + report_period`,
   existing streams should report `stale`. Restart the bridge and reconnect the simulator;
   the first returning message starts a new rate window.
4. After some motion/history, reset the simulator. Inspect reset-candidate and timestamp
   events. Zero-history resets may not be observable; wall-clock header timestamps need
   not reverse. Do not substitute normal lap-time rollover for reset evidence.
5. Stop diagnostics and rosbag with Ctrl+C. The diagnostic JSONL should end with
   `session_end`. Check bag metadata using `ros2 bag info`.
6. Stop the simulator and bridge before exiting the container shell. Because the container
   uses `--rm`, verify that all desired data is already in the bind-mounted directory.

From Windows, validate the newest log after shutdown/container removal:

```powershell
$log = Get-ChildItem autodrive_devkit/diagnostics -Filter 'sensor_diagnostics_*.jsonl' -Recurse |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
Get-Content -LiteralPath $log.FullName | ForEach-Object { $_ | ConvertFrom-Json } |
    Select-Object -Last 1
```

Expected final record type: `session_end`. Preserve the log and bag paths plus observations
for Feature 1 review. No future driving features should be implemented before that review.

### Existing autonomous controller command, retained for later reference

These are the user's existing commands; they are not the Feature 1 manual test:

```powershell
docker exec -it penn-challenge bash
```

```bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 run autodrive_roboracer wall_follow_node
```

## Automated checks

Host PowerShell, at repository root:

```powershell
$env:PYTHONPATH = (Resolve-Path autodrive_devkit).Path
python -B -m unittest discover -s autodrive_devkit/test -p test_gap_controller.py -v
python -B -m unittest discover -s autodrive_devkit/test -p 'test_sensor_diagnostics*.py' -v
```

Expected: 20 existing tests and 20 diagnostic-core tests pass; the ROS integration test
skips on a host without `rclpy`. In a sourced ROS container, run the same discovery with
`python3`; the transport test should pass rather than skip:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/autodrive_devkit/src/autodrive_devkit:$PYTHONPATH
python3 -B -m unittest discover \
  -s /home/autodrive_devkit/src/autodrive_devkit/test \
  -p 'test_sensor_diagnostics*.py' -v
```

Preserve `$PYTHONPATH` after sourcing ROS; overwriting it hides `rclpy` and can cause a
misleading skip. The integration test publishes synthetic sensors only in the isolated
`/feature1_transport_test` namespace and never publishes actuator commands.

## Agent runtime experiments and troubleshooting

Verified on 2026-10-24:

- Docker CLI initially failed because `dockerDesktopLinuxEngine` did not exist.
- Starting Docker Desktop restored the engine; `penn-challenge:latest` was already local.
- WSL Ubuntu starts, but did not provide ROS or Xvfb commands on PATH. The existing Docker
  image provides ROS 2 Humble, Python 3.10, Xvfb, and Mesa OpenGL libraries, but no working
  Vulkan rendering backend was found for this Unity build.
- Built in `/tmp/feature1_ws` after copying the mounted devkit there, avoiding changes to
  the user's installed container workspace.
- A temporary container named `roboracer-feature1-check` uses source and read-only
  simulator bind mounts. Runtime logs go to the persistent `diagnostics/` directory.

Docker Desktop startup that worked from this machine's PowerShell:

```powershell
Start-Process -FilePath 'C:/Users/alexr/AppData/Local/Programs/DockerDesktop/Docker Desktop.exe' -WindowStyle Hidden
docker version
```

For agent shell scripts, piping a PowerShell here-string into `docker exec -i ... bash`
avoids nested Windows/bash quote loss. Finish the script with a comment line: PowerShell's
trailing CR then affects the comment instead of the last command.

The first no-graphics attempt ran the Linux executable with:

```bash
timeout --kill-after=3 35 './AutoDRIVE Simulator.x86_64' \
  -batchmode -nographics -ip 127.0.0.1 -port 4567 -logFile simulator.log
```

It initialized Unity, emitted unsupported shader errors, and yielded no observed sensor
messages. A plain `timeout` without `--kill-after` was insufficient because the simulator
did not exit promptly on TERM. This is a failed telemetry check, not a working simulation.

Additional Linux experiments:

- Xvfb with `-batchmode -force-glcore`: Unity reported OpenGL Core was not built into
  the player, graphics initialization failed, and it segfaulted. No telemetry.
- Xvfb with `-batchmode` and default renderer: Unity reported `Vulkan detection: 0`
  and `No supported renderers found, exiting`. No telemetry.
- These failures do not apply to the user's working Windows/D3D11 simulator.
- Synthetic ROS recording succeeded separately: 195 messages (97 LiDAR, 98 IMU) in
  `diagnostics/synthetic_full_20261024T205102Z/bag`. The first shorter bag attempt contained zero messages because
  publishing stopped before recorder discovery; it is not accepted as recording evidence.
- The installed launch was checked with `timeout --signal=INT --kill-after=3 4 ros2 launch
  ...`; diagnostics finalized JSONL with `session_end`. Timeout exit 124 is expected for
  that deliberately bounded test.

## Verified Windows live session

The user started Unity with `-force-d3d11`, selected Autonomous mode, and initially saw
Disconnected. No Windows listener existed on port 4567: the first isolated test container
had not published that port. The agent started a separate container with `-p 4567:4567`.
The user manually pressed **Connect**, after which the bridge logged `Connected!` and
real sensor messages arrived. **Always include this manual Connect step.**

The known-working temporary live setup was created from repository-root PowerShell:

```powershell
docker run -d --platform linux/amd64 --name roboracer-feature1-live -p 4567:4567 `
  --mount "type=bind,source=${PWD}/autodrive_devkit,target=/home/autodrive_devkit/src/autodrive_devkit" `
  --entrypoint /bin/bash penn-challenge -lc 'sleep infinity'
```

Inside this container, direct module execution was used to avoid relying on an older
installed package. Each long-running command goes in its own `docker exec -it ... bash`:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/autodrive_devkit/src/autodrive_devkit:$PYTHONPATH
python3 -u -m autodrive_roboracer.autodrive_bridge
```

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/autodrive_devkit/src/autodrive_devkit:$PYTHONPATH
python3 -u -m autodrive_roboracer.sensor_diagnostics_node --ros-args \
  -p output_dir:=/home/autodrive_devkit/src/autodrive_devkit/diagnostics/manual_live \
  -p include_ground_truth:=true
```

Use the earlier rosbag command in another terminal. Prefer the main build/launch workflow
for normal ongoing use. Before creating any port-published container, inspect `docker ps`:
`penn-challenge` and `roboracer-feature1-live` cannot both publish host port 4567.

Live evidence is in `diagnostics/windows_live_20261024T205316Z/`:

- Initial publication rates about 17 Hz; LiDAR has 1081 beams with 1035 finite returns,
  46 positive-infinity no-returns, and zero invalid ranges at the initial position.
- Stationary feedback: throttle/steering zero; both encoder changes zero.
- Before any completed lap, `last_lap_time` and `best_lap_time` are positive infinity.
  The code now represents these as unavailable lap results, not corrupt scalar data.
  The first live log used the previous interpretation; subsequent sessions use the fix.
- During the user's manual driving, both encoder positions increased, world position
  changed, and IMU yaw rate changed sign (examples: -1.3522 and +0.9467 rad/s).
- Steering feedback included values near +/-0.5236. Its units are recorded raw and must
  not be assumed identical to the normalized steering-command topic. Encoder conversion
  also remains unverified. Motion recordings include collisions and are sensor checks,
  not clean vehicle-identification experiments.

Completed live acceptance:

- After the user stopped/reset, the agent restarted its own bridge. All streams became
  stale; the user pressed Connect again, and messages recovered. Manual Connect is also
  needed after restarting/replacing the bridge or a reset that disconnects the simulator.
- Resumed telemetry showed lap count 5 -> 0, collision count 42 -> 0, and zero encoder
  positions. Diagnostics recorded one reset candidate and reset derivative baselines.
  A reset cannot be observed while no packets are arriving; evidence is reported on recovery.
- A separate corrected diagnostics session after reset verified every stream fresh,
  with `last_lap_time` and `best_lap_time` explicitly unavailable instead of invalid.
- The live bag finalized with **70,642 messages / 13 topics / about 488 seconds**;
  `bag_info.txt` records exact counts. Both JSONL sessions end with `session_end`.
- The agent removed `roboracer-feature1-check` and read its synthetic JSONL/bag from
  Windows successfully. The live bag was also opened read-only from Windows and its
  message count verified. Bind-mounted recording persistence is confirmed.
- Intermittent incoming gaps exceeded 0.5 seconds. Preserve these observations for
  future timing/watchdog design rather than assuming LiDAR's advertised 40 Hz is delivered.

End-of-iteration state: diagnostics and rosbag recording are stopped; the bridge in
`roboracer-feature1-live` is still running and owns host port 4567. Check `docker ps` to
refresh this state in a later session. To release that port before using the user's
normal `penn-challenge` command:

```powershell
docker stop --timeout 2 roboracer-feature1-live
docker rm roboracer-feature1-live
```

These commands stop/remove only the agent-created container; they leave bind-mounted
logs intact. Recreate/start the preferred bridge and manually press Connect afterward.

## Feature 2: watchdog with the retained gap follower

Feature 2 is a bridge command gate, plus a temporary launch using the existing gap
controller. It does not implement mapping, local odometry, or a measured-speed controller.
Read the derivation and acceptance evidence in `RACING_PROGRESS.md`.

### Normal bounded-throttle driving test

1. Check `docker ps` and stop any old bridge before starting another on port 4567.
2. Start Unity with the user's Windows command, reset to the track start, and select
   Autonomous mode. The test controller starts driving as soon as valid scans arrive.
3. Start the user's `penn-challenge` container with the existing bind mount and `-p 4567:4567`.
4. Inside the container:

```bash
cd /home/autodrive_devkit
source /opt/ros/humble/setup.bash
colcon build --packages-select autodrive_roboracer
source install/setup.bash
ros2 launch autodrive_roboracer feature2_gap_test.launch.py \
  max_throttle:=0.12 command_timeout:=0.25
```

This launch starts **both** the bridge and the existing wall-follow node. Do not also
start `bringup_headless.launch.py`, another gap follower, or teleop alongside it. The
bridge restricts commands to `/wall_follow_node`; the original YAML is loaded with a
0.12 throttle override, leaving the normal saved tuning untouched.

5. Press **Connect** manually in Unity when the bridge is ready. If Unity reconnects
   automatically, confirm real telemetry rather than pressing Connect unnecessarily.
6. Observe gate state from another sourced shell:

```bash
ros2 topic echo /autodrive/roboracer_1/command_status
```

`enabled: true` means an exclusive fresh command pair is available. It does not mean
throttle must be nonzero: the follower can legitimately command zero for blocked/stale
LiDAR. A second publisher on either command topic makes both outputs zero.

The test launch shuts down its group if either main process exits. Ctrl+C stops the test.
Stopping input removes propulsion and allows coasting; this is not calibrated braking.
Use Autonomous mode for these actuator checks: Unity's Manual controls can produce
feedback independently of the bridge's permitted command pair.

### Agent fault-injection procedure

For testing bridge survival after a controller crash, start bridge and follower as
separate processes instead of the combined launch. This was the live acceptance method.
Inside the sourced container, start the bridge in one terminal:

```bash
export PYTHONPATH=/home/autodrive_devkit/src/autodrive_devkit:$PYTHONPATH
python3 -u -m autodrive_roboracer.autodrive_bridge --ros-args \
  -p command_timeout:=0.25 -p command_owner:=/wall_follow_node
```

Press Connect after bridge startup; reset/select Autonomous before the driving phase.
In a second sourced shell:

```bash
export PYTHONPATH=/home/autodrive_devkit/src/autodrive_devkit:$PYTHONPATH
python3 -u -m autodrive_roboracer.wall_follow_node --ros-args \
  --params-file /home/autodrive_devkit/src/autodrive_devkit/config/wall_follow.yaml \
  -p max_throttle:=0.12 > /tmp/feature2_gap.log 2>&1 &
controller_pid=$!
```

Use only the PID captured from that newly started test process. Do not use broad process
name matching. Record status, feedback, odometry, collision count, and raw commands with
rosbag before these tests:

```bash
kill -STOP "$controller_pid"   # no shutdown callback can send a stop
# Observe command timeout and both normalized outputs zero.
kill -CONT "$controller_pid"   # wait for fresh-pair recovery
```

While the normal follower is alive, another terminal can introduce a zero-valued competing
publisher. The value itself cannot explain stopping: the bridge must report two publishers
and disable the complete pair:

```bash
ros2 topic pub --rate 20 /autodrive/roboracer_1/throttle_command \
  std_msgs/msg/Float32 '{data: 0.0}'
```

Ctrl+C the extra publisher. Confirm sole ownership and fresh-pair recovery. Finally:

```bash
kill -KILL "$controller_pid"   # no controller cleanup; bridge must expire commands
```

Confirm both bridge outputs and then actuator feedback become zero. Measure speed
separately: coasting continues after zero throttle. Stop/finalize rosbag and diagnostics
with SIGINT before removing a container. Ask the user to reset if the car hits an obstacle
or a new test needs the start position. Do not implement later features as part of testing.

### Reproducible automated checks

Host PowerShell:

```powershell
$env:PYTHONPATH = (Resolve-Path autodrive_devkit).Path
python -B -m unittest discover -s autodrive_devkit/test -p 'test_command_safety*.py' -v
```

In the sourced ROS container, preserve the ROS Python paths and use `python3`:

```bash
export PYTHONPATH=/home/autodrive_devkit/src/autodrive_devkit:$PYTHONPATH
python3 -B -m unittest discover \
  -s /home/autodrive_devkit/src/autodrive_devkit/test \
  -p 'test_command_safety*.py' -v
```

Expected: 16 core tests and one actual ROS test. The ROS test remaps the command and
status topics to `/feature2_test/*`, so it does not actuate the simulator. It uses the
same Humble metadata-preserving executor as the real bridge. Also rerun the 20 retained
gap-controller tests and the 21 diagnostics tests in ROS: **58 tests total**.

Compatibility note: normal subscription callbacks in this image's rclpy do not expose
DDS message metadata. Do not wire `CommandSafetyAdapter` to a standard executor: use
`CommandSafetyExecutor`, which retains `take_message` timestamps for its command callbacks.
The image does not expose a per-message publisher GID; ownership is checked through
publisher discovery, with source-time rejection of pre-transition queued messages.

The legacy keyboard teleop publishes on keystrokes rather than continuously. Its old
nonzero command now expires after 0.25 seconds without another key-driven publication.
The reused gap follower already refreshes both axes at 20 Hz. Do not increase the timeout
to hide a controller that has stopped publishing; add a suitable command heartbeat when
integrating another controller.

### Live acceptance evidence

Run directory: `diagnostics/feature2_20261024T211445Z/`. It contains bridge/controller
logs, diagnostic JSONL, raw rosbag, `phases.jsonl`, 20 Hz `state_samples.jsonl`, and
`live_test_summary.json`. The user reset the Windows simulator and selected Autonomous;
all driving and fault injection after that were performed by the agent.

- Max observed normalized throttle: 0.12; max observed speed: about 2.98 m/s.
- Controller paused with SIGSTOP: gate output zero observed after 0.241 s; zero actuator
  feedback after 0.291 s. The car was still coasting when resumed two seconds later.
- Competitor: 61 samples over about 3 seconds reported two throttle publishers, with
  both outputs zero throughout that detected conflict; fresh-pair operation resumed
  after competitor removal.
- Controller killed with SIGKILL: gate output zero observed after 0.267 s, actuator
  feedback zero after 0.367 s, and measured speed below 0.05 m/s after about 1.17 s.
- Collision count stayed **0** throughout the test.

Timing values are observations sampled at 20 Hz, not tighter hard-real-time guarantees.
The 0.25 s rule is evaluated at each final socket emission; observed status/feedback
include sampling, scheduling, and transport delays. Background load also caused two
expired-transport samples after the pause; those were rejected before normal operation
recovered. No tire/friction or braking parameter was inferred from this test.

Final artifacts: the raw bag has **52,228 messages over approximately 462 seconds**;
the diagnostic JSONL has 453 records and ends in `session_end`. The command test's
finite phase/observation logs are separate from the full-duration rosbag. Both setup
time and post-test stationary time are included in the bag duration.

Current end-of-Feature-2 state: the final bridge is running in the existing agent container
`roboracer-feature1-live`, connected to Unity, with `command_timeout:=0.25` and
`command_owner:=/wall_follow_node`. The controller, diagnostics, and bag recorder are
stopped. No command publishers remain, and permitted output is zero. Check `docker ps`
and `/command_status` to refresh this state in the next session. This container name is
historical; it now hosts the Feature 2 bridge. Normal `penn-challenge` setup instructions
above remain the preferred reproducible workflow.
