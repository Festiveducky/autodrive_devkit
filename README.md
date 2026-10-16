# AutoDRIVE Devkit

<p align="justify">
This directory hosts ROS 2 API (a meta-package), which supports modular algorithm development targetted towards autonomous driving. It uses client libraries for Python and C++, and can be therefore exploited by the users to develop their algorithms flexibly.
</p>

## SETUP

1. Clone the `AutoDRIVE-RoboRacer-Sim-Racing` repository.
    ```bash
    $ git clone https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing.git
    ```
2. Give executable permissions to the Python scripts.
   ```bash
   $ cd autodrive_devkit
   $ sudo chmod +x *.py
   ```
4. Install the necessary dependencies as mentioned below.
    [AutoDRIVE Devkit's ROS 2 API](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/tree/main/autodrive_devkit) has the following dependencies (tested with Python 3.8, 3.9 and 3.10):
    
    - Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/tree/main/autodrive_simulator) and [AutoDRIVE Devkit](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/tree/main/autodrive_devkit) (version sensitive):
    
      | Package            | Python 3.8 | Python 3.9 | Python 3.10 |
      |--------------------|------------|------------|-------------|
      | eventlet           | 0.33.3     | 0.33.3     | 0.33.3      |
      | Flask              | 1.1.1      | 1.1.1      | 1.1.1       |
      | Flask-SocketIO     | 4.1.0      | 4.1.0      | 4.1.0       |
      | python-socketio    | 4.2.0      | 4.2.0      | 4.2.0       |
      | python-engineio    | 3.13.0     | 3.13.0     | 3.13.0      |
      | greenlet           | 1.0.0      | 1.0.0      | 1.1.0       |
      | gevent             | 21.1.2     | 21.1.2     | 21.12.0     |
      | gevent-websocket   | 0.10.1     | 0.10.1     | 0.10.1      |
      | Jinja2             | 3.0.3      | 3.0.3      | 3.0.3       |
      | itsdangerous       | 2.0.1      | 2.0.1      | 2.0.1       |
      | werkzeug           | 2.0.3      | 2.0.3      | 2.0.3       |
    
    - Generic dependencies for data processing and visualization (usually any version will do the job):
    
      | Package               | Tested Version |
      |-----------------------|----------------|
      | attrdict              | 2.0.1          |
      | numpy                 | 1.22.2         |
      | pillow                | 9.0.1          |
      | opencv-contrib-python | 4.10.0.84      |
      | transforms3d          | 0.4.2          |
  
    - Install dependencies using `requirements.txt` file (use the file specific to your Python version &#8594; check using `python3 --version`):

      ```bash
      $ pip3 install -r requirements_python_3.8.txt # Python 3.8
      $ pip3 install -r requirements_python_3.9.txt # Python 3.9
      $ pip3 install -r requirements_python_3.10.txt # Python 3.10
      ```

    - ROS 2 dependencies for data processing and visualization (usually any version will do the job):

      ```bash
      $ sudo apt install ros-$ROS_DISTRO-tf-transformations
      $ sudo apt install ros-$ROS_DISTRO-imu-tools
      ```

## USAGE

- **Bringup:**
    - **Headless Mode Bringup:**
      ```bash
      $ ros2 launch autodrive_roboracer bringup_headless.launch.py
      ```
      **[OR]**
    - **Graphics Mode Bringup:**
      ```bash
      $ ros2 launch autodrive_roboracer bringup_graphics.launch.py
      ```

- **Teleoperation:**
  ```bash
  $ ros2 run autodrive_roboracer teleop_keyboard
  ```

## LiDAR driving controller

`autodrive_roboracer/wall_follow_node.py` subscribes to `sensor_msgs/LaserScan`
on `/autodrive/roboracer_1/lidar` and publishes `std_msgs/Float32` commands on
`/autodrive/roboracer_1/throttle_command` and
`/autodrive/roboracer_1/steering_command`. Running it now enables autonomous
commands. Use the simulator's Autonomous mode, enable its bridge connection,
and stop keyboard teleoperation or any other controller publishing commands.

### Build and run

Inside your running devkit container (from the host: `docker exec -it penn-challenge bash`):

```bash
source /opt/ros/humble/setup.bash
cd /home/autodrive_devkit
colcon build --symlink-install --packages-select autodrive_roboracer
source install/setup.bash
```

Start the bridge in one terminal, if it is not already running:

```bash
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

In another terminal, source the same ROS and workspace setup files and run:

```bash
ros2 run autodrive_roboracer wall_follow_node
```

Press Ctrl+C to send zero throttle/steering and exit. The node also commands a
stop while waiting for scans, if scans stop arriving for 0.5 seconds, if forward
readings are unknown/blocked, or if a side wall is dangerously close. A stop
command is not an active brake, and abrupt process/container termination cannot
send one. The controller keeps running until stopped; continuous collision-free
laps still require simulator validation and tuning.

For diagnostics with zero movement commands:

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args -p enabled:=false
```

### How it works

The ROS-independent functions live in `autodrive_roboracer/gap_controller.py`:

1. `sanitize_ranges`: treat positive infinity as no hit within sensor range;
   treat NaN, negative infinity, and below-minimum readings as blocked. Finite
   readings above `range_max` are capped. Geometry comes from the scan message,
   so both 1,080 and 1,081 beams work.
2. `inflate_obstacles`: spread each obstacle over neighboring directions by
   `asin(clearance / distance)`. This prevents choosing a long ray immediately
   beside an obstacle or through a gap too narrow for the clearance envelope.
3. `choose_target`: search the forward +/-80 degrees for deep open space, with
   a mild preference for forward travel. Group rays within 5% of the best score
   and aim at the middle of an opening; prefer the previous direction when
   several openings are equally good. This reduces left/right target flicker.
4. `calculate_steering`: use pure-pursuit steering and blend in a sharp-turn
   boost as the target angle increases from 35 to 70 degrees. At full boost,
   lookahead is multiplied by 0.60 and steering gain by 1.35. This makes the car
   commit harder to a corner without abruptly switching steering modes.
5. `calculate_throttle`: scale throttle with usable path distance and reduce it
   by up to 40% as steering increases (previously 75%). A separate sharp-turn cap
   blends in with the turn boost. `compute_command` combines these functions
   with wall avoidance and emergency stops. The throttle topic is a normalized
   motor command, not a speed in m/s; physical speed is not directly proportional.

The wall clearance is a planning envelope measured from the LiDAR, including
car half-width plus a margin; it is not a guaranteed distance from the body to
all walls during a turn. The front stop distance is also measured from LiDAR.
This reactive controller does not reverse out of dead ends or use a track map.

### Tuning and terminal output

Edit **`autodrive_devkit/config/wall_follow.yaml`**, then restart the node.
After your next build, the new launcher loads the installed config:

```bash
ros2 launch autodrive_roboracer wall_follow.launch.py
```

To always use the editable source file in your mounted devkit directory, with
no rebuild required after config edits, run this inside the container:

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args --params-file /home/autodrive_devkit/src/autodrive_devkit/config/wall_follow.yaml
```

The launch file also accepts `params_file:=/absolute/path/to/wall_follow.yaml`.
Plain `ros2 run` uses the Python defaults, which match the supplied YAML, but
will not load YAML edits unless you pass `--params-file`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `max_throttle` | `0.20` | Straight-line throttle limit (previously 0.12) |
| `throttle_gain` | `0.08` | Throttle per clear meter (previously 0.04) |
| `turn_slowdown` | `0.40` | Fraction of throttle removed at full steering; smaller is faster in corners |
| `sharp_turn_throttle` | `0.14` | Additional throttle cap at full sharp-turn boost |
| `steering_gain` | `1.0` | Overall steering multiplier |
| `min_lookahead` | `0.6` | Minimum ordinary steering lookahead, meters |
| `max_lookahead` | `1.5` | Maximum ordinary steering lookahead, meters |
| `lookahead_gain` | `0.5` | Path distance multiplier before lookahead clamping |
| `sharp_turn_angle` | `35.0` | Absolute target angle where boost starts, degrees |
| `sharp_turn_full_angle` | `70.0` | Absolute target angle where boost reaches full strength, degrees |
| `sharp_turn_steering_gain` | `1.35` | Steering multiplier at full boost |
| `sharp_turn_lookahead_scale` | `0.60` | Lookahead multiplier at full boost; smaller turns harder |
| `clearance` | `0.35` | Obstacle inflation radius, meters |
| `stop_distance` | `0.45` | Minimum frontal clearance before stopping, meters |
| `forward_angle` | `80.0` | Search half-angle in degrees |
| `wheelbase` | `0.33` | Steering model wheelbase, meters |
| `max_steering_angle` | `0.52` | Radians corresponding to full steering command |
| `scan_timeout` | `0.5` | Stop after this many seconds without a scan |
| `enabled` | `true` | Set false to publish zero commands while inspecting scans |
| `debug_period` | `1.0` | Seconds between terminal reports |
| `print_ranges` | `false` | Also print every range from the latest scan |

Raise `max_throttle` for faster straights. Raise `throttle_gain` if the car is
still slow with limited distance ahead. Lower `turn_slowdown` and/or raise
`sharp_turn_throttle` for faster corners. Raise `sharp_turn_steering_gain` or
lower `sharp_turn_lookahead_scale` for stronger sharp-turn steering. Commands
remain clamped to full steering, so tuning beyond saturation cannot turn tighter.
These settings do not force a minimum throttle near obstacles.

Reports include scan metadata, front/left/right distances, nearest finite return,
controller state, target angle, usable path distance, sharp-turn boost percentage,
throttle, and steering.
Positive infinity now prints as `no return (>10.00 m)` rather than `invalid`.
For raw range arrays:

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args -p enabled:=false -p print_ranges:=true
```

Positive throttle is forward and positive steering turns left. Both commands
are bounded to `[-1, 1]`. Zero degrees points forward; +90 degrees points left.

Run the planner behavior tests without ROS, from the repository root:

```bash
PYTHONPATH=autodrive_devkit python3 -m unittest discover -s autodrive_devkit/test -p test_gap_controller.py -v
```

The obstacle inflation and gap target selection are inspired by the
[F1TENTH Follow the Gap lab](https://github.com/f1tenth/f1tenth_lab4_template).
