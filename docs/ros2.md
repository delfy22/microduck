# ROS 2 bridge and visualisation

The C++ bridge connects ROS 2 Lyrical to the existing duck daemons through their
shared Unix sockets. The gait policy still controls balancing and joints. ROS
requests forward velocity and yaw rate; it does not implement wheel control.

## Build and start

From the repository root, with the usual desktop `XAUTHORITY` environment:

```bash
podman-compose --profile dev build dev
podman-compose --profile dev run --rm -T dev scripts/ros-build
podman-compose up -d sim
podman-compose --profile ros up -d bridge lichtblick
```

The bridge service uses the development image and the built workspace overlay.
Rebuild the workspace after C++ changes, then restart the bridge:

```bash
podman-compose --profile dev run --rm -T dev scripts/ros-build
podman-compose --profile ros restart bridge
podman-compose --profile ros logs --tail=30 bridge
```

The simulator and bridge can start independently; the bridge retries unavailable
sockets. The `ros` profile adds the bridge and browser application without changing
the normal simulation startup. Stop just these services with
`podman-compose --profile ros stop bridge lichtblick`.

## Lichtblick

Open <http://localhost:8080>, choose **Open connection**, select **Foxglove
WebSocket**, and connect to `ws://localhost:8765`. Import
[`microduck.layout.json`](../ros2_ws/src/microduck_bringup/config/microduck.layout.json)
from the layout menu. An existing desktop Lichtblick installation can connect to
the same address. Both published container ports bind only to localhost.

The layout displays the moving skeleton, contact-odometry trail, sensor frames and
valid ToF points, alongside joint plots, odometry, IMU and diagnostics. The skeleton
is a minimal representation using daemon geometry, rather than a mesh model.
MuJoCo continues to show the simulated physical duck. No map frame is published;
mapping will own `map -> odom` in the next milestone.

Lichtblick 1.30.0 reports schema-change alerts for Lyrical's `PointField` and
`Marker` definitions. Live points and markers render despite these alerts; they
are a visualisation compatibility issue, rather than a TF or timestamp error.

## Send velocity commands

Open a development shell:

```bash
podman-compose --profile dev run --rm dev bash
```

Commands must be stamped and refreshed while movement is wanted:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.15}, angular: {z: 0.0}}}'
```

Stop publishing with Ctrl-C. Positive `angular.z` turns left; x is metres per
second and yaw is radians per second. Lateral velocity is always zero. The bridge
defaults to limits of 0.20 m/s and 1.0 rad/s, and a 0.25 s command timeout. Requested
speed is not a guarantee of measured walking speed.

Use exactly one `/cmd_vel` publisher. The bridge inhibits commands when it detects
multiple publishers; select either teleoperation or navigation, then start that
publisher. Do not run `duck-sim drive`, a gamepad, or another direct daemon command
source alongside ROS control: those clients bypass ROS ownership checks. A proper
command multiplexer belongs with Nav2 integration.

The bridge never enables the duck or overrides the daemon's fallen-state and
deadman protections. Use the existing simulation controls to initialise it when
needed. Commands are discarded across bridge/daemon reconnection; resuming
movement requires fresh messages.

## Check and record

```bash
podman-compose --profile dev run --rm -T dev bash -c \
  'cd ros2_ws && colcon test && colcon test-result --verbose'
podman-compose --profile dev run --rm -T dev scripts/check-ros-bridge
```

The live check is read-only by default. Add `--motion` to explicitly test forward
and left-turn commands, timeout stopping and the absence of stale command replay.
Run it with no other command publisher active.

Run `scripts/check-ros-bridge-failures` in the development container for isolated
fake-daemon checks, including socket loss and stale commands. This uses temporary
sockets and a separate ROS domain, so it does not contact the simulator.

For a recording, open a development shell and run:

```bash
ros2 bag record -s mcap -o /workspace/.cache/rosbags/duck --topics \
  /odom /imu/data /joint_states /tof/points /tf /tf_static \
  /robot_markers /diagnostics /cmd_vel
```

Stop with Ctrl-C; choose a new output directory for each recording. Open the MCAP
file directly in Lichtblick for inspection without the live simulator. Avoid
replaying `/cmd_vel` into the live robot when playing recordings through ROS.

## Sources

- [Daemon protocol](../3rd_party/microduck/duck-ipc-proto/src/lib.rs)
- [Foxglove bridge](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md)
- [Lichtblick local container](https://github.com/lichtblick-suite/lichtblick#-from-docker)
- [Roadmap and outstanding acceptance checks](ros2-roadmap.md)
