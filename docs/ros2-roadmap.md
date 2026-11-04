# ROS 2 integration roadmap

Build towards sensor-based mapping and Nav2 goal navigation using ROS 2 Lyrical,
the existing MuJoCo duck, and a local Lichtblick instance. Implement and validate
one milestone at a time. The first functional milestone is ROS velocity control
and sensor visualisation; reliable mapping is a separate piece of work.

## Working with this plan

- Tick a task only after its acceptance checks pass; record evidence below it.
- Expand the next milestone as interfaces become concrete. Keep later milestones
  at the level needed to guide current decisions.
- Record unresolved choices under Open decisions. Update decisions to describe
  the agreed implementation, with links to the relevant code or documentation.
- Commit each coherent part with its associated checks and documentation.

## Architecture and boundaries

```text
Teleoperation / Nav2 -- cmd_vel --> ROS bridge -- robot.move --> robotd
                                        ^                       |
                                        |                       v
                                state / sensors             MuJoCo body
                                        |
                                  ROS topics / TF
                                   /           \
                         Mapping / localisation  WebSocket bridge --> Lichtblick
```

- Keep the existing gait policy and daemon responsible for balancing and joints.
- Present a nonholonomic navigation interface: forward velocity and yaw rate,
  with commanded lateral velocity fixed to zero. Do not add wheel controllers.
- Put ROS packages in `ros2_ws/src`, outside the upstream submodules.
- Use shared Unix sockets for the daemon API. Use the Compose network for the
  simulation body and ROS communication; publish the visualisation WebSocket
  only on localhost.
- The ROS bridge owns protocol conversion, units, timestamps and command freshness.
- MuJoCo displays physical motion. Lichtblick displays sensors, transforms, maps,
  paths and goals. MuJoCo path/goal overlays are a later feature.

## Milestone 0 — Development foundation

- [x] Build and run the existing sim with a GPU-rendered MuJoCo viewer.
- [x] Control the same duck from temporary containers through shared sockets.
- [x] Make `drive`, `status` and `realtime` work across container boundaries.
- [x] Establish development rules in the root `AGENTS.md`.
- [x] Provide a Helix development container with ROS 2 Lyrical dependencies.
- [x] Verify C++, Python and YAML language support and ROS dependency navigation.
- [x] Document editing, building and checking the workspace.

Evidence: the existing simulation has run at 1.00× real time with a healthy 50 Hz
control loop. A temporary client successfully drove it 59 cm over an eight-second
test. Bringup and shared access are committed in the main repository and the
`microduck` submodule.

Development environment verified on 4 November 2026: Helix 25.07.1 in the Lyrical
container; disposable ROS C++ compilation and clangd checks passed; Python/Jedi
resolved `rclpy` definitions; the interactive YAML language server reported no
Compose diagnostics. The development container also reached the running duck.
See the [development guide](development.md) and `scripts/check-dev-env`.

## Milestone 1 — ROS control and sensor visualisation

### Bridge and control

- [ ] Create `microduck_ros_bridge` and `microduck_bringup` packages.
- [ ] Subscribe to daemon state and ToF streams, with reconnect handling.
- [ ] Translate `TwistStamped` commands to `robot.move`: x in m/s, yaw in rad/s,
  positive yaw turning left, lateral command zero.
- [ ] Enforce conservative speed limits and a command timeout; resend only while
  a command is fresh. Preserve the daemon's deadman and fallen-state protections.
- [ ] Expose connection, sensor freshness and enabled/fallen state in diagnostics.
- [ ] Decide command ownership so teleoperation and autonomous navigation cannot
  fight over the same actuator interface.

### Data contract

| Topic | ROS type | Source and interpretation |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Forward/yaw request in the robot frame |
| `/odom` | `nav_msgs/msg/Odometry` | Contact odometry; derive/filter measured velocity from pose changes |
| `/imu/data` | `sensor_msgs/msg/Imu` | Trunk quaternion and gyro; mark acceleration unavailable |
| `/joint_states` | `sensor_msgs/msg/JointState` | Measured positions and available velocities |
| `/tof/points` | `sensor_msgs/msg/PointCloud2` | Valid depth returns, using published beam geometry |
| `/tf`, `/tf_static` | TF messages | Base and sensor transforms, with one publisher per transform |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Health and data freshness |

- [ ] Convert scalar-first duck quaternions into ROS quaternion field ordering.
- [ ] Preserve ToF status semantics: valid return, no target and failed measurement
  are different. Do not turn invalid returns into obstacles or clear space.
- [ ] Use the daemon's timestamped sensor poses for the moving head.
- [ ] Start with ROS wall time and a consistent conversion from daemon monotonic
  timestamps. Define missing-timestamp behaviour explicitly.
- [ ] Define meaningful covariance assumptions; do not claim perfect measurements.
- [ ] Publish `odom -> base_footprint -> base_link -> sensor frames`, preserving
  trunk height, roll and pitch beneath the planar navigation frame.
- [ ] Provide a minimal robot representation for visualisation. Avoid duplicating
  head kinematics already supplied by the daemon.

### Visualisation and acceptance

- [ ] Add a ROS WebSocket bridge and a saved Lichtblick layout.
- [ ] Show the robot pose, trajectory, joints, sensor frames and ToF points.
- [ ] Verify forward motion and positive yaw against the MuJoCo viewer.
- [ ] Verify stopping on command timeout and bridge disconnect.
- [ ] Restart bridge and simulator independently and verify reconnect behaviour.
- [ ] Record a short ROS bag for repeatable sensor/TF inspection.

Complete when ROS teleoperation controls the existing duck, the sensor geometry
matches MuJoCo, and Lichtblick shows live data without transform/time errors.

## Milestone 2 — Mapping inputs and localisation

The simulated ToF is 8×8, approximately 45° field of view, up to 4 m and 15 Hz.
It is useful for nearby obstacles, but sparse data and head motion make robust
conventional planar SLAM an open question. A point cloud converted to `LaserScan`
does not acquire the coverage or geometry of a planar lidar.

- [ ] Create a room scene with obstacles while retaining the walking model needed
  by the current gait; validate walking before evaluating mapping.
- [ ] Record straight, turning and head-moving trajectories.
- [ ] Compare contact odometry with separately labelled MuJoCo ground truth.
- [ ] Check whether accumulated wall returns remain stationary; inspect floor and
  self returns and the treatment of failed/no-target measurements.
- [ ] Choose a mapping/localisation route based on these recordings.
- [ ] Evaluate ToF occupancy mapping and, if needed, deliberate head scanning.
- [ ] Optionally add a labelled synthetic planar lidar profile to unblock Nav2
  development independently of realistic ToF mapping.
- [ ] Produce and save an occupancy map; demonstrate repeatable localisation.

Ground-truth odometry may be an explicit diagnostic mode. It must remain
distinguishable from the robot's estimate. Mapping/localisation owns `map -> odom`.

## Milestone 3 — Nav2 goals

- [ ] Configure a nonholonomic controller and consistent stamped velocity topics.
- [ ] Measure useful forward/reverse speeds, turn rates, acceleration and stopping
  distance. Tune from measured movement rather than assuming requested speed.
- [ ] Define the footprint, including space needed by body and leg movement.
- [ ] Configure obstacle observations, inflation, velocity smoothing and command
  arbitration.
- [ ] Validate short goals in open space, followed by obstacle avoidance.
- [ ] Add a goal adapter from a Lichtblick-published pose to `NavigateToPose`, with
  visible status and cancellation. Do not assume the UI can call ROS actions.
- [ ] Show map, costmaps, paths, current pose and goal in Lichtblick.
- [ ] Test cancellation, stale sensors, lost localisation and interrupted clients.

Complete when a selected goal moves the same simulated duck, cancellation stops
it reliably, and navigation state agrees between ROS and MuJoCo.

## Milestone 4 — Refinements

- [ ] Add camera images, calibration and the required frame transforms.
- [ ] Improve state estimation and validate transfer to the physical sensor set.
- [ ] Add fuller robot geometry and optional MuJoCo navigation overlays.
- [ ] Introduce simulation time only with coherent timestamp, pause and reset
  handling across the bridge and wall-clock-driven daemons.

## Open decisions

| Decision | Starting position | Resolve by |
| --- | --- | --- |
| Bridge implementation | Python/rclpy for the existing JSON-RPC interfaces | Milestone 1 package creation |
| TF/robot representation | Minimal geometry; consume published sensor poses | First visualisation |
| Mapping sensor | Evaluate existing ToF; synthetic lidar is an explicit development aid | Milestone 2 recordings |
| Mapping/localisation packages | Select after sensor evaluation | Milestone 2 |
| Nav2 controller and limits | Measure walking response first | Milestone 3 |

## References

- [Daemon protocol](../3rd_party/microduck/duck-ipc-proto/src/lib.rs)
- [Simulation usage](../3rd_party/microduck/docs/robot/simulation.md)
- [Simulated ToF](../3rd_party/microduck_rl/src/mjlab_microduck/sim/tof.py)
- [Nav2 Lyrical guide](https://docs.nav2.org/lyrical/getting_started/quickstart/quickstart/)
- [Nav2 frame conventions](https://docs.nav2.org/rolling/getting_started/navigation_concepts/state_estimation/)
- [Lichtblick ROS 2 connections](https://lichtblick-suite.github.io/docs/docs/connecting-to-data/frameworks/ros2/)
