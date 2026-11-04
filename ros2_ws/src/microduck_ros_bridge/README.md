# microduck_ros_bridge

`microduck_bridge` translates the Microduck daemon's Unix socket streams into ROS 2 Lyrical topics. It does not enable robot policies. The daemon's own deadman remains in force, and this bridge also sends one zero `robot.move` when a fresh command expires or is cancelled.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `robot_socket` | `/var/lib/duck-sim/duck-a.sock` | robotd JSON-RPC NDJSON socket |
| `tof_socket` | `/var/lib/duck-sim/duck-a-tof.sock` | tofd JSON-RPC NDJSON socket |
| `command_timeout_sec` | `0.25` | Maximum age and hold time for `/cmd_vel` |
| `max_forward_speed_mps` | `0.20` | Clamp for forward and reverse velocity |
| `max_yaw_rate_radps` | `1.0` | Clamp for yaw rate |
| `state_timeout_sec` | `0.25` | State age beyond which commands are inhibited |
| `tof_timeout_sec` | `0.5` | ToF freshness diagnostic threshold |

## Topics and frames

- `/cmd_vel` (`geometry_msgs/msg/TwistStamped`): forward velocity in m/s and yaw rate in rad/s. Header stamp must be current and no more than the command timeout old. More than one publisher inhibits motion. Lateral velocity is ignored and `vy=0` is sent to the daemon.
- `/odom` (`nav_msgs/msg/Odometry`): contact odometry in `odom`, planar pose in `base_footprint`; pose covariance is 0.01 m² in x/y and 0.04 rad² in yaw. z/roll/pitch covariance is marked unavailable with 1e6. Twist is filtered from successive pose samples, with 0.04 (x/y) and 0.09 (yaw) covariance.
- `/imu/data` (`sensor_msgs/msg/Imu`): trunk quaternion and gyro in `base_link`; linear acceleration is unavailable (`covariance[0] = -1`).
- `/joint_states` (`sensor_msgs/msg/JointState`): measured joint positions and reported velocities in radians and radians/s.
- `/tof/points` (`sensor_msgs/msg/PointCloud2`): valid ToF returns in metres in frame `tof`. Only ST statuses 5 and 9 with positive distance become points; no-target and failed measurements are omitted.
- `/robot_markers` (`visualization_msgs/msg/MarkerArray`): skeleton body spheres in `base_link` and a bounded 3,000-pose trajectory line in `odom`.
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): socket, freshness, policy, fallen/limp, and command ownership status.
- `/tf`: `odom -> base_footprint -> base_link -> tof` and, when available, `base_link -> camera`. The live sensor transforms are only published when supplied in state; absent poses are not replaced with identities.

Daemon `CLOCK_MONOTONIC` timestamps are mapped to ROS wall time using one offset captured at node startup. Missing/invalid daemon timestamps are dropped and reported; `use_sim_time` is unsupported. Robot skeleton poses and sensor poses are trunk-relative, as defined by `robot.model` and `robot.state`.

Covariances are provisional assumptions for development, not calibrated error
estimates. Tune them against recorded trajectories before feeding a state
estimator. Acceleration is absent from the daemon interface and is explicitly
marked unavailable rather than estimated from commanded motion.

The diagnostics include per-frame counts of ToF no-target (`status 255`) and failed zones, so omitted points remain distinguishable. State and ToF freshness use daemon monotonic sample timestamps; delayed, repeated, future, or missing samples do not refresh the bridge health state.
