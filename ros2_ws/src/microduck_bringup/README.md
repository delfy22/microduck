# Microduck bringup

After building and sourcing `ros2_ws`, start the bridge and the Foxglove
WebSocket server with:

```sh
ros2 launch microduck_bringup sim.launch.xml
```

The launch file reads the simulator sockets at `/var/lib/duck-sim/duck-a.sock`
and `/var/lib/duck-sim/duck-a-tof.sock`. Override `robot_socket` or `tof_socket`
if the simulator uses different paths. The WebSocket server listens on port
8765; connect Lichtblick to `ws://localhost:8765`.

Import `config/microduck.layout.json` in Lichtblick using **Layouts → Import from
file…**. It places TF, ToF points and robot markers in the 3D panel, plots
joint positions, and exposes diagnostics, odometry and IMU messages for
inspection.
