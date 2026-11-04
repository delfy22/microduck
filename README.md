# Microduck simulation container

Development towards ROS 2, mapping and navigation is tracked in the
[ROS 2 integration roadmap](docs/ros2-roadmap.md).
The [Helix development guide](docs/development.md) covers the ROS 2 editor and
toolchain container for C++, Python and YAML.

Build the Rust daemons and run them against the MuJoCo body from the two pinned
submodules. Requires Linux, Docker with Compose (or Podman with podman-compose),
an X11 or XWayland desktop, and an Intel/AMD GPU exposed at `/dev/dri`.

```bash
git submodule update --init --recursive
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
docker compose up --build -d --force-recreate
docker compose logs -f sim
```

With Podman, use `podman-compose` in place of `docker compose` in every command.
GPU rendering and device passthrough are enabled in `compose.yaml`. The image includes Mesa from
Bookworm backports for newer AMD GPUs, including RDNA 4. NVIDIA needs its own
container driver setup and is not covered by this configuration.

The MuJoCo window opens and the duck enables its standing policy. The first build
downloads dependencies and compiles Rust; first startup downloads the official
ONNX policy set. Later launches reuse the build and policy volumes.

```bash
docker compose exec sim scripts/duck-sim status
docker compose exec sim scripts/duck-sim drive
docker compose exec sim scripts/duck-sim ctl health
docker compose exec sim scripts/duck-sim monitor
docker compose exec sim scripts/duck-sim log
docker compose down
```

The workspace is mounted at `/workspace`. Edit either submodule on the host, then
restart the service to rebuild the daemons. You can also compile or open a shell
inside the running container:

```bash
docker compose exec sim cargo build --locked -p robotd -p robotctl
docker compose exec sim bash
```

To control the running duck from a separate, temporary container:

```bash
docker compose run --rm sim bash
robotctl health
robotctl robot look 1 0.5 0
robotctl robot do sit_toggle
scripts/duck-sim drive 0.15 0
exit
```

Start the simulation service with `up -d` first. The temporary container shares
the simulation's socket volume and build artifacts. Its `robotctl` wrapper selects
the shared sockets and simulator params, so the usual CLI commands work directly.
Exiting this shell removes the temporary container and leaves the duck running.
The same `robotctl` wrapper is available through `compose exec`.

Other client services can connect to `/var/lib/duck-sim/duck-a.sock` by mounting
the `sim-state` volume there and using compatible socket permissions. No network
gateway is needed for this local JSON-RPC connection. Helpers such as
`scripts/duck-sim drive`, `status` and `realtime` also query MuJoCo at `sim:7801`
on the Compose network. The body listens on the container's network interfaces;
no host port is published. `drive` is a launcher helper, so run
`scripts/duck-sim drive`, not `scripts/duck-sim ctl drive` or `robotctl drive`.

Python contains the body server's dependencies, pinned from the RL `uv.lock`,
rather than the training environment. Rebuild the image after changing dependency
locks. The GPU accelerates the viewer; physics and the Rust control loop
still run on the CPU.
This setup launches the MuJoCo viewer and core daemons; the optional web camera
stack and systemd simulation mode are not configured.

`DUCK_SIM_SCENE=apartment docker compose up -d` selects another scene.
`DUCK_SIM_VIEWER=0 docker compose up -d` disables the viewer (the Compose display
mounts still require a valid X authority file). Display access uses your existing
X11 cookie; no `xhost` access changes are needed. If `XAUTHORITY` changes after
logging in again, recreate the service with `docker compose up -d --force-recreate`.

Build caches and state survive `docker compose down`. `docker compose down -v`
also removes those volumes, including downloaded policies and simulated identities.

## ROS 2 development

See the [ROS bridge and Lichtblick guide](docs/ros2.md) for velocity control and
sensor visualisation, the [Helix development guide](docs/development.md) for
editing and building C++, and the [integration roadmap](docs/ros2-roadmap.md)
for the staged mapping and Nav2 work.
