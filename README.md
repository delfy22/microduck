# Microduck simulation container

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
