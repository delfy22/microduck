FROM ghcr.io/astral-sh/uv:0.9.28 AS uv
FROM docker.io/library/rust:1.94.1-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
    alsa-utils ca-certificates curl git pkg-config libudev-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev libgl1-mesa-dri libglx-mesa0 \
    libegl1 libglfw3 libx11-6 libxcursor1 libxi6 libxinerama1 \
    libxrandr2 tini xauth \
    && rm -rf /var/lib/apt/lists/*
COPY --from=uv /uv /usr/local/bin/uv
ENV UV_PYTHON_INSTALL_DIR=/opt/python
RUN uv python install 3.12 && uv venv --python 3.12 /opt/microduck_rl/.venv

# The body server imports only these three packages; use the RL checkout's pins
# without pulling in CUDA, Torch and the training tools.
COPY 3rd_party/microduck_rl/uv.lock /tmp/rl-uv.lock
RUN /opt/microduck_rl/.venv/bin/python -c \
    'import tomllib; p=tomllib.load(open("/tmp/rl-uv.lock", "rb")); print("\n".join(x["name"]+"=="+x["version"] for x in p["package"] if x["name"] in {"mujoco", "numpy", "onnxruntime"}))' \
    > /tmp/sim-requirements.txt \
    && uv pip install --python /opt/microduck_rl/.venv/bin/python -r /tmp/sim-requirements.txt

ENV DUCK_SIM_RL=/opt/microduck_rl \
    DUCK_SIM_STATE=/var/lib/duck-sim \
    PYTHONPATH=/workspace/3rd_party/microduck_rl/src \
    CARGO_BUILD_JOBS=4
WORKDIR /workspace/3rd_party/microduck
COPY 3rd_party/microduck/ ./
COPY 3rd_party/microduck_rl/ /workspace/3rd_party/microduck_rl/
RUN cargo build --locked -p robotd -p robotctl -p tof -p sounds -p configd -p updater
# RDNA 4 GPUs need newer Mesa than Bookworm's original 22.x drivers.
RUN printf '%s\n' 'deb http://deb.debian.org/debian bookworm-backports main' \
    > /etc/apt/sources.list.d/backports.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends -t bookworm-backports \
       libgl1-mesa-dri libglx-mesa0 libegl-mesa0 libgbm1 \
    && rm -rf /var/lib/apt/lists/*
RUN ln -s /workspace/3rd_party/microduck_rl/src /opt/microduck_rl/src
ENV PATH=/opt/microduck_rl/.venv/bin:$PATH
COPY docker/sim-entrypoint.sh /usr/local/bin/sim-entrypoint
COPY --chmod=755 docker/robotctl /usr/local/bin/robotctl
ENTRYPOINT ["/usr/bin/tini", "--", "/bin/sh", "/usr/local/bin/sim-entrypoint"]
CMD ["sim"]
