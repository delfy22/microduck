FROM docker.io/library/ros:lyrical-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config git curl ca-certificates xz-utils \
    clangd clang-format python3-pylsp python3-autopep8 python3-pytest \
    python3-colcon-common-extensions python3-rosdep python3-yaml \
    nodejs npm ripgrep shellcheck yamllint bash-completion less tini \
    ros-lyrical-sensor-msgs ros-lyrical-nav-msgs ros-lyrical-diagnostic-msgs \
    ros-lyrical-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Install the editor together with its matching runtime and language grammars.
RUN case "$(dpkg --print-architecture)" in \
      amd64) hx_arch=x86_64; hx_sha=3f08e63ecd388fff657ad39722f88bb03dcf326f1f2da2700d99e1dc40ab2e8b ;; \
      arm64) hx_arch=aarch64; hx_sha=ce23fa8d395e633e3e54c052012f11965d91d8d5c2bfa659685f50430b4f8175 ;; \
      *) exit 1 ;; \
    esac \
    && curl -fL --retry 3 "https://github.com/helix-editor/helix/releases/download/25.07.1/helix-25.07.1-${hx_arch}-linux.tar.xz" -o /tmp/helix.tar.xz \
    && printf '%s  %s\n' "$hx_sha" /tmp/helix.tar.xz | sha256sum -c - \
    && mkdir -p /opt/helix \
    && tar -xJf /tmp/helix.tar.xz -C /opt/helix --strip-components=1 \
    && ln -s /opt/helix/hx /usr/local/bin/hx \
    && rm /tmp/helix.tar.xz
RUN npm install --global yaml-language-server@1.18.0 && npm cache clean --force

ENV HELIX_RUNTIME=/opt/helix/runtime \
    EDITOR=hx \
    VISUAL=hx \
    COLCON_HOME=/workspace/.cache/colcon \
    XDG_CACHE_HOME=/workspace/.cache
WORKDIR /workspace
COPY docker/dev-entrypoint.sh /usr/local/bin/dev-entrypoint
COPY --chmod=755 docker/robotctl /usr/local/bin/robotctl
ENTRYPOINT ["/usr/bin/tini", "--", "/bin/bash", "/usr/local/bin/dev-entrypoint"]
CMD ["hx", "."]
