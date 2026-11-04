#!/bin/sh
set -eu

if [ -r /tmp/host.xauth ]; then
    # FamilyWild makes the host cookie valid under the container's hostname.
    export XAUTHORITY=/tmp/sim.xauth
    touch "$XAUTHORITY"
    chmod 600 "$XAUTHORITY"
    cp /tmp/host.xauth /tmp/source.xauth
    xauth -f /tmp/source.xauth nlist | sed 's/^..../ffff/' | xauth nmerge -
    rm /tmp/source.xauth
fi

if [ "${1:-sim}" != sim ]; then
    exec "$@"
fi

cleanup() {
    trap - EXIT INT TERM
    if [ -n "${startup:-}" ]; then
        kill "$startup" 2>/dev/null || true
    fi
    scripts/duck-sim down || true
}
trap cleanup EXIT
trap 'exit 0' INT TERM

# Running startup asynchronously lets TERM interrupt the build or startup wait.
scripts/duck-sim up &
startup=$!
if wait "$startup"; then
    startup=
else
    startup=
    tail -n 40 "$DUCK_SIM_STATE/body.log" "$DUCK_SIM_STATE/robotd.log" >&2 || true
    exit 1
fi
if [ "${DUCK_SIM_VIEWER:-1}" != 0 ] && grep -q 'no viewer' "$DUCK_SIM_STATE/body.log"; then
    cat "$DUCK_SIM_STATE/body.log" >&2
    exit 1
fi

tail -n 20 "$DUCK_SIM_STATE/body.log" "$DUCK_SIM_STATE/robotd.log"
while sleep 2; do
    for name in body duck-a duck-a-tof duck-a-config duck-a-updater; do
        pid=$(cat "$DUCK_SIM_STATE/$name.pid")
        if ! kill -0 "$pid" 2>/dev/null || [ "$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)" = Z ]; then
            echo "Simulation process $name exited; stopping." >&2
            tail -n 40 "$DUCK_SIM_STATE/body.log" "$DUCK_SIM_STATE/robotd.log" >&2
            exit 1
        fi
    done
done
