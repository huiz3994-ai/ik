#!/usr/bin/env bash
set -eo pipefail

if [ -z "${DISPLAY:-}" ]; then
    echo "FAIL: DISPLAY is empty; Gazebo GUI cannot start." >&2
    exit 1
fi
if [ ! -S "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
    echo "FAIL: X11 socket for DISPLAY=${DISPLAY} is not mounted." >&2
    exit 1
fi

install -d -o 1000 -g 1000 -m 0755 /tmp/rm75-gui
install -d -o 1000 -g 1000 -m 0700 /tmp/rm75-gui/runtime
# rm75_ws is mounted below /root. Allow traversal to that known mount without
# granting directory listing or write access to the container root home.
chmod 0711 /root

# roslaunch normally supplies a log path below /root. Remove only that remap
# because the GUI is deliberately dropped to the desktop user's UID.
gzclient_args=()
for arg in "$@"; do
    case "$arg" in
        __log:=*) ;;
        *) gzclient_args+=("$arg") ;;
    esac
done

exec env HOME=/tmp/rm75-gui USER=zjh XDG_RUNTIME_DIR=/tmp/rm75-gui/runtime \
    setpriv --reuid=1000 --regid=1000 --clear-groups \
    /usr/bin/gzclient "${gzclient_args[@]}"
