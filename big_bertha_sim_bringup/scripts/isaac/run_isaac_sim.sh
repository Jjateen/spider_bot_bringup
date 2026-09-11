#!/usr/bin/env bash
# Entry point for the Isaac Sim bringup path: source the Isaac Lab venv
# (the only setup Isaac Sim needs here) then run launch_isaac.py inside it.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source ~/isaac.sh

# The login shell's ~/.bashrc sources /opt/ros/humble/setup.bash unconditionally
# (see prepare_urdf.sh for the same fact re: ROS_DISTRO), which puts the
# system rclpy on PYTHONPATH. isaacsim.ros2.bridge appends its own bundled
# rclpy to sys.path rather than prepending it, so with PYTHONPATH inherited,
# Python resolves `import rclpy` to the system one first -- built for
# Python 3.10, not this venv's 3.11, so it fails on a missing compiled
# submodule with a confusingly generic "No module named '...pybind11'"
# error that looks unrelated to the real cause. Isaac Sim's own venv is
# self-contained; nothing here needs the host's ROS on PYTHONPATH.
unset PYTHONPATH

# The bundled rclpy's compiled extension needs two runtime dependency dirs
# on LD_LIBRARY_PATH that aren't found by default:
#   - isaacsim.ros2.bridge/humble/lib: the bundled rcl/rmw/rosidl .so's.
#   - isaacsim/kit/kernel/plugins: Kit's own libpython3.11.so (the pybind11
#     extension links against libpython dynamically; there's no system
#     libpython3.11 on this host to fall back to since it's Ubuntu 22.04's
#     default Python 3.10).
# Without both, the bridge silently no-ops instead of erroring: its
# ROS2Context node's output context stays null, so every publisher/
# subscriber node downstream is a dead end. Must be set before the process
# starts, not from inside launch_isaac.py -- consulted at dlopen, and
# setting it late enough to be certain of ordering isn't worth the risk when
# the shell can just guarantee it up front.
ROS2_BRIDGE_EXT="$(find "$VIRTUAL_ENV/lib" -maxdepth 6 -type d \
  -path "*/isaacsim/exts/isaacsim.ros2.bridge" 2>/dev/null | head -1)"
KIT_PLUGINS="$(find "$VIRTUAL_ENV/lib" -maxdepth 6 -type d \
  -path "*/isaacsim/kit/kernel/plugins" 2>/dev/null | head -1)"
if [ -n "$ROS2_BRIDGE_EXT" ] && [ -n "$KIT_PLUGINS" ]; then
  export LD_LIBRARY_PATH="$ROS2_BRIDGE_EXT/humble/lib:$KIT_PLUGINS:${LD_LIBRARY_PATH:-}"
fi

exec python3 "$SCRIPT_DIR/launch_isaac.py" "$@"
