#!/usr/bin/env bash
# Expand big_bertha_description's URDF for Isaac Sim import.
#
# The xacro source is untouched -- this just calls it the same way
# big_bertha_description/launch/rsp.launch.py does, with use_gz:=false so the
# gz_ros2_control + gz sensor blocks (gated behind that arg in
# big_bertha.urdf.xacro) are skipped, leaving plain links/joints/meshes.
#
# xacro's $(find big_bertha_description) needs the package on the ROS 2
# package index. The Gazebo path runs inside a Jazzy Docker container where
# the workspace is built; Isaac Sim runs natively on the host, which only has
# ROS 2 Humble installed. big_bertha_description has no distro-specific code
# (pure ament_cmake install of urdf/meshes/launch), so it builds fine under
# Humble -- this builds a throwaway host-side workspace for that purpose only.
#
# Isaac's URDF importer resolves "package://NAME/..." by walking up from the
# URDF file to find a directory named NAME, so the expanded URDF is written
# into a scratch tree shaped like the real package (urdf/ next to a symlinked
# meshes/), not next to the real xacro.
set -eo pipefail

# readlink -f, not just dirname on BASH_SOURCE directly: when this script is
# invoked through a colcon --symlink-install share/ path (e.g. via
# `ros2 launch big_bertha_sim_bringup ...`), BASH_SOURCE is the symlink's own
# path (share/big_bertha_sim_bringup/scripts/isaac/prepare_urdf.sh), and bash
# does not resolve it -- the "../../.." below would then land inside the
# install tree instead of the repo root. Resolving the symlink first makes
# this work identically whether run from the git checkout directly or
# through an installed/symlinked package.
REPO_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../.." && pwd)"
WS="$REPO_ROOT/.cache/isaac_ws"
OUT_PKG="$REPO_ROOT/.cache/isaac_urdf/big_bertha_description"

mkdir -p "$WS/src"
ln -sfn "$REPO_ROOT/big_bertha_description" "$WS/src/big_bertha_description"

if [ ! -f "$WS/install/setup.bash" ]; then
  (
    cd "$WS"
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select big_bertha_description \
      > /dev/null
  )
fi

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

mkdir -p "$OUT_PKG/urdf"
ln -sfn "$REPO_ROOT/big_bertha_description/meshes" "$OUT_PKG/meshes"

xacro "$REPO_ROOT/big_bertha_description/urdf/big_bertha.urdf.xacro" \
  use_gz:=false > "$OUT_PKG/urdf/big_bertha.urdf"

echo "$OUT_PKG/urdf/big_bertha.urdf"
