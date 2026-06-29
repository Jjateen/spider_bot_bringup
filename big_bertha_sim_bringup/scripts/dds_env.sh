#!/usr/bin/env bash
# Source before running plain ROS 2 CLI tools (ros2 topic hz, ros2 node list,
# ...) to talk to a demo launched with dds_shm:=true -- those are separate
# processes from the launch tree and don't inherit its env automatically.
# demo_straight.launch.py sets RMW_IMPLEMENTATION + starts RouDi itself by
# default; you don't need to source this for the demo, only for CLI tools
# run alongside it. This profile has SharedMemory OFF (cyclonedds.xml); use
# cyclonedds_shm.xml instead to match an SHM-on process.
_dds_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_dds_env_dir}/../config/dds/cyclonedds.xml"
unset _dds_env_dir
