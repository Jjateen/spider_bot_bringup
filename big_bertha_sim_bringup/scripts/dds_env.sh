#!/usr/bin/env bash
# Source before `ros2 launch` to opt into Cyclone DDS (+ iceoryx shared memory)
# instead of the default Fast DDS. Reversible: unset RMW_IMPLEMENTATION to revert.
#   source $(ros2 pkg prefix big_bertha_sim_bringup)/share/big_bertha_sim_bringup/scripts/dds_env.sh
_dds_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_dds_env_dir}/../config/dds/cyclonedds.xml"
unset _dds_env_dir
