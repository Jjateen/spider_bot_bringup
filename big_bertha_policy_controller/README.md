# big_bertha_policy_controller

C++ ONNX Runtime gait controller for the Big Bertha quadruped. Turns a
`/cmd_vel` velocity command into the learned PPO gait and streams 12 joint
position targets to `gz_ros2_control`.

## Data flow

    /odom, /imu, /joint_states, /cmd_vel
        -> 52-d observation (48 state dims + 4 gait-clock sin dims;
           see include/.../observation_builder.hpp for the exact layout)
        -> policy.onnx (ONNX Runtime, input "obs"[1,52] -> output "actions"[1,12])
        -> joint_target = 0.25 * action + default_joint_pos  (clamped)
        -> /position_controller/commands  (std_msgs/Float64MultiArray, 12)

Also publishes `spider_msgs/PolicyStatus` and offers the
`set_policy_enabled` (arm/disarm) and `load_policy` (hot-swap onnx) services.

## Build

The CMake configure step downloads ONNX Runtime (C++) automatically if it is
not already present at the default location (`<workspace>/.onnxruntime`). No
manual download is needed for `x86_64` or `aarch64` hosts:

    colcon build --packages-select big_bertha_policy_controller

To use a different path or skip the auto-download, set `ONNXRUNTIME_ROOT`:

    colcon build --packages-select big_bertha_policy_controller \
      --cmake-args -DONNXRUNTIME_ROOT=/opt/onnxruntime

`models/policy.onnx` (the exported Big Bertha PPO weights, ~452 KB) is
committed directly, **not** via Git LFS.

## Run

    ros2 launch big_bertha_policy_controller policy_controller.launch.py

## Sim-transfer status

The v1.0.0 policy transfers to Gazebo Harmonic and walks 1:1 at the demo
speed (0.136 vs Isaac 0.165 m/s at cmd 0.12) after the calf-armature
emulation and mass corrections in big_bertha_description. Higher-speed
tracking still trails Isaac (0.185 vs 0.273 m/s at cmd 0.30) -- a
training-side issue documented in the training repo's HANDOFF_TRAINING.md.
The node guards against NaN/divergence (obs sanitization + target clamping).
