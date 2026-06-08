# big_bertha_policy_controller

C++ ONNX Runtime gait controller for the Big Bertha quadruped. Turns a
`/cmd_vel` velocity command into the learned PPO gait and streams 12 joint
position targets to `gz_ros2_control`.

## Data flow

    /odom, /imu, /joint_states, /cmd_vel
        -> 48-d observation (PLAN.md section 2)
        -> policy.onnx (ONNX Runtime, input "obs"[1,48] -> output "actions"[1,12])
        -> joint_target = 0.25 * action + default_joint_pos  (clamped)
        -> /position_controller/commands  (std_msgs/Float64MultiArray, 12)

Also publishes `spider_msgs/PolicyStatus` and offers the
`set_policy_enabled` (arm/disarm) and `load_policy` (hot-swap onnx) services.

## Build

ONNX Runtime (C++) is not packaged for Jazzy; download it once and point
CMake at it:

    curl -fsSL -o /tmp/ort.tgz \
      https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
    mkdir -p .onnxruntime && tar -xzf /tmp/ort.tgz -C .onnxruntime --strip-components=1
    colcon build --packages-select big_bertha_policy_controller \
      --cmake-args -DONNXRUNTIME_ROOT=$(pwd)/.onnxruntime

`models/policy.onnx` (the exported Big Bertha PPO weights, ~452 KB) is
committed directly, **not** via Git LFS.

## Run

    ros2 launch big_bertha_policy_controller policy_controller.launch.py

## Sim-transfer status

The node is verified to load the policy, build the 48-d observation, run
inference at 50 Hz, and produce correct topic/service I/O. The Isaac-Lab
trained weights, however, do **not** transfer cleanly to Gazebo Harmonic: the
raw action diverges (norm ~1e13) and the targets saturate at the joint clamp.
The node guards against NaN/divergence (obs sanitization + target clamping) so
the controller stays stable, but a Gazebo-side gait demo needs either a
fine-tuned/retrained policy or a sim-to-sim adaptation. Tracked on issue #5.
