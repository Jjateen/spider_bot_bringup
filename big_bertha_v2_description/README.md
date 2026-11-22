# Big Bertha v2 description

URDF and referenced meshes/materials from
`spider_bot_training/assets/URDF/big_bertha_v2/big_bertha_v2.urdf`.
Only mesh URI packaging and a provenance comment were changed. Geometry,
joint names, scales, limits, and inertials are preserved. Referenced assets
total approximately 611 MB; no mesh simplification has been applied.

Build and launch from a sourced ROS workspace:

```bash
colcon build --packages-select big_bertha_v2_description
source install/setup.bash
ros2 launch big_bertha_v2_description rsp.launch.py
```

Publishes `/robot_description` and link TF from `/joint_states`. Standalone
launch enables `joint_state_publisher` and uses wall time by default.
For Isaac use `publish_jsp:=false use_sim_time:=true`; the sim bringup passes
these automatically. Isaac then supplies matching joint names and `/clock`.
Isaac owns `odom -> base_link`;
robot_state_publisher owns transforms below `base_link`. Disable duplicate
Isaac link TF publication.

The model has 13 links and 12 movable joints. It currently has no dedicated
IMU or lidar frames; add those only with verified mounting transforms.
The Isaac robot USD remains a separate simulation asset.
