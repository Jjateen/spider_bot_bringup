# Issues — Spider Bot Bringup

## P0 — Robot won't stand or walk

| # | Title | Description | Tags | File |
|---|---|---|---|---|
| 1 | ~~`fix(config): default_joint_pos mismatches training asset`~~ | ✅ Fixed: `ros2_control.xacro` knees `0.5→-0.32`, ankles `0.0→2.00`. All 5 config files updated to ankles `2.00`. | `bug` `fix` `done` | — |
| 2 | `fix(hardware_bridge): rate limiter at 0.12 rad/step allows 4× servo overspeed` | Rate limiter exists at `hardware_bridge_node.cpp:202-206` (0.12 rad/step) but allows 0.12×200 = **24 rad/s** vs MG995 max **6.54 rad/s** — 4× overspeed. Servos perpetually lag, accumulating positional error the policy never sees (see Issue #15). At 50 Hz effective rate: 0.12×50 = 6 rad/s, still dangerously close to limit. Reduce to **0.03 rad/step** (6.54/200) and make configurable via YAML. Also add EWMA smoothing on outbound targets to filter 50 Hz policy steps. | `bug` `fix` | `hardware_bridge_node.cpp:202-206` |
| 15 | `fix(control): synthetic joint feedback creates positive feedback loop` | `legged_odometry_node.cpp:74-87` echoes **commanded** positions as `/joint_states`. Policy reads this as `joint_pos - default_joint_pos` in its observation (`observation_builder.hpp:98-99`). Since `joint_pos == target_pos` (what the policy just commanded), tracking error appears zero regardless of real servo lag. The policy never "sees" physical lag and generates progressively more aggressive actions — the gait **oscillates violently**. In simulation this is masked by Gazebo's real `joint_state_broadcaster` overwriting synthetic values. Fix: add a 1st-order servo dynamics filter in leg_odometry so feedback reflects realistic servo lag. | `bug` `fix` `critical` | `legged_odometry_node.cpp:58-92` `observation_builder.hpp:98-99` |

## P1 — Sensor orientation

| # | Title | Description | Tags | File |
|---|---|---|---|---|
| 3 | `fix(imu): verify MPU9250 orientation — may be 180° flipped` | MPU9250 on STM32 carrier may be physically rotated 180° relative to body. Policy uses `projected_gravity_b` (obs[6:9]) to know which way is down. If IMU X points backward, the gait direction inverts and the robot walks backward. Test: tilt nose-down, read `ax` — positive = correct, negative = flipped. | `bug` `verification` | URDF / node |
| 16 | `fix(steering): PI integrator windup on warmup→moving transition` | `steer_i_` integrates heading error during warmup (3s) and gated-stance idle. It's reset only on `just_stopped` transitions (`policy_controller_node.cpp:380`), NOT on warmup→moving. If the robot drifts or is placed at an angle during warmup, the accumulated integral applies a sudden steering jerk when the gait activates. Fix: reset `steer_i_ = 0.0` on `warming→!warming` and `!moving→moving` transitions; also rate-limit `steer_cmd_` changes. | `bug` `fix` | `policy_controller_node.cpp:437-443` |

## P2 — Calibration needs physical verification

| # | Title | Description | Tags | File |
|---|---|---|---|---|
| 4 | `calibration(servo): verify direction, offset, and physical limits per joint` | `servo_direction` has two `+1` entries (rest `-1`). `servo_offset` ranges 0-10°. `servo_lower_limit`/`servo_upper_limit` define physical stops. None have been verified by commanding each joint independently. A wrong direction makes the joint move opposite to policy expectation. | `verification` `calibration` | `hardware_bridge.yaml` |
| 5 | `calibration(servo): verify PCA9685 channel wiring matches servo_channel config` | `servo_channel` maps Isaac index → PCA9685 output pin. Physical wiring done by hand — one swapped channel means commanding the hip moves the knee. | `verification` | `hardware_bridge.yaml` |
| 6 | ~~`question(servo): verify ankle default (1.82 rad) is within physical servo range`~~ | ✅ Resolved: default changed to `2.00 rad` (training match). After conversion chain: `(2.00 - 1.57) * 57.3 * -1 + 0 + 90 = 65.4°` — well within MG995 range (40-180°). | `question` `verification` `done` | — |
| 17 | `tuning: multiple outer loops sum to aggressive combined heading correction` | Heading hold (`heading_kp=2.0`), hip-bias steering (`steer_kp=1.8`, `steer_ki=0.8`), and lateral-hold cross-track yaw bias (`lateral_yaw_kp=0.4`) all act on heading error simultaneously. Combined effective gain ~4.2× on heading error causes **aggressive overshoot corrections** when transitioning from stand→walk. Reduce gains: `heading_kp` 2.0→0.5, `steer_kp` 1.8→1.2, `steer_ki` 0.8→0.4, `lateral_yaw_kp` 0.4→0.2. | `enhancement` `tuning` | `policy.yaml:43-78` |

## P3 — Pipeline architecture

| # | Title | Description | Tags | File |
|---|---|---|---|---|
| 7 | `architecture: JointEffortPdController bypassed on real hardware — add software damping` | Sim runs PD at 500 Hz (Kp=20, Kd=2) providing smooth damped motion. On real hardware, the bridge converts rad→PWM directly — zero damping. The servo's internal PID acts as infinite stiffness. No compliance means the gait will be rigid and oscillations go uncorrected. Addressed by the rate limiter (Issue #2) and EWMA smoothing on outbound targets. | `architecture` `enhancement` | `hardware_bridge_node.cpp` |
| 8 | `fix(perf): increase I2C bus speed from 50 kHz to 400 kHz` | `Wire.setClock(50000)` limits I2C to 50 kHz. Each 12-channel servo write takes ~2.4 ms. Fast mode (400 kHz) would cut this to ~0.3 ms, freeing bandwidth for the 20 Hz IMU poll. The PCA9685 and MPU9250 both support 400 kHz. | `fix` `performance` | `sketch.ino:484` |
| 18 | `enhancement: add servo dynamics simulation in leg_odometry for realistic feedback` | No encoders on MG995 servos — real joint position is unknown. Leg_odometry publishes commanded pos as "measured" (Issue #15). Add a 1st-order low-pass filter (EWMA, `tau ≈ 0.06s` matching MG995 BW) to produce realistic lagged positions. This lets the policy see tracking error as it would in sim, breaking the positive feedback loop. | `enhancement` `fix` | `legged_odometry_node.cpp` |

## P4 — Launch / deployment

| # | Title | Description | Tags | File |
|---|---|---|---|---|
| 14 | `fix(launch): joint_state_publisher not launched — policy never publishes commands` | `big_bertha.launch.py` omits `publish_jsp:=true` when including `rsp.launch.py`. The policy controller's `control_loop()` gates on `!have_joints_` (`policy_controller_node.cpp:489`) and returns early without ever publishing to `/position_controller/commands`. Without joint state feedback, the servos never receive targets (limp). Fix: add `publish_jsp: 'true'` to the RSP launch include arguments. | `bug` `fix` | `big_bertha.launch.py` `policy_controller_node.cpp:489` |
| 19 | `fix(launch): goal_y default 3.5 aims at blocked diagonal instead of bottom corridor` | `demo.launch.py:161` defaults `goal_y='3.5'`, aiming at (3.5, 3.5) — the diagonal blocked by box_1/pillar_1/box_2. Every docstring (lines 34-41, 162) and the patrol convention (goal3 at (3.5, -3.5)) say B should be (3.5, -3.5), the straight East bottom-corridor traverse the gait can manage. The docstring explicitly warns the diagonal "needs hard turns the gait cannot make." Fix: change `default_value='3.5'` to `'-3.5'` on line 161. | `bug` `fix` | `demo.launch.py:160-162` |
| 20 | `fix(launch): map_server loads amcl.yaml config it doesn't need` | `localization.launch.py:74` passes `amcl_config` (the entire `amcl.yaml`) as a parameter to `map_server`. The file contains both `amcl:` and `map_server:` blocks; `map_server` only consumes its own block. Harmless — unused params are ignored — but sloppy and misleading. Fix: remove `amcl_config` from `map_server`'s `parameters` list at lines 73-74. | `bug` `cleanup` | `localization.launch.py:73-74` |
| 21 | `fix(slam): scan_topic uses raw /scan instead of filtered /scan_filtered` | `slam_toolbox.yaml:20` sets `scan_topic: /scan`. The body-mounted lidar tilts with the gait, producing floor returns ("ghost walls") that slam_toolbox bakes into the saved map, corrupting it for later known-map AMCL. All other consumers (AMCL, costmaps) use `/scan_filtered`. Fix: change to `scan_topic: /scan_filtered`. | `bug` `fix` | `slam_toolbox.yaml:20` |
| 22 | `fix(dep): missing nav2_navfn_planner exec_depend` | `nav2_params.yaml:287` configures the planner as `plugin: "nav2_navfn_planner::NavfnPlanner"` but `package.xml` does not list `nav2_navfn_planner` as an `exec_depend`. If not pulled transitively, `planner_server` fails to load the plugin at runtime. Fix: add `<exec_depend>nav2_navfn_planner</exec_depend>`. | `bug` `fix` | `package.xml` `nav2_params.yaml:287` |
| 23 | `fix(config): dead amcl section in nav2_params.yaml has wrong scan_topic` | `nav2_params.yaml:39` has a stale `amcl:` section with `scan_topic: scan` (relative, no leading `/`, not `/scan_filtered`). This section is unused (AMCL loads from `amcl.yaml`), but if launch order ever changed it would match against raw ghost-wall scans. Fix: delete the unused `amcl:` section from `nav2_params.yaml`. | `bug` `cleanup` | `nav2_params.yaml:39` |
| 24 | `fix(config): 162 lines of dead config for 6 unlaunched Nav2 modules` | `nav2_params.yaml:328-489` fully configures `waypoint_follower`, `route_server`, `velocity_smoother`, `collision_monitor`, `docking_server`, and `loopback_simulator`. None are launched — `nav2.launch.py:81-83` explicitly omits them. The sections reference packages not in `package.xml` (`opennav_docking`, `nav2_route`), which would cause missing-plugin errors if triggered. Fix: delete all six sections. | `bug` `cleanup` | `nav2_params.yaml:328-489` |
| 25 | `fix(config): dead static_layer blocks in both costmaps` | `nav2_params.yaml:206-208, 255-260` define `static_layer` parameter blocks for both costmaps, but neither costmap includes `static_layer` in its `plugins:` list (local uses `voxel_layer+inflation_layer`, global uses `obstacle_layer+inflation_layer`). Orphaned config with no effect. Fix: delete the dead blocks. | `bug` `cleanup` | `nav2_params.yaml:206-208, 255-260` |
| 26 | `fix(launch): dead x/y/yaw args in localization.launch.py never consumed` | `localization.launch.py:147-149` declares `x`, `y`, `yaw` args which are passed from `bringup.launch.py:138-139` but never referenced anywhere in the launch. If someone intended these to seed the AMCL `initial_pose`, they are silently ignored. Fix: remove the dead args from both files. | `bug` `cleanup` | `localization.launch.py:147-149` `bringup.launch.py:138-139` |
| 27 | `fix(launch): OnProcessExit chain breaks when spawn_controllers=false` | `simulation.launch.py:178-183` chains `spawn_robot -> jsb_spawner -> position_spawner` via `OnProcessExit`. Both spawners have `condition=IfCondition(spawn_controllers)`. If `spawn_controllers` is false, `jsb_spawner` is never created, so its `OnProcessExit` never fires and `position_spawner` stays dead. All callers pass `true`, so not hit in production, but a latent trap. Fix: conditionally register the event handlers. | `bug` `latent` | `simulation.launch.py:178-183` |
| 28 | `fix(doc): rviz.launch.py docstring omits straight config` | `rviz.launch.py:20-23` lists available configs as `simulation`, `mapping`, `planning`, `integration` but omits `straight`, even though `config/rviz/straight.rviz` exists and is used by `demo_straight.launch.py:161`. Fix: add `straight` to the docstring list. | `docs` | `rviz.launch.py:20-23` |
| 29 | `docs: three conflicting A→B coordinate descriptions` | `worlds/obstacle_world.sdf:15` says B = (3.5, 3.5), `bringup.launch.py:206-208` describes the "bottom corridor (y=-3.5)", and `demo.launch.py:161` defaults to (3.5, 3.5) but its description says world B = -3.5. Contradictory coordinate stories for the same demo. Fix: reconcile all docs to the same goal coordinate. | `docs` | `obstacle_world.sdf:15` `bringup.launch.py:206-208` `demo.launch.py:34-41,160-162` |
| 30 | `fix(tf): map frame never published in ground-truth mode` | `localization.launch.py:97-109` used `tf2_ros::static_transform_publisher` to publish `map->odom` identity. That executable publishes a single transform at construction with a wall-clock timestamp. When `use_sim_time` is active and `/clock` starts later, `tf2_ros::Buffer::onTimeJump` detects the clock change and clears its transform cache, permanently losing the one-shot static transform. Nav2's global costmap (`global_frame: map`) cannot find the `map` frame → lifecycle activation stalls → `bt_navigator` says "Action server is inactive." Fix: replace with a custom `map_to_odom_publisher` that re-publishes the identity transform on a 10 s timer, surviving buffer clears. | `bug` `fix` | `localization.launch.py:97-109` `src/map_to_odom_publisher.cpp` |

## P5 — Config validation (compliance checks)

| # | Title | Description | Tags | File |
|---|---|---|---|---|
| 9 | `compliance: add cross-config array length and joint order validation` | Validate all 12-element arrays (`servo_direction`, `servo_channel`, `joint_names`, `default_joint_pos`) have length 12 and agree on joint order. Catches misaligned configs before they reach the robot. | `enhancement` `compliance` | `scripts/check_config_sanity.py` |
| 10 | `compliance: add PCA9685 channel uniqueness check` | `servo_channel` must have 12 unique values in 0-15 range. Two servos on the same channel is undefined behavior. | `enhancement` `compliance` | `scripts/check_config_sanity.py` |
| 11 | `compliance: add PWM range sanity check` | Verify `pwm_min < pwm_mid < pwm_max` and all values within 0-4095 range. Invalid PWM values silently produce wrong servo positions. | `enhancement` `compliance` | `scripts/check_config_sanity.py` |
| 12 | `compliance: add URDF ↔ ros2_control joint cross-reference check` | Every joint declared in `ros2_control.yaml` must exist in the URDF, and vice versa. An unreferenced joint silently does nothing. | `enhancement` `compliance` | `scripts/check_config_sanity.py` |
| 13 | `compliance: add ONNX model input/output dimension validation` | Validate `policy.onnx` has 48-input / 12-output dimensions matching the code constants (`kObsDim`, `kActionDim`). A mismatched model silently produces garbage inference. | `enhancement` `compliance` | `scripts/check_config_sanity.py` |

---

### Quick reference

```
Prefixes:   fix / calibration / compliance / architecture / question
Priority:   P0 (blocker → robot won't stand) → P1 (critical → wrong direction)
              → P2 (needs physical verification) → P3 (improvement)
              → P4 (tooling / compliance)
Tags:       bug / fix / verification / calibration / enhancement / architecture
              / performance / compliance / question
```
