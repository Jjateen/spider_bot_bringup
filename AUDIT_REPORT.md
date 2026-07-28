# Policy Controller Audit Report

## Completed Fixes

### 1. Reduce publish/timer rate: 200 Hz → 50 Hz

**What**
Changed `pd_rate_` from 200 Hz to 50 Hz in both `policy_controller_node.cpp` (default) and `policy.yaml`.

**Why**
- The timer fired at 200 Hz, but `use_effort: false` means no PD is computed in this node — it just echoes `target_pos_` (which only changes at the 50 Hz policy rate)
- Identical position targets were published 4× per policy cycle: wasteful, no benefit
- The PD was moved to the separate `JointEffortPdController` in the ros2_control layer

**How**
- `policy.yaml:31`: `pd_rate: 200.0` → `pd_rate: 50.0`
- `policy_controller_node.cpp:169`: default `200.0` → `50.0`
- `policy_decimation_ = round(50/50) = 1` → policy runs every tick, publishes every tick at 50 Hz

---

### 2. Fix `/odom` QoS: BEST_EFFORT → RELIABLE

**What**
Changed the `/odom` subscription from `rclcpp::SensorDataQoS()` (BEST_EFFORT, depth 5) to `rclcpp::QoS(1)` (RELIABLE, depth 1).

**Why**
| Pub | QoS |
|---|---|
| `legged_odometry_node` (hw) | RELIABLE |
| Gazebo bridge (sim) | RELIABLE |
| Policy subscription (was) | **BEST_EFFORT** ✗ |

All odom publishers use RELIABLE. The policy's heading-hold, lateral-hold, and position-hold outer loops depend on accurate odometry. BEST_EFFORT silently drops messages — the outer loops act on stale data.

**How**
- `policy_controller_node.cpp:199`: `rclcpp::SensorDataQoS()` → `rclcpp::QoS(1)`

---

## Completed Fixes (continued)

### 3. `default_joint_pos` ankles: 1.82 rad → 2.00 rad (matches training)

**What**
The default joint positions used in all deployment configs have ankles at **1.82 rad**, but the training environment's articulation config (`big_bertha.py:104-117`) initializes ankles at **2.00 rad**.

**Why**
Two critical equations depend on `default_joint_pos`:

1. **Observation** (what the policy sees):
   `obs[12:24] = joint_pos - default_joint_pos`
   At the same physical pose, training computed `pos - 2.00`, deployment computes `pos - 1.82`. The 0.18 rad offset shifts the observation out-of-distribution for the ONNX empirical normalizer.

2. **Action output** (what the robot does):
   `target = action_scale * action + default_joint_pos`
   Training: `target = 0.25 * action + 2.00`
   Deployment: `target = 0.25 * action + 1.82`
   Same action → different absolute angle. The robot stands with less knee bend than the policy expects.

**Impact:**
The policy's "zero action" (stand still) produces a pose the policy never trained on → it tries to correct with non-zero actions → the robot oscillates around the wrong equilibrium → **jittery, unstable stance**.

✅ Fixed in 5 files: `policy.yaml`, `policy_controller_node.cpp`, `observation_builder.hpp`, `ros2_control.yaml`, `legged_odometry.yaml`

### 4. `joint_limit: 1.5708` → `3.14159` (stop clipping ankle default)

✅ Fixed in `policy.yaml:23`. Raised from 1.57 to full ±180° so the ankle default (2.00 rad) is no longer clipped.

### 5. `b.ros2_control.xacro` initial values match training defaults

**What**
The Gazebo spawn initial values (`big_bertha.ros2_control.xacro`) set `knee=0.5, ankle=0.0` per leg. Training default is `knee=-0.32, ankle=2.00`.

**Impact**
The robot spawns in a collapsed/awkward pose. The warmup period (3s at default pose) is supposed to correct this, but the transition from spawn→warmup→policy is violent because the spawn pose is so far from the training default.

✅ Fixed in `big_bertha.ros2_control.xacro:36-49`. Knees `0.5→-0.32`, ankles `0.0→2.00`.

---

## Proposed Additions

### 6. Standalone test script

**What**
A Python ROS 2 node at `big_bertha_policy_controller/test/test_policy.py` that feeds synthetic sensor data and verifies the node's output without requiring Gazebo.

**Why**
- The only existing test (`verify_locomotion.sh`) requires a full simulation stack
- No way to validate the observation builder, command processing, or gating logic in isolation
- Cannot be run in CI (no GPU)

**How**
The script will:
1. Launch the policy controller node via `ros2 run`
2. Publish fake `/joint_states` (all at default_joint_pos), `/odom` (zero twist, identity pose), `/imu` (level orientation)
3. Publish a `/cmd_vel` sequence: `[stop] → [vx=0.3] → [vx=0.0]`
4. Subscribe to `/position_controller/commands` and `/policy_status`
5. Assert:
   - Node publishes targets at 50 Hz
   - Action norm > 0 when moving, 0 when gated
   - `vx=0` produces `moving=false` (gated to default pose)
   - `set_policy_enabled(false)` stops publishing
6. Report PASS/FAIL per check, exit code matches result

---

### 7. Debug inspector

**What**
A Python ROS 2 node at `big_bertha_policy_controller/scripts/inspector.py` that subscribes to all policy node topics and dumps the full observation + action chain.

**Why**
- No way to see what the policy "sees" without adding printf to C++ code
- Debugging the 48-d observation requires cross-referencing 5 topics
- The conversion chain (action → rad → deg → PWM) is spread across 2 nodes + firmware

**How**
The inspector will subscribe to: `/joint_states`, `/odom`, `/imu`, `/cmd_vel`, `/policy_status`, `/position_controller/commands` and for each policy cycle log:
```
[Obs] lin_vel: [0.00, 0.00, 0.00]  ang_vel: [0.00, 0.00, 0.00]
[Obs] gravity: [0.00, 0.00, -1.00]  cmd: [0.30, 0.00, 0.00]
[Obs] joint_delta: [0.00, 0.00, ...]  joint_vel: [0.00, 0.00, ...]
[Obs] prev_actions: [0.00, 0.00, ...]
[Act] raw: [0.12, -0.05, ...]  clipped: [0.12, -0.05, ...]
[Act] target_rad: [0.03, -0.01, ...]  (with hip_bias + steer_bias)
[PWM] ch0: 307, ch1: 210, ...  (hardware conversion chain)
```

Optional: `--bag` flag to record all topics to a ROS 2 bag for offline analysis.

---

### 8. Bag recording launch argument

**What**
Add a `record:=true` argument to `policy_controller.launch.py` that starts `ros2 bag record` for all relevant topics.

**Why**
- No bag recording infrastructure exists in the workspace
- Offline replay is essential for diagnosing intermittent gait issues
- PlotJuggler can replay bags for post-mortem analysis

**How**
```
ros2 launch big_bertha_policy_controller policy_controller.launch.py record:=true
```
Launches `ros2 bag record -o gait_<timestamp> /joint_states /odom /imu /cmd_vel /position_controller/commands /policy_status` in a `TimerAction` alongside the node.
