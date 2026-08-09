# Big Bertha Hardware Bridge App

Arduino App for the UNO Q STM32U585 co-processor.

Uses **Bridge RPC** — the sketch registers providers (`set_servo_pwms`,
`scan_i2c`, `ping`, `servo_diag`) and pushes notifications (`imu`,
`hw_status`, `imu_diag`, `i2c_scan`, `servo_diag_result`, `pong`) over the
internal UART, which the `arduino-router` manages transparently.

## Upload & run

```bash
# 1. Copy this App to the UNO Q
scp -r firmware/hardware_bridge_app user@<uno-q-ip>:~/ArduinoApps/

# 2. SSH into the UNO Q and flash the sketch
arduino-app-cli app start ~/ArduinoApps/hardware_bridge_app

# 3. On the ROS host, launch the C++ bridge node (talks to the router
#    socket directly — no Python relay, no TCP):
ros2 launch big_bertha_bringup hardware_bringup.launch.py
```

## Architecture

```
ROS 2 Node (C++) ──MsgPack-RPC unix socket── arduino-router ──UART── STM32 sketch
  Sub: /position_controller/commands                   Providers:
  Pub: /imu (fused), /imu/mag                             set_servo_pwms(pwms)
  Svc: ~/status, ~/scan_i2c, ~/servo_diag, ~/imu_diag    scan_i2c, ping, servo_diag
```

## Diagnostics

The C++ node exposes the firmware's diagnostics as ROS 2 services:

| Service | Purpose |
|---|---|
| `~/status` | Last `hw_status` (I2C present, PCA9685/BNO055 IMU health, counters) |
| `~/scan_i2c` | Triggers an I2C bus scan, returns device addresses |
| `~/servo_diag` | Runs the on-MCU servo write/readback diagnostic |
| `~/imu_diag` | Last BNO055 identity/fusion-mode/calibration-status diagnostic string |
