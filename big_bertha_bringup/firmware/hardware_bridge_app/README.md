# Big Bertha Hardware Bridge App

Arduino App for the UNO Q STM32U585 co-processor.

Uses **Bridge RPC** — the sketch registers two providers (`set_servo_pwms`,
`get_imu_data`) callable from the Python side over the internal UART, which
the `arduino-app-cli` manages transparently. No raw UART conflicts.

## Upload & run

```bash
# 1. Copy this App to the UNO Q
scp -r firmware/hardware_bridge_app user@<uno-q-ip>:~/ArduinoApps/

# 2. SSH into the UNO Q and start the App
#    (compiles sketch + uploads to MCU + starts Python container)
arduino-app-cli app start ~/ArduinoApps/hardware_bridge_app

# 3. The Python relay listens on 127.0.0.1:50007. Run the ROS node:
ros2 launch big_bertha_bringup hardware_bringup.launch.py
```

No need to stop `arduino-router` or disable the bridge — the RPC layer
is designed to coexist with the router.

## Architecture

```
ROS 2 Node (C++) ──TCP── Python relay (main.py) ──Bridge RPC── STM32 sketch
  Sub: /position_controller/commands                Providers:
  Pub: /imu                                           set_servo_pwms(pwms)
                                                      get_imu_data() → imu
```

## Protocol (ROS ↔ Python)

JSON lines over TCP on `127.0.0.1:50007`:

| Direction | Request | Response |
|---|---|---|
| ROS → Python | `{"cmd":"servo","pwms":[102,512,...]}\n` | `{"ok":true}\n` |
| ROS → Python | `{"cmd":"imu"}\n` | `{"ax":...,"ay":...,"az":...,"gx":...,"gy":...,"gz":...}\n` |
