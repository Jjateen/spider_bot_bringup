from arduino.app_utils import App, Bridge


PCA9685 = 0x40
MPU6050 = 0x68


def on_scan(addrs):
    print(f"I2C scan: {len(addrs)} device(s) found")
    for a in sorted(addrs):
        tag = ""
        if a == PCA9685:
            tag = " (PCA9685)"
        elif a == MPU6050:
            tag = " (MPU6050)"
        print(f"  0x{a:02X}{tag}")


def main():
    Bridge.provide("i2c_scan_result", on_scan)
    App.run()


if __name__ == "__main__":
    main()
