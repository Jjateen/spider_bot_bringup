#!/usr/bin/env python3
# I2C Scanner — scans all three I2C buses on the UNO Q and reports devices found.
#
# Run:
#   arduino-app-cli app start ~/ArduinoApps/i2c_scanner
#   arduino-app-cli app logs -f i2c_scanner
#   arduino-app-cli app stop i2c_scanner

from arduino.app_utils import App, Bridge
import time

BUS_LABELS = {0: "Wire (D20/D21)", 1: "Wire1 (Qwiic)", 2: "Wire2 (A4/A5)"}
DEVICE_NAMES = {0x40: "PCA9685", 0x68: "MPU6050"}

bus_scans = {}
pca_result = None
last_scan_ts = 0


def on_bus_scan_result(bus_id, addrs):
    bus_scans[bus_id] = list(addrs)


def on_pca9685_result(present, mode1, pre_scale, ai_ok):
    global pca_result
    pca_result = {
        "present": bool(present),
        "mode1": mode1,
        "pre_scale": pre_scale,
        "ai_ok": bool(ai_ok),
    }


def log_bus(bus_id, devices):
    label = BUS_LABELS.get(bus_id, f"Bus {bus_id}")
    if not devices:
        print(f"[bus] {label}: no devices found")
        return
    print(f"[bus] {label}:")
    for addr in sorted(devices):
        name = DEVICE_NAMES.get(addr, "")
        line = f"       0x{addr:02X}"
        if name:
            line += f"  <-  {name}"
        print(line)


def log_pca():
    if pca_result is None:
        return
    r = pca_result
    if not r["present"]:
        print("[pca] PCA9685 at 0x40: NOT FOUND")
        return
    ai = "OK" if r["ai_ok"] else "FAIL"
    print(f"[pca] FOUND  mode1=0x{r['mode1']:02X}  pre_scale={r['pre_scale']}  ai={ai}")


def print_results():
    any_found = False
    for bus_id in sorted(bus_scans.keys()):
        devices = bus_scans[bus_id]
        log_bus(bus_id, devices)
        if devices:
            any_found = True
    if not any_found:
        print("[bus] *** NO DEVICES FOUND ON ANY BUS ***")
    log_pca()


def trigger_scan():
    global bus_scans, pca_result, last_scan_ts
    bus_scans = {}
    pca_result = None
    Bridge.notify("scan_bus")
    time.sleep(0.5)
    Bridge.notify("check_pca9685")
    time.sleep(0.3)
    print_results()
    last_scan_ts = time.time()


def user_loop():
    global last_scan_ts
    if last_scan_ts == 0:
        print("[startup] Scanning all 3 I2C buses...")
        trigger_scan()
        return
    if time.time() - last_scan_ts >= 10:
        print("--- re-scan ---")
        trigger_scan()


def main():
    Bridge.provide("bus_scan_result", on_bus_scan_result)
    Bridge.provide("pca9685_result", on_pca9685_result)
    App.run(user_loop=user_loop)


if __name__ == "__main__":
    main()
