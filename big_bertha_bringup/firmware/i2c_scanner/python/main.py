#!/usr/bin/env python3
# Copyright 2026 Jjateen Gundesha
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# I2C Scanner — scans all three I2C buses on the UNO Q and
# reports devices found.
#
# Run:
#   arduino-app-cli app start ~/ArduinoApps/i2c_scanner
#   arduino-app-cli app logs -f i2c_scanner
#   arduino-app-cli app stop i2c_scanner

import time

from arduino.app_utils import App, Bridge

BUS_LABELS = {0: 'Wire (D20/D21)', 1: 'Wire1 (Qwiic)', 2: 'Wire2 (A4/A5)'}
DEVICE_NAMES = {0x40: 'PCA9685', 0x68: 'MPU9250'}

bus_scans = {}
buses_done = set()
pca_result = None
last_scan_ts = 0


def on_bus_device(bus_id, addr):
    if bus_id not in bus_scans:
        bus_scans[bus_id] = []
    bus_scans[bus_id].append(addr)


def on_bus_done(bus_id):
    buses_done.add(bus_id)


def on_pca9685_result(present, mode1, pre_scale, ai_ok):
    global pca_result
    pca_result = {
        'present': bool(present),
        'mode1': mode1,
        'pre_scale': pre_scale,
        'ai_ok': bool(ai_ok),
    }


def log_bus(bus_id, devices):
    label = BUS_LABELS.get(bus_id, f'Bus {bus_id}')
    if not devices:
        print(f'[bus] {label}: no devices found')
        return
    print(f'[bus] {label}:')
    for addr in sorted(devices):
        name = DEVICE_NAMES.get(addr, '')
        line = f'       0x{addr:02X}'
        if name:
            line += f'  <-  {name}'
        print(line)


def log_pca():
    if pca_result is None:
        return
    r = pca_result
    if not r['present']:
        print('[pca] PCA9685 at 0x40: NOT FOUND')
        return
    ai = 'OK' if r['ai_ok'] else 'FAIL'
    print(
        f'[pca] FOUND  mode1=0x{r["mode1"]:02X}'
        f'  pre_scale={r["pre_scale"]}  ai={ai}'
    )


def print_results():
    any_found = False
    for bus_id in sorted(bus_scans.keys()):
        devices = bus_scans[bus_id]
        log_bus(bus_id, devices)
        if devices:
            any_found = True
    if not any_found:
        print('[bus] *** NO DEVICES FOUND ON ANY BUS ***')
    log_pca()


def trigger_scan():
    global bus_scans, buses_done, pca_result, last_scan_ts
    bus_scans = {}
    buses_done = set()
    pca_result = None
    Bridge.notify('scan_bus')
    time.sleep(0.5)
    Bridge.notify('check_pca9685')
    time.sleep(0.3)
    print_results()
    last_scan_ts = time.time()


def user_loop():
    global last_scan_ts  # noqa: PLW0602
    if last_scan_ts == 0:
        print('[startup] Scanning all 3 I2C buses...')
        trigger_scan()
        return
    if time.time() - last_scan_ts >= 10:
        print('--- re-scan ---')
        trigger_scan()


def main():
    Bridge.provide('bus_device', on_bus_device)
    Bridge.provide('bus_done', on_bus_done)
    Bridge.provide('pca9685_result', on_pca9685_result)
    App.run(user_loop=user_loop)


if __name__ == '__main__':
    main()