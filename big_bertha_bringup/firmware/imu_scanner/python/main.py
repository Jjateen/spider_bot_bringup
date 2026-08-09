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
# IMU Scanner — standalone diagnostic tool for the Big Bertha UNO Q.
#
# Probes the BNO055 and PCA9685 on the M33's I2C bus via Bridge RPC.
# Runs in continuous polling mode — output goes to logs (no stdin).
#
# Run:
#   arduino-app-cli app start ~/ArduinoApps/imu_scanner
#   arduino-app-cli app logs -f imu_scanner    # tail live data
#   arduino-app-cli app stop imu_scanner
#
# The M33 pushes IMU data from loop() at ~1 Hz — no request needed.

import time

from arduino.app_utils import App, Bridge

# ── State ─────────────────────────────────────────────────────────────────

# IMU data is pushed by the M33 from loop() at ~1 Hz (Bridge.notify inside
# provider handlers is not reliably delivered, so request-response doesn't
# work for IMU). The bus / PCA / servo handlers use the request-response
# pattern since those are triggered less frequently.

imu_data = None
bus_scan = None
pca_result = None

# ── Bridge RPC handlers ──────────────────────────────────────────────────


def on_imu_data(found, qw, qx, qy, qz, ax, ay, az, gx, gy, gz):
    global imu_data
    imu_data = {
        'found': bool(found),
        'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
        'ax': ax, 'ay': ay, 'az': az,
        'gx': gx, 'gy': gy, 'gz': gz,
    }


def on_bus_scan_result(addrs):
    global bus_scan
    bus_scan = list(addrs)


def on_pca9685_result(present, mode1, pre_scale, ai_ok):
    global pca_result
    pca_result = {
        'present': bool(present),
        'mode1': mode1,
        'pre_scale': pre_scale,
        'ai_ok': bool(ai_ok),
    }


# ── Output helpers ───────────────────────────────────────────────────────

def log_imu():
    if imu_data is None:
        return
    r = imu_data
    if r['found']:
        print(
            f'[imu] FOUND'
            f'  qw={r["qw"]:7.4f} qx={r["qx"]:7.4f} qy={r["qy"]:7.4f} qz={r["qz"]:7.4f}'
            f'  ax={r["ax"]:7.3f}  ay={r["ay"]:7.3f}  az={r["az"]:7.3f}'
            f'  gx={r["gx"]:7.4f}  gy={r["gy"]:7.4f}  gz={r["gz"]:7.4f}'
        )
    else:
        print('[imu] NOT FOUND  at 0x28')


def log_bus():
    if bus_scan is None:
        return
    if not bus_scan:
        print('[bus] No I2C devices found — bus may be locked')
        return
    names = {0x40: 'PCA9685', 0x28: 'BNO055', 0x29: 'BNO055'}
    for addr in sorted(bus_scan):
        label = names.get(addr, '')
        line = f'[bus] 0x{addr:02X}'
        if label:
            line += f'  ←  {label}'
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
        f'[pca] FOUND'
        f"  mode1=0x{r['mode1']:02X}"
        f'  pre_scale={r["pre_scale"]}'
        f'  ai={ai}'
    )


# ── Polling ──────────────────────────────────────────────────────────────

SAMPLE = 0
started = False


def startup_scan():
    print('[startup] Scanning I2C bus...')
    Bridge.notify('scan_bus')
    time.sleep(0.3)
    log_bus()
    Bridge.notify('check_pca9685')
    time.sleep(0.3)
    log_pca()


def log_latest_imu():
    global SAMPLE
    SAMPLE += 1
    print(f'[sample #{SAMPLE}]', end=' ')
    log_imu()


def user_loop():
    global started
    if not started:
        started = True
        startup_scan()
        return

    log_latest_imu()


# ── Entry point ──────────────────────────────────────────────────────────

def main():
    Bridge.provide('imu_data', on_imu_data)
    Bridge.provide('bus_scan_result', on_bus_scan_result)
    Bridge.provide('pca9685_result', on_pca9685_result)

    App.run(user_loop=user_loop)


if __name__ == '__main__':
    main()
