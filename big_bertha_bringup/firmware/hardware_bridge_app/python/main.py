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
# Stub Python component for the hardware_bridge_app.
#
# `arduino-app-cli app start` requires a python/main.py to build and flash the
# MCU sketch, so this no-op stub exists purely to satisfy that check. The real
# bridge is the native C++ hardware_bridge_node (ROS 2), which talks to the
# arduino-router MsgPack-RPC unix socket directly. This process intentionally
# does nothing: no TCP relay, no ports, no Bridge RPC.
import threading


def main():
    # Block forever without waking. Polling round an hourly sleep achieved the
    # same thing while looking like it was meant to do something on each pass.
    threading.Event().wait()


if __name__ == '__main__':
    main()
