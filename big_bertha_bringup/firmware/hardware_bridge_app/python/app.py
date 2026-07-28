#!/usr/bin/env python3
# Big Bertha Hardware Bridge App — Python entry point
#
# Runs the TCP relay that bridges the ROS 2 C++ node (TCP socket) to the
# STM32U585 sketch (Bridge RPC over UART).  This is the entry point that
# arduino-app-cli invokes for the "python-relay" brick.
#
# Protocol: JSON lines over TCP on 127.0.0.1:50007
#   Request:  {"cmd":"servo","pwms":[12 ints]}\n
#   Response: {"ok":true}\n
#   Request:  {"cmd":"imu"}\n
#   Response: {"ax":f,"ay":f,"az":f,"gx":f,"gy":f,"gz":f}\n

from main import main  # relay logic lives in main.py

if __name__ == "__main__":
    main()
