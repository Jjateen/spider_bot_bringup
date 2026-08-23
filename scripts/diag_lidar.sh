#!/usr/bin/env bash
# One-shot lidar diagnosis. Run it, paste the whole output back.
#
# Answers, in order: does the device node exist, does it point at the LIDAR or
# at some other USB-serial device, is the driver running, is it publishing, and
# what did the kernel see. Each section is separately useful, so it keeps going
# rather than stopping at the first failure.
echo "=================== 1. device node ==================="
ls -l /dev/ttyLIDAR 2>&1
echo "--- all USB serial devices present ---"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1

echo
echo "=================== 2. WHAT is it ==================="
# The important one. 99-big-bertha-lidar.rules matches three vendor ids
# (CP210x, CH341, FTDI) because nobody had confirmed the adapter, so the
# symlink can land on the wrong device if more than one is attached. A driver
# that opens the wrong port stays alive and silent, which is indistinguishable
# from a dead lidar unless you check here.
lsusb 2>&1
for d in /dev/ttyUSB*; do
  [ -e "$d" ] || continue
  echo "--- $d ---"
  udevadm info -q property -n "$d" 2>&1 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|DEVLINKS" || true
done

echo
echo "=================== 3. is the port readable ==================="
# If the port opens but returns nothing, the lidar is not talking: wrong
# baud, wrong model settings, or the motor has no power. X2 is 115200.
if [ -e /dev/ttyLIDAR ]; then
  timeout 3 stty -F /dev/ttyLIDAR 115200 raw -echo 2>&1
  echo "reading 3s at 115200 (any hex output below = the device IS talking):"
  timeout 3 od -A x -t x1z /dev/ttyLIDAR 2>&1 | head -5
else
  echo "SKIP: /dev/ttyLIDAR does not exist"
fi

echo
echo "=================== 4. ROS side ==================="
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(ros2 pkg prefix big_bertha_bringup 2>/dev/null)/share/big_bertha_bringup/config/dds/cyclonedds.xml"
pgrep -af ydlidar || echo "driver NOT running"
echo "--- /scan ---"
timeout 8 ros2 topic hz /scan 2>&1 | head -2
echo "--- tf frames that exist ---"
timeout 8 ros2 topic echo /tf_static --once --field transforms 2>/dev/null | grep -E "frame_id|child" | head -6

echo
echo "=================== 5. kernel ==================="
# error -71 / -32 here means the rail is browning out (power), not software.
echo arduinoverse | sudo -S dmesg 2>/dev/null | grep -iE "usb|tty|cp210|ch341|ftdi" | tail -15
