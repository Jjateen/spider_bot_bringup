#!/usr/bin/env python3
import os

target = {"68473", "69817"}
for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    try:
        fd_dir = f"/proc/{pid}/fd"
        for fd in os.listdir(fd_dir):
            try:
                link = os.readlink(f"{fd_dir}/{fd}")
                if link.startswith("socket:["):
                    inode = link[8:-1]
                    if inode in target:
                        cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode()
                        print(f"PID {pid} FD {fd}: {cmd.strip()}")
            except (FileNotFoundError, PermissionError):
                pass
    except (FileNotFoundError, PermissionError):
        pass
