#!/usr/bin/env python3
import socket, time, json

s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(3)
s.connect("/var/run/arduino-router.sock")
time.sleep(0.5)
req = json.dumps({"jsonrpc": "2.0", "method": "rpc.list", "id": 1})
s.sendall((req + "\n").encode())
try:
    data = s.recv(4096)
    print("Response:", repr(data))
except socket.timeout:
    print("No response (timeout)")
except Exception as e:
    print("Error:", e)
s.close()
