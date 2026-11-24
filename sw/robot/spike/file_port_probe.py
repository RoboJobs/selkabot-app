"""Upload a file to the controller without the SDK's connect race. No motion.

The SDK's TrajectoryJUpLoad / LuaUpload (__FileUpLoad) do three steps and return
-1 from any of them:
  A. XML-RPC FileUpload(type, name) on port 20003 -> controller says "ready" (0)
  B. TCP connect to port 20010                    -> opens about 50 ms after A
  C. send header + file + end, read reply         -> controller says "SUCCESS"
The SDK connects once, immediately after A, so B is usually refused. upload_file()
retries B, then always runs C. See the design doc, "Controller file transfer".

WARNING: after A, connecting and closing without sending leaves an EMPTY file at
the target path (observed). upload_file() always sends once connected.

Usage (from the repo root):
    uv run python sw/robot/spike/file_port_probe.py FILE           # type 20 (traj)
    uv run python sw/robot/spike/file_port_probe.py FILE --type 0  # type 0 (Lua)

Only types 0 and 20 are allowed; the others are firmware and upgrade packages.
Type 20 files land at /fruser/traj/<basename>.
"""

import hashlib
import os
import socket
import sys
import time

from fairino import Robot

ROBOT_IP = "192.168.58.2"  # real robot
DATA_PORT = 20010
FILE_TYPE_TRAJ_J = 20  # protocol manual 5.1.50: 20 = Trajectory J file
ALLOWED_TYPES = (0, 20)  # 0 = Lua file
POLL_S = 5.0
POLL_STEP_S = 0.05


def wait_for_port(ip, port, limit_s):
    """Return (socket, seconds) once the port accepts, or (None, last_error)."""
    start = time.monotonic()
    last = None
    while time.monotonic() - start < limit_s:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1)
        try:
            s.connect((ip, port))
            return s, time.monotonic() - start
        except OSError as e:
            last = e
            s.close()
            time.sleep(POLL_STEP_S)
    return None, last


def send_file(sock, data):
    """Same framing as the SDK's __FileUpLoad (Robot.py:7181)."""
    total = len(data) + 46 + 4
    md5 = hashlib.md5(data).hexdigest()
    sock.settimeout(20)
    sock.sendall(f"/f/b{total:10d}{md5}".encode())
    sock.sendall(data)
    sock.sendall(b"/b/f")
    time.sleep(0.5)
    return sock.recv(1024)


def upload_file(r, ip, local, file_type=FILE_TYPE_TRAJ_J, log=print):
    """Upload `local` with steps A, B (retried), C. Return (ok, stage, detail).

    stage is "request", "connect", "transfer" or "done".
    """
    if file_type not in ALLOWED_TYPES:
        raise ValueError("file_type must be one of %s" % (ALLOWED_TYPES,))
    name = os.path.basename(local)
    with open(local, "rb") as f:
        data = f.read()

    rtn = r.robot.FileUpload(file_type, name)
    log("A. FileUpload(%d, %r) -> %s" % (file_type, name, rtn))
    if rtn != 0:
        return False, "request", rtn

    sock, info = wait_for_port(ip, DATA_PORT, POLL_S)
    if sock is None:
        log("B. port %d did not open within %.1f s: %r" % (DATA_PORT, POLL_S, info))
        return False, "connect", info
    log("B. port %d accepted after %.3f s" % (DATA_PORT, info))
    try:
        reply = send_file(sock, data)
    finally:
        sock.close()
    log("C. controller reply: %r" % reply[:64])
    if reply[:7] != b"SUCCESS":
        return False, "transfer", reply[:64]
    return True, "done", None


def main():
    args = sys.argv[1:]
    file_type = FILE_TYPE_TRAJ_J
    if "--type" in args:
        i = args.index("--type")
        file_type = int(args[i + 1])
        del args[i : i + 2]
    if file_type not in ALLOWED_TYPES or len(args) != 1:
        print(__doc__)
        return
    local = os.path.abspath(args[0])
    if not os.path.exists(local):
        print("Stop: local file not found:", local)
        return

    r = Robot.RPC(ROBOT_IP)
    try:
        ok, stage, detail = upload_file(r, ROBOT_IP, local, file_type)
        print("Result:", "upload succeeded" if ok else "failed at %s" % stage)
    finally:
        r.CloseRPC()


if __name__ == "__main__":
    main()
