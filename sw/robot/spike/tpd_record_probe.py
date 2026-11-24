"""Q2 probe: record a short joint-space TPD trajectory on the robot and try to
download it, to learn the controller's trajectory file format.

Records 8 ms samples while the robot moves between two home-near joint sets.
"""

import os
import sys

from fairino import Robot

ROBOT_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.58.2"
NAME = "probe_tpd"
PERIOD_MS = 8
HOME = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
OFFSET = [15.0, -80.0, 80.0, -90.0, -90.0, 0.0]
OUT_DIR = os.path.join(os.path.dirname(__file__), "out")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    r = Robot.RPC(ROBOT_IP)
    r.SetSpeed(10)
    r.MoveJ(HOME, 0, 0, vel=10)
    print("Mode(manual):", r.Mode(1))
    print("DragTeachSwitch(1):", r.DragTeachSwitch(1))
    print("SetTPDParam :", r.SetTPDParam(NAME, PERIOD_MS, 1, 0, 0))
    print("SetTPDStart :", r.SetTPDStart(NAME, PERIOD_MS, 1, 0, 0))
    print("(drag mode: no robot motion sent during recording)")
    print("SetWebTPDStop:", r.SetWebTPDStop())
    print("DragTeachSwitch(0):", r.DragTeachSwitch(0))
    print("Mode(auto):", r.Mode(0))

    local = os.path.join(OUT_DIR, f"{NAME}.txt")
    open(local, "a").close()  # SDK requires the destination to exist
    rtn = r._RPC__FileDownLoad(20, f"{NAME}.txt", local)
    print("download rtn:", rtn)
    if os.path.exists(local) and os.path.getsize(local) > 0:
        with open(local) as f:
            head = [next(f, "") for _ in range(6)]
        print("first lines:")
        for line in head:
            print("   ", repr(line))
    else:
        print("no downloaded content")
    r.CloseRPC()


if __name__ == "__main__":
    main()
