"""Q5 probe: can a second Robot.RPC connection stop a motion that the first
connection is blocked in? Uses a slow joint move with a known target.
"""

import sys
import threading
import time

from fairino import Robot

ROBOT_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.58.2"
HOME = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
FAR = [60.0, -90.0, 90.0, -90.0, -90.0, 0.0]


def main():
    main_conn = Robot.RPC(ROBOT_IP)
    main_conn.SetSpeed(5)
    main_conn.MoveJ(HOME, 0, 0, vel=5)

    result = {}

    def long_move():
        t0 = time.time()
        result["move_rtn"] = main_conn.MoveJ(FAR, 0, 0, vel=5)
        result["move_s"] = time.time() - t0

    worker = threading.Thread(target=long_move, daemon=True)
    worker.start()
    time.sleep(0.8)

    stop_conn = Robot.RPC(ROBOT_IP)
    t0 = time.time()
    rtn = stop_conn.StopMotion()
    result["stop_rtn"] = rtn
    result["stop_call_s"] = time.time() - t0

    worker.join(timeout=20)
    print("StopMotion rtn      :", result.get("stop_rtn"))
    print("StopMotion call (s) :", round(result.get("stop_call_s", -1), 3))
    print("worker finished     :", not worker.is_alive())
    print(
        "MoveJ rtn / dur (s) :",
        result.get("move_rtn"),
        round(result.get("move_s", -1), 2),
    )
    _, joints = main_conn.GetActualJointPosDegree()
    print("joint 1 after stop  :", round(joints[0], 2), "(target 60.0)")

    main_conn.MoveJ(HOME, 0, 0, vel=5)
    main_conn.CloseRPC()
    stop_conn.CloseRPC()


if __name__ == "__main__":
    main()
