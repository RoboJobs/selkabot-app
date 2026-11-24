"""Check and optionally play a TPD recording on the FR5.

Usage:
    python tpd_play_check.py                  # load + read only (no motion)
    python tpd_play_check.py --play           # also move to start and play
"""

import sys
import time

from fairino import Robot

ROBOT_IP = "192.168.58.2"
NAME = "tracktest1"  # TPD name, no extension and no path
PLAY_OVL = 50  # speed scaling %, keep low for the first run
BLEND = 0  # 0 = no smoothing, 1 = smooth
MOVE_OVL = 50  # speed for the move to the start pose


def main():
    play = "--play" in sys.argv
    r = Robot.RPC(ROBOT_IP)
    try:
        err = r.LoadTPD(NAME)
        print("LoadTPD:", err)
        if err != 0:
            print("Stop: LoadTPD failed")
            return

        err, start = r.GetTPDStartPose(NAME)
        print("GetTPDStartPose:", err, start)
        if err != 0:
            print("Stop: could not read start pose")
            return

        err, joints = r.GetActualJointPosDegree()
        print("Current joints:", joints)
        print("Robot error code:", r.GetRobotErrorCode())

        if not play:
            print("Load-only run. Re-run with --play to move and play.")
            return

        print("\nThe robot will MOVE to the start pose, then play the track.")
        print("Cell must be clear and the e-stop in hand.")
        if input("Type YES to continue: ").strip() != "YES":
            print("Aborted.")
            return

        err = r.Mode(0)  # 0 = automatic
        print("Mode(0):", err)
        # robot_state_pkg is a cache refreshed by the SDK in the background;
        # give it time to pick up the new mode (same as the SDK examples).
        time.sleep(1)
        mode = r.robot_state_pkg.robot_mode
        print("robot_mode:", mode)
        if err != 0 or mode != 0:
            print("Stop: controller is not in automatic mode")
            return

        r.SetSpeed(MOVE_OVL)
        # Start pose is Cartesian; tool 0 / user 0 are the defaults.
        print("MoveCart to start:", r.MoveCart(start, 0, 0, MOVE_OVL))
        input("Press Enter to play the track: ")

        print("MoveTPD:", r.MoveTPD(NAME, BLEND, PLAY_OVL))
    except KeyboardInterrupt:
        print("\nInterrupted, stopping motion")
        r.StopMotion()
    finally:
        r.CloseRPC()


if __name__ == "__main__":
    main()
