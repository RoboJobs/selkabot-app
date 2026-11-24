"""Read the current tool-1 pose and joint angles from the Fairino robot.

This script is read-only: it does not move the robot or change its settings.

Usage:
    uv run python sw/read_tool1_state.py
    uv run python sw/read_tool1_state.py --ip 192.168.58.2
"""

import argparse

from fairino import Robot

DEFAULT_IP = "192.168.58.2"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=DEFAULT_IP, help="Fairino controller IP")
    args = parser.parse_args()

    robot = Robot.RPC(args.ip)
    try:
        code, tool_id = robot.GetActualTCPNum()
        if code != 0:
            print(f"GetActualTCPNum failed: {code}")
            return 1
        if tool_id != 1:
            print(f"Refusing to read: active tool is {tool_id}, expected tool 1")
            return 1

        pose_code, pose = robot.GetActualTCPPose()
        joint_code, joints = robot.GetActualJointPosDegree()
        if pose_code != 0:
            print(f"GetActualTCPPose failed: {pose_code}")
            return 1
        if joint_code != 0:
            print(f"GetActualJointPosDegree failed: {joint_code}")
            return 1

        print("Tool 1 pose [x, y, z, rx, ry, rz]:")
        print("  " + ", ".join(f"{value:.6f}" for value in pose))
        print("Joint angles [j1, j2, j3, j4, j5, j6] (degrees):")
        print("  " + ", ".join(f"{value:.6f}" for value in joints))
        return 0
    finally:
        robot.CloseRPC()


if __name__ == "__main__":
    raise SystemExit(main())
