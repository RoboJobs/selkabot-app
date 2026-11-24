"""Q1 probe: confirm the SDK's Euler convention on the robot (simulator or real).

Moves joint 1 by a known angle and checks that the TCP rotates about the base
z axis by the same angle, and that the rz component follows it. Run with the
robot in auto or manual mode and nothing in the workspace.
"""

import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROBOT_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.58.2"
HOME = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]


def pose_rotation(p, convention):
    return Rotation.from_euler(convention, p[3:6], degrees=True)


def main():
    from fairino import Robot

    r = Robot.RPC(ROBOT_IP)
    r.SetSpeed(10)
    r.MoveJ(HOME, 0, 0, vel=10)
    _, pose0 = r.GetActualTCPPose()
    _, fk0 = r.GetForwardKin(HOME)
    print("home TCP pose      :", np.round(pose0, 3))
    print("home FK pose       :", np.round(fk0, 3))

    for conv in ("xyz", "XYZ", "zyx"):
        z_axis = pose_rotation(pose0, conv).apply([0, 0, 1])
        print(f"  tool z under '{conv}':", np.round(z_axis, 3))

    target = HOME.copy()
    target[0] += 10.0
    r.MoveJ(target, 0, 0, vel=10)
    _, pose1 = r.GetActualTCPPose()
    print("after +10 deg J1  :", np.round(pose1, 3))

    dz = pose1[5] - pose0[5]
    dx0, dy0 = pose0[0], pose0[1]
    dx1, dy1 = pose1[0], pose1[1]
    ang_pos = np.degrees(np.arctan2(dy1, dx1) - np.arctan2(dy0, dx0))
    print(f"TCP rz change      : {dz:.3f} deg (expect +10 for fixed-axis xyz)")
    print(f"TCP xy rotation    : {ang_pos:.3f} deg about base z (expect +10)")

    r.MoveJ(HOME, 0, 0, vel=10)
    r.CloseRPC()


if __name__ == "__main__":
    main()
