"""Upload a TPD recording and check it loads on the controller. No motion.

Steps:
  1. upload_file(local, type 20)       -> /fruser/traj/<basename> (retrying upload,
                                          not the SDK's TrajectoryJUpLoad)
  2. ComputeFileMD5(remote path)       -> must equal the local MD5
  3. LoadTPD("traj/<stem>")            -> load it as a TPD recording
  4. GetTPDStartPose("traj/<stem>")    -> compare with the first row of the file

After this passes, play it with sw/fairino_basic_query.py (NAME = "traj/<stem>").

Usage (from the repo root):
    uv run python sw/robot/spike/tpd_upload_check.py sw/tracktestpc.txt
"""

import hashlib
import os
import sys

from fairino import Robot
from file_port_probe import FILE_TYPE_TRAJ_J, upload_file

ROBOT_IP = "192.168.58.2"  # real robot (the simulator is 192.168.50.2)
REMOTE_DIR = "/fruser/traj"
POSE_TOL = 0.01  # the controller returns float32 values


def first_row_tcp(path):
    """Columns 7-12 of the first data row are the TCP pose (mm, deg)."""
    with open(path) as f:
        f.readline()  # header
        cols = f.readline().split(",")
    return [float(v) for v in cols[6:12]]


def local_md5(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return
    local = os.path.abspath(sys.argv[1])
    if not os.path.exists(local):
        print("Stop: local file not found:", local)
        return
    fname = os.path.basename(local)
    remote = REMOTE_DIR + "/" + fname
    tpd_name = "traj/" + os.path.splitext(fname)[0]
    print("File:", local)

    r = Robot.RPC(ROBOT_IP)
    try:
        ok, stage, detail = upload_file(r, ROBOT_IP, local, FILE_TYPE_TRAJ_J)
        if not ok:
            print("Stop: upload failed at %s: %r" % (stage, detail))
            return

        err, remote_md5 = r.ComputeFileMD5(remote)
        want = local_md5(local)
        print("ComputeFileMD5(%s): %s %s" % (remote, err, remote_md5))
        if err != 0 or str(remote_md5).lower() != want:
            print("Stop: controller copy does not match (local MD5 %s)" % want)
            return
        print("MD5 match: True")

        err = r.LoadTPD(tpd_name)
        print("LoadTPD(%r): %s" % (tpd_name, err))
        if err != 0:
            print("Stop: LoadTPD failed (8 = tpd file does not exist)")
            return

        err, start = r.GetTPDStartPose(tpd_name)
        expected = first_row_tcp(local)
        print("GetTPDStartPose:", err, start)
        print("First row of file (TCP):", expected)
        if err != 0 or not start:
            print("Stop: could not read the start pose")
            return
        diff = max(abs(a - b) for a, b in zip(start, expected))
        verdict = "OK" if diff < POSE_TOL else "MISMATCH"
        print(
            "Max abs pose difference: %.6f (%s, limit %s)" % (diff, verdict, POSE_TOL)
        )
        if diff < POSE_TOL:
            print("Ready to play with NAME = %r" % tpd_name)
    finally:
        r.CloseRPC()


if __name__ == "__main__":
    main()
