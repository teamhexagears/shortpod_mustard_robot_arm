#!/usr/bin/env python3
"""
camera_to_robot_transform.py

Compute the rigid 3D transform from camera coordinates to robot coordinates:

    robot_xyz = R @ camera_xyz + t

This version takes the calibration table you already recorded and converts
joint-angle samples into robot-space end-effector points using forward
kinematics for a 4-DOF reach arm.

Calibration data format used here:
    camera_xyz = [x, y, z] in mm (from the depth camera)
    pose_deg = [base, arm1, elbow, tilt_wrist, twist_wrist, claw]

The robot point used for calibration is computed from the first four joints:
    base, arm1, elbow, tilt_wrist

This matches your custom arm configuration where motion is dominated by:
- base rotation
- arm1 / shoulder lift
- elbow bend
- wrist pitch

The last two values (twist_wrist, claw) are ignored for the kinematic reach
calculation, but kept in the calibration table for completeness.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------
# Your calibration table
# ---------------------------------------------------------------------
# Each item is a calibration sample:
#   camera_xyz (mm), pose_deg (base, arm1, elbow, tilt, twist, claw)
# NOTE: The camera positions below are from your notes; keep them as-is.
CALIBRATION_TABLE = [
    {
        "camera_xyz": [-167.0, -54.0, 237.0],
        "pose_deg": [10, 150, 30, 90, 5, 150],
    },
    {
        "camera_xyz": [-106.0, 30.0, 165.0],
        "pose_deg": [20, 140, 80, 90, 5, 150],
    },
    {
        "camera_xyz": [-58.0, -47.0, 229.0],
        "pose_deg": [25, 140, 50, 90, 5, 150],
    },
    {
        "camera_xyz": [30.0, -29.0, 191.0],
        "pose_deg": [40, 150, 60, 90, 5, 150],
    },
    {
        "camera_xyz": [111.0, -54.0, 170.0],
        "pose_deg": [55, 140, 60, 90, 5, 150],
    },
    {
        "camera_xyz": [78.0, -34.0, 202.0],
        "pose_deg": [50, 150, 50, 90, 5, 150],
    },
]

# ---------------------------------------------------------------------
# Robot geometry from your arm notes
# ---------------------------------------------------------------------
# Units: mm
L1 = 220.0
L2 = 150.0
L3 = 70.0
BASE_HEIGHT = 60.0

# ---------------------------------------------------------------------
# Forward kinematics for your 4-DOF reach arm
# ---------------------------------------------------------------------
def fk_from_pose_deg(pose_deg: list[float]) -> np.ndarray:
    """
    Convert a 6D servo pose into a robot-space tool point.

    We use the first four values to define reach:
        q1 = base
        q2 = arm1
        q3 = elbow
        q4 = tilt_wrist

    The last two values (twist_wrist, claw) are intentionally ignored for the
    target-position computation, because they do not change the reach point
    used for the stem target.
    """
    q1, q2, q3, q4, _, _ = pose_deg

    q1 = math.radians(q1)
    q2 = math.radians(q2)
    q3 = math.radians(q3)
    q4 = math.radians(q4)

    x = math.cos(q1) * (
        L1 * math.cos(q2)
        + L2 * math.cos(q2 + q3)
        + L3 * math.cos(q2 + q3 + q4)
    )
    y = math.sin(q1) * (
        L1 * math.cos(q2)
        + L2 * math.cos(q2 + q3)
        + L3 * math.cos(q2 + q3 + q4)
    )
    z = BASE_HEIGHT + (
        L1 * math.sin(q2)
        + L2 * math.sin(q2 + q3)
        + L3 * math.sin(q2 + q3 + q4)
    )

    return np.array([x, y, z], dtype=np.float64)

# ---------------------------------------------------------------------
# Rigid transform solve using SVD / Kabsch
# ---------------------------------------------------------------------
def solve_rigid_transform(camera_pts: np.ndarray, robot_pts: np.ndarray):
    if len(camera_pts) != len(robot_pts):
        raise ValueError("camera_pts and robot_pts must have the same length")
    if len(camera_pts) < 3:
        raise ValueError("Need at least 3 corresponding points")

    cam_mean = camera_pts.mean(axis=0)
    rob_mean = robot_pts.mean(axis=0)

    cam_centered = camera_pts - cam_mean
    rob_centered = robot_pts - rob_mean

    H = cam_centered.T @ rob_centered
    U, _, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    # Ensure a proper rotation matrix (det = +1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = rob_mean - R @ cam_mean
    return R, t


def rmse(camera_pts: np.ndarray, robot_pts: np.ndarray, R: np.ndarray, t: np.ndarray) -> float:
    pred = (camera_pts @ R.T) + t
    err = pred - robot_pts
    return float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))

# ---------------------------------------------------------------------
# Build calibration arrays
# ---------------------------------------------------------------------
def build_camera_robot_pairs() -> tuple[np.ndarray, np.ndarray]:
    camera_pts = []
    robot_pts = []

    for sample in CALIBRATION_TABLE:
        camera_xyz = np.asarray(sample["camera_xyz"], dtype=np.float64)
        pose_deg = sample["pose_deg"]
        robot_xyz = fk_from_pose_deg(pose_deg)

        camera_pts.append(camera_xyz)
        robot_pts.append(robot_xyz)

    return np.asarray(camera_pts), np.asarray(robot_pts)


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------
def main() -> None:
    camera_pts, robot_pts = build_camera_robot_pairs()

    R, t = solve_rigid_transform(camera_pts, robot_pts)
    err_mm = rmse(camera_pts, robot_pts, R, t)

    print("Rotation matrix R:")
    print(np.array2string(R, precision=6, suppress_small=False))
    print()
    print("Translation vector t:")
    print(np.array2string(t, precision=6, suppress_small=False))
    print()
    print(f"RMSE: {err_mm:.3f} mm")

    output = {
        "R": R.tolist(),
        "t": t.tolist(),
        "rmse_mm": err_mm,
    }

    output_path = Path("camera_to_robot_transform.json")
    output_path.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(f"Saved calibration transform to: {output_path}")

    # Example use:
    print()
    print("Use this in your detector code:")
    print("  robot_xyz = R @ camera_xyz + t")


if __name__ == "__main__":
    main()
