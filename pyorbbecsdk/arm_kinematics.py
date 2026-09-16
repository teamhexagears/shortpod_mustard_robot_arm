"""Simple 5-DOF position IK for the mustard robot arm."""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, pi, sin


@dataclass(frozen=True)
class ArmGeometry:
    base_height_mm: float
    upper_arm_mm: float
    forearm_mm: float
    wrist_mm: float
    wrist_pitch_deg: float = 0.0


@dataclass(frozen=True)
class ServoCalibration:
    base_zero_deg: float = 80.0
    arm_zero_deg: float = 60.0
    elbow_zero_deg: float = 90.0
    tilt_wrist_zero_deg: float = 50.0
    twist_wrist_deg: float = 90.0
    claw_deg: float = 150.0
    base_direction: float = 1.0
    arm_direction: float = 1.0
    elbow_direction: float = 1.0
    tilt_wrist_direction: float = 1.0


def solve(target_x_mm: float, target_y_mm: float, target_z_mm: float, geometry: ArmGeometry, calibration: ServoCalibration) -> tuple[int, int, int, int, int, int]:
    """Return base, arm, elbow, tilt wrist, twist wrist, and claw angles."""
    radius_mm = hypot(target_x_mm, target_y_mm)
    base_rad = atan2(target_y_mm, target_x_mm)
    wrist_pitch_rad = geometry.wrist_pitch_deg * pi / 180.0
    wrist_x_mm = radius_mm - geometry.wrist_mm * cos(wrist_pitch_rad)
    wrist_z_mm = target_z_mm - geometry.base_height_mm - geometry.wrist_mm * sin(wrist_pitch_rad)
    distance_squared = wrist_x_mm**2 + wrist_z_mm**2
    cosine_elbow = (distance_squared - geometry.upper_arm_mm**2 - geometry.forearm_mm**2) / (2 * geometry.upper_arm_mm * geometry.forearm_mm)
    if cosine_elbow < -1.0 or cosine_elbow > 1.0:
        raise ValueError("Target is outside the arm workspace")

    elbow_rad = acos(cosine_elbow)
    arm_rad = atan2(wrist_z_mm, wrist_x_mm) - atan2(
        geometry.forearm_mm * sin(elbow_rad),
        geometry.upper_arm_mm + geometry.forearm_mm * cos(elbow_rad),
    )
    tilt_wrist_rad = wrist_pitch_rad - arm_rad - elbow_rad

    def servo_angle(zero: float, direction: float, joint_rad: float) -> int:
        return int(round(zero + direction * degrees(joint_rad)))

    return (
        servo_angle(calibration.base_zero_deg, calibration.base_direction, base_rad),
        servo_angle(calibration.arm_zero_deg, calibration.arm_direction, arm_rad),
        servo_angle(calibration.elbow_zero_deg, calibration.elbow_direction, elbow_rad),
        servo_angle(calibration.tilt_wrist_zero_deg, calibration.tilt_wrist_direction, tilt_wrist_rad),
        int(round(calibration.twist_wrist_deg)),
        int(round(calibration.claw_deg)),
    )