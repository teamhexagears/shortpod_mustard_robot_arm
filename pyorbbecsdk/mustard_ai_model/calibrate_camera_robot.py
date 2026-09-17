#!/usr/bin/env python3
"""Calibrate Gemini 330 camera coordinates to a robot-base coordinate frame.

The camera is fixed beside the robot. This utility collects corresponding points:
- camera point: measured from the aligned Gemini depth image
- robot point: entered by the operator in robot-base millimeters

Workflow:
1. Put a visible marker on the claw TCP.
2. Move the claw to a safe, known robot-base XYZ position.
3. Click the marker in the camera window.
4. Press C and enter the robot X, Y, Z values in millimeters.
5. Repeat for at least 4 non-coplanar points; 6-10 points are recommended.
6. Press S to solve and save the transform.

Controls:
    C       capture the selected image pixel and enter its robot XYZ
    R       clear the selected pixel
    S       solve and save after at least 4 points
    Q/ESC   quit

Important: The robot XYZ values must come from your robot coordinate system.
Servo angles alone are not XYZ coordinates; use measured positions or forward
kinematics from the arm's link lengths and servo zero offsets.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.utils import frame_to_bgr_image
from pyorbbecsdk import AlignFilter, Config, Context, OBError, OBSensorType, OBStreamType, Pipeline


class CalibrationError(ValueError):
    """Raised when calibration points cannot produce a reliable transform."""


def solve_rigid_transform(camera_points: np.ndarray, robot_points: np.ndarray):
    """Return rotation, translation, and RMS error for camera -> robot points."""
    if camera_points.shape != robot_points.shape or camera_points.shape[0] < 4:
        raise CalibrationError("At least 4 matching 3D point pairs are required.")

    camera_center = camera_points.mean(axis=0)
    robot_center = robot_points.mean(axis=0)
    camera_zeroed = camera_points - camera_center
    robot_zeroed = robot_points - robot_center

    if np.linalg.matrix_rank(camera_zeroed) < 3 or np.linalg.matrix_rank(robot_zeroed) < 3:
        raise CalibrationError("Points must span 3D space. Do not put every point on one flat line or plane.")

    covariance = camera_zeroed.T @ robot_zeroed
    u, _, vh = np.linalg.svd(covariance)
    rotation = vh.T @ u.T
    if np.linalg.det(rotation) < 0:
        vh[-1, :] *= -1
        rotation = vh.T @ u.T

    translation = robot_center - rotation @ camera_center
    predicted = (rotation @ camera_points.T).T + translation
    errors = np.linalg.norm(predicted - robot_points, axis=1)
    return rotation, translation, float(np.sqrt(np.mean(errors**2)))


def depth_point(depth_frame, pixel_x: int, pixel_y: int) -> Optional[np.ndarray]:
    """Read a robust XYZ camera point in millimeters around an image pixel."""
    try:
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(height, width)
        depth_mm = raw.astype(np.float32) * depth_frame.get_depth_scale()
        x = max(0, min(width - 1, pixel_x))
        y = max(0, min(height - 1, pixel_y))
        radius = 4
        samples = depth_mm[max(0, y - radius):min(height, y + radius + 1), max(0, x - radius):min(width, x + radius + 1)]
        samples = samples[np.isfinite(samples) & (samples > 0)]
        if samples.size == 0:
            return None

        z = float(np.median(samples))
        intrinsic = depth_frame.get_stream_profile().get_intrinsic()
        return np.array([(x - intrinsic.cx) * z / intrinsic.fx, (y - intrinsic.cy) * z / intrinsic.fy, z], dtype=np.float64)
    except (TypeError, ValueError, AttributeError, ZeroDivisionError):
        return None


def save_calibration(path: Path, rotation: np.ndarray, translation: np.ndarray, rms_error: float, points: int) -> None:
    data = {
        "source_frame": "gemini330_camera",
        "target_frame": "robot_base",
        "units": "millimeters",
        "camera_to_robot_rotation": rotation.tolist(),
        "camera_to_robot_translation_mm": translation.tolist(),
        "rms_error_mm": rms_error,
        "calibration_point_count": points,
    }
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate a fixed Gemini 330 to an Arduino robot-base frame.")
    parser.add_argument("--output", type=Path, default=ROOT / "mustard_ai_model" / "camera_robot_calibration.json")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    context = Context()
    if context.query_devices().get_count() == 0:
        print("No Orbbec camera found.")
        return

    pipeline = Pipeline()
    config = Config()
    try:
        color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        config.enable_stream(color_profiles.get_default_video_stream_profile())
        config.enable_stream(depth_profiles.get_default_video_stream_profile())
        pipeline.start(config)
    except OBError as exc:
        print(f"Could not start Orbbec pipeline: {exc}")
        return

    align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    camera_points = []
    robot_points = []
    selected_pixel = [None]
    latest_depth = [None]

    def on_mouse(event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            selected_pixel[0] = (x, y)

    window = "Gemini 330 Robot Calibration"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    print("Click the claw marker, then press C. Press S after collecting points.")

    try:
        while True:
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                continue
            frames = align_filter.process(frames)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                continue
            image = frame_to_bgr_image(color_frame)
            if image is None:
                continue
            latest_depth[0] = depth_frame

            display = image.copy()
            if selected_pixel[0] is not None:
                px, py = selected_pixel[0]
                cv2.drawMarker(display, (px, py), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
            cv2.putText(display, f"Points: {len(camera_points)} | C capture | S solve | Q quit", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("r"), ord("R")):
                selected_pixel[0] = None
            elif key in (ord("c"), ord("C")):
                if selected_pixel[0] is None:
                    print("Click the claw marker first.")
                    continue
                camera_point = depth_point(depth_frame, *selected_pixel[0])
                if camera_point is None:
                    print("No valid depth at that pixel. Move the marker or try again.")
                    continue
                print(f"Camera XYZ: {camera_point.round(1).tolist()} mm")
                try:
                    values = input("Enter robot-base X Y Z in mm: ").split()
                    if len(values) != 3:
                        raise ValueError
                    robot_point = np.array([float(value) for value in values], dtype=np.float64)
                except ValueError:
                    print("Enter exactly three numbers, for example: 120 40 180")
                    continue
                camera_points.append(camera_point)
                robot_points.append(robot_point)
                selected_pixel[0] = None
                print(f"Captured point {len(camera_points)}")
            elif key in (ord("s"), ord("S")):
                if len(camera_points) < 4:
                    print("Collect at least 4 points first; 6-10 points are recommended.")
                    continue
                try:
                    rotation, translation, rms_error = solve_rigid_transform(np.array(camera_points), np.array(robot_points))
                except CalibrationError as exc:
                    print(f"Cannot solve calibration: {exc}")
                    continue
                save_calibration(args.output, rotation, translation, rms_error, len(camera_points))
                print(f"Saved {args.output}")
                print(f"RMS calibration error: {rms_error:.2f} mm")
                print("Review this error before allowing the arm to move automatically.")
                break
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()


if __name__ == "__main__":
    main()
