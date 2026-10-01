#!/usr/bin/env python3
import argparse
import importlib.util
import sys
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import serial

ROOT = Path(__file__).resolve().parent
PYORBBECSDK_ROOT = ROOT / "pyorbbecsdk"

for candidate in [str(ROOT), str(PYORBBECSDK_ROOT)]:
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

try:
    from pyorbbecsdk.examples.utils import frame_to_bgr_image
except ModuleNotFoundError:
    utils_path = PYORBBECSDK_ROOT / "examples" / "utils.py"
    if utils_path.exists():
        spec = importlib.util.spec_from_file_location("local_examples_utils", utils_path)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        frame_to_bgr_image = module.frame_to_bgr_image
    else:
        raise

from pyorbbecsdk import AlignFilter, Config, OBSensorType, OBStreamType, Pipeline

try:
    from pyorbbecsdk.mustard_ai_model.detect_mustard import MustardDetector
except ModuleNotFoundError:
    from mustard_ai_model.detect_mustard import MustardDetector

# --- load calibration transform ---
with open("camera_to_robot_transform.json", "r") as f:
    calib = json.load(f)

R = np.array(calib["R"], dtype=np.float64)
t = np.array(calib["t"], dtype=np.float64)

# --- arm geometry ---
L1 = 220.0
L2 = 150.0
L3 = 70.0
BASE_HEIGHT = 60.0

JOINT_LIMITS = {
    "base": (0, 160),
    "arm1": (30, 150),
    "elbow": (10, 130),
    "tiltWrist": (0, 90),
    "twistWrist": (0, 179),
    "claw": (100, 179),
}

CURRENT_POSE = {
    "base": 80.0,
    "arm1": 60.0,
    "elbow": 90.0,
    "tiltWrist": 50.0,
    "twistWrist": 90.0,
    "claw": 150.0,
}

MAX_TARGET_Z_MM = 600.0
DETECTION_WAIT_SECONDS = 1.0


def fk(q):
    q1, q2, q3, q4 = q
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


def jacobian_numeric(q, eps=1e-6):
    p0 = fk(q)
    J = np.zeros((3, len(q)))
    for i in range(len(q)):
        dq = np.zeros_like(q)
        dq[i] = eps
        p1 = fk(q + dq)
        J[:, i] = (p1 - p0) / eps
    return J


def ik_point(target_xyz, q0=None, max_iter=400, tol=1e-3, damping=1e-2):
    if q0 is None:
        q = np.array([
            0.0,
            math.radians(60.0),
            math.radians(-35.0),
            math.radians(20.0),
        ], dtype=float)
    else:
        q = np.array(q0, dtype=float)

    for _ in range(max_iter):
        p = fk(q)
        e = target_xyz - p
        if np.linalg.norm(e) < tol:
            break

        J = jacobian_numeric(q)
        JTJ = J.T @ J
        reg = damping * np.eye(JTJ.shape[0])
        dq = np.linalg.solve(JTJ + reg, J.T @ e)
        q = q + dq

        q[0] = np.clip(q[0], math.radians(0), math.radians(160))
        q[1] = np.clip(q[1], math.radians(30), math.radians(150))
        q[2] = np.clip(q[2], math.radians(10), math.radians(130))
        q[3] = np.clip(q[3], math.radians(0), math.radians(90))

    return q


def q_to_pose_deg(q):
    pose = {
        "base": math.degrees(q[0]),
        "arm1": math.degrees(q[1]),
        "elbow": math.degrees(q[2]),
        "tiltWrist": math.degrees(q[3]),
        "twistWrist": CURRENT_POSE["twistWrist"],
        "claw": CURRENT_POSE["claw"],
    }
    for name, (lo, hi) in JOINT_LIMITS.items():
        pose[name] = int(round(np.clip(pose[name], lo, hi)))
    return pose


def depth_to_camera_xyz(depth_frame, x_px, y_px, radius=4):
    depth_map = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
    h = depth_frame.get_height()
    w = depth_frame.get_width()
    depth_map = depth_map.reshape(h, w).astype(np.float32)

    x = int(round(x_px))
    y = int(round(y_px))
    x = max(0, min(w - 1, x))
    y = max(0, min(h - 1, y))

    x_start = max(0, x - radius)
    x_end = min(w, x + radius + 1)
    y_start = max(0, y - radius)
    y_end = min(h, y + radius + 1)
    raw_samples = depth_map[y_start:y_end, x_start:x_end]
    scale = float(depth_frame.get_depth_scale())
    samples_mm = raw_samples * scale
    valid_samples = samples_mm[np.isfinite(samples_mm) & (samples_mm > 0)]
    if valid_samples.size == 0:
        return None, 0

    # Median rejects isolated depth holes/outliers near the box-bottom pixel.
    z_mm = float(np.median(valid_samples))
    if not np.isfinite(z_mm) or z_mm <= 0:
        return None, int(valid_samples.size)

    intr = depth_frame.get_stream_profile().get_intrinsic()
    fx = intr.fx
    fy = intr.fy
    cx = intr.cx
    cy = intr.cy

    X = (x - cx) * z_mm / fx
    Y = (y - cy) * z_mm / fy
    Z = z_mm
    return np.array([X, Y, Z], dtype=np.float64), int(valid_samples.size)


def camera_to_robot(camera_xyz):
    return R @ camera_xyz + t


def build_move_command(pose):
    return "MOVE {} {} {} {} {} {}".format(
        int(round(pose["base"])),
        int(round(pose["arm1"])),
        int(round(pose["elbow"])),
        int(round(pose["tiltWrist"])),
        int(round(pose["twistWrist"])),
        int(round(pose["claw"])),
    )


def send_move(ser, pose):
    cmd = build_move_command(pose)
    print("Arduino MOVE command:", cmd, flush=True)
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    time.sleep(0.3)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect plant stem, convert to robot target, and send MOVE to Arduino.")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Arduino serial port")
    parser.add_argument("--dry-run", action="store_true", help="Print target and pose without sending to Arduino")
    parser.add_argument("--weights", type=str, default=None, help="Path to model weights (.pt)")
    parser.add_argument("--conf", type=float, default=0.90, help="Detection confidence threshold")
    parser.add_argument("--iou", type=float, default=0.7, help="NMS IoU threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="Model image size")
    parser.add_argument("--device", type=str, default="cpu", help="Model device")
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print low-confidence detections and camera/depth/IK diagnostics",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    detector = MustardDetector(model_path=args.weights, conf=args.conf, iou=args.iou, imgsz=args.imgsz, device=args.device)

    pipeline = Pipeline()
    config = Config()
    color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    config.enable_stream(color_profiles.get_default_video_stream_profile())
    config.enable_stream(depth_profiles.get_default_video_stream_profile())
    pipeline.start(config)
    align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    frame_number = 0
    ser = None

    try:
        if not args.dry_run:
            print(f"Opening Arduino serial connection on {args.port}...", flush=True)
            ser = serial.Serial(args.port, 9600, timeout=1)
            # Opening a Uno serial port may reset the board; allow setup() to finish once.
            time.sleep(2)

        while True:
            frame_number += 1
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                if args.debug:
                    print(f"[debug] frame {frame_number}: no frames", flush=True)
                continue
            frames = align.process(frames)
            if frames is None:
                #if args.debug:
                #    print(f"[debug] frame {frame_number}: alignment failed", flush=True)
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                if args.debug:
                    print(
                        f"[debug] frame {frame_number}: missing color/depth frame "
                        f"(color={color_frame is not None}, depth={depth_frame is not None})",
                        flush=True,
                    )
                continue

            bgr = frame_to_bgr_image(color_frame)
            if bgr is None:
                if args.debug:
                    print(f"[debug] frame {frame_number}: color conversion failed", flush=True)
                continue

            inference_conf = 0.05 if args.debug else detector.conf
            results = detector.model(
                bgr,
                conf=inference_conf,
                iou=detector.iou,
                imgsz=detector.imgsz,
                device=detector.device,
                verbose=False,
            )
            result = results[0]

            if result.boxes is not None and len(result.boxes) > 0:
                confidences = result.boxes.conf.cpu().numpy()
                best_index = int(np.argmax(confidences))
                best = result.boxes[best_index]
                x1_box, y1_box, x2_box, y2_box = map(int, best.xyxy[0].tolist())
                confidence = float(best.conf[0])
                color = (0, 255, 0)
                cv2.rectangle(bgr, (x1_box, y1_box), (x2_box, y2_box), color, 2)
                cv2.putText(
                    bgr,
                    f"mustard {confidence:.2f}",
                    (x1_box, max(20, y1_box - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

                best_x1, best_y1, best_x2, best_y2 = map(float, best.xyxy[0].tolist())
                stem_x = int(round((best_x1 + best_x2) / 2.0))
                stem_y = int(round(best_y2 - (best_y2 - best_y1) / 10.0))
                cv2.drawMarker(
                    bgr,
                    (stem_x, stem_y),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    markerSize=14,
                    thickness=2,
                )

            cv2.imshow("stem target", bgr)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

            if result.boxes is not None and len(result.boxes) > 0:
                if args.debug:
                    confidences = result.boxes.conf.cpu().numpy()
                    #print(
                    #    f"[debug] frame {frame_number}: {len(result.boxes)} detection(s), "
                    #    f"confidence range {confidences.min():.3f}-{confidences.max():.3f}",
                    #    flush=True,
                    #)
                    for index, box in enumerate(result.boxes):
                        coords = [round(value, 1) for value in box.xyxy[0].tolist()]
                        # print(
                        #     f"[debug]   box {index}: conf={float(box.conf[0]):.3f}, "
                        #     f"xyxy={coords}",
                        #     flush=True,
                        # )

                x1, y1, x2, y2 = map(float, best.xyxy[0].tolist())

                x_center = (x1 + x2) / 2.0
                stem_y = y2 - (y2 - y1) / 10.0

                camera_xyz, depth_sample_count = depth_to_camera_xyz(depth_frame, x_center, stem_y)
                if camera_xyz is None:
                    print(
                        f"Depth unavailable around stem target pixel "
                        f"({x_center:.1f}, {stem_y:.1f}); no valid samples in 9x9 window.",
                        flush=True,
                    )
                    continue

                if camera_xyz[2] > MAX_TARGET_Z_MM:
                    print(
                        f"Target Z {camera_xyz[2]:.1f} mm exceeds maximum "
                        f"{MAX_TARGET_Z_MM:.1f} mm; skipping target.",
                        flush=True,
                    )
                    continue

                robot_xyz = camera_to_robot(camera_xyz)
                q = ik_point(robot_xyz)
                pose = q_to_pose_deg(q)
                command = build_move_command(pose)
                ik_error_mm = float(np.linalg.norm(fk(q) - robot_xyz))

                print(f"detection_confidence: {float(best.conf[0]):.3f}")
                print(f"stem_pixel: ({x_center:.1f}, {stem_y:.1f}, {camera_xyz[2]:.1f})")
                print(f"valid_depth_samples: {depth_sample_count}")
                print("camera_xyz:", np.round(camera_xyz, 2))
                print("robot_xyz:", np.round(robot_xyz, 2))
                print(f"ik_error_mm: {ik_error_mm:.2f}")
                print("pose_deg:", pose)
                print("Arduino MOVE command:", command, flush=True)

                if args.dry_run:
                    print("Dry-run: not sending MOVE command.")
                else:
                    assert ser is not None
                    send_move(ser, pose)

                print(
                    f"Waiting {DETECTION_WAIT_SECONDS:.0f} seconds before detecting again.",
                    flush=True,
                )
                time.sleep(DETECTION_WAIT_SECONDS)

            elif args.debug:
                print(f"[debug] frame {frame_number}: no detections", flush=True)

    finally:
        cv2.destroyAllWindows()
        try:
            pipeline.stop()
        finally:
            if ser is not None and ser.is_open:
                ser.close()


if __name__ == "__main__":
    main()
