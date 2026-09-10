#!/usr/bin/env python3
"""Real-time mustard detection using the Orbbec camera and the restored YOLO model."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.utils import frame_to_bgr_image
from pyorbbecsdk import (
    AlignFilter,
    Config,
    OBError,
    OBSensorType,
    OBStreamType,
    Pipeline,
)

from mustard_ai_model.detect_mustard import MustardDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run real-time mustard detection on Orbbec camera frames.")
    parser.add_argument("--weights", type=str, default=None, help="Path to model weights (.pt)")
    parser.add_argument("--conf", type=float, default=0.90, help="Minimum confidence, from 0.0 to 1.0")
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument("--window-name", type=str, default="Mustard Detector", help="OpenCV window title")
    return parser.parse_args()


class OrbbecMustardApp:
    def __init__(self, weights: str | None = None, conf: float = 0.90, iou: float = 0.7, imgsz: int = 640, device: str = "cpu"):
        self.detector = MustardDetector(model_path=weights, conf=conf, iou=iou, imgsz=imgsz, device=device)
        self.pipeline = None
        self.align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)

    def start(self) -> None:
        try:
            self.pipeline = Pipeline()
            config = Config()
            color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            config.enable_stream(color_profiles.get_default_video_stream_profile())
            config.enable_stream(depth_profiles.get_default_video_stream_profile())
            self.pipeline.start(config)
            print("Orbbec color + depth pipeline started. Press Q or ESC to quit.")
        except OBError as exc:
            raise RuntimeError(f"Could not start Orbbec pipeline: {exc}") from exc

    def stop(self) -> None:
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None

    def run(self, window_name: str = "Mustard Detector") -> None:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        try:
            while True:
                if self.pipeline is None:
                    break

                frames = self.pipeline.wait_for_frames(1000)
                if frames is None:
                    continue

                frames = self.align_filter.process(frames)
                if frames is None:
                    continue

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if color_frame is None or depth_frame is None:
                    continue

                frame = frame_to_bgr_image(color_frame)
                if frame is None:
                    continue

                results = self.detector.model(frame, conf=self.detector.conf, iou=self.detector.iou, imgsz=self.detector.imgsz, verbose=False)
                result = results[0]
                display = frame.copy()
                selected_box = None
                if result.boxes is not None and len(result.boxes) > 0:
                    confidence_values = result.boxes.conf.cpu().numpy()
                    selected_box = result.boxes[int(np.argmax(confidence_values))]

                detections = 1 if selected_box is not None else 0
                cv2.putText(display, f"Best detection: {detections}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                if selected_box is not None:
                    x1, y1, x2, y2 = map(int, selected_box.xyxy[0].tolist())
                    confidence = float(selected_box.conf[0])
                    class_id = int(selected_box.cls[0])
                    class_name = result.names.get(class_id, "object")
                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(
                        display,
                        f"{class_name} {confidence:.1%}",
                        (max(5, x1), max(25, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

                    xyz = self._get_xyz(depth_frame, x1, y1, x2, y2)
                    if xyz is not None:
                        x_mm, y_mm, z_mm = xyz
                        label = f"X:{x_mm:.0f} Y:{y_mm:.0f} Z:{z_mm:.0f} mm"
                        cv2.putText(
                            display,
                            label,
                            (max(5, x1), min(display.shape[0] - 8, y2 + 22)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 255, 255),
                            2,
                        )
                    else:
                        cv2.putText(
                            display,
                            "XYZ: depth unavailable",
                            (max(5, x1), min(display.shape[0] - 8, y2 + 22)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 0, 255),
                            2,
                        )

                cv2.imshow(window_name, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
        finally:
            cv2.destroyAllWindows()
            self.stop()

    @staticmethod
    def _depth_data(depth_frame):
        try:
            raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth = raw.reshape(depth_frame.get_height(), depth_frame.get_width())
            return depth.astype(np.float32) * depth_frame.get_depth_scale()
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _get_xyz(depth_frame, x1, y1, x2, y2):
        depth_data = OrbbecMustardApp._depth_data(depth_frame)
        if depth_data is None:
            return None

        depth_height, depth_width = depth_data.shape
        intrinsic = depth_frame.get_stream_profile().get_intrinsic()
        center_x = max(0, min(depth_width - 1, (x1 + x2) // 2))
        center_y = max(0, min(depth_height - 1, (y1 + y2) // 2))

        # The depth frame has been aligned to color, so detection pixels map
        # directly to depth pixels. Use the inner box region to reject edges.
        box_width = max(1, x2 - x1)
        box_height = max(1, y2 - y1)
        half_width = max(3, box_width // 5)
        half_height = max(3, box_height // 5)
        x0 = max(0, center_x - half_width)
        x_end = min(depth_width, center_x + half_width + 1)
        y0 = max(0, center_y - half_height)
        y_end = min(depth_height, center_y + half_height + 1)
        samples = depth_data[y0:y_end, x0:x_end]
        samples = samples[np.isfinite(samples) & (samples > 0)]
        if samples.size == 0:
            return None

        z_mm = float(np.median(samples))
        x_mm = (center_x - intrinsic.cx) * z_mm / intrinsic.fx
        y_mm = (center_y - intrinsic.cy) * z_mm / intrinsic.fy
        return x_mm, y_mm, z_mm


def main() -> None:
    args = parse_args()
    app = OrbbecMustardApp(
        weights=args.weights,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        device=args.device,
    )

    try:
        app.start()
        app.run(window_name=args.window_name)
    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception as exc:
        print(f"[ERROR] {exc}")
        print("Make sure the camera is connected and the model file exists.")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
