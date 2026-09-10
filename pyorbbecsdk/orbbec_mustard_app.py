#!/usr/bin/env python3
"""Real-time mustard detection using the Orbbec camera and the restored YOLO model."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.utils import frame_to_bgr_image
from pyorbbecsdk import OBError, Pipeline

from mustard_ai_model.detect_mustard import MustardDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run real-time mustard detection on Orbbec camera frames.")
    parser.add_argument("--weights", type=str, default=None, help="Path to model weights (.pt)")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument("--window-name", type=str, default="Mustard Detector", help="OpenCV window title")
    return parser.parse_args()


class OrbbecMustardApp:
    def __init__(self, weights: str | None = None, conf: float = 0.25, iou: float = 0.7, imgsz: int = 640, device: str = "cpu"):
        self.detector = MustardDetector(model_path=weights, conf=conf, iou=iou, imgsz=imgsz, device=device)
        self.pipeline = None

    def start(self) -> None:
        try:
            self.pipeline = Pipeline()
            self.pipeline.start()
            print("Orbbec pipeline started. Press Q or ESC to quit.")
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

                color_frame = frames.get_color_frame()
                if color_frame is None:
                    continue

                frame = frame_to_bgr_image(color_frame)
                if frame is None:
                    continue

                results = self.detector.model(frame, conf=self.detector.conf, iou=self.detector.iou, imgsz=self.detector.imgsz, verbose=False)
                result = results[0]
                annotated = result.plot()
                if annotated is not None:
                    display = annotated
                else:
                    display = frame.copy()

                cv2.putText(
                    display,
                    f"Detections: {len(result.boxes)}" if result.boxes is not None else "Detections: 0",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

                cv2.imshow(window_name, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
        finally:
            cv2.destroyAllWindows()
            self.stop()


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
