#!/usr/bin/env python3
"""Mustard detector wrapper for the custom YOLO model.

This file is restored because the original app file went missing.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Optional, Union

import cv2
from ultralytics import YOLO


DEFAULT_MODEL_NAMES = ["best.pt", "last.pt"]


class MustardDetector:
    def __init__(
        self,
        model_path: Optional[Union[str, Path]] = None,
        *,
        conf: float = 0.25,
        iou: float = 0.7,
        imgsz: int = 640,
        device: str = "cpu",
    ) -> None:
        self.model_path = self._resolve_model_path(model_path)
        self.model = YOLO(str(self.model_path))
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz
        self.device = device

    @staticmethod
    def _resolve_model_path(model_path: Optional[Union[str, Path]]) -> Path:
        base_dir = Path(__file__).resolve().parent

        if model_path is not None:
            p = Path(model_path).expanduser()
            if p.exists():
                return p.resolve()
            raise FileNotFoundError(f"Model file not found: {p}")

        for root in [base_dir, base_dir / "weights"]:
            for name in DEFAULT_MODEL_NAMES:
                candidate = root / name
                if candidate.exists():
                    return candidate

        for child in base_dir.rglob("*.pt"):
            if child.name in DEFAULT_MODEL_NAMES or "mustard" in child.name.lower():
                return child

        raise FileNotFoundError(
            "No model weights found. Train a model or pass --weights to a valid .pt file."
        )

    def predict(self, source):
        return self.model.predict(
            source=source,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )

    @staticmethod
    def draw_boxes(image, result) -> None:
        if result is None:
            return
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            label = f"mustard {conf:.2f}"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mustard detector via YOLO")
    parser.add_argument("--weights", type=str, default=None, help="Path to model weights .pt")
    parser.add_argument("--source", type=str, default="0", help="Image path or camera index")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    detector = MustardDetector(
        model_path=args.weights,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        device=args.device,
    )
    results = detector.predict(args.source)

    if args.show:
        for result in results:
            image = result.orig_img.copy()
            detector.draw_boxes(image, result)
            cv2.imshow("Mustard Detection", image)
            if cv2.waitKey(0) & 0xFF == 27:
                break
        cv2.destroyAllWindows()
    else:
        print(f"Processed {len(results)} result(s).")


if __name__ == "__main__":
    main()
