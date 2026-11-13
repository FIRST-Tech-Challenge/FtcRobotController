"""Try a trained YOLO detector on a PC image and print each bounding box.

Example:
    python aiController/training/predict.py path/to/photo.jpg
"""

import argparse
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = WORKSPACE / "runs" / "baseline" / "weights" / "best.pt"


def main() -> None:
    parser = argparse.ArgumentParser(description="Run FTC object detection on a test image")
    parser.add_argument("image", type=Path, help="Image to analyze on the computer")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--conf", type=float, default=0.35, help="Minimum model score")
    args = parser.parse_args()

    if not args.image.is_file():
        parser.error(f"Image not found: {args.image}")
    if not args.model.is_file():
        parser.error(f"Model not found: {args.model}. Train the model first.")
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf must be between 0 and 1")

    from ultralytics import YOLO

    model = YOLO(str(args.model))
    results = model.predict(source=str(args.image), conf=args.conf, verbose=False)
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls.item())
            confidence = float(box.conf.item())
            x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
            name = result.names[class_id]
            print(f"{name}: score={confidence:.2f} box=({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f})")


if __name__ == "__main__":
    main()
