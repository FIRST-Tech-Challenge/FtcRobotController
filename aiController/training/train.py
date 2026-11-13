"""Train a custom YOLO detector on a computer (not on the FTC Control Hub).

Example:
    python aiController/training/train.py --data aiController/datasets/dataset.yaml
"""

import argparse
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a vision model on labeled FTC images")
    parser.add_argument("--data", type=Path, default=WORKSPACE / "datasets" / "dataset.yaml")
    parser.add_argument("--weights", default="yolo11n.pt", help="Pretrained YOLO checkpoint")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--imgsz", type=int, default=640)
    args = parser.parse_args()

    dataset_yaml = args.data.expanduser().resolve()
    if not dataset_yaml.is_file():
        parser.error(f"Dataset config not found: {dataset_yaml}. See datasets/README.md")

    from ultralytics import YOLO

    model = YOLO(args.weights)
    model.train(
        data=str(dataset_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        project=str(WORKSPACE / "runs"),
        name="baseline",
        exist_ok=True,
    )
    print("Training complete. Check aiController/runs/baseline/weights/best.pt")


if __name__ == "__main__":
    main()
