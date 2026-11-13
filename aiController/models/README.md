# Trained models

This folder is intentionally empty of model weights.

A model has to be trained/evaluated on the team's images before it can claim
to recognize the competition objects. Training with `training/train.py`
produces weights under `aiController/runs/baseline/weights/best.pt` on a PC.

Do not claim these `.pt` weights run in FTC's `VisionPortal` automatically.
Robot deployment of a custom TFLite/ONNX model needs an appropriate compatible
runtime/vision processor, preprocessing, output decoding, and latency testing.

You can instead use the optional HuskyLens hardware example for an initial
on-sensor learned recognizer, without deploying PC weights to the Control Hub.
