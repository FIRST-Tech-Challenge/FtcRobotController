# AI Vision / Perception - Fulgur Robotics

Starter workspace for the team's FIRST Tech Challenge (FTC) vision work.
This directory holds **documentation, dataset conventions, and PC-side ML tools**.
The Java classes that the FTC Robot Controller actually compiles live under
[`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/).

The old `aiController` placeholder was a *file*. This foundation replaces it with
an actual directory so the project can grow cleanly.

## Repository layout

```text
aiController/
  README.md                   <- overview and setup
  docs/
    GLOSSARY.md               <- standard terminology / nomenclature
    TEST_PLAN.md              <- validation checklist
  datasets/
    README.md                 <- data and annotations conventions
    dataset.yaml.example      <- example classes and splits for YOLO
  models/
    README.md                 <- trained-model storage/deployment notes
  training/
    requirements.txt          <- PC Python dependencies (optional)
    train.py                  <- train a YOLO object detector on a computer
    predict.py                <- test a trained detector on an image
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
  VisionConfig.java           <- hardware names and thresholds
  TargetDetection.java       <- immutable detection result
  VisionSubsystem.java       <- webcam + color blobs + AprilTags
  VisionDebugOpMode.java     <- read-only Driver Station test
  HuskyLensAiRecognitionOpMode.java <- optional true AI sensor
```

## Path A: FTC webcam (works without an AI model)

1. In the Robot Controller configuration, add a compatible webcam called **`Webcam 1`**.
2. Open the complete FTC project in Android Studio and let Gradle sync with FTC SDK 12.0.0.
3. In `VisionConfig.java`, adjust `TARGET_COLOR`, `TARGET_LABEL`, and `MIN_BLOB_AREA_PX`
   to the real game piece under your actual field lighting.
4. Build/deploy the Robot Controller app and select **`Vision: Recognition Test`** in the Driver Station.
5. Look at telemetry and the Camera Stream. No motors or servos are driven.
6. When reusing vision in autonomous code, instantiate `VisionSubsystem` once,
   call `findLargestColorTarget()` or `getAprilTags()` repeatedly, and `close()` at the end.

Example result: the target's center in pixels, blob area, and `horizontalError`:
`-1` means left, `0` centered, `+1` right **in image coordinates**. Whether
positive error means "turn left" or "turn right" depends on camera orientation.

This is **classical computer vision**, NOT a learned neural-network classifier.
A yellow field marking could generate a false positive if it looks like the
configured color. The simple class label is a heuristic name, not an AI prediction.

## Path B: actual AI object recognition (optional hardware)

The included `HuskyLensAiRecognitionOpMode` shows how to read learned object IDs
from a **DFRobot HuskyLens** over I2C. It is marked `@Disabled` because the repository
does not establish that this sensor is installed. Configure a device called
`huskylens`, train/select its object recognition algorithm, then remove `@Disabled`.
Its learned IDs are specific to the sensor configuration: make an ID-to-label map
once you know what the team has trained.

## Path C: custom neural network (PC training foundation)

The optional `training/` scripts can train YOLO to classify and locate **your own
competition objects** from labeled images. See [`datasets/README.md`](datasets/README.md).
This is a **PC inference/training demo**, not a working on-robot TFLite deployment.
An exported YOLO model cannot simply be dropped into the FTC `VisionPortal`:
a compatible on-robot inference runtime/processor, model format, preprocessing,
and performance testing would still need to be implemented.

## Standards and sources

- SDK version in this repo: `org.firstinspires.ftc:Vision:12.0.0`.
- Project sources of truth:
  [`ConceptVisionColorLocator_Rectangle.java`](../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVisionColorLocator_Rectangle.java),
  [`ConceptAprilTagEasy.java`](../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptAprilTagEasy.java),
  [`SensorHuskyLens.java`](../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorHuskyLens.java).
- This is a baseline for experimentation; it has **not** been deployed to or verified on a physical robot.
