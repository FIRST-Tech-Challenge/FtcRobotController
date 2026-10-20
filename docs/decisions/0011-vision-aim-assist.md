# 0011: Aim assist: hold a button, the robot faces the nearest AprilTag

Status: Accepted. Not yet tried on the robot. The turn direction must be checked first.
Date: 2026-10-15

## Context

The coach wants a button that turns the robot to face the nearest AprilTag, with the
Driver Station showing what the robot knows about that tag. This is the first piece of the
launcher team's aim assist: later, the tag will be the hive's, and the launcher will use
the range.

Decode had this on the left bumper, with a Limelight 3A. It never worked: the next line of
code overwrote the turn every loop (PORT-PLAN P6, decision 0002). Which camera this
season's robot has is not settled. Last year's robot has a Limelight; FTC's standard is a
USB webcam read by the SDK's own AprilTag processor.

## Decision

- **A `Vision` rule**, like `Imu`: `SeesTag()`, `GetTagId()`, `GetBearing()`, `GetRange()`,
  `CountTags()`. Bearing is degrees to the nearest tag, **left positive, like yaw**.
- **Three parts follow it.** `Vision_Limelight` (Limelight 3A, an AprilTag pipeline),
  `Vision_Webcam` (the SDK's `VisionPortal` + `AprilTagProcessor`, from the sample
  ConceptAprilTag), and `Vision_None` for a robot with no camera.
- **`VisionSelector.Pick`** looks at the robot configuration: a `limelight` wins, then a
  `Webcam 1`, else none. The Driver Station says which. Same idea as `ImuSelector`.
- **"Nearest"**: with a webcam, the smallest `ftcPose.range`. With a Limelight, the tag that
  fills the most of the picture (`getTargetArea`), since a nearer tag looks bigger.
- **A tag older than `visionMaxAgeMs` (250 ms) counts as gone.** The robot never chases a
  tag the camera lost.
- **The turn** is in `objects/AimAssist.RotateToward`: `kP x bearing`, never past
  `aimMaxRotate`, and 0 within `aimDoneDegrees` of center so the robot does not wobble.
  No hardware, so it is tested on a laptop.
- **In `Drive_Manual`**, holding the **left bumper** swaps the right stick's turn for the
  aim assist turn. Drive and strafe stay on the sticks. Let go and the stick is back.
  Safe Mode still limits every wheel.
- **The Bot builds the camera** unless `visionDisabled` is true, before the drive, so the
  drive always aims at this loop's picture. It hands the drive a `Vision`.
- **Debug on the Driver Station, every loop:** tags in view, one line per tag (id, range
  or size, bearing) with the nearest marked `<- nearest`, then the nearest tag's id,
  bearing with left or right spelled out, range, how long since it was seen, and the aim
  assist's turn. Several tags in view is the test, not a problem.
  `Test: Vision` runs the camera alone, with the drive and launcher off.
- **Signs.** The Limelight counts right as positive (`tx`); `Vision_Limelight` flips it.
  The SDK's bearing already counts left as positive. In `Drive_Manual`, a positive rotate
  turns left (the stick is negated, `-right_stick_x`). So positive bearing gives positive
  rotate gives a left turn, toward the tag.

## Check on the robot, in this order

1. `Test: Vision`, wheels off the ground. Hold a tag in front of the camera. `Nearest tag`
   shows its id. Move the tag to the camera's left: the bearing says `left`. If it says
   `right`, the camera is reading backwards: make `aimKP` negative in `BotProperties`.
2. `HyDrive`, wheels off the ground. Hold the left bumper with the tag on the left. The
   left wheels should roll forward and the right wheels backward: a left turn.
3. On the floor, Safe Mode key in. Hold the left bumper. The robot should swing to face the
   tag and stop within a degree or so. If it swings back and forth, lower `aimKP` or raise
   `aimDoneDegrees`. If it stops short, raise `aimKP`.

## Consequences

- HyDrive now looks for a camera at INIT. With none it says so and runs as before.
- The webcam needs the SDK to know the tag's size to give a range. The game tags are in
  the SDK's library; a printed test tag of another family or size shows up with no pose and
  is skipped. The Limelight needs its AprilTag pipeline chosen in its own web page.
- `Vision_Webcam` builds a `VisionPortal`, which needs Android's camera stack, so its tests
  stop at the nearest-tag rule. `Vision_Limelight` is tested with pretend results.
- "Nearest" is a stand-in. The hive's tag, not the nearest, is what the launcher wants.
  That is one `if` in `Process()` when the tag ids are known.
- Tests: `AimAssistTest` (6), `DriveAimTest` (6), `VisionLimelightTest` (7),
  `VisionSelectorTest` (2), two more in `BotTest`.

## Added 2026-10-16: the Limelight's web page through the hub

The first robot test found the pipeline's ID filter dropping every tag. Fixing it meant
unplugging the Limelight from the hub and plugging it into a laptop, every time.

- **`tools/LimelightWebProxy`** starts when the Robot Controller app starts and passes
  ports 5800 to 5807 through from the hub's Wi-Fi to the Limelight's USB network. On a
  laptop joined to the robot's Wi-Fi, `http://192.168.43.1:5801` is the Limelight's page.
  The same trick WPILib's PortForwarder does on an FRC robot. Not yet tried on the robot.
- **The Driver Station says whether the Limelight is connected** and prints that address
  during INIT.
- **Next, if the filter bites again:** keep the AprilTag pipeline as a JSON file in the
  repo and have the robot upload it with `Limelight3A.uploadPipeline()`. The SDK added
  that so teams can version-control their pipelines. Then the setting lives in git, not
  in the camera.
