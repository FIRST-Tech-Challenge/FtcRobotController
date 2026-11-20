# Pollen tracking and autonomous collection

Select **Test: Pollen Navigation (Pedro + Ivy)** in the Driver Station's Test group. This is an experiment, separate from the match autonomous. It searches for pollen with the Logitech C270, aligns with a tracked blob, approaches, and runs the intake through a short final pickup. Pedro Pathing v3 controls all movement; Ivy schedules the collection command through Zenith's runtime. It then searches again until the transfer sensor confirms full or a navigation limit is reached. See [autonomous setup](Autonomous.md) for installed versions, tuning, and match routines.

The current configuration is **diagnostic only**. Pedro dynamics must be measured and `PedroConfiguration.TUNING_VERIFIED` set after verification. The C270 mount is confirmed; collection also requires pickup and color calibration plus calibrated intrinsics, or explicit approximate-intrinsics eligibility. Enter camera values in `PollenVision.CONFIG` / `PollenConfiguration`; see [SoftwareConfiguration.md](SoftwareConfiguration.md). Zero offsets may be measured values; measured status is independent of the numbers.

Current scope is pollen detection and navigation. Match paths and scoring/parking routes are deferred and are not required to configure this routine. Physical checks described below are for later; none were performed during the source review.

## Hardware and camera configuration

| Device | Configuration / mounting |
| --- | --- |
| Logitech C270 | Configure as `webcam_1`, connected through the powered USB hub to the Control Hub. Team-confirmed lens mount: **195 mm above floor**, **248 mm forward**, **0 mm sideways** from robot center; straight forward, upright, optical axis **15 degrees below horizontal**. |
| Limelight 3A | Remains the turret's AprilTag camera. The pollen routine uses the C270 and does not initialize the turret or Limelight. |
| Drivetrain | Existing `MecanumDrivetrain` hardware names and motor configuration. |
| Pinpoint | Control Hub **I2C port 0**, configured as `pinpoint`; leave the robot stationary during INIT calibration. |
| Intake / transfer / gate | Existing `intake_motor`, `transfer_motor_1`, `transfer_motor_2` (linked to stage 1), `distance_sensor`, and `gate_servo` names. |

`PollenVision.HW_WEBCAM` controls the camera name. Color detection currently assumes **yellow pollen**, using configured HSV thresholds. Tune `PollenConfiguration` hue/saturation/value thresholds and the area, density, aspect-ratio, and circularity filters under actual field lighting.

The supplied Section 9 TU06 reference confirms yellow pollen approximately 2.8 in. (71 mm) in diameter, with size/shape variation; see [FieldReference.md](FieldReference.md). This supports the chosen color target, but does not calibrate HSV thresholds or turn the lowest contour point into an exact floor-contact measurement.

The camera runs at 320 x 240 with MJPEG. Image processing runs on the camera thread and publishes an immutable detection with its original capture timestamp. The control loop never reads hardware maps or waits for images.

## Verify vision before driving

1. Run **Test: Pollen Camera Calibration** in the Test group. It initializes only the C270, so the drivetrain, Pinpoint, turret, and intake can be disconnected.
2. Inspect the Driver Station Camera Stream during INIT. Contours show the raw color detections; telemetry reports the selected, filtered target. Place individual pollen pieces at known distances and on both sides of the image.
3. Confirm that a target on the robot's right has a positive bearing, a left target has a negative bearing, and a lower floor contact point produces a shorter distance estimate. Keep the camera upright and facing forward.
4. The confirmed mount is already set to **195 mm high, 15 degrees below horizontal, 248 mm forward and 0 mm right**, with mount status measured. Update it only if the installed mount changes; the earlier 180-mm/zero-forward example is superseded.
5. Check **Pollen Intrinsics** telemetry. The code uses valid SDK camera calibration when available. Otherwise it uses an explicitly approximate 60-degree horizontal field of view with square pixels. Calibrate the C270 at the selected resolution and provide its calibration through the SDK, or tune the fallback FOV against measured floor distances. The fallback is not a measured C270 specification.
6. Measure Pedro tuning with the [Pedro web tuner](Autonomous.md#pedro-web-tuner), run **Test: Pedro Drive** to verify axes, paths and holds, and verify final pickup distances at low speed in a cleared area. Java's initial dynamics and the editor's robot dimensions are unmeasured examples.

Distance is estimated by intersecting a ray through the blob's lowest contour point with the floor, using lens intrinsics and the confirmed 195-mm height / 15-degree downward pitch, then adding the 248-mm lens offset to report distance from robot center. This assumes flat ground and estimates floor contact rather than the pollen center. Clipped blobs and rays at/above the horizon are rejected. Range accuracy depends on calibration, lens distortion, shadows, and the visible contour.

## Autonomous behavior and tuning

`PollenNavigator` owns the non-blocking state machine and requests STOP, HOLD, or FOLLOW with an absolute pose target. It computes no motor powers. `PollenPathFollower` turns those targets into Pedro holds and straight paths, limits replanning, and immediately stops on target loss. `PollenAuto` loads the test JSON through Zenith; `BioBuzzRobot` schedules collection on Ivy and reads Pinpoint once per loop.

| State | Behavior / default limit |
| --- | --- |
| Waiting for hardware | Motors remain stopped until fresh camera frames and READY Pinpoint data are available. Fault after 4 seconds. |
| Searching | Pedro holds the search position with a heading target 90 degrees clockwise from entry. Stop after 90 degrees or 4 seconds. Select the largest valid blob initially. Subsequent frames prefer the blob closest to its previous image position, within a 65 px gate; the lock expires after 0.4 seconds without a match. |
| Approaching | Pedro turns in place until within 12 degrees of alignment, then follows a line toward the pickup stand-off point. Path velocity fraction and wheel power are 1 (no cap; `PedroConfiguration`). Stop on missing detections; wait 0.35 seconds before searching again. Approach timeout is 7 seconds. |
| Collecting | Enter after at least 3 distinct close, aligned frames spanning 0.12 seconds. The uncalibrated example entry is within 650 mm forward of robot center and 4 degrees bearing error. Pedro follows one fixed forward pickup path for up to 450 mm or 0.9 seconds, allowing pollen to leave camera view. |
| Settling | Stop driving and keep intaking for 0.4 seconds, then search again. Telemetry counts **pickup attempts**, since no individual pickup sensor is present. |
| Finished / fault | Stop drivetrain, intake, and both transfer motors; keep the gate closed. |

`PollenConfiguration.pickupStartMm`, `pickupTravelMm`, and `PollenNavigator.PICKUP_TIME_LIMIT_SECONDS` must match the camera's near blind spot, the intake's leading edge, and measured robot speed. A bounded blind pickup begins only after visual confirmation. Target loss during the approach stops movement; stale camera frames or unavailable Pinpoint data stop the routine, including during pickup.

With the current approximate 60-degree horizontal FOV and square-pixel model at 320x240, an unclipped bottom contact at pixel y=237 projects to about **500 mm from robot center** (498.6 mm in the model). The team updated the downward pitch from 10 to **15 degrees on November 5, 2026**, bringing this boundary closer than the earlier 550-mm estimate. The old 350-mm transition remains invisible. The explicitly **uncalibrated 650-mm transition / 450-mm blind-travel example** remains visible with the new mount, but still requires measured pickup calibration. `PollenCameraModel` computes the near/far visible floor boundary using the active intrinsics, optional distortion and contour clipping margin. An enabled pickup outside that region is rejected (including a 20-mm margin); SDK intrinsics are checked when the camera starts. Configuration, color and pickup calibration remain ineligible by default.

Stage 1 of the transfer (both linked transfer motors) loads during active collection, and the gate stays closed. The existing full sensor must stay HIGH for 0.5 seconds to confirm a full queue. This routine collects pollen; it does not perform a shooting sequence. All movement is additionally bounded by **28 seconds** and **2400 mm of accumulated translation**. Use a cleared collection area: field routes and obstacle avoidance require a separate autonomous strategy.

Motor shutdown runs before camera shutdown, including on Driver Station STOP or exceptions.

## Files and verification

- `PollenVision.java`: camera hardware, SDK color processing, target continuity, fresh-frame checks, telemetry, and camera shutdown.
- `PollenTarget.java` / `PollenCameraModel.java`: immutable geometry, active intrinsics, distortion removal and floor visibility.
- `PollenNavigator.java`: hardware-independent state machine and pose targets.
- `PollenPathFollower.java`: pose targets to Pedro paths/holds, with bounded replanning.
- `PedroDrive.java` / `BioBuzzRobot.java`: drivetrain/localization adapters and shared Ivy/Zenith runtime.
- `PollenVisionTestOpMode.java`: standalone camera test.
- `PollenAuto.java`: sole Test-group entry point; team deployment retains its isolated hardware profile.
- `autos/pollen-navigation-test.auto.json`: canonical editor source; APK assets are managed deployment output. Use [UI edit/save → team deploy → build/install](ZenithAuthoringGuide.md).

From the repository root:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

The unit tests cover floor geometry, steering signs, startup calibration, acquisition, target loss and reacquisition, duplicate-frame rejection, bounded blind pickup, heading wrap, full-queue stopping, hardware faults, and time/travel limits. Robot motion, color thresholds, and camera calibration still require physical testing.

November 5, 2026 pitch update: the shared C270 pitch is now 15 degrees down. All 27 focused pollen tests passed and `:TeamCode:assembleDebug` packaged successfully using JDK 25 and offline dependencies; log: `build/webcam-pitch-verification.log`. The visibility regression checks the approximately 498.6-mm near boundary and retained pickup visibility with the active mount. Full unit tests, lint and simulation were omitted for this mounting-value adjustment; the current simulator does not model USB webcam processing. No physical camera/pickup validation or robot deployment was performed, and calibration eligibility remains unchanged.

November 11, 2026 follow-up: the synthetic 180-mm camera fixtures in `PollenConfigurationTest`, `PollenTargetTest` and `PollenNavigatorTest` also moved from 10 to 15 degrees down, so no camera-pitch value in code or tests still uses 10 degrees. The October 10-degree figures above and in the review documents are kept only as history.
