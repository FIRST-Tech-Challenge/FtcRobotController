# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A fork of the *FIRST* Tech Challenge SDK (`FtcRobotController` v11.2.1, DECODE 2025-2026 season) containing team robot code for the "BioBuzz" team. The output is an Android APK that runs on the Robot Controller phone/Control Hub.

- `FtcRobotController/` — upstream SDK app module and sample OpModes. **Do not edit**; it is replaced wholesale on SDK updates. Read it for reference (samples live in `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/`).
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` — all team code. This is the only place changes belong.
- `build.common.gradle` / `build.dependencies.gradle` — shared SDK build config; upstream files, avoid editing (customizations go in `TeamCode/build.gradle`).

## Build

```bash
./gradlew :TeamCode:compileDebugJavaWithJavac   # fastest check that team code compiles
./gradlew :TeamCode:assembleDebug               # build the APK
./gradlew installDebug                          # build + install onto a connected RC device
```

Add `--offline` when there is no network; the Gradle 9.1 wrapper and AGP 8.13.2 deps are already cached.

There are **no unit tests and no test source set** — `TeamCode/src/main` only. Verification means: it compiles, and someone runs the OpMode on the robot. Do not invent test commands or add a test framework unless asked.

## OpMode conventions

Any class in `teamcode` annotated `@TeleOp` / `@Autonomous` shows up in the Driver Station menu. Two OpMode styles are both in use here:

- `extends OpMode` — SDK calls `init()` then `loop()` repeatedly (`AprilTagTest`, `FaceAprilTagOpMode`, `FlywheelTuner`, `ServoTest`).
- `extends LinearOpMode` — you own the loop inside `runOpMode()` via `while (opModeIsActive())` (`MecanumOpMode`, `BioBuzzOpMode`, `BB2DecodeOpMode`, `AkiMecanumTeleOp`).

`FlywheelTuner.java` sits in the `teamcode` directory but declares `package ...teamcode.tutorials`. It compiles, but move the file rather than "fixing" the package if you touch it.

## Architecture: three parallel robot codebases

The repo carries several generations of robot code side by side. They share hardware-config names but are otherwise independent — **changing a helper class affects only its own OpMode**.

1. **`MecanumOpMode`** (current composed design) — the OpMode is a thin shell that owns `MecanumDriveTrain`, `Launcher`, `Camera`, and `Auto`. During init it cycles through modes (TELEOP / AUTO BLUE|RED GOAL|WALL) using the gamepad PS button, then dispatches after `waitForStart()`. `Auto` is pure sequencing built from `driveTrain.driveWithTime(...)` and `launcher.shootWithTime(...)` — time-based dead reckoning, no odometry.
2. **`BioBuzzOpMode`** ("Lox", newest robot) — pairs with `MecanumDrive`, which adds an intake motor plus two CRServos and uses BRAKE zero-power behavior. Currently TeleOp only.
3. **`BB2DecodeOpMode`** (legacy) — a single ~400-line monolith for the older two-motor tank robot (`leftDrive`/`rightDrive`), with flywheel, servo, AprilTag, and autonomous all inline. Superseded by (1); left in place deliberately.

The subsystem helpers are constructed with a concrete OpMode type, not `LinearOpMode` (`MecanumDriveTrain(MecanumOpMode)`, `MecanumDrive(BioBuzzOpMode)`), so a helper cannot be reused across robots without changing that field. `MecanumDriveTrain` reaches back through it for `lox.camera` and `lox.opModeIsActive()`.

Motor direction assignments differ between `MecanumDrive` and `MecanumDriveTrain` because the two robots are wired differently — do not "harmonize" them.

## Hardware configuration names

`hardwareMap.get(...)` names must match the config saved on the Robot Controller. Names in use: `frontLeft`, `frontRight`, `backLeft`, `backRight` (mecanum); `leftDrive`, `rightDrive` (legacy tank); `flywheel` (DcMotorEx, velocity-controlled), `coreHex` (feeder), `servo` (launch-angle deflector), `intake`, `leftIntakeServo`, `rightIntakeServo`, `Webcam 1`, `pinpoint`. A typo here compiles fine and fails at runtime on the robot.

## Launcher tuning

`Launcher` and `BB2DecodeOpMode` hold duplicated shot constants (`nearVelocity`/`midVelocity`/`farVelocity` in encoder ticks/sec, `nearAngle`/`midAngle`/`farAngle` as servo positions) and PIDF gains `P = 265`, `F = 14.098` applied via `setPIDFCoefficients(RUN_USING_ENCODER, ...)`. These are empirical, tuned on the physical robot with `FlywheelTuner` — treat them as data, not code to refactor. If a change alters them, say so explicitly.

## Pedro Pathing

`com.pedropathing:revhub:3.0.0` and `com.pedropathing:tuning:1.0.0` (Pedro 3 + AutoTune) are
declared in `TeamCode/build.gradle` (not the upstream `build.dependencies.gradle`) and resolve from
Maven Central; they pull in `com.pedropathing:core` and `kotlin-stdlib`. AutoTune also pulls
`dev.frozenmilk.sinister:Sloth`, which is why `https://repo.dairy.foundation/releases/` is declared
alongside the Panels repo. Pedro 3 is a full rewrite — `FollowerBuilder`, `FollowerConstants`,
`MecanumConstants`/`PinpointConstants`, `startTeleopDrive`/`setTeleOpDrive` and
`com.pedropathing.geometry.Pose` are all 2.x API that no longer exists; anything found online for
Pedro 2 has to be translated, not copied.

`pedroPathing/Constants.java` holds the config for Lox and exposes `create(hardwareMap)`, which
returns `new Follower(localizer, drivetrain, algorithm)`. Its Pinpoint values mirror the ones tuned
in `SensorGoBildaPinpoint`, and its motor names/directions mirror `MecanumDrive` — those are three
independent copies, so a re-wire has to be applied to each.

**The Foresight constants in `Constants.java` are placeholders, not Lox's numbers.** They are the
example values from Pedro's docs, measured on someone else's robot. Pedro 3 gives these no defaults
(they are `ConfigVar.required()` and throw when read), so something has to be there for a Follower
to exist at all. Paths will not track accurately until AutoTune output replaces them. Autonomous
still runs on the time-based `Auto` class.

**Tuning is AutoTune**, a web UI the Robot Controller serves at `http://192.168.43.1:10158`.
`pedroPathing/Tuning.java` registers the procedures with `@Tuner` — these are not OpModes and do not
appear in the Driver Station menu; Sinister scans for them at startup. The procedures themselves
(`pedroPathing/procedures/`: `MecanumTuner`, `PinpointTuner`, `ForesightTuner`, `Tests`, ~2000 lines)
are vendored verbatim from the Pedro Quickstart with only the `package` line changed — treat them as
a copied dependency and re-copy rather than edit. Tune in the order Mecanum → Pinpoint → Foresight,
pasting each step's generated block into `Constants.java` before the next. `Tests` verifies the
result and produces no constants; its `Localization`, `Odometry`, `Driving` and `Pose` tests work
with a null algorithm, so they are usable before Foresight is tuned.

**Panels** (`com.bylazar:fullpanels`, plus `com.pedropathing:telemetry`) is the on-robot dashboard,
served at `http://192.168.43.1:8001`. It is published to bylazar's own Maven repo, declared in
`TeamCode/build.gradle` — not Maven Central. Pedro 3 does not replace it: AutoTune's
`Tests → Localization` drives the robot and prints the pose as text to the Driver Station, and
AutoTune's whole display API is static images plus the wheel diagram used for motor directions. It
has no field view, so the robot-drawn-on-the-field picture in `PedroLocalizationTest` is still only
available through Panels.

Panels forced two build changes worth knowing about:

- `TeamCode/build.gradle` overrides `compileSdk 36`, because its androidx dependencies refuse to be
  consumed below 34 while `build.common.gradle` pins 30. The override is scoped to TeamCode so the
  upstream SDK files stay untouched; `minSdk` (24) and `targetSdk` (28) are unchanged, so on-robot
  behavior is not affected. Anyone building this repo needs the `android-36` platform installed.
- The debug APK grew from ~52 MB to ~76 MB (16 dex files).

**Panels draws Lox mirrored.** The stock `PEDRO_PATHING` canvas preset made driving forward move the
marker left; `PedroLocalizationTest.REVERSE_XY` corrects it. The odometry itself was verified correct
on the robot, so this is a drawing-only correction - never fix a Panels orientation problem by
changing encoder or motor directions in `Constants`. Panels preset values also differ between
versions (`com.bylazar:field:1.0.6` uses `DEG_90`, upstream source uses `DEG_270`), so the OpMode
prints the transform actually in effect at init.

## Vision

AprilTag detection goes through `AprilTagProcessor` + `VisionPortal` on `Webcam 1`. `Camera` is the reusable wrapper (`findAprilTag(id)`, `update()`); ID 20 is the blue goal tag. Aiming is a proportional turn on `detection.ftcPose.bearing` (see `MecanumDriveTrain.driveWithGamepad` and `FaceAprilTagOpMode`).

## Git

Work happens on short-lived branches (`aki/...`, `bb2/...`, `test/...`) merged to `main` via PR; `dev` also exists. Season code is written by students during competition — keep diffs small and readable, and preserve existing commented-out code (it is often a hardware option toggled between matches).
