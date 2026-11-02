# Driver Station Testing Design

> Historical document: FTC Knowledge Bank was removed from this project on 2026-10-28. FTCKB integration steps, rules, and check commands below are retained as history and no longer apply.

**Date:** 2026-09-13  
**Project:** FTC-20827 BioBuzz<br>
**Scope:** Replace host-side JUnit tests with FTC Driver Station test OpModes.

## Goal

Keep all team-authored Java code under the normal TeamCode package tree while placing diagnostic code in its own `testing` package. Remove JUnit so Android Studio does not show unresolved JUnit imports when the project is opened or synced without the host-test dependency.

## Directory layout

Delete `TeamCode/src/test/` and remove the JUnit dependency from `TeamCode/build.gradle`.

Add these files under:

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/testing/`

- `FrameworkSelfTest.java`: hardware-free checks for the command scheduler and mecanum mixer.
- `DriveHardwareTest.java`: guarded, low-power individual drivetrain motor test.

No test class is placed directly under `TeamCode/src/main/java`, and no separate Java source root is introduced.

## Framework self-test

`FrameworkSelfTest` is a `LinearOpMode` listed in the Driver Station `Testing` group. After PLAY, it runs deterministic checks without accessing the hardware map. It covers:

- command initialization, execution, completion, and requirement release;
- requirement conflicts and interruption;
- default-command scheduling;
- cancellation during a scheduler cycle;
- initialization-failure cleanup;
- multi-requirement default-command blocking;
- shutdown cleanup and terminal scheduler state;
- mecanum zero, forward, strafe, turn, and normalization calculations.

A small private assertion/reporting helper records each named check. Telemetry shows each PASS/FAIL result, the totals, and the first failure message. Failures do not require JUnit and do not crash before the result is shown. The OpMode remains on the result screen until STOP.

## Drive hardware test

`DriveHardwareTest` is an iterative `OpMode` in the Driver Station `Testing` group. It creates the drivetrain through `RobotHardware`, preserving centralized names, directions, braking, and motor ownership.

The four face buttons map spatially to individual wheels:

- Y: front-left
- B: front-right
- X: back-left
- A: back-right

The selected wheel receives `0.15` power through `DriveSubsystem.drive(WheelPowers)` only while exactly one button is held. No button, multiple buttons, initialization failure, or STOP commands all four wheels to zero. Telemetry continuously shows the mapping, selected wheel, configured motor names, test power, and the reminder to lift the robot before running.

## Documentation and validation

Update `TeamCode/README.md` to describe the `testing` package, Driver Station procedures, and the removal of the Gradle/JUnit test command. Validation consists of:

1. search confirming no `org.junit`, `@Test`, or JUnit dependency remains;
2. `:TeamCode:assembleDebug` passing;
3. FTCKB project check and integration verification;
4. manual robot validation remaining explicitly unverified until the robot is lifted and both test OpModes are run.

The existing FTCKB Limelight regex limitation remains a known checker false positive for full historical diffs; no meaningless Limelight calls will be added.
