# BIOBUZZ 2026–2027 Command Framework Design

> Historical document: FTC Knowledge Bank was removed from this project on 2026-10-28. FTCKB integration steps, rules, and check commands below are retained as history and no longer apply.

## Goal

Build a small, team-owned command-based framework under `TeamCode` for FTC team 20827's 2026–2027 BIOBUZZ season. The framework must remain understandable to student programmers, avoid an unmaintained FTCLib runtime dependency, and include a minimal mecanum TeleOp that proves the architecture works without claiming unverified robot behavior.

## Required execution order

1. Pin and integrate FTC Knowledge Bank commit `105c8e47ddee734365af50dd2305caa13646ab21` into this project.
2. Configure the integration for team `20827`, season `2026-2027`, and the explicit `command-based` profile.
3. Run the integration dry-run before its write operation.
4. Run FTCKB `resolve` and treat the returned active rules as development requirements.
5. Implement the TeamCode framework and example.
6. Run unit tests, `:TeamCode:assembleDebug`, FTCKB `check`, and FTCKB `verify`.

The Knowledge Bank is a project-level pinned submodule and Agent skill. It is not an Android runtime dependency. Integration must not commit personal SDK or JDK paths, API keys, hooks, or generated build output.

## Architecture

### Command core

`core/command` owns the framework contracts and scheduler:

- `Subsystem` exposes periodic work and a safe stop operation.
- `Command` exposes initialize, execute, completion, interruption, and required-subsystem behavior.
- `CommandScheduler` owns command lifecycle, prevents simultaneous ownership of the same subsystem, runs subsystem periodic methods, schedules defaults, and cancels all work on shutdown.
- Small reusable commands and command groups may be included only where they are required by the initial example. The first version will not reproduce the full FTCLib/WPILib API.

Commands receive live input through suppliers instead of snapshotting gamepad values during construction. Every command explicitly declares its subsystem requirements. Interrupted commands must leave their outputs safe.

### Robot configuration and hardware

`config` centralizes hardware-map names and tunable constants. Values that depend on the physical robot remain clearly marked as candidate values until the team verifies them on hardware.

`hardware` centralizes SDK device lookup and motor initialization. It applies explicit direction, zero-power behavior, and run mode in one place. Hardware names, directions, gear ratios, dimensions, odometry offsets, PID values, and power limits must not be copied from another robot as if verified.

### Subsystems and commands

`subsystems/DriveSubsystem` owns the four drivetrain motors and is the only component allowed to command their power. It accepts normalized wheel powers, clamps output, and provides an explicit `stop()` operation.

`commands/TeleopDriveCommand` reads forward, strafe, and turn suppliers each scheduler cycle, applies deadband and configured scaling, calls a pure mecanum mixer, and sends the normalized result to `DriveSubsystem`. Its end path stops the drivetrain.

### OpModes

`opmodes/MainTeleOp` is the initial runnable example. It constructs the hardware, drivetrain subsystem, scheduler, and default drive command. Its lifecycle must:

1. initialize hardware without moving actuators;
2. expose configuration status through telemetry;
3. run the scheduler only after start;
4. cancel commands and stop all registered subsystems when the OpMode ends or is interrupted.

The example uses centralized placeholder hardware-map names. Compilation proves only software compatibility; the OpMode remains unverified on a real robot until names, directions, controls, and emergency-stop behavior are checked by the team.

## Data flow

During TeleOp, gamepad suppliers feed `TeleopDriveCommand`. The command transforms inputs through deadband and scaling, then calls the pure mecanum mixer. The mixer returns four normalized wheel powers. `DriveSubsystem` writes those powers to the SDK motors. The scheduler owns when commands start, execute, end, or yield a subsystem.

No subsystem reads gamepads directly, no OpMode writes motors directly, and no command performs hardware lookup.

## Error and safety behavior

- Missing or incorrectly typed required hardware fails during initialization with a message that identifies the configuration name.
- Initialization never applies nonzero actuator power.
- Mixer output is normalized and drivetrain output is clamped.
- A command collision interrupts the existing interruptible owner before the replacement starts; invalid duplicate ownership is not silently allowed.
- Scheduler shutdown cancels commands, resets internal state, and stops registered subsystems.
- Exceptions must not be swallowed to keep a robot moving. Cleanup runs before an initialization or loop failure is surfaced.

## Verification

Pure JVM unit tests cover:

- mecanum forward, strafe, turn, mixed-input normalization, zero input, and output bounds;
- command initialize/execute/end lifecycle;
- requirement conflicts and interruption;
- default-command scheduling;
- cancel-all and scheduler reset behavior.

Project verification includes:

- FTCKB installation verification and active-rule resolution before coding;
- Java/unit-test execution using an isolated Gradle home when needed;
- `:TeamCode:assembleDebug` against FTC SDK 12.0;
- FTCKB `check` and `verify`, with hard failures distinguished from known false positives and soft guidance;
- `git diff --check` and a final scope review.

Android compilation and unit tests do not establish Android Studio Sync success on every teammate's machine, deployment success, or physical robot correctness. Hardware validation remains a separate team-owned step.

## Initial scope exclusions

The first version does not include autonomous path following, localization, vision, mechanism-specific subsystems, PID tuning, logging infrastructure, CI branch protection, Control Hub deployment, or a full clone of FTCLib. These may be added after the corresponding robot design and validation requirements are known.
