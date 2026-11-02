# FTC-20827 BioBuzz TeamCode

Team-owned FTC code for BIOBUZZ. The command architecture uses SolversLib's official command system (`org.solverslib:core:0.3.4`), following its subsystem, command, and `CommandOpMode` lifecycle model.

## Source layout

- `teleops/`: Driver Station TeleOp entry points. `FieldCentricDrive` extends SolversLib's `CommandOpMode` directly and installs `DriveCommand` as the drivetrain default.
- `autos/`: autonomous OpModes and autonomous command compositions.
- `subsystems/`: hardware ownership. `Drive` owns the four drive motors; `Shooter` and `Intake` each own one motor; `OctoQuadSubsystem` provides a validated heading relative to the direction captured at START.
- `commands/`: `DriveCommand` reads live driver axes. `BaseDriveTeleOp.run()` handles shooter and intake buttons directly through `GamepadConfig`.
- `tests/`: Driver Station diagnostic OpModes.

Subsystems encapsulate hardware. Commands declare their subsystem requirements with `addRequirements(...)`, implement the command lifecycle, and use subsystem methods instead of writing hardware directly. SolversLib's `CommandOpMode` runs its scheduler and handles scheduler reset when the OpMode ends.

`FieldCentricDrive` is registered as **20827 Headless TeleOp** in the Driver Station. The left stick translates relative to the direction the robot faced at START, and right stick X turns the robot. Hold gamepad 1 LB to cap each normalized wheel output at 0.30; release LB for normal power. No gamepad button changes the fixed forward reference.

For example, after the robot turns counterclockwise 90 degrees, stick-left commands robot-forward movement, which is still the driver's original left. Stick-forward-left commands robot-forward-right, which is still the driver's original forward-left. These directions remain fixed for the whole run; the program does not track the driver's later movement or turning.

Keep the robot still during INIT calibration and wait for `RUNNING`, then point its front along the driver's desired forward direction before START. SolversLib **0.3.4 does not call a `preRun()` hook**. This TeleOp captures one fresh valid OctoQuad heading at the beginning of its first active `run()`, before `super.run()` advances the scheduler. Invalid data at that point aborts startup instead of choosing a later direction. OctoQuad MK2 is required, configured as `octoquad`; CRC/status/nonfinite heading failures during driving command zero output.

On STOP or a runtime exception, the OpMode explicitly stops the drivetrain, shooter and intake, and lets SolversLib reset its scheduler. Every available motor receives a stop attempt even when cancellation or another stop write fails. Each mechanism command also stops on interruption or input/output failure. Existing OctoQuad pod settings and heading scale are preserved.

`Drive`, `Shooter` and `Intake` fetch their hardware directly from `HardwareMap` in their constructors. The drive motor names are written at the lookup calls, and the class preserves the current left-reverse/right-forward direction setup. Verify the names and mapping on the robot before operating it.

## Shooter and intake controls

In **20827 Headless TeleOp**, use gamepad 1:

- Hold **A** to run the intake; release A to stop it.
- Hold **B** to run the shooter; release B to stop it.
- Hold A and B together to run both. The left/right sticks and LB retain their existing drive behavior.

Configure a motor named `shooter` and a motor named `intake`. `Shooter.SHOOT_POWER` and `Intake.INTAKE_POWER` are initial open-loop commands of `1.0`, not tuned or measured performance. Change these constants to adjust power. Each subsystem's `DIRECTION` constant starts at FORWARD; verify the actual collection/firing direction on the current robot. Both use BRAKE and RUN_WITHOUT_ENCODER and initialize with zero power. This version controls the two motors directly; it does not regulate shooter RPM or actuate a separate feeder.

The subsystem APIs are `shooter.shoot()`, `intake.intake()`, `setPower(double)`, `stop()` and `getCommandedPower()`. Power is clamped to [-1,1]; nonfinite inputs throw after attempting to stop. `getCommandedPower()` and the TeleOp telemetry report the last successfully written command, not motor speed or a confirmation of ball movement.

The drive default command is installed during INIT and executes after START. `BaseDriveTeleOp.run()` reads the shared `GamepadConfig` each cycle: left bumper runs the intake and right bumper runs the shooter. The two buttons work independently and together; releasing either button stops its motor. `end()` stops both mechanisms on STOP.

The current drive names are `frontLeft`, `frontRight`, `backLeft` and `backRight`. Raise the wheels and verify the individual wheel directions, a 90/180-degree turn followed by left/forward-left joystick input, LB slow/release and STOP before floor driving. Software tests and compilation do not prove deployment, sensor calibration or physical robot behavior.

The project keeps SolversLib `0.3.4` and FTC SDK `12.0.0`. `TeamCode/build.gradle` excludes only SolversLib's transitive `org.firstinspires.ftc` SDK artifacts because its strict 11.1 constraints conflict with the application's explicit SDK 12.0 dependencies. No library version is upgraded by the headless restoration.

## Reference

- [SolversLib command system](https://docs.seattlesolvers.com/command-base/command-system)
- [SolversLib CommandOpMode](https://docs.seattlesolvers.com/command-base/command-system/robot-and-commandopmode)
- [SolversLib installation](https://docs.seattlesolvers.com/installation)
