# PVI-FTC Implementation Status

PVI-FTC Codex Sequential Repository Build — Master Instructions

PVI-FTC | Editable master guide

## Repository baseline
- Source repository: PVI-FTC fork of FtcRobotController
- Current sequential prompt: Prompt 7 complete
- Last completed prompt: Prompt 7: Implement the TeleOp InputManager.
- Last verified commit: 7d11d07 (Prompt 1 package-structure merge)
## Completed work
- Added repository instructions and architecture documentation.
- Added sequential student workflow documentation.
- Completed Prompt 1: confirmed the TeamCode Java source root is
  `TeamCode/src/main/java` and its root package is `org.firstinspires.ftc.teamcode`.
- Added documented package-level structure under that root: `core.fsm`, `core.robot`,
  `core.input`, `common.hardware`, `common.subsystems.drive`,
  `common.subsystems.intake`, `common.subsystems.vision`, `common.autonomous`,
  `robots.teamA`, `robots.teamB`, `robots.teamC`, `opmodes.teleop`,
  `opmodes.autonomous`, and `opmodes.testing`.
- Confirmed `TeamCode/project-docs` exists as the TeamCode documentation directory.
- Completed Prompt 2: added the `Subsystem` lifecycle contract and the `Robot` base class in
  `core.robot`.
- `Robot` owns registered subsystems in deterministic registration order. It initializes each
  subsystem once, updates each subsystem once per FTC loop, and stops each subsystem before
  calling its protected `onStop()` safety hook.
- Duplicate subsystem instances and registration after initialization are rejected with clear
  exceptions.
- Completed Prompt 3: added the reusable `State`, `Transition`, and `FSM` types in `core.fsm`.
- `FSM` is explicitly activated by `initialize()`, which enters its configured initial state once.
  Each `update()` evaluates transitions from the current state in registration order and permits
  only the first satisfied transition to fire. A firing transition exits the old state, enters the
  target state, then updates the target state in that same cycle.
- `Transition` compares source states by instance identity and uses `BooleanSupplier` for its
  condition. The FSM reports update-before-initialization with a clear exception.
- Completed Prompt 4: added `DriveHardware`, `IntakeHardware`, `VisionHardware`, and
  `RobotHardware` in `common.hardware`.
- `DriveHardware` requires the configured `frontLeft`, `frontRight`, `rearLeft`, and `rearRight`
  `DcMotorEx` motors. It configures left motors reverse, right motors forward, BRAKE zero-power
  behavior, and `RUN_WITHOUT_ENCODER` mode without resetting encoders. It clamps requested motor
  powers and retains their last commanded values for telemetry.
- `IntakeHardware` treats the configured `intake` `DcMotorEx` as optional. A missing intake leaves
  it unavailable and makes intake commands safe no-ops, without preventing drivetrain startup.
- `VisionHardware` provides only the safe optional lifecycle. Camera, AprilTag, OpenCV, and
  VisionPortal setup remain deferred, so it currently reports unavailable.
- `RobotHardware` owns all three wrappers, initializes them in drivetrain, intake, vision order,
  and provides `stopAll()`.
- Required versus optional policy: drive motors are required and fail clearly when absent; intake
  and vision are optional during early testing and must not disable the drivetrain.
- Completed Prompt 5: added `DriveSubsystem` and its `DisabledDriveState`,
  `ManualDriveState`, and `HeadingHoldState` in `common.subsystems.drive`.
- `DriveSubsystem` owns the drive FSM and depends on `DriveHardware` through its constructor. It
  stores requested forward, strafe, and rotate values; its public request methods select manual,
  disabled, or heading-hold mode without exposing FSM state manipulation.
- Mecanum calculations live only in `DriveSubsystem`. It applies the standard four-wheel equations,
  normalizes all four powers when needed, and sends the results through `DriveHardware`.
- Disabled drive continuously stops the motors. Heading hold is an explicit safe manual-drive
  fallback with no IMU target or correction; IMU heading correction remains deferred.
- Drive requests made before initialization are stored safely. The FSM initializes in disabled
  mode, and transitions are registered in deterministic order.
- Completed Prompt 6: added `TeamARobot` in `robots.teamA` as the Team A composition root.
- `TeamARobot` owns a `RobotHardware` composition and one registered `DriveSubsystem`. Its
  `initialize(HardwareMap)` method initializes hardware first and then invokes the inherited
  subsystem lifecycle once, preventing duplicate initialization on repeated calls.
- OpModes use Team A's public drive request methods rather than accessing mechanisms directly.
  Telemetry can read the drivetrain state, requested inputs, and last commanded motor powers
  without access to the drive FSM.
- During stop, the inherited Robot lifecycle stops the registered subsystem before Team A's
  `onStop()` calls `RobotHardware.stopAll()` once for robot-wide hardware safety.
- Completed Prompt 7: added the robot-agnostic `InputManager` in `core.input` for one FTC
  `Gamepad`.
- Construct `InputManager` with a non-null gamepad and call `update()` exactly once at the start
  of every TeleOp loop before reading its queries. The first update records initial button state,
  so already-held buttons produce no just-pressed or just-released event.
- The manager exposes held, just-pressed, and just-released queries for A, B, X, Y, and both
  bumpers, plus all requested stick and trigger values. It has no robot, hardware, subsystem,
  FSM, telemetry, or autonomous dependency.
## Current public APIs
- `org.firstinspires.ftc.teamcode.core.robot.Subsystem`
  - `initialize()`, `update()`, `stop()`, and `getName()`
- `org.firstinspires.ftc.teamcode.core.robot.Robot`
  - `protected final registerSubsystem(Subsystem)`
  - `public final initialize()`, `update()`, and `stop()`
  - `protected onStop()` for robot-level safety work after subsystem shutdown
- `org.firstinspires.ftc.teamcode.core.fsm.State`
  - `enter()`, `update()`, `exit()`, and `getName()`
- `org.firstinspires.ftc.teamcode.core.fsm.Transition`
  - `Transition(State, State, BooleanSupplier)`
  - `appliesTo(State)`, `isConditionSatisfied()`, `getSourceState()`, and `getTargetState()`
- `org.firstinspires.ftc.teamcode.core.fsm.FSM`
  - `FSM()`, `FSM(State)`, `setInitialState(State)`, `addTransition(Transition)`
  - `initialize()`, `update()`, `getCurrentState()`, and `getCurrentStateName()`
- `org.firstinspires.ftc.teamcode.common.hardware.DriveHardware`
  - `initialize(HardwareMap)`, `setMotorPowers(double, double, double, double)`, `stop()`,
    `setBrakeMode()`, `setFloatMode()`, and individual last-commanded-power getters
- `org.firstinspires.ftc.teamcode.common.hardware.IntakeHardware`
  - `initialize(HardwareMap)`, `forward(double)`, `reverse(double)`, `stop()`, and `isAvailable()`
- `org.firstinspires.ftc.teamcode.common.hardware.VisionHardware`
  - `initialize()`, `update()`, `stop()`, and `isAvailable()`
- `org.firstinspires.ftc.teamcode.common.hardware.RobotHardware`
  - `initialize(HardwareMap)`, hardware-wrapper getters, and `stopAll()`
- `org.firstinspires.ftc.teamcode.common.subsystems.drive.DriveSubsystem`
  - `DriveSubsystem(DriveHardware)`
  - `initialize()`, `update()`, `stop()`, and `getName()`
  - `drive(double, double, double)`, `enableManualDrive()`, `disableDrive()`, and
    `enableHeadingHold()`
  - `getCurrentStateName()`, `getRequestedForward()`, `getRequestedStrafe()`, and
    `getRequestedRotate()`
- `org.firstinspires.ftc.teamcode.common.subsystems.drive.DisabledDriveState`,
  `ManualDriveState`, and `HeadingHoldState`
  - public `State` implementations with `DriveSubsystem` constructors
- `org.firstinspires.ftc.teamcode.robots.teamA.TeamARobot`
  - `TeamARobot()`, `TeamARobot(RobotHardware)`, and `initialize(HardwareMap)`
  - `drive(double, double, double)`, `enableManualDrive()`, `disableDrive()`, and
    `enableHeadingHold()`
  - drivetrain state, requested-input, and last-commanded-power telemetry getters
- `org.firstinspires.ftc.teamcode.core.input.InputManager`
  - `InputManager(Gamepad)` and `update()` once per TeleOp loop
  - held, just-pressed, and just-released button queries for A, B, X, Y, and both bumpers
  - left/right stick-axis and trigger getters
## Build status
- Approved JDK: Record the team-approved version here.
- Android Studio version: Record the team-approved version here.
- FTC SDK version or tag: Record here.
- TeamCode build command (Windows): `.\gradlew.bat TeamCode:assembleDebug`
- Last result: Not independently verified by Codex because its shell session has no configured JDK
  (`JAVA_HOME` and `java` are unavailable). The student confirmed the baseline build works; run
  the documented command in a configured local environment to verify Prompt 7.
## Known limitations and TODO items
- Configure branch protection and pull-request review.
- Consider adding compile-only GitHub Actions validation.
- Vision hardware integration is intentionally deferred until a future prompt defines camera and
  processor requirements.
- IMU heading correction remains intentionally deferred. `HeadingHoldState` currently provides a
  safe manual-drive fallback.
## Next planned task
Prompt 8: To be assigned.
## Update instructions
After every completed prompt, replace or extend the sections above with:
- prompt number and title;
- completed classes and behavior;
- actual package names and public APIs;
- important implementation decisions;
- build command and result;
- known limitations;
- next prompt.
