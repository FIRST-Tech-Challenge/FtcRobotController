# Pollen autonomous: how it works and how it compares

This document explains the initial pollen autonomous code for students and mentors.
For installation and the bench/robot test procedure, see
[POLLEN_AUTONOMOUS.md](POLLEN_AUTONOMOUS.md).

The comparison is with the existing `Auto.java`, `MecanumOpMode.java`,
`MecanumDriveTrain.java`, and `Launcher.java` in this repository. It describes
the source code, not a claim that every old routine was used in competition.

## What the robot is supposed to do

There are two choices:

1. **Park:** move from a chosen start to the parking location, then stop.
2. **Shoot, then park:** move to a shooting position, stop and shoot, then move
   to the parking location and stop.

Both use timed driving, like the existing autonomous routines. Neither measures
the robot's driving position or detects that it has arrived at a parking zone.
The initial durations and directions are short examples requiring physical
calibration. A step labeled “PARK” does not establish that the robot is parked.

The optional shooting routine assumes the goBILDA StarterBot mechanism reflected
in `BioBuzzStarterbotOpMode.java`. Confirm that this matches the actual pollen bot.

## Who assigns the motor names?

**The team assigns names through Configure Robot on the Driver Hub. Java uses
those names to retrieve the configured devices.** Java does not rename ports.

Example: suppose the front-left wheel motor is plugged into Control Hub motor
port 0. That port number is only an example; use the robot's actual wiring.

| Location | Responsibility |
| --- | --- |
| Driver Hub's Configure Robot screen | Select the motor type and name that port `frontLeft`. |
| Configuration stored on the Robot Controller | Record the association between the hub, port, device type, and name. |
| Java initialization | Retrieve the motor with `hardwareMap.get(DcMotor.class, "frontLeft")`. |
| Java while running | Send commands such as `setPower(0.2)` to that motor. |

The name in Java must match the active configuration exactly, including case.
Plugging in a motor does not tell the system which wheel it drives.

The existing `MecanumDriveTrain.java` already contains:

```java
frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
frontRight = hardwareMap.get(DcMotor.class, "frontRight");
backLeft = hardwareMap.get(DcMotor.class, "backLeft");
backRight = hardwareMap.get(DcMotor.class, "backRight");
```

The new code does the same lookup in a loop, using `DRIVE_NAMES` from
`PollenConfig.java`. It uses these same four names. **If the robot already uses
them, there is nothing to rename.** The check is that the active configuration
and Java agree. Keep working team names wherever possible.

Motor direction is separate: `setDirection(REVERSE)` changes which electrical
direction counts as positive. A name such as `frontLeft` does not automatically
set the correct direction for that wheel's mounting and gearing.

## The familiar part: a sequence of actions

The existing `doAutoBlueWall()` routine in `Auto.java` reads:

```java
driveTrain.driveWithTime(1, 0, 0, 0.3, 200);
driveTrain.driveWithTime(0, 0, -1, 0.3, 450);
launcher.shootWithTime(3, 7000);
driveTrain.driveWithTime(0, 0, 1, 0.3, 450);
driveTrain.driveWithTime(0, -1, 0, 1, 750);
```

That requests forward movement, a turn, shooting, a turn back, and a strafe.
The `3` selects the existing launcher's **far-shot preset**; it is not a shot count.
The `7000` gives the shooting method a seven-second overall window, including
time spent waiting for launcher speed.

The new `parkRoute()` and `shootRoute()` also describe actions in order. They
create a list of steps instead of immediately calling each hardware method.
The sequencer executes that list after START.

## Translating an old driving command

Existing argument order:

```java
driveWithTime(forward, strafe, turn, powerMultiplier, milliseconds)
```

New argument order:

```java
PollenSequence.Step.drive(label, seconds, forwardPower, strafePower, turnPower)
```

For example, these request the same forward command and duration, assuming the
same motor mapping and directions:

```java
// Existing: 30% forward for 200 milliseconds.
driveTrain.driveWithTime(1, 0, 0, 0.3, 200);

// New: 30% forward for 0.2 seconds.
PollenSequence.Step.drive("Forward", 0.2, 0.3, 0, 0);
```

To translate a movement, divide milliseconds by 1000, and multiply each of the
old forward/strafe/turn inputs by its power multiplier. The new mixer also scales
combined wheel commands proportionally if any would exceed magnitude 1. Thus
high-power diagonal or combined movements need particular care when comparing
the versions; their limiting behavior differs.

Positive inputs mean robot-forward, robot-right, and clockwise turn, provided
the physical wheel layout and direction configuration match the assumed layout.

## What changed, and why?

| Area | Existing code | New pollen code | Reason |
| --- | --- | --- | --- |
| Basic driving | Timed movement | Timed movement | Keep the first routine simple. |
| Execution | `LinearOpMode`; a method waits for its action to finish before returning | Iterative `OpMode`; each loop checks the current action and time | Run the same decision logic on a computer or a hub. |
| Selection | `MecanumOpMode` is `@TeleOp`; the PS/Home button cycles through modes during INIT | Separate `@Autonomous` entries for park, shoot, and bench tests | Make each choice visible directly in the autonomous list; no gamepad needed for selection. |
| Hardware initialization | The OpMode initializes drivetrain, launcher, and camera | Park needs drivetrain only; bench modes look up no hardware | Test parts independently, including without motors. |
| Shooting mechanism | `flywheel`, angle `servo`, and `coreHex` feeder | StarterBot `launcher`, `windmillServo`, and intake devices | Match the StarterBot mechanism rather than the older launcher. |
| Shooting timing | One overall timed shooting window | Separate bounded spinup and feeding windows | Make readiness and timeout behavior explicit. |
| Route editing | Change method calls in `Auto.java` | Change step lists in `PollenConfig.java` | Keep the route and tuning values together. |

The existing timed driving and shooting methods already check
`opModeIsActive()`. The old launcher already gates feeding on measured speed.
These were useful existing behaviors, not features invented for the new version.
Both execution styles are legitimate FTC approaches. A small move-and-park
routine could have been written in the existing sequential style with less code.

The additional structure primarily supports hardware-free tests and separates
the route decisions from the actual motor commands. It is a design choice,
not a requirement imposed by FTC.

## What happens in the new shooting step?

1. The drivetrain stops for shooting.
2. The launcher is commanded to the configured target velocity.
3. Speed must remain at or above the configured minimum for 0.25 seconds before
   feeding begins. The supplied target/minimum are 1250/1200 encoder ticks per
   second, taken from the starter example and still needing robot validation.
4. If speed does not qualify within four seconds, the launcher is stopped,
   a warning is shown, and the routine continues to its parking leg.
5. Once feeding starts, a two-second feeding window runs. Feeding pauses whenever
   speed is too low; that pause does not extend the window.
6. Launcher and feeder commands return to zero before movement to parking.

`Step.shoot(2.0)` means a two-second feeding window after successful spinup,
not two projectiles and not two seconds for the whole shooting operation.
There is no projectile counter, automatic aiming, or confirmation of a scored shot.
The route also has a 27-second command cutoff; it stops output, but cannot
guarantee parking after a delay or failure. This is a local guard, not a statement
of game rules.

## What are the BENCH modes?

| Driver Hub choice | What it exercises |
| --- | --- |
| `Pollen BENCH Park` | Movement/hold/completion sequence with displayed commands only. |
| `Pollen BENCH Shoot` | Shooting sequence with simulated launcher speed. |
| `Pollen BENCH Timeout` | Launcher stays at simulated zero speed; verify no feeding and continuation to parking. |
| `Pollen ROBOT Park` | Actual drivetrain commands, once enabled. |
| `Pollen ROBOT Shoot` | Actual drivetrain and StarterBot mechanism commands, once enabled. |

Bench and robot modes use the same `PollenSequence` logic. Bench modes do not
call `hardwareMap.get` or send motor/servo commands. Their numbers on screen are
requests from the simulated sequence, not measurements of physical movement.
Running BENCH on the hub still requires a powered Control Hub and a valid hub
configuration. The separate Java test script runs on a computer without either hub.

## Which files should students read or edit?

All new Java files are under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/`.

| File | Role | Usually edited when |
| --- | --- | --- |
| `PollenConfig.java` | Names, directions, enabling flags, launcher settings, and both route lists | Calibrating the robot and route. Start here. |
| `PollenSequence.java` | Decide which action is active and what commands it requests | Changing sequence behavior. |
| `PollenAutoBase.java` | Connect the sequencer to FTC hardware or simulated telemetry | Adapting the mechanism or hardware interface. |
| `PollenBench*.java` and `PollenRobot*.java` | Small classes that register Driver Hub choices | Adding or changing selectable modes. |

For routine route tuning, students should normally only need `PollenConfig.java`.
Keep the existing tested TeleOp available while checking the autonomous settings.

## What still needs the real robot?

The source includes different wheel-direction conventions:

| Existing class | Motors set to REVERSE |
| --- | --- |
| `BioBuzzStarterbotOpMode` | frontLeft, backLeft |
| `MecanumDrive` | frontLeft, backRight |
| `MecanumDriveTrain` | frontLeft, frontRight |

The new configuration provisionally follows `BioBuzzStarterbotOpMode`. This is
not evidence that the other configurations were wrong for their original robots.
Use the TeleOp that actually drives this pollen bot correctly as the reference,
then verify names, wiring, and wheel directions with the wheels lifted.

Before treating either route as ready, establish the chosen starting position
and heading, measure driving behavior, verify the real parking destination,
and repeat the parking test. Only then validate launcher/feeder operation and
the optional shooting route. Its parking leg starts from the shooting position,
so it needs its own calibration.

The supplied software has computer tests and can be built before practice;
those checks do not establish that the robot drives, parks, or scores correctly.

## FTC reference

[FIRST's Java OpMode guide](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html)
explains `hardwareMap.get` and the requirement that device names match the
Robot Controller's configuration exactly.
