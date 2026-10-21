# Pollen autonomous: how it works and how it compares

This document explains the pollen autonomous code for students and mentors.
The current Option B is **Pollen ROBOT Shoot + Park**: start already aimed at
the hive, shoot while stationary, then translate diagonally. Its current
controls are in the [operating manual](POLLEN_AUTONOMOUS.md), and field assumptions
are in [POLLEN_SHOOT_FIRST.md](POLLEN_SHOOT_FIRST.md). For installation and laptop
tests, see the [code README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md).

The comparison is with the existing `Auto.java`, `MecanumOpMode.java`,
`MecanumDriveTrain.java`, and `Launcher.java` in this repository. It describes
the source code, not a claim that every old routine was used in competition.

## What the robot is supposed to do

There are two choices:

1. **Park:** move from a chosen start to the parking location, then stop.
2. **Shoot, then park:** start at the end wall already aimed at the hive, spin
   up and feed continuously, then translate toward the parking location and stop.

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

Option A's `parkRoute()` and Option B's `PollenShootTuning.sequence()` also
describe actions in order. They
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
| Selection | `MecanumOpMode` is `@TeleOp`; the PS/Home button cycles through modes during INIT | Two `@Autonomous` entries: park and shoot + park | Make each choice visible directly in the autonomous list; no gamepad needed for selection. |
| Hardware initialization | The OpMode initializes drivetrain, launcher, and camera | Park needs drivetrain only; shooting INIT can fall back to its own departure | Keep the basic park mode independent of shooting hardware. |
| Shooting mechanism | `flywheel`, angle `servo`, and `coreHex` feeder | Current team `pollenLauncher`, `intake`, and `agitator` motors | Match the StarterBot mechanism rather than the older launcher. |
| Shooting timing | One overall timed shooting window | Bounded spinup followed by a continuous, adjustable feed window | Make readiness and timeout behavior explicit. |
| Route editing | Change method calls in `Auto.java` | INIT controller tuning; geometry/defaults in `PollenShootTuning.java` for B | Keep the route and tuning values together. |

The existing timed driving and shooting methods already check
`opModeIsActive()`. The old launcher already gates feeding on measured speed.
These were useful existing behaviors, not features invented for the new version.
Both execution styles are legitimate FTC approaches. A small move-and-park
routine could have been written in the existing sequential style with less code.

The additional structure primarily supports hardware-free tests and separates
the route decisions from the actual motor commands. It is a design choice,
not a requirement imposed by FTC.

## What happens in the current shooting step?

The new `Step.continuousShoot(10)` qualifies flywheel speed once (96% of the
chosen target for 0.25 seconds), then requests ten continuous seconds of intake,
agitator and flywheel operation. Spinup is additional, bounded to four seconds.
Normal speed dips during feeding do not pause it. It then zeros the mechanisms
and translates diagonally with no chassis turn. It does not count balls or
confirm a scored shot. A spinup timeout skips feeding and continues departure;
invalid velocity data or a hardware exception stops the routine.

The earlier pause-and-recover shooting example has been removed. The sequencer
now supports timed drive steps and the continuous-feed shooting behavior used
by the current robot, with the same independent 27-second cutoff.

## One parking routine, two physical starting placements

The robot does not need to know which end of the loading zone it approaches.
Humans place and angle it at one of two measured starts. Both use the same
`PARK_FORWARD_POWER` and `PARK_FORWARD_SECONDS`, followed by a zero-output hold.
These are timed commands, not measured-distance control.

During INIT, Gamepad 1 D-pad up/down changes only the starting delay, in
half-second increments. `PollenParkTuning` ignores held-button repeats, bounds
the delay by the route budget (at most 20 seconds), and locks on START. A fresh
INIT returns to the configured default. Neither heading nor motor signs change.
Option B has its own INIT menu, starting pose, delay and diagonal departure.

Coordinate the partner's full path, timing, and finishing footprint. A delay
can separate a crossing in time; it cannot make an occupied starting or parking
position compatible. The program does not sense the partner or guarantee clearance.

## Why fallback only during INIT?

If shooter initialization throws, the code tries to stop every mapped shooter
motor. When those commands succeed, current Option B skips its shooting step
and uses its own diagonal departure, displaying the reason. It never substitutes
Option A, which has a different starting pose. Drive initialization failure or
a failed shooter stop blocks the run.

After movement starts, a hardware exception stops the routine. Switching to the
original park-only route at that point would assume the robot was still at its
starting position. A normal spinup timeout instead continues the
shooting route's own parking leg from the shooting position.

Some hub failures do not throw an exception, or prevent the SDK from starting
an OpMode. The fallback is not a substitute for checking hub connectivity.

## Which modes are on the Driver Hub?

- **Pollen ROBOT Park:** forward-only Option A, with INIT controls for time, power and delay.
- **Pollen ROBOT Shoot + Park:** stationary shooting and calibrated diagonal departure.

Both command real hardware after START and preselect `PollenBot` for TeleOp.
Earlier BENCH/simulation modes were removed after physical robot testing. The
laptop test script still exercises the logic without either hub or motors.

## Which files should students read or edit?

All new Java files are under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/`.

| File | Role | Usually edited when |
| --- | --- | --- |
| `PollenRobotPark.java` | Option A hardware and lifecycle | Changing the park mode's hardware behavior. |
| `PollenRobotShoot.java` | Option B hardware and lifecycle | Changing the shooting mode's hardware behavior. |
| `PollenConfig.java` | Shared drive mapping and Option A defaults | Saving permanent Option A defaults. |
| `PollenParkTuning.java` | Option A INIT controls and memory | Changing its controller settings. |
| `PollenShootTuning.java` | Option B INIT controls, calibration and route | Saving permanent B defaults or calibrating travel. |
| `PollenSequence.java` | Shared timed drive / spinup / feed execution | Changing sequencing or stop behavior. |

For routine tuning, use the INIT controls; save validated defaults in the
corresponding configuration class when ready.
Keep the existing tested TeleOp available while checking the autonomous settings.

## What still needs the real robot?

The source includes different wheel-direction conventions:

| Existing class | Motors set to REVERSE |
| --- | --- |
| `BioBuzzStarterbotOpMode` (current PollenBot) | frontLeft, backRight |
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

## Improvements to evaluate after calibration

- **Pre-spin:** may save some approach/spinup time per shooting step, not per ball.
  Verify the launcher can run safely during approach before adding overlap.
- **Alliance mirroring:** negating strafe and turn assumes reflected starting
  poses/headings and equivalent paths. Test both field routes before registering
  red/blue choices; a color label alone does not establish a valid route.
- **Voltage compensation:** a raw `12 / voltage` multiplier is not a distance
  controller. It needs valid sensor readings, bounded powers, and tests across
  battery states. Keep the baseline unchanged until drift is measured.
- **Acceleration ramps:** can reduce slip at higher power but change travel for
  every timed leg. Add and recalibrate together if testing demonstrates a need.
- **Odometry:** reuse ideas from the existing Pinpoint/Pedro work, but verify
  hardware, pod offsets, direction conventions, and tuning for this robot.
  Constants for another robot do not establish localization on this chassis.

## FTC reference

[FIRST's Java OpMode guide](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html)
explains `hardwareMap.get` and the requirement that device names match the
Robot Controller's configuration exactly.
