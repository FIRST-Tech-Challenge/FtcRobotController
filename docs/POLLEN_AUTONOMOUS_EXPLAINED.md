# Pollen autonomous: how it works and how it compares

This document explains the pollen autonomous code for students and mentors.
The current Option B is **Pollen ROBOT Shoot + Park**: start already aimed at
the hive, shoot while stationary, then translate at the angle selected during INIT. Its current
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
The team has reported successful robot testing of the movement calibration,
cleanup build and controller-adjustable angle. This confirms the tested setups;
it does not establish every selectable angle or a full-field parking route.
Short code defaults remain distinct from verified match settings. A step labeled
“PARK” does not establish that the robot is parked.

The shooting routine uses the current PollenBot motor mapping reflected in
`BioBuzzStarterbotOpMode.java`: `pollenLauncher`, `intake` and `agitator`.
There is no windmill feeder or feeder servo in this implementation.

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

Positive drive inputs mean robot-forward, robot-right, and clockwise turn,
provided the wheel layout and direction configuration match the assumed layout.
The operator's **travel angle** uses a different convention: positive degrees
mean left of forward. `PollenShootTuning` converts a positive angle into a negative
right-strafe command. These are two representations of the same movement.

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

The default `Step.continuousShoot(10)` qualifies flywheel speed once (96% of the
chosen target for 0.25 seconds), then requests ten continuous seconds of intake,
agitator and flywheel operation. INIT can change that feed duration; zero skips
the shot. Spinup is additional, bounded to four seconds.
Normal speed dips during feeding do not pause it. It then zeros the mechanisms
and translates at the selected angle with zero commanded rotation. There is no
heading correction if the chassis drifts. It does not count balls or
confirm a scored shot. A spinup timeout skips feeding and continues departure;
invalid velocity data or a hardware exception stops the routine.

The earlier pause-and-recover shooting example has been removed. The sequencer
now supports timed drive steps and the continuous-feed shooting behavior used
by the current robot, with the same independent 27-second cutoff.

## Starting placements and INIT settings

**Option A** always commands forward movement. Humans choose and mark a starting
pose that points along the tested path. Either physical start can use this same
routine; verify each path and endpoint rather than assuming two placements are
equivalent. `PARK_FORWARD_POWER` and `PARK_FORWARD_SECONDS` supply defaults,
which the operator can change before START.

During INIT, Gamepad 1 D-pad up/down adjusts delay, right/left adjusts drive
time, and R1/L1 adjusts power. Tap and release; held buttons do not repeat.
START locks the settings. **Only delay resets each INIT**; power and time remain
in memory until the Robot Controller app restarts. Triangle restores all defaults.
See the [operating manual](POLLEN_AUTONOMOUS.md) for increments and bounds.

**Option B** has its own shooting start and menu. Up/down selects a row and
right/left edits it. The travel angle defaults to +25.87 degrees, with positive
left, zero forward and negative right. START locks it along with the other
settings. Selected values remain in process memory except for delay, which resets
each INIT; an app restart loses that memory.

The corresponding B starts on opposite alliances are related by a 180-degree
rotation of the whole starting pose. The same robot-relative commands work for
both; no alliance selector or automatic sign reversal is needed for those poses.
A different route or starting orientation requires its own validation.

Coordinate the partner's full path, timing, and finishing footprint. A delay
can separate a crossing in time; it cannot make an occupied starting or parking
position compatible. The program does not sense the partner or guarantee clearance.

## Why the distance setting depends on calibration

Option B converts the selected distance into a duration:

`drive_seconds = selected_distance_cm / (129 * selected_power)`

Angle changes the forward/sideways command mixture, but does not change this
duration. The coefficient 129 was fitted near the default angle. It is not a
calibration for every direction, power, surface or battery voltage. Likewise,
the selected angle describes the command, not a measured travel heading.

For example, 100 cm at power 0.20 requests about 3.876 seconds at every angle.
If the earlier forward-only coefficient of 150 at 12.86 V still applies, that
would predict about 116 cm at zero degrees. This is an illustration of the
model's limitation, not a new measured result.

For the current route, keep power fixed, measure forward and left displacement,
adjust angle for direction, then adjust distance for the endpoint. Repeat the
combined shooting/departure run under representative conditions. Entering
forward/left distances instead would describe the destination differently; it
would not by itself correct timed-driving errors. Separate forward/strafe
calibrations could improve an estimate, but would still need diagonal validation.
See the [calibration reference](POLLEN_SHOOT_FIRST.md) for the measured data.

## Shooter initialization fallback and runtime failures

If shooter initialization throws, the code tries to stop every mapped shooter
motor. When those commands succeed, current Option B skips its shooting step
and uses its own selected departure, displaying the reason. It never substitutes
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
- **Pollen ROBOT Shoot + Park:** stationary shooting followed by travel at the selected angle and estimated distance.

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

## What testing establishes

The source includes different wheel-direction conventions:

| Existing class | Motors set to REVERSE |
| --- | --- |
| `BioBuzzStarterbotOpMode` (current PollenBot) | frontLeft, backRight |
| `MecanumDrive` | frontLeft, backRight |
| `MecanumDriveTrain` | frontLeft, frontRight |

The Pollen configuration matches the current `BioBuzzStarterbotOpMode` and has
been exercised on the robot. Different conventions in older classes are not
evidence that those configurations were wrong for their original robots.
After wiring or direction changes, recheck the drivetrain before reusing a route.

The team reported successful tests, but this document does not contain the
measured endpoints, battery conditions or a full-field clearance record. Keep
those results with the installed commit in the operating manual's settings record.
Option B's departure starts from the shooting position, so Option A's successful
parking test does not validate it. Retest after changing angle, power, starting
pose or hardware. Computer tests verify logic; physical trials establish travel
and shooting results.

## Keep the next development step focused

The current routine starts at its shooting position. It has no approach leg in
which to pre-spin, and its paired starting poses do not need an alliance selector.
Keep the tested controls while measuring the actual route; add ramps or voltage
compensation only if measurements justify changing and recalibrating movement.

For longer routines, the useful upgrade is position feedback. First verify
reported position and heading while moving the robot manually. Then make one
short movement stop at a measured destination, with a timeout, before combining
travel with flower collection and repeated shooting.

Keep hardware control, mechanism actions and route sequencing separate. A future
movement action can use measured position to decide when it has arrived instead
of relying solely on elapsed time. Limelight observations, odometry and heading
information will need consistent coordinates and handling for unavailable or
stale measurements. None of that localization is implemented in these Pollen modes.

The repository's Pedro/Pinpoint material is a starting reference, not a ready
configuration for this robot. Verify the installed sensor hardware, pod offsets,
directions and units. Its Foresight values are explicitly marked as placeholders
and must be tuned before relying on path following.

## FTC reference

[FIRST's Java OpMode guide](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html)
explains `hardwareMap.get` and the requirement that device names match the
Robot Controller's configuration exactly.
