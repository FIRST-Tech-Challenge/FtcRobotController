# Option B: shoot from the end wall, then travel toward park

Select **Pollen ROBOT Shoot + Park**. Option A remains **Pollen ROBOT Park**.
These modes have different starting poses. Neither detects a parking zone.

## What happens after START

1. Wait for the selected start delay, with every output zero.
2. Stay still and command the selected flywheel velocity. Wait until measured
   speed is at least 96% of the target continuously for 0.25 seconds.
3. Run the flywheel, main intake and `agitator` feeder together for the selected
   duration: **10 seconds by default, starting after speed qualification**.
   Both feed motors default to 0.50 power, matching the current TeleOp's neutral
   trigger / right-bumper feed command. Change their shared power in INIT if needed.
4. Stop all three shooting mechanisms. Translate diagonally forward-left without
   turning the chassis, for the selected estimated distance. Default: **60.96 cm
   (2 feet)** at vector power 0.20, estimated **2.363 seconds**.
5. Hold still, then remain in DONE with zero commands until STOP.

The flywheel runs for spinup plus the feed duration. Normal speed dips after
feeding starts do **not** pause either feed motor; this implements the requested
continuous run. There is no ball counter, jam detection, hive-tip detection or
shot-success detection. A nonfinite velocity reading stops the sequence.

If initial speed never qualifies within 4 seconds, skip feeding and continue
this mode's diagonal departure with a warning. If shooter mapping/configuration
throws during INIT, attempt to stop each mapped mechanism. If that succeeds,
telemetry says **READY FOR DEPARTURE ONLY - SHOOTER FAILED**. The fallback uses
this mode's selected departure, never Option A's forward-only route. A drive
initialization failure, unsuccessful shooter stop, or runtime hardware exception
stops the whole mode. Some disconnected-hub failures occur before the OpMode or
do not throw; this cannot guarantee recovery from every electrical failure.

## Operator instructions

Use the [operating manual](POLLEN_AUTONOMOUS.md) for the current controller
settings table, pre-run checks, warning meanings and ordered robot tests.
Use the [code README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md)
for architecture, hardware details, build and deployment. This document preserves
the geometry assumptions and calibration reasoning behind Option B.

## Starting pose, alliance and the provisional geometry

The team confirmed: **start at the end wall facing the open end of the hive,
and shoot toward the robot's calibrated forward direction.** Keep the chassis
square to that wall, centered on the hive axis; angling it to aim changes the
departure heading too.

Use the orientation in BIOBUZZ manual figure 9-2 (red driver area on the left):

| Alliance | Starting wall and heading | Field departure | Robot-relative departure |
| --- | --- | --- | --- |
| Red | Bottom end wall, facing up toward red hive | Up and left | Forward-left |
| Blue | Top end wall, facing down toward blue hive | Down and right | Forward-left |

The blue pose is the red pose rotated 180 degrees. Both starts use the same
robot-relative forward-left command, so **there is no alliance selector**.
Square and Circle do nothing. The program trusts the humans' starting orientation;
there is no field-coordinate transform or IMU requirement. Shooting settings and
travel distance work the same on both alliances.

**Geometry estimate, not a validated field route:** assume an 18-by-18-inch
robot, a 144-inch field and hive center lines 25.5 inches apart. Coordinates in
inches use x to the right, y downward in figure 9-2:

- Red start center: `(59.25, 135)`; proposed finish center: `(11, 35.5)`.
- Blue start center: `(84.75, 9)`; proposed finish center: `(133, 108.5)`.
- Delta magnitudes: 48.25 inches sideways and 99.5 inches forward.
- Straight-line length: `hypot(48.25, 99.5) * 2.54 = 280.88 cm`, about 9.22 ft.
- Angle: `atan2(48.25, 99.5) = 25.87 degrees` left of forward.

The code fixes this angle and comments the nominal full-field distance. At 0.20
vector power, the formula estimates 10.89 seconds for 280.88 cm. Default worst
case would be approximately 25.39 seconds including spinup, feed and hold, with
no start delay. The nominal destination leaves about 2 inches between an
18-inch-wide robot and the side wall while overlapping its loading zone.

**Before attempting that distance, measure the actual robot footprint, hive
alignment and tape, and check the swept footprint past the hive frame/feet.**
This direct line passes near that structure; a center-line drawing is not enough
to prove clearance. If it clips the frame, use a waypoint route rather than
increasing the distance. The current patch does not implement obstacle avoidance.
Coordinate the entire route and timing with the partner; delay cannot guarantee
clearance from a partner that remains on the route or occupies the destination.

Source: [official BIOBUZZ manual](https://ftc-resources.firstinspires.org/ftc/game/manual),
TU05, figures 9-2 and 9-10, sections 9.2–9.6 and 10.5.4. These are approximate
field dimensions; our robot size, chosen finish center and resulting route are
planning assumptions, not prescribed FIRST coordinates.

## What the distance formula does and does not tell us

Team 31452 measured `distance_cm = seconds * power * 150` at **12.86 V** for
forward travel. A later diagonal run with **100 cm selected** produced **86 cm
of total travel, including 76 cm forward**. That gives a provisional diagonal
factor of `150 * (86 / 100) = 129`, so Option B now uses:

`seconds = distance_cm / (129 * power)`

This increases travel time by `100 / 86`, approximately 16.3%, for the same
selected distance and power. At 0.20 power, 100 cm now requests approximately
3.876 seconds, and 60.96 cm requests 2.363 seconds. This is a single-run fit;
repeat before extrapolating to the full field. The voltage of this diagonal
run was not reported. Option A retains its earlier forward calibration.

If 86 cm was straight-line displacement, the inferred lateral displacement
is 40.25 cm and the observed angle is about 27.9 degrees left of forward.
The commanded 25.87-degree angle is unchanged pending repeat measurements.

For diagonal motion, power means the magnitude of the forward/strafe command
vector, not the largest individual motor power. At default settings the mixer
requests approximately 0.093, 0.267, 0.267, 0.093 for FL, FR, BL, BR.

Strafe efficiency, wheel slip, battery voltage and acceleration can change both
distance and angle. The code neither measures distance nor compensates voltage.
Record voltage after shooting too: ten seconds of launcher/intake load may
change departure behavior. A short diagonal test is required before extrapolation.
Changing distance cannot correct an incorrect travel angle.

## Validation status

The team reported that the calibrated movement worked. That does not establish
full-field clearance or confirm the final cleanup installation's robot tests.
Record those results against the installed commit in the
[operating manual](POLLEN_AUTONOMOUS.md#team-settings-record).
The nominal full-field distance above remains a planning estimate until measured
on the actual field with the complete robot footprint and partner route.
