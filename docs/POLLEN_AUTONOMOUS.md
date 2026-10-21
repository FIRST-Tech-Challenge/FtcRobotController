# Pollen autonomous operating manual

**For drivers and students.** Covers Park build `pollen-prep-v3.3-clean` and
Shoot + Park build `pollen-prep-v3.4-angle`.
Keep this manual in the same branch as the code installed on the robot.
For installation or code changes, use the [programmer README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md).

## Choose the routine

| Choice on the Driver Hub | What happens after START | How to position the robot |
| --- | --- | --- |
| **Pollen ROBOT Park** — Option A | Wait, drive straight forward, stop | Aim the robot along the team's tested straight path to park. |
| **Pollen ROBOT Shoot + Park** — Option B | Wait, spin up, feed while stationary, stop the shooter, drive at the selected angle (default forward-left), stop | Start at the end wall, square to it and facing the hive, in the team's tested shooting position. |

The modes use **different starting poses and different controller controls**.
Option B commands translation with zero rotation; it does not correct heading
drift. Left always means the robot's
own left. For the corresponding opposite-alliance shooting start, rotate the
whole starting pose 180 degrees; the same selected angle gives the corresponding robot-relative departure.
There is no alliance or start-position selector.

Neither routine detects the parking zone, other robots, or obstacles. Humans
choose and verify the starting position, path and destination. Agree on the
whole route and delay with your partner; a delay alone does not ensure clearance.
Use the current event rules to confirm starting placement and ball loading.

## Before every run

1. Connect a charged robot battery, power on the robot and Driver Hub, and wait
   for the Driver Station to connect to the Control Hub. Use the existing robot
   configuration. The laptop is not needed to change the settings below.
2. Confirm Gamepad 1 is connected and assigned. Select the intended routine in
   the autonomous list. Enable the Driver Station autonomous timer.
3. Put the robot in the marked starting position for **that routine**. Clear
   the route and shooting area; have someone ready to press Driver Hub STOP.
4. Press **INIT**. Motors should remain stopped. Check the build identifier,
   status and every setting against your team's verified settings record.
5. Adjust settings using the tables below. **Tap and release** for each change;
   holding a button does not repeat. Check the displayed value after editing.
6. Read any warning before START. A shooter failure can leave Option B ready to
   **move without shooting**; that is not a successful shooting initialization.
7. At the appropriate start signal, press **START**. Settings are now locked.
   The programmed sequence runs without controller steering or further input.
8. Watch the phase and robot. **STOP** interrupts the routine. At DONE, motor
   commands stay zero. Press STOP before setting up another test.

Both routines preselect **PollenBot** for TeleOp. That does not start TeleOp;
select/check, INIT and START it at the appropriate time. For another autonomous
trial, reselect the intended autonomous routine before INIT.

## Option A controls: Pollen ROBOT Park

Use Gamepad 1 between INIT and START.

| Button | Action |
| --- | --- |
| D-pad UP / DOWN | Increase / decrease start delay by 0.5 seconds |
| D-pad RIGHT / LEFT | Increase / decrease forward drive time by 0.05 seconds |
| R1 / L1 | Increase / decrease drive power by 0.05 |
| Triangle | Restore all Option A code defaults |

| Setting | Code default | Adjustment limits |
| --- | ---: | --- |
| Drive time | 2.00 seconds | 0.05–5.00 seconds |
| Drive power | 0.20 | 0.05–0.50 |
| Start delay | 0 seconds | Up to 20 seconds, subject to the time budget |

It drives forward for the chosen time, holds zero commands for 0.5 seconds,
then enters DONE. Increasing drive time can reduce an excessive selected delay;
check both displayed values afterward. There is no distance entry in Option A.

For a first estimate only, the team's forward experiment at **12.86 V** gave:

`distance in cm = drive time in seconds × power × 150`

For example, 2 seconds at 0.20 estimates 60 cm. Measure the actual endpoint on
our robot; battery voltage, floor and load affect timed travel.

## Option B controls: Pollen ROBOT Shoot + Park

Use Gamepad 1 between INIT and START:

- **D-pad UP / DOWN:** select a row (marked `>` on the screen).
- **D-pad RIGHT / LEFT:** increase / decrease the selected value.
- **Triangle:** restore all Option B defaults, including the short two-foot move.

| Screen row | Code default | Change per tap | Limits / special meaning |
| --- | ---: | ---: | --- |
| Flywheel ticks/s | 1250 | 50 | 500–2500; encoder ticks per second, not RPM |
| Feed seconds (0 = skip shot) | 10 seconds | 0.5 seconds | 0–20; zero skips spinup and feeding |
| Travel cm (0 = shoot only) | 60.96 cm | 5 cm | 0–350; zero skips driving |
| Drive vector power | 0.20 | 0.05 | 0.10–0.50 |
| Travel angle deg (+left) | +25.87° | 1° after the first edit | −90° right to +90° left; 0° straight forward |
| Intake + agitator power | 0.50 | 0.05 | 0.10–1.00; both feed motors use this power |
| Start delay seconds | 0 seconds | 0.5 seconds | 0–20, subject to the time budget |

**60.96 cm is the short two-foot test default, not a full-field parking setting.**
Defaults are not a substitute for your team's verified match settings.

The flywheel first runs alone. After it reaches at least 96% of the selected
speed for 0.25 seconds, the main intake and agitator run with it for the chosen
feed duration. At the default, this is **10 seconds of feeding plus spinup**.
Ordinary speed dips during feeding do not pause it. There is no ball counter:
feed time means motor-running time, not proof that all balls were launched.
The outside ball must be within intake reach while the robot stays stationary.

The shooter then stops and the robot translates at the selected angle, with no
chassis turn. **Positive = left of forward; negative = right; zero = forward.**
The default remains **+25.87°**. From that default, the first RIGHT tap selects
+26° and the first LEFT tap selects +25°; subsequent taps change one whole degree.
Triangle restores +25.87° along with every other default. The angle is remembered
between runs and locked at START like the other settings.

Travel distance adjusts how
long it drives. It then holds zero commands for 0.5 seconds and enters DONE.
The [calibration reference](POLLEN_SHOOT_FIRST.md) explains the estimate and
why the full route needs a field clearance test. The distance formula stays the
same when you change angle, but actual travel can change. Measure each new angle
with feed duration zero and a short travel distance before combining it with shooting.

If an edit would use 26 seconds or more in the worst case, it is rejected and
the old value stays. Read the notice; reduce the requested duration, distance or
delay. Changing drive power also changes travel and requires a physical retest.

## Settings memory: check every INIT

Settings are remembered separately for A and B while the Robot Controller app
stays running. **Start delay resets each INIT** (zero in this build).
Rebooting, restarting the app or installing an APK loses remembered settings.
Triangle restores code defaults, not your last successful match setup.

Photograph the settings screen and keep a written record. To make tested values
survive an app restart, a programmer must update defaults, rebuild and install.

## What warnings mean

| Screen / behavior | Meaning and action |
| --- | --- |
| BLOCKED | Initialization failed. Do not START expecting a route; read the error and have a programmer check the configuration or hardware. |
| READY FOR DEPARTURE ONLY - SHOOTER FAILED | Option B could not initialize shooting but could stop the mapped shooter devices. START will run its selected departure without shooting. Before START, decide whether that result is acceptable from this pose. |
| SPINUP, then a timeout warning | Initial speed did not qualify within 4 seconds. Feeding is skipped automatically and Option B continues its selected departure. No driver intervention is required. |
| CONTINUOUS FEED - speed below target band | Feeding continues as requested. Record the result; change settings before a later run if needed. |
| Hardware error / mode stops | The code attempts to stop each device and ends the mode. Diagnose before retrying. |
| DONE | The programmed steps finished with zero motor commands. This does not confirm physical parking or successful shots. |

A 27-second software cutoff also stops commands. STOP and software cutoffs cannot
prevent physical coasting or guarantee recovery from an electrical failure.

## Robot verification after changes

The team reported successful testing of the cleanup and adjustable-angle builds.
Use this checklist after further changes or to validate a new route; it is not a
record that every field configuration has been tested. Start with short trials.
Record the installed commit and photograph the settings.

1. **INIT check:** verify both menus, build identifier, settings changes, delay
   reset and no movement before START.
2. **A:** use the known short forward settings on a clear floor. Check direction,
   distance, stopping and a repeat run. Test STOP during a short movement.
3. **B departure only:** set feed duration to zero and travel to 60.96 cm.
   Verify forward-left travel at the default angle without chassis turning.
   Try a nearby angle using the new menu row, then return to the default. Measure forward and
   sideways displacement and the endpoint; test STOP.
4. **B shooting only:** set travel to zero. Begin without balls and a short feed
   duration, then use a suitable catch area and the tested shooting settings.
   Confirm the flywheel, intake and agitator work, including the outside ball.
   Test STOP during shooting. Allow mechanisms to stop before handling them.
5. **B combined:** restore the chosen feed duration and short departure distance.
   Verify that feeding finishes before driving and repeat with the expected load.
6. **Field check:** mark starts and endpoints, check the full robot footprint
   along each path, then test full travel and partner timing for both alliance
   placements. Only record a route as field-verified after these checks.

Setting both B feed duration and distance to zero only waits/holds. Check these
values again after isolated tests. Zero settings do not bypass hardware checks.

## Team settings record

Fill this out from actual tests; blank means not yet recorded. Keep separate
records if different starting positions require different settings.

- Installed commit / build: __________
- Date, tester, battery voltage, floor and load: __________
- Starting position / orientation (attach a photo or marked diagram): __________
- Alliance placement and partner route: __________
- A: drive seconds _____; power _____; start delay _____
- B: flywheel ticks/s _____; feed seconds _____; intake + agitator power _____;
  travel cm _____; drive vector power _____; travel angle degrees _____; start delay _____
- Actual endpoint, shot results and STOP check: __________
- Full field route verified by / date: __________

Update this manual in the same commit whenever menu names, controls or behavior
change. Documentation-only changes do not require a robot rebuild or installation.
