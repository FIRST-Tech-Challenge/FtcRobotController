# Pollen autonomous: Sunday bench test and Monday robot test

Prepared October 18, 2026 against team `main` at `49c36cb` (October 17,
"add starter bot example code (#14)"). Branch: `pollen-auto-prep`.

This adds two routines: move to park and stop; or move to a shooting position,
shoot for a bounded window, then move to park and stop. Each has a hardware-free
bench version using the same sequencer. A third bench mode simulates a launcher
that never reaches speed. No gamepad is needed to select or run these modes.

**The supplied routes are short examples, not field coordinates or a promise of
scoring. Monday's robot and field testing must establish direction, travel time,
shooting position, launcher speed, and the legal start/parking areas.** There is
no camera aiming, position sensing, or projectile counting in this first version.

## What tonight can establish

| Test | Two hubs and laptop | Needs the robot/field |
| --- | --- | --- |
| Clone, compile, install Java app | Yes | No |
| Driver Hub pairing and autonomous selection | Yes | No |
| INIT waits, START runs, STOP interrupts | Yes | No |
| Move/shoot/park ordering and launcher timeout logic | Yes, simulated outputs | No |
| Hardware names and wheel directions | Inspect configuration only | Physical verification |
| Travel distance, heading, traction, parking | No | Yes |
| Launcher velocity feedback, feeding, actual scoring | No | Yes |

## 1. Equipment and power

- Laptop with Android Studio and a USB **data** cable that fits the Control Hub's USB-C port.
- Control Hub plus a charged robot 12 V battery, XT30 connection, and power switch.
  Use normal 12 V hub power for this test; the laptop USB cable is not the robot's power supply.
- Charged REV Driver Hub. The hubs communicate over the Control Hub's Wi-Fi.
- Motors, servos, gamepad, camera, and Expansion Hub are not needed for BENCH modes.

If you literally have only the two hubs and no robot battery, collect the battery
before attempting the normal hub run. You can still clone and build on the laptop.

## 2. Get the project onto the laptop

Use the team's project, not a new Android/Kotlin application. The FTC project is
already Java. Make a separate checkout if your existing checkout has uncommitted
work; do not reset or overwrite that work.

Once the branch has been pushed, run in Terminal:

```bash
git clone https://github.com/FtcBagelBits/FtcRobotController.git FtcRobotController-pollen
cd FtcRobotController-pollen
git switch pollen-auto-prep
```

If using the provided source overlay before publication, clone the repository,
create the branch with `git switch -c pollen-auto-prep`, and copy the overlay's
`TeamCode`, `docs`, and `tools` contents into the matching project directories.
These are new paths; preserve existing files and directory contents. The overlay
alone is not an Android Studio project.

Open the **project root** (`FtcRobotController-pollen`) in Android Studio. It is
the folder containing `settings.gradle`, `gradlew`, `TeamCode`, and
`FtcRobotController`. Do not open just a Java file or the TeamCode subfolder.

The checked-out repository uses FTC SDK **12.0.0**, Gradle **9.1.0**, Android
Gradle Plugin **8.13.2**, and TeamCode compile SDK **36**. The repo's release notes
require Android Studio **Narwhal 3 Feature Drop or later** for this AGP. Let Gradle
sync download the project's dependencies while the laptop has internet. Install
Android SDK Platform 36 if prompted. Use Android Studio's bundled compatible
Gradle JDK. Do not accept a suggested AGP downgrade or automatic project upgrade.

Build the project before connecting to the hub. Terminal alternative:

```bash
bash gradlew :TeamCode:assembleDebug
```

If it fails, record the first actual error, file, and line. Existing Pedro or
AprilTag dependency errors can block the entire app even though the new pollen
code does not use those components. Do not delete the team's existing classes
to hide a build error.

## 3. Pair the hubs and preserve configuration

1. Power the Control Hub using the robot battery; wait for it to boot.
2. Open **FTC Driver Station** on the Driver Hub.
3. If not already paired: three-dot menu → **Settings** → **Pair with Robot
   Controller** → **Wi-Fi Settings**. Select this Control Hub's network and enter
   its configured password. The factory password is `password` if unchanged.
4. Return to Driver Station. Confirm a connection, ping reading, and robot battery
   voltage. Use **About** to record the RC and DS app versions. Prefer matching
   12.0.x apps for this SDK 12.0 project; resolve any version warning before tests.
5. Record the current active robot configuration name and its hardware names.
   If this hub belongs to the robot, preserve that configuration for Monday.
6. If the active configuration expects an absent Expansion Hub or other hardware,
   use **Configure Robot** to create a separate `BenchOnly` configuration: scan,
   keep the detected embedded Control Hub, and leave external device ports unused.
   Save and activate it. Do not replace the team's actual robot configuration.

The BENCH modes never call `hardwareMap.get`, so they need no pretend motor names.
The Driver Station still needs a valid configuration for the connected hub itself.

## 4. Install from Android Studio

1. Keep the Control Hub on 12 V power.
2. Connect laptop USB to the **Control Hub USB-C port**. Deploy to the Control Hub,
   not the Driver Hub. Leave the Driver Hub paired over Wi-Fi.
3. Select the **TeamCode** run configuration and the connected Control Hub as the
   deployment target. FTC's TeamCode module builds the complete Robot Controller app.
4. Click Run. Wait for the build/install and the Driver Station to reconnect.
5. On Driver Station, select the autonomous list and find `Pollen BENCH Park`.
   INIT should show build identifier **pollen-prep-v1**.

If the Control Hub is absent from the device list, check its battery power and
try a known data cable first. The Control Hub comes with developer options enabled.
Installing a generic stock RC APK later replaces the custom Android Studio app;
redeploy TeamCode to restore your Java OpModes.

## 5. Run these tests tonight

| Select this autonomous OpMode | Expected result after INIT then START |
| --- | --- |
| `Pollen BENCH Park` | MOVE TO PARK → PARK / HOLD → DONE; about 1.3 seconds |
| `Pollen BENCH Shoot` | MOVE TO SHOT → SETTLE → SPINUP → FEED → MOVE TO PARK → PARK / HOLD → DONE; about 6 seconds |
| `Pollen BENCH Timeout` | Same start, 4-second spinup timeout, no feeding, warning that the shot was skipped, then park and DONE; about 6.5 seconds |
| `Pollen ROBOT Park` | BLOCKED because physical robot testing is disabled in the supplied settings |
| `Pollen ROBOT Shoot` | BLOCKED for the same reason |

For each BENCH mode:

1. Select it and press **INIT**. Wait several seconds: the sequence must not start.
2. Press **START** (play). Observe Phase, requested wheel powers, launcher
   target/measured velocity, feeder request, and elapsed time.
3. In the successful shot test, simulated launcher speed becomes 1250 ticks/s after
   about one second of spinup; feeding starts after a further 0.25 second stable period.
4. At DONE, all requested wheel, launcher, and feeder commands must be zero. Press STOP.
5. Re-run, pressing STOP midway through SPINUP and then midway through FEED on
   separate runs. Driver Station must exit the running mode promptly. Re-INIT
   and START must restart from the first step, never resume partway through.
6. Power-cycle the Control Hub, reconnect, and repeat BENCH Park to confirm the
   custom app is installed persistently. Keep a record of version/build and results.

BENCH displays intended commands; it does not test motor wiring or prove that a
physical motor stops. A real launcher would still coast after power is removed.

## 6. Monday: establish park-only first

1. Confirm this is the **pollen StarterBot mechanism**, with the launcher and
   feeder arrangement reflected in `BioBuzzStarterbotOpMode`. If the actual robot
   is different, adapt the hardware mapping before running robot modes.
2. Restore/select the actual robot's configuration. Check the names below against
   real ports. Use the team configuration on its own hub; a spare hub has different
   serial numbers and may need its own configuration.
3. Inspect wiring and lift/support the chassis so the wheels can turn freely.
   Keep mechanisms unloaded for the first direction check and have STOP ready.
4. In `PollenConfig.java`, set `ENABLE_ROBOT_PARK_TEST = true`; keep the shooting
   flag false. Rebuild/redeploy. Run `Pollen ROBOT Park`, which requests 20% forward
   power for only 0.75 seconds. Check each wheel's physical direction and STOP.
   If wrong, correct that wheel's `DRIVE_REVERSED` entry and redeploy.
5. Mark the chosen starting position and heading on the floor/field. Put the robot
   down in clear space, run a short move, and measure actual travel and drift.
6. Edit `parkRoute()` to reach the agreed legal parking location. Change one
   duration/direction at a time. Positive forward means robot-forward; positive
   strafe means robot-right; positive turn is clockwise under the assumed wiring.
7. Repeat at least three times from the same start, including another realistic
   battery charge level. Confirm the whole robot parks with margin to the boundary.
8. Commit and push the proven parking settings before working on shooting.

These are timed moves; they do not detect arrival. Wheel slip, battery voltage,
starting heading, and contact change the final position. If the short route is
not repeatable enough, use encoder/heading feedback as the next improvement.

## 7. Monday: optional shooting, then parking

The shooting routine currently expects these **team StarterBot** devices:

| Name | Device | Required for |
| --- | --- | --- |
| `frontLeft`, `frontRight`, `backLeft`, `backRight` | DC drive motors | Both robot modes |
| `launcher` | DC motor with working encoder velocity feedback | Shoot mode |
| `intake` | DC motor | Shoot mode |
| `windmillServo` | Continuous-rotation servo | Shoot mode |
| `leftIntakeServo`, `rightIntakeServo` | Continuous-rotation servos | Shoot mode |

This is six DC motors; the full setup requires more than the four motor ports
on a single Control Hub (normally the robot also has an Expansion Hub). That does
not prevent tonight's hardware-free bench tests or park-only with four drive motors.

1. First verify launcher direction, encoder connection, motor configuration, and
   measured positive velocity. The 1250 target / 1200 minimum ticks/s and
   PIDF 40/0/0/12.5 are inherited from goBILDA's sample; tune for the actual robot.
2. Establish a fixed shooting position/heading and test shots from there. This
   code does not find AprilTags or correct its aim. Confirm feeder direction and
   decide how long to feed. Two seconds is a test window, not a fixed shot count.
3. With the robot unloaded for the initial mechanism test, set both physical-test
   flags true and redeploy. After verifying operation, load and test in a clear
   shooting area under supervision.
4. Tune `shootRoute()` to drive from the marked start to the verified shooting
   position. Tune the final drive leg from that position to parking independently.
   The provided reverse leg simply returns approximately toward the example start.
5. Verify the launcher spins up with drivetrain stopped. The feeder must remain
   off below minimum speed and stop whenever velocity drops below minimum during
   the feeding window. If speed never stabilizes, the four-second timeout skips
   the shot and continues the parking leg.
6. Test STOP during movement and spinup, then during a controlled feeding test.
7. Keep the validated park-only option available as Monday's fallback. The 27-second
   cutoff stops all commanded outputs; it cannot guarantee parking after a failure.

## Repo findings that affect this work

- Team starter code already uses `frontLeft`/`frontRight`/`backLeft`/`backRight`.
  The original goBILDA download instead uses `left_front_drive` and similar names.
- The new StarterBot TeleOp reverses the **left pair**. `MecanumDrive` reverses
  **frontLeft/backRight**. `MecanumDriveTrain` reverses the **front pair**. We use
  the new StarterBot convention provisionally; only a physical check can settle this.
- Existing `Auto` and `Launcher` classes refer to a different mechanism (`flywheel`,
  `servo`, `coreHex`). The new routines do not reuse that hardware mapping.
- Existing TeleOp, camera, and Pedro files are unchanged by this addition.

## GitHub access and handoff

On October 18, the connected account was `ari1974`; repository metadata returned
`pull: true`, `push: false`. A separate collaborator-permission lookup was denied
by the integration. Publication was therefore not attempted. Ask the repository
owner to grant `ari1974` Write access and accept the invitation. If access has
already been granted, check that the GitHub connection is authorized for this repo.

Reading/cloning a public repository does not establish permission to push. On a
laptop with GitHub CLI installed and signed into the intended account:

```bash
gh auth status
gh repo view FtcBagelBits/FtcRobotController --json viewerPermission --jq .viewerPermission
```

`WRITE`, `MAINTAIN`, or `ADMIN` means repository write access; `READ` or `TRIAGE`
does not. Branch rules or credential scope can still restrict a particular push.
For missing access, a repo/org administrator can invite the correct GitHub account
with Write access; accept the invitation. Do not share passwords or access tokens.

After reviewing and building changes on your branch:

```bash
git status
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen docs/POLLEN_AUTONOMOUS.md tools/pollen-tests tools/test-pollen-sequence.sh
git commit -m "Add pollen autonomous routines and hub-only bench tests"
git push -u origin pollen-auto-prep
```

Open the pull request against **FtcBagelBits/FtcRobotController:main**, not the
upstream FIRST SDK. A pushed branch can be checked out and tested without merging.
Record the tested commit and configuration name so Monday's laptop and hub match.

## Validation record

- New Java sources compiled against the published FTC RobotCore **12.0.0** API
  using Eclipse's Java compiler in Java 8 language/bytecode mode.
- **62** checks passed for ordering, timeouts, speed gating, STOP, clock failure,
  power normalization, invalid routes, and completion. This used real pure Java
  sequence code, not an Android device simulator.
- The full Android APK build was attempted but blocked by this preparation
  environment's inability to fetch the Gradle distribution. Gradle project sync,
  complete APK packaging, installation, and hardware execution remain unverified.
- No physical robot or hubs were connected to the preparation environment.

To rerun the sequence checks with a JDK installed:

```bash
bash tools/test-pollen-sequence.sh
```

The distribution-settings check expects both physical-test flags to be false;
restore them to false when running that release check after robot testing.

## References

- [Team repository](https://github.com/FtcBagelBits/FtcRobotController)
- [Original goBILDA sample ZIP](https://www.gobilda.com/content/downloads/3200-2627-0004_example-code.zip)
- [REV pairing instructions](https://docs.revrobotics.com/duo-control/menu/control-hub-gs/driver-station-pairing-to-control-hub)
- [FIRST Android Studio tutorial](https://ftc-docs.firstinspires.org/en/latest/programming_resources/android_studio_java/Android-Studio-Tutorial.html)
- [GitHub CLI repository permission field](https://cli.github.com/manual/gh_repo_view)
