# Pollen autonomous runbook

Two ROBOT modes execute timed example routes: park, or shoot then park. **They
command real hardware on START; there are no compile-time enable flags.** The
routes are not calibrated field paths. Establish an exact start, heading, and
finish on the physical robot before competition use. Build telemetry: `pollen-prep-v2`.
For the design and comparison with existing team code, see
[POLLEN_AUTONOMOUS_EXPLAINED.md](POLLEN_AUTONOMOUS_EXPLAINED.md).

## Build and laptop tests

Open this repository's root in Android Studio. Use the existing project versions
(FTC 12.0.0, Gradle 9.1.0, AGP 8.13.2, Android SDK Platform 36).
From the project root on the Mac, this function selects Android Studio's Java for
both the Gradle launcher and daemon; add it to `~/.zshrc` to keep it:

```bash
ftcgradle() {
  local ftc_jdk="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
  JAVA_HOME="$ftc_jdk" bash ./gradlew \
    "-Dorg.gradle.java.home=$ftc_jdk" "$@"
}
ftcgradle :TeamCode:assembleDebug
PATH="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin:$PATH" \
  bash tools/test-pollen-sequence.sh
```

The script checks sequence behavior, including speed recovery, timeout, STOP,
delay, and budget validation. It does not test Android or physical hardware.
Laptop builds/tests need neither hub nor a battery. A normal hub run requires
12 V robot-battery power to the Control Hub; laptop USB is not that power supply.

## Deploy and select modes

1. Preserve the robot's active configuration. Pair the Driver Hub with the powered
   Control Hub and resolve app-version warnings.
2. Connect a USB data cable to the Control Hub's USB-C port. Deploy Android
   Studio's **TeamCode** run configuration to the Control Hub, not the Driver Hub.
3. On Driver Station, enable the 30-second autonomous timer, select the intended
   OpMode and press INIT. Check build, active route, delay, and any BLOCKED/FALLBACK
   message. INIT must not start the sequence. START begins it; STOP interrupts it.
4. Both ROBOT modes preselect `BioBuzz StarterBot` for TeleOp. This selects the
   next mode; it does not start it or make its required hardware optional.

| Mode | Default menu visibility | Expected result with supplied routes and zero delay |
| --- | --- | --- |
| `Pollen ROBOT Park` | Visible | Actual drive commands; drivetrain required. |
| `Pollen ROBOT Shoot` | Visible | Actual driving and shooting, or INIT-time park fallback. |
| `Pollen BENCH Park` | Hidden with `@Disabled` | Display-only move/hold/DONE; about 1.3 s. |
| `Pollen BENCH Shoot` | Hidden with `@Disabled` | Simulated speed qualifies after about 1.25 s of spinup; feed and park; about 6 s total. |
| `Pollen BENCH Timeout` | Hidden with `@Disabled` | Four-second spinup timeout, no feed, warning, then park; about 6.5 s total. |

To run a hub bench test, temporarily remove `@Disabled` from the desired
`PollenBench*.java` file, rebuild, and install. Use a valid hub configuration
without absent devices; BENCH itself maps no motors or servos. Verify INIT waits,
START runs, DONE zeros requests, and STOP interrupts driving/spinup/feeding.
**Restore `@Disabled`, rebuild, and install the final app before competition.**
Bench telemetry is simulated; it proves neither wiring nor physical stopping.

## Device table

Names must match the active Robot Controller configuration exactly. Java looks
up those names; it does not assign names to ports.

| Name | Device | Required for |
| --- | --- | --- |
| `frontLeft`, `frontRight`, `backLeft`, `backRight` | DC drive motors | Both ROBOT modes |
| `launcher` | DC motor with encoder velocity feedback | Shooting |
| `intake` | DC motor | Shooting |
| `windmillServo` | Continuous-rotation servo | Shooting |
| `leftIntakeServo`, `rightIntakeServo` | Continuous-rotation servos | Shooting |

Full hardware needs six DC motor ports, normally Control Hub plus Expansion Hub.
Park-only needs four. Directions provisionally follow `BioBuzzStarterbotOpMode`;
verify them against this robot, not another robot's helper classes.

## Calibrate on the robot

1. Restore the actual robot configuration. With the chassis securely supported
   and mechanisms unloaded, check wheel names/directions and STOP using ROBOT Park.
2. Establish the exact wall contact, tile reference, offset, and heading. From
   that repeatable start, measure short drives and tune `parkRoute()` one leg at
   a time. The supplied 0.75 s at 20% is only an example.
3. Check the current [BIOBUZZ manual](https://ftc-resources.firstinspires.org/ftc/game/manual).
   In TU05, G304 requires a start touching the perimeter, wholly on your side,
   outside the loading zone, clear of FLOWERS, in starting configuration, and
   contacting four POLLEN. Section 10.5.4 scores PARK for partial loading-zone
   overlap; LEAVE requires no wall contact at AUTO's end. Aim for obvious overlap
   plus a wall gap, not full containment. See Figures 9-3, 10-7, and 11-1.
4. Repeat from the same start at least three times and at another realistic
   battery charge. Check drift, clearance, and parking with a partner robot in
   its intended position. Record the tested commit, configuration, and battery.
5. Validate shooter direction/encoder feedback and feeder direction while unloaded.
   Then test loaded shots from a fixed shooting pose. Starter values are 1250
   target / 1200 minimum ticks/s, PIDF 40/0/0/12.5; they are not calibrated aiming data.
6. Measure velocity dip and time to regain stable speed after each shot. Tune
   `ACTIVE_FEED_SECONDS` (default 2), `MAX_FEED_WINDOW` (default 6), and launcher
   settings using measured results. Do not lower the threshold merely to empty
   the feeder. Commanded feed time is not a ball count or proof of scoring.
7. Tune `shootRoute()` and its parking leg independently. Both route choices must
   use the SAME initial pose so the INIT-time fallback is valid. The example
   reverse leg returns toward the start; it is not a proven scoring park.
8. Coordinate paths, final footprints, and arrival order with the alliance partner.
   Set `START_DELAY_SECONDS` if needed, rebuild, and retest. It is included in the
   route's worst-case budget. No automatic alliance mirroring is implemented.
9. Verify STOP during motion, spinup, and feeding. Keep the validated park-only
   option available. Hide BENCH modes and verify the exact deployed menu before use.

## Failure behavior to verify

- Missing/invalid drive hardware: INIT is BLOCKED; every mapped device is stopped.
- Shooter initialization exception: all mapped shooter devices are individually
  commanded off. If those commands succeed, INIT reports **FALLBACK: PARK ONLY**
  and selects `parkRoute()`. If any stop command fails, INIT remains BLOCKED.
- To test fallback, use a separate test configuration with a shooter name absent,
  then a drive name absent. Confirm fallback in the first case and blocking in
  the second. Preserve and restore the working configuration.
- A missing/unpowered hub may be rejected by the SDK before the OpMode runs, or
  yield hardware objects without throwing. This fallback cannot guarantee recovery
  from an unplugged Expansion Hub. Inspect Driver Station connection warnings.
- Launcher never qualifies within four seconds: skip shooting, continue the
  shooting route's remaining parking leg. Once feeding begins, recovery pauses do
  not consume active-feed time, but a six-second wall-clock window bounds them.
- Runtime hardware exception: stop all devices and request OpMode stop. Never
  restart a route from the original start after the robot has moved.
- A 27-second global cutoff stops outputs; it cannot guarantee parking after a
  failure or scheduler delay. A launcher can coast after power is removed.

## Validation limits

Host checks and compilation against RobotCore 12.0.0 cover logic and API usage.
Build the full APK with the command above after each revision. Verify deployment,
SDK/hub failure behavior, route repeatability, and shooting on the actual robot.
Keep results tied to a commit; a successful build of an older revision does not
validate the current one.
