# Pollen autonomous code

Two standalone FTC autonomous OpModes implement timed parking and shoot-first
parking. This README covers `pollen-prep-v3.3-clean` after the cleanup patch.
For Driver Hub operation, controller tables and test records, read the
[operating manual](../../../../../../../../../docs/POLLEN_AUTONOMOUS.md).

## Start reading here

| File | Responsibility |
| --- | --- |
| [PollenRobotPark.java](PollenRobotPark.java) | Option A: FTC lifecycle, drive hardware, controller input, telemetry and motor output. |
| [PollenRobotShoot.java](PollenRobotShoot.java) | Option B: same integration plus launcher/intake/agitator setup, velocity readback and shooter initialization fallback. |
| [PollenConfig.java](PollenConfig.java) | Shared drive names/directions, Option A defaults and forward route construction. |
| [PollenParkTuning.java](PollenParkTuning.java) | Hardware-free Option A controller editing, limits, locking and settings snapshot. |
| [PollenShootTuning.java](PollenShootTuning.java) | Hardware-free Option B settings, distance-to-time calculation, fixed angle and route construction. |
| [PollenSequence.java](PollenSequence.java) | Hardware-free state machine: delay, timed drive, spinup, continuous feed, deadlines, STOP and mecanum mixing. |

The two OpModes call the FTC SDK. The other four classes can run in laptop Java
tests. The former base class, bench OpModes, start-delay helper and paused-feed
shooting path were removed. There are two Driver Hub autonomous choices:
`Pollen ROBOT Park` and `Pollen ROBOT Shoot + Park`. Both preselect `PollenBot`.

## Lifecycle and data flow

1. **init:** map/configure hardware, command zero output and restore the
   process-local settings snapshot. A hardware error is either reported as
   BLOCKED or handled by the specific shooter fallback below.
2. **init_loop:** read Gamepad 1, update settings and show telemetry. Buttons
   are edge-triggered: tap/release. No motion is requested by setting changes.
3. **start:** freeze settings, reset the elapsed clock and construct the
   sequence from the selected values.
4. **loop:** call `sequence.tick(...)` with elapsed seconds and, for B, measured
   launcher velocity. Apply its drive vector through the mecanum mixer and apply
   its shooter outputs. Publish phase and measurements.
5. **stop:** latch the sequence stopped and attempt to zero each device
   individually. A failure stopping one device does not skip attempts on others.

Settings are static, process-local snapshots, not files or Android preferences.
A and B keep separate snapshots; start delay resets each INIT. Restart/reinstall
loses the snapshots. START locks editing. No controller command cycles modes
while autonomous is running.

## Routes and shooting behavior

**A:** selected delay → timed forward drive → 0.5-second zero-output hold → DONE.

**B:** selected delay → spinup → continuous feed → diagonal drive → hold → DONE.
A zero feed duration removes the shot; zero distance removes the drive. Both
zero leave delay/hold only. Hardware initialization still runs.

B commands encoder velocity in **ticks/second**, not RPM. It qualifies initial
speed at 96% of the target for 0.25 seconds, with a 4-second spinup timeout.
Once qualified, flywheel, intake and agitator run for the selected feed duration.
Normal speed dips do not pause feeding. A nonfinite velocity stops the sequence.
`activeFeedSeconds` reports commanded feed time, not sensed ball delivery.
The shoot OpMode sends the mechanism outputs before the drive outputs, so the
shooting motors are commanded stopped before departure begins.

A spinup timeout skips feeding and continues B's own diagonal departure. If
shooter setup throws during INIT, fallback is allowed only when every mapped
shooting device accepts its stop command. The warning remains visible and the
shot is omitted. **Fallback does not use A's route**, because A assumes a
different starting pose. Drive initialization errors and unsuccessful shooter
stops block the mode. Runtime hardware exceptions attempt to stop everything
and request OpMode stop. Recovery is not guaranteed for SDK/electrical failures
outside this handling, or for disconnected devices that do not report an error.

## Hardware contract

Names are exact, case-sensitive lookups in the active Control Hub configuration.
Java does not name ports; the configuration names and code lookups must agree.

| Configuration name | SDK device | Direction | Mode |
| --- | --- | --- | --- |
| `frontLeft` | DcMotor | REVERSE | RUN_WITHOUT_ENCODER, BRAKE |
| `frontRight` | DcMotor | FORWARD | RUN_WITHOUT_ENCODER, BRAKE |
| `backLeft` | DcMotor | FORWARD | RUN_WITHOUT_ENCODER, BRAKE |
| `backRight` | DcMotor | REVERSE | RUN_WITHOUT_ENCODER, BRAKE |
| `pollenLauncher` | DcMotorEx | REVERSE | RUN_USING_ENCODER; velocity PIDF 40 / 0 / 0 / 12.5 |
| `intake` | DcMotor | FORWARD | RUN_WITHOUT_ENCODER, BRAKE |
| `agitator` | DcMotor | REVERSE | RUN_WITHOUT_ENCODER, BRAKE |

The launcher and two feed motors are used only by B. Directions and shooter
constants follow the team's `BioBuzzStarterbotOpMode` / `PollenBot` setup.
There are no windmill or intake servos in this implementation. Seven DC motors
normally require the Control Hub and Expansion Hub; A maps only four drives.

## Coordinates, units and calibration

Drive axes are robot-relative: positive forward, positive right strafe, positive
clockwise turn. The mixer is:

```text
frontLeft  = forward + strafe + turn
frontRight = forward - strafe - turn
backLeft   = forward - strafe + turn
backRight  = forward + strafe - turn
```

All four are divided by the largest absolute value when it exceeds one. Motor
directions are then handled by the configured SDK direction flags.

A uses time and power directly. The team's forward estimate is
`distance_cm = seconds * power * 150`, measured at 12.86 V; the factor is not
used to close a feedback loop.

B's active constants are **129 cm/second per unit power** and **25.87 degrees**:

```text
drive_seconds = selected_distance_cm / (129 * selected_vector_power)
forward = selected_vector_power * cos(25.87 degrees)
strafe  = -selected_vector_power * sin(25.87 degrees)
turn = 0
```

This preserves the calibration in the pushed branch plus cleanup. The original
100-cm diagonal request traveled 86 cm, with 76 cm forward, leading to the scalar
factor `150 * 0.86 = 129`. That trial's voltage was not reported. At power 0.20,
100 cm now requests about 3.876 seconds; the 60.96-cm default requests 2.363 seconds.
Power here is the forward/strafe vector magnitude, not each wheel's power.

There is no alliance transform: the paired physical starting poses are related
by a 180-degree rotation, so both use robot-relative forward-left. Humans place
and aim the robot. The nominal 280.88-cm full-field geometry is a planning value,
**not an approved route or the default**. See the
[geometry and calibration reference](../../../../../../../../../docs/POLLEN_SHOOT_FIRST.md)
for assumptions and required footprint/clearance checks.

Timed travel has no distance feedback, voltage compensation, heading correction,
obstacle avoidance or parking detection. Preserve a known calibration unless
new measured evidence justifies changing it. Record voltage, power, surface,
load, starting orientation, forward displacement and sideways displacement.

## Time and failure limits

Route construction validates the worst case against a 27-second deadline with
one second reserved. B rejects edits when the total is 26 seconds or more:

```text
start delay + drive seconds + 0.5-second hold
+ (feed seconds > 0 ? 4-second spinup timeout + feed seconds : 0)
```

An independent cutoff stops commands at 27 seconds. STOP, a backwards clock and
an invalid clock latch zero outputs. Hardware commands are best-effort; zero
commands do not prove that moving parts stopped instantly. Keep the Driver
Station autonomous timer enabled and physically test STOP.

## Build, deploy and test

From the repository root on the team's Mac, use the existing `ftcgradle` helper:

```bash
PATH="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin:$PATH" \
  bash tools/test-pollen-sequence.sh &&
ftcgradle :TeamCode:assembleDebug
```

If `ftcgradle` is not defined in this shell:

```bash
ftcgradle() {
  local ftc_jdk="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
  JAVA_HOME="$ftc_jdk" bash ./gradlew \
    "-Dorg.gradle.java.home=$ftc_jdk" "$@"
}
```

The [host test script](../../../../../../../../../tools/test-pollen-sequence.sh)
compiles the four hardware-free classes and tests with Java 8 compatibility.
Tests cover sequence behavior, controller edits, calibration arithmetic, limits
and stop conditions. They do not verify wiring, physical travel or shooting.
The Android build checks SDK integration and creates the APK.

Use the team's working Wi-Fi connection to deploy Android Studio's **TeamCode**
run configuration to the **Control Hub**, not the Driver Hub. Keep the active
hardware configuration. Check `pollen-prep-v3.3-clean` in INIT telemetry and run
the ordered physical checks in the operating manual. Laptop logic tests and
APK compilation need no powered robot; normal hub operation needs its battery.

A controller setting change needs only STOP, reselect, INIT, edit and START.
A Java change needs tests, rebuild and reinstall. A Markdown-only change needs
no APK rebuild or reinstall.

## Where to make changes

| Intended change | Location / follow-through |
| --- | --- |
| Permanent A power, duration or delay | `PollenConfig`; retest forward travel. |
| Permanent B menu defaults or ranges | `PollenShootTuning.Field`; update the operating manual's table and retest. |
| B distance calibration or angle | `PollenShootTuning`; record measurements and retest both starting placements. |
| Controller buttons | Respective tuning class and OpMode input mapping; update the manual and controller tests together. |
| Hardware name, direction or PIDF | Shared drive config or shoot OpMode; compare with working TeleOp and verify on the robot. |
| Sequence behavior or fault handling | `PollenSequence` and/or OpMode; add a meaningful regression test and verify relevant physical behavior. |

Keep the operator manual as the source for button instructions; keep this README
as the code map. Preserve the student's
[comparison with the old code](../../../../../../../../../docs/POLLEN_AUTONOMOUS_EXPLAINED.md)
for learning. Update documentation with the code in the same commit, record the
installed commit with test results, and never equate a passing laptop test with
a verified full-field route.
