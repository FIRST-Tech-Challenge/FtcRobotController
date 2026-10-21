# Pollen autonomous runbook

Choose **Pollen ROBOT Park** for Option A or **Pollen ROBOT Shoot + Park** for
Option B. Both command real motors after START and preselect `PollenBot` for
TeleOp. The Driver Station autonomous timer must be enabled. A menu name or
successful build does not establish that a route reaches the loading zone.

Option A remains the forward-only routine already used for calibration.
For Option B's end-wall starting pose, continuous shooting, controller settings,
geometry assumptions and ordered tests, read [POLLEN_SHOOT_FIRST.md](POLLEN_SHOOT_FIRST.md).
The two modes do not share the same physical starting pose.

For students comparing with last year's functions, see
[POLLEN_AUTONOMOUS_EXPLAINED.md](POLLEN_AUTONOMOUS_EXPLAINED.md).

## Build and deploy

From the repository root on the Mac, use the existing `ftcgradle` function:

```bash
PATH="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin:$PATH" \
  bash tools/test-pollen-sequence.sh &&
ftcgradle :TeamCode:assembleDebug
```

If the function is not available in this shell, define it:

```bash
ftcgradle() {
  local ftc_jdk="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
  JAVA_HOME="$ftc_jdk" bash ./gradlew \
    "-Dorg.gradle.java.home=$ftc_jdk" "$@"
}
```

Use the already-working Wi-Fi connection to the powered Control Hub. Deploy
Android Studio's **TeamCode** run configuration to the Control Hub, not the
Driver Hub. Retain the robot's active hardware configuration. A laptop build
needs no robot; normal Control Hub operation needs the robot's 12 V battery.

Select the intended mode, INIT, and inspect build, settings and any failure
message. INIT does not start the route. START locks the settings and begins;
STOP interrupts it. New Option B telemetry identifies `pollen-prep-v3.2.2-diagonal`;
Option A retains `pollen-prep-v3.1-tuning`.

## Option A: controller calibration

Use Gamepad 1 during INIT in **Pollen ROBOT Park**:

| Control | Change |
| --- | --- |
| D-pad UP / DOWN | Start delay + / - 0.5 seconds |
| D-pad RIGHT / LEFT | Forward drive duration + / - 0.05 seconds |
| R1 / L1 | Drive power + / - 0.05 |
| Triangle | Restore the values in `PollenConfig` |

Tap and release; held buttons do not repeat. Power is bounded to 0.05–0.50;
duration to 0.05–5 seconds. Delay is capped by the remaining sequence budget.
The screen reports the values and whether they are locked. Power/time persist
through STOP/INIT while the Robot Controller app remains open; delay resets
each INIT. Reboot, app restart or reinstall loses remembered settings.

Position and aim the robot manually. It always drives forward; it does not know
start A/B or alliance. Coordinate the whole path and destination with the
partner. Use the team's empirical estimate as a starting point:

`distance_cm = seconds * power * 150`, measured at **12.86 V**.

This is timed control, not distance feedback. Validate at the battery voltages,
surface and load used in practice. Copy successful values to `PollenConfig`
when ready to make them permanent, then rebuild once.

## Hardware

Java looks up the existing configuration names; it does not assign names to ports.

| Name | Device/direction | Required by |
| --- | --- | --- |
| `frontLeft` | Drive motor, REVERSE | A and B |
| `frontRight` | Drive motor, FORWARD | A and B |
| `backLeft` | Drive motor, FORWARD | A and B |
| `backRight` | Drive motor, REVERSE | A and B |
| `pollenLauncher` | DC motor with encoder, REVERSE | B shooting |
| `intake` | DC motor, FORWARD | B shooting |
| `agitator` | DC motor, REVERSE | B shooting |

The directions match the team's current `BioBuzzStarterbotOpMode` / `PollenBot`
TeleOp. Option B maps no windmill or intake servos. Seven DC motors normally
require the Control Hub plus Expansion Hub. Option A needs only its four drives.

## Bench choices and failures

Bench modes are hidden by `@Disabled`. If needed, temporarily enable only the
chosen bench class, build/install, test, then restore it for the match build.
They display simulated commands and map no hardware; they do not test wiring.

| Hidden mode | Exercises |
| --- | --- |
| `Pollen BENCH Park` | Option A settings and forward/hold/STOP |
| `Pollen BENCH Shoot First` | Current Option B settings, continuous shooting, diagonal departure |
| `Pollen BENCH Shoot` / `Pollen BENCH Timeout` | Legacy speed-gated shooting example; not current Option B |

A missing drive device blocks either physical mode. Option B can skip shooting
on a shooter INIT exception only if all mapped shooter devices accept stop
commands; it then uses its own diagonal departure. It never invokes Option A's
route from the end-wall shooting pose. A runtime hardware exception stops all
available motors individually and requests OpMode stop. A four-second spinup
timeout skips feeding and continues Option B's departure. Inspect the persistent
telemetry warnings. Some electrical/SDK failures cannot be recovered in an OpMode.

The independent 27-second command cutoff does not guarantee parking after a
failure or prevent physical coasting. Test STOP, actual direction, shot behavior,
route clearance and endpoint on the robot. Keep results tied to a commit.
