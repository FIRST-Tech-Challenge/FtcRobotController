# Final TeleOp software consistency audit

## Physical checklist status - corrected November 5, 2026

Prithvi confirms tasks 1-6 of the 12-task remaining-work checklist COMPLETE / PASS:
Pinpoint X correction, turret/hood, flywheel, loaded transfer/feed, status lights
and depollinator integration. Previously reported build/configuration, drivetrain,
intake, queue sensor and gate checks also remain complete.

Task 7 (Limelight/field geometry) subsequently passed per the operator, including
a 35 ms button-to-detection result (not calibrated camera latency). Task 8
(localization and auto aim) is COMPLETE / VERIFIED by functional check on
2026-11-14: the operator reports Test: Auto Aim localizing and tracking the goal
after the Limelight inverted-mount and Pinpoint X pod fixes.
Tasks 10 and 12 remain PENDING / NOT TESTED: combined competition controls and
full match rehearsal. Task 9 (shot calibration/release timing) is not required
(team decision 2026-11-19: current values work and feeding is continuous only);
task 11 (autonomous) is not required.
This replaces the earlier incorrect
all-stages sign-off. See [the verification matrix](TeleOpVerificationMatrix.md)
for the numbered status table. Production calibration data and flags are unchanged.

Updated November 4, 2026. Scope: source review and software integration for both final
alliance TeleOps. No physical testing or deployment is required for this audit.
Existing PIDF, feed timing, geometry and calibration flags remain provisional.
The user's latest decision to keep the newer working-tree controls is authoritative.

## Current controls

| Input / default | Both competition TeleOps |
| --- | --- |
| GP1 sticks | Robot-centric translation and rotation |
| GP1 RT | Automatic yaw, hood and distance RPM preparation; ordinary collection continues |
| GP1 RT + LT | Readiness-checked automatic fire request; ordinary collection pauses |
| GP1 A or GP2 RT | Explicit manual readiness bypass; immediate continuous feed (gate and both linked stage-1 motors together) |
| GP2 LT | Close gate, stop transfer and pause automatic intake; explicit intake bumper overrides remain available |
| Flywheel | Idles at 3000 output RPM from START (was 3500 until 2026-11-19); returns to the manual setting when RT is released |
| GP2 RS-X | Manual yaw with RT released; clockwise-positive fine yaw offset while RT is held |
| GP2 Dpad Up/Down | Hood travel +/-0.5 degrees; trims the automatic hood only while RT is held with a calibrated solution |
| GP2 Y/A | RPM +/-50; trims the automatic RPM only while RT is held with a calibrated solution |
| GP2 Dpad Right | Clear all auto offsets; also cleared by turret rehome |
| GP2 X | Turret rehome with auto aim released |
| GP2 Dpad Left | Explicit alliance-corner field reference and optional camera refinement |
| GP2 Back | Show/hide full diagnostics telemetry; the short driver view is always on top |
| GP1 RS click / LS click | Deploy / home depollinator; home priority |
| Either controller RB / LB | Force intake + stage 1 forward / reverse intake and pause stage 1; opposing bumpers stop collection |
| GP1 rumble (2026-11-19) | `DriverRumble`: two pulses when the queue becomes full; steady rumble while RT is held until the shot is ready. GP2 does not rumble |
| Fault lights (2026-11-19) | A new turret homing, slide, Pinpoint or firing fault blinks all three lights red for 2 s. See [CompetitionTeleOp.md](CompetitionTeleOp.md#gp1-rumble-and-fault-lights-november-19) |

Offsets persist across RT release during this OpMode session. Bounds are yaw +/-10
degrees at 15 deg/s full stick, hood +/-5 degrees, RPM +/-500. Positive hood travel
lowers launch elevation. Offset changes restart feed qualification. A resulting
setpoint outside mechanism limits blocks automatic firing; offsets cannot turn an
uncalibrated profile into a valid one. Release GP1 RT for the existing manual fallback.
Feeding is continuous only; GP2 B has no competition function (2026-11-19).

Shooting requests, gate travel, recovery and inhibit pause ordinary intake/loading;
RT alone does not. Explicit intake direction overrides remain available. Actual
feeding takes priority over stage-1 collection controls. Full confirmation is
0.5 s HIGH, which stops the intake and starts a 0.5 s stage-1 run-on; LOW resumes ordinary loading.

## Gaps fixed

- Corrected competition collection arbitration: aim-only keeps loading; shooting
  pauses ordinary intake as well as stage 1. Existing bumper priorities remain.
- Added bounded GP2 yaw/hood/RPM adjustments during automatic aiming, reset controls,
  and readiness requalification after edits. Both alliance wrappers share the code.
- Replaced gate-open-as-ready telemetry with the actual automatic tracking checks;
  manual bypass and inhibit have explicit labels. Green shooter light means current
  checks pass; the firing state machine still applies its stable-readiness timer.
- Changed AutoAimTestOpMode to the competition TELEOP_ODOMETRY mode, nearest-cell
  selection and encoder correction. Its independent hold-to-spin and no-feed policy
  remain. It now acquires a field reference rather than using a fixed starting-pose
  diagnostic configuration. RS-X manually scans with RT released.
- Added three independent PWM status lights and an isolated LightsTestOpMode.
- Corrected stale turret/intake encoder-port and two-second full-latch documentation.

## Review of test and autonomous functions

| Source / function | Competition disposition |
| --- | --- |
| DrivetrainTestOpMode | Same MecanumDrivetrain motor setup, gearing, robot-centric commands; single-wheel debug remains isolated |
| IntakeTestOpMode | Shared motor setup; competition adds sensor-based automatic direction and shooting coordination |
| TransferTestOpMode | Shared full debounce, 0.5 s stage-1 run-on, stage-1 loading on both linked motors and inhibit; competition adds selected feed sequence |
| GateTestOpMode | Shared positions/PWM; direct diagnostic gate commands stay isolated |
| IntakeShooterTestOpMode | Shooting policy shared with competition since 2026-11-19 (`CompetitionFeedController` continuous feed, GP2 RT manual feed (plus bench-only GP1 B), GP2 LT inhibit, collection paused while shooting, fusion flywheel, rumble/lights); bench arming, GP1 Back stop, RT for selected RPM, bench yaw readiness and live tuning remain diagnostic. Bench-only: no flywheel idle (the 3000 RPM idle is competition-only) and stage 1 starts 0.10 s after the gate-open command |
| TurretTestOpMode | Shared homing, hood, flywheel and bounded yaw; competition deliberately uses measured-angle release hold and encoder correction |
| AutoAimTestOpMode | Now exercises the competition localization/nearest-cell path; no intake, gate, transfer or drivetrain ownership |
| PinpointTestOpMode | Same odometry implementation; raw calibration/reset controls remain isolated, competition uses explicit corner rehome |
| LimelightTagTestOpMode | Tag/pipeline inspection stays diagnostic; competition consumes the shared vision source and validated pose observations |
| GoalStateTestOpMode | Classifier remains diagnostic: driver decides whether the nearest alliance cell is raised |
| DepollinatorTestOpMode | Same subsystem and GP1 stick-click deployment/home controls; user reports installed and working |
| ShotCalibrationOpMode | Measurement/recording tool only; no fabricated profile or calibration flags imported |
| ServoZeroCalibrationOpMode | Disconnected-servo raw positioning stays outside match controls |
| LightsTestOpMode | Same light subsystem/palette; diagnostic lets operator select channels independently |
| PedroDriveTestOpMode / PollenVisionTestOpMode / PollenAuto | Navigation/camera experiments remain outside manual TeleOp; no automatic pollen-driving migration |
| Both competition autonomous wrappers / BioBuzzRobot / AutonomousOutputCoordinator | Shared mechanism implementations retained; shared intake recovery applies, while autonomous output ownership, presets and route feed policy remain explicit |

## Three independent lights

Expansion Hub servo 5 is the linear slide (`slide_servo`, CR Servo entry) and Control Hub analog 1 is its feedback (`slide_encoder`); Control Hub servos 0-2 are unused and the lights use servo ports 3-5. The claw was removed 2026-11-19.
Configure standard positional Servo entries with these exact names; software finds
names, not hardcoded port numbers.

| Connection | Name | Meaning |
| --- | --- | --- |
| Control Hub servo 3 | led_intake | Blue loading/forward override; green confirmed full; orange reverse; off paused/stopped |
| Control Hub servo 4 | led_shooter | Yellow standby/preparation/recovery; green automatic tracking checks pass; violet gate opening/feeding; blinking violet manual bypass; red inhibit/fault |
| Control Hub servo 5 | led_status | Green homed with fresh field pose; orange field reference unavailable/stale; yellow homing; red turret unavailable/home fault |

Fault alert (2026-11-19): when a turret homing, slide, Pinpoint or firing fault starts, all three
lights blink red for 2 s, then return to the colors above.

Full/empty and successful launches cannot be inferred from motor commands. A full
light reports the queue sensor's confirmed state; green reference reports available
localization, not verified field accuracy. Missing lights appear in telemetry and
allow the rest of TeleOp to initialize. STOP commands all configured lights off.

Pulse colors follow the [goBILDA product chart](https://www.gobilda.com/rgb-indicator-light-pwm-controlled/).
The subsystem explicitly uses a 600-2400 us range: off 600, red 1100, orange 1200,
yellow 1300, green 1500, blue 1700, violet 1900, white 2400 us. Off is below 1100;
white is above 1900. This avoids relying on the Hub's selected servo type/range.

## Remaining external inputs, not software migration blockers

- Final flywheel PIDF/controller selection and loaded gate/transfer timing.
  Staggered timing is now one shared, robot-tuned set (2026-11-15) for competition, autonomous, bench,
  transfer-only and calibration: 0.10 s gate delay / 0.20 s feed / 0.00 s recovery, stage 1 (both linked motors) feeds
  (0.30 s total open). Stage 1 keeps running with the gate closed between held timed shots;
  continuous starts the transfer immediately. There is no separate stage 2 since 2026-11-18. Loaded timing is unmeasured.
  Bench unrestricted continuous feed must not be mistaken for automatic readiness.
- Release timing. The measured 12-60 in shot table (2026-11-15) drives auto-aim hood/RPM; release
  timing no longer blocks automatic firing (gate removed 2026-11-15) and the current values are unmeasured.
- Physical verification of Pinpoint scale/heading, camera convention and nominal
  tag geometry; those measurements cannot be obtained through this code audit.
- Actual light wiring/configuration and mounting orientation.

No robot tests are requested now. These items are recorded so software integration
is not confused with completed calibration. No production calibration flag was enabled.

## Desktop verification

Compile completed with the explicit installed JDK. The initial default-Java test
launch stalled and was stopped without producing test results. Subsequent checks
use the explicit JDK, offline dependencies, selected tests and a 120-second cutoff.
The completed run passed 43 focused unit tests and 7 selected simulation methods
in 64 seconds, with zero failures/errors/skips. See the verification matrix for
exact scenario scope. Full suite/lint/full simulation, APK packaging and
physical testing are intentionally omitted for this pass per the revised testing scope.

Simulation decision: use selected actual Red/Blue wrapper scenarios for collection,
manual controls, offsets and lights, plus isolated intake/light scenarios. Synthetic
shot profiles and normalized camera convention are explicitly test-only. Motor/servo
commands and timing are observable; loaded RPM, actual balls,
servo load and optical color are not modeled.


November 5 gate wiring and policy: The gate servo is wired directly to Expansion Hub servo port 3, bypassing the 6 V power injector. It receives immediate full-position targets without software ramping for maximum available speed. The 0.043 s/60-degree rating at 6 V is catalog reference only; actual direct-hub loaded speed is unmeasured.

Both final Red/Blue TeleOps start CONTINUOUS: once a feed is authorized, the gate-open command and the full-power transfer command (both linked transfer motors) occur together, without a gate delay. Automatic readiness and GP2 LT inhibit still apply; manual feed bypasses readiness. STAGGERED opens the gate, starts stage 1 at 0.10 s, and closes the gate after the shared 0.20 s feed (0.30 s total open window). Stage 1 keeps running while the gate is closed through the recovery and any subsequent readiness wait. At the next opening the transfer stops, then repeats the shared 0.10 s delay. Release, inhibit, invalid automatic solution, tracking loss or STOP cancels transfer feeding; tracking-loss recovery does not continue closed-gate feed. Mode changes reset the sequence. Autonomous and diagnostic defaults are unchanged.
