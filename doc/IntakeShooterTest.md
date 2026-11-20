# Intake, index and shooter bench test

November 4 software integration: [final TeleOp audit](TeleOpConsistencyChecklist.md) records collection while aiming, GP2 auto offsets, independent status lights and the retained current controls. Bench tuning/interlocks remain explicitly documented diagnostic policies.

**Test status: complete October 26, 2026.** The operator confirmed all transfer and intake tests complete, and turret rotation, shooting/angle and gate tests good. See [mechanism test sign-off](HardwareReviewImplementationStatus.md#mechanism-tests--complete-october-26-2026). Numerical calibration remains separate.

Select **Test: Intake + Index + Shooter** in the Driver Station's Test group.
It initializes the intake, both transfer motors, full sensor, gate, turret servos,
turret encoder and flywheel. It does not initialize the drivetrain, Pinpoint,
Limelight or depollinator. Use Gamepad 1; Gamepad 2 has the competition manual
feed (RT), inhibit (LT) and bumpers plus the flywheel tuning controls. The status
lights are optional and only reported when missing.

The turret homes during INIT to robot-right (90 degrees), with the hood at its
highest launch elevation (+15 degrees). Clear its travel before INIT. After START,
release Back, B, both bumpers and all triggers once to enable motor controls. Once
armed, intake and stage 1 begin automatic loading; the flywheel stays off until RT is held.

**Encoder wiring (November 13, 2026):** The intake has no encoder. The turret
Through-Bore encoder is plugged into the encoder input for `intake_motor` (Expansion
Hub port 1) and read through `intake_motor`; the turret never resets that port or
changes intake power, direction or mode, and it removes the SDK's motor-direction
sign so the channel's direction setting cannot flip yaw calibration. Standalone
configurations can still use a `turret_encoder` alias for port 1. Transfer stage 1
(`transfer_motor_1`, Expansion Hub port 2) now has its own motor encoder on port 2.
The intake runs the requested power directly in every OpMode; the intake jam
"jiggle" recovery was removed, so there is no automatic stall detection or reversal.

**Stage 1 speed:** stage 1 (goBILDA 5203 6000 RPM Yellow Jacket) runs open-loop
full power (`RUN_WITHOUT_ENCODER`) for loading, the intake-forward override, the
0.5 s fourth-ball run-on and shot feeding. Its own encoder is read only for the
"Transfer Stage 1 Speed (RPM, motor encoder)" telemetry line.

Simulation decision: the fixture now drives the `intake_motor` encoder from physical
turret yaw. The Mechanisms and both competition-wrapper scenarios assert that stage 1
stays out of velocity control at full open-loop power while loading and feeding.
No ball-contact or loaded-transfer model is claimed. Five encoder-routing tests cover
initialization order, direction-sign removal, the alias and independence from
transfer loading. No robot deployment was verified.
Verification after the wiring update: `tools/verify-teleop.ps1 -Sim -Scenario
Mechanisms,Sensors,Competition` passed 195 ordinary tests and 15 simulations with
zero failures, errors or skips, plus debug APK packaging and lint. These existing
scenarios include the actual bench TeleOp, turret diagnostic and both competition
wrappers; modeled shaft motion does not establish loaded intake behavior.

**Shooting = competition TeleOp (November 19, 2026).** This bench test now runs the
same shooting policy as Red/Blue `MecanumTeleOp`, through the shared
`CompetitionFeedController`: CONTINUOUS feed only (the bench's STAGGERED mode, its
GP2 RB toggle and the GP2 LB timing controls were removed, matching the competition
change the same day), an automatic RT + LT shot that waits for readiness, a GP2 RT
manual feed, a GP2 LT inhibit, automatic collection paused while shooting and
bang-bang + feedforward fusion regulation. GP1 rumble (queue full; RT held until
ready) and the shooter/intake status lights also follow competition. Bench-only
safeguards remain: release-to-arm after START, and GP1 Back stops and disarms (it was
B until GP1 B became a bench-only manual feed).

**Bench-only differences (later November 19, 2026, by request; this test only).**
There is no flywheel idle: the 3000 RPM idle is a competition-TeleOp-only change, so
the bench flywheel is off unless RT is held, then runs at the selected RPM. Every
feed (RT + LT or manual) commands the gate open first and starts transfer stage 1
(both linked motors) 0.10 s later
(`IntakeShooterTestController.GATE_OPEN_TO_TRANSFER_SECONDS`); competition still
starts the gate and stage 1 in the same loop. During that 0.10 s the gate is open,
stage 1 is off and automatic collection is paused; FEED STATUS shows GATE OPENING.
The check runs once per loop, so the transfer starts 0.10 s after the gate command or
up to one loop later.

| Control | Action |
| --- | --- |
| Bumpers released, armed, not shooting | Automatic intake/index: forward on LOW; intake stops (no reversal) after 0.5 s HIGH, stage 1 runs on 0.5 s then stops; LOW resumes forward/loading |
| Hold GP1 or GP2 RB | Force intake and stage-1 loading forward, overriding full (competition override) |
| Hold GP1 or GP2 LB | Reverse intake only and pause stage-1 loading; a shot already feeding continues, as in competition |
| Both bumpers | Stop intake and loading |
| Armed, RT released | Flywheel off (no bench idle; the 3000 RPM idle is competition-only) |
| Hold RT | Run the flywheel at the selected RPM; release turns it off (coast). RT alone does not pause collection |
| Hold RT + LT | Automatic shot: once readiness holds for 0.15 s, open the gate, then run the transfer (both motors) at full power 0.10 s after the gate command; keep feeding until either trigger is released or feed readiness is lost |
| Hold GP1 B or GP2 RT | Manual feed (GP1 B is bench-only; competition uses GP1 A / GP2 RT): open the gate at once and run the transfer 0.10 s later, bypassing homing/RPM/aim readiness (works with the flywheel off) |
| Hold GP2 LT | Inhibit: force the gate closed and stop the transfer; overrides every feed without disarming |
| Tap Y / A | Increase / decrease flywheel output RPM by 100 |
| Tap D-pad right / left | Increase / decrease flywheel output RPM by 10 (fine); all RPM steps clamp to 0..6000 RPM; starts at 3000 RPM |
| D-pad up / down | Lower / raise launch elevation by 0.5 degrees per press |
| Right stick X | Rotate turret; release holds its target |
| X | Rehome yaw and hood when bumpers and triggers are released |
| GP1 Back | Stop intake, transfer and flywheel and close gate; release Back, B, bumpers and triggers to resume |
| Press GP1 left stick | Toggle **open-loop max**: RT then runs the flywheel at full power (full battery voltage, `RUN_WITHOUT_ENCODER`, no velocity loop); press again to return to the selected controller. Toggling stops the flywheel and closes the gate like a controller switch |

Each button acts once per press. RPM changes apply on the next loop; a change of
up to 133 RPM does not interrupt a feed already running (see below), but a new shot
requalifies at the new target.

**Automatic shot readiness.** RT + LT requires a homed turret, measured flywheel
speed within 100 output RPM of the selected target, the bench yaw limits (up to 4
degrees of error, the retained bench-specific tolerance, and 3 degrees/s of measured
motion), hood travel complete, the yaw stick released and no RPM or hood change this
loop, all holding for the shared 0.15-second ready delay. The gate is then commanded
open and stage 1 starts 0.10 s later (bench-only). Feeding continues, with no timed window or recovery pause,
while the competition's looser continuation check holds: yaw error within 4 degrees,
hood settled and flywheel within 133 output RPM of the target. If that is lost the
gate closes, the coordinator records RECOVERY, and the shot requalifies the full
0.15 s before reopening. Releasing either trigger, pressing B, losing home or GP2 LT
stops feeding. The competition auto-aim's odometry/camera solution is not available
on the bench, so bench readiness stands in for it. Software counts feed starts, not
launched balls.

**Manual feed (GP1 B or GP2 RT)** opens the gate and, 0.10 s later, runs the transfer at
full forward power while held, with no readiness, feed window or recovery. GP2 LT, GP1 Back,
STOP and controller switches retain priority; startup and controller switches still require
releasing the motor controls to re-arm. A manual feed with RT released feeds with the
flywheel off (there is no bench idle); hold RT first to shoot at the selected RPM.

**Collection while shooting:** an RT + LT request, a manual feed, an open gate or
RECOVERY pauses automatic intake and stage-1 loading, exactly as in competition. The
bumpers still override the intake direction. Feeding always wins over the loading
pause.

Simulation decision for the competition shooting port (November 19, 2026): run
`Mechanisms`. The four actual-bench scenarios now check, through the real OpMode, that
the gate and both linked transfer motors start together with automatic collection
paused (`benchLoadsHoldsFullFeedsAndReloadsThroughActualMotorOutputs`), the GP2 RT
manual feed with the flywheel off, the GP2 LT inhibit and B stop/re-arm
(`benchManualFeedAndInhibitFollowCompetitionThroughActualOpMode`), fusion regulation
from START and the GP2 B switch to PIDF
(`benchFusionControllerSpinsUpHoldsAndFeedsThroughActualOpMode`), and +/-10 RPM fine
steps in telemetry (`benchActualOpModeStartsAndStopsSafely`). These scenarios had been
failing in INIT since at least November 18 because FTC Dashboard's static setup needs
the robot's native library; the OpMode now skips Dashboard off Android. `Competition`
was not rerun because the shared competition classes were not changed. Loaded ball
spacing, actual gate clearance and flywheel recovery under load still require
physical testing.

Simulation decision for removing the bench idle and adding the 0.10 s gate-to-stage-1
delay (November 19, 2026): ran `MechanismsSimTest` (11 passed). Through the real OpMode,
the transfer-cycle, manual-feed and fusion scenarios now assert that both transfer
motors stay off for the first 0.10 s after every gate opening and then feed (traces
showed 0.10-0.12 s at the 0.02 s tick), and that the flywheel stays at rest with
fusion OFF until RT and drops to OFF on release. `IntakeShooterTestControllerTest`
(21) covers the delay for automatic, manual, post-inhibit and requalified feeds, plus
a check that the competition coordinator still feeds in the same loop as the gate.
`Competition` was not rerun: no shared competition class changed. Whether 0.10 s is
enough for the physical gate to clear still needs robot testing.

The flywheel starts with the bang-bang + feedforward fusion controller, the
competition TeleOp's `MecanumTeleOp.FLYWHEEL_CONTROLLER` (it started with PIDF until
November 19, 2026). **Tap GP2 B** to switch between fusion and the FTC SDK's encoder
velocity PIDF controller in `RUN_USING_ENCODER` mode.
Switching stops the motors and closes the gate; release GP1/GP2 B, bumpers and triggers
to re-arm, then hold RT again. A held GP2 B toggles only once. The selected RPM and
both controllers' tuning are retained, while comparison measurements reset.

**Open-loop max (GP1 left-stick press)** bypasses both controllers: while RT is held,
`TurretSubsystem.runShooterOpenLoop(1.0)` applies full power with the motor in
`RUN_WITHOUT_ENCODER`, so speed depends on battery voltage and load. The encoder is still
read for telemetry and readiness. RPM readiness becomes "measured at or above the selected
RPM", so RT + LT auto shots still wait for spin-up; manual feed is unchanged. The next
closed-loop command or stop restores the selected controller's run mode (PIDF goes back to
`RUN_USING_ENCODER`).

Fusion uses `RUN_WITHOUT_ENCODER` (encoder readings still work) and runs every main
loop, including recovery, using one cached velocity sample per loop:

- **Speed error above 80 motor ticks/s (about 171 flywheel RPM):** bang-bang. Full
  power when slow, zero power (FLOAT coast) when fast. There is no reverse braking.
- **Error within 80 ticks/s:** feedforward, `kS + kV * target + kP * error`, scaled by
  `12 V / battery`. `kS` overcomes static friction, `kV` holds the target speed and
  `kP` is a small correction for the remaining error.

Starting values are `kS` 0.05, `kV` about 0.000392 and `kP` 0.001 (motor power at
12 V; `kV` and `kP` per motor tick/s). `kS` and `kP` are estimates and `kV` is
solved from the 6100 RPM at 14 V observation (November 20, 2026); none is tuned on this flywheel. Edit
`FUSION_KS`, `FUSION_KV` and `FUSION_KP` live in Dashboard. Releasing RT or stopping
commands zero power and coasts in FLOAT. The command never opposes the spin
direction, so lowering the target by 200 RPM or more simply coasts down to the
band, which takes longer than a powered brake would. PIDF remains P=40, I=0, D=0 with
voltage-compensated F. The GP2 D-pad and Y/A tuning controls only affect PIDF;
GP2 X resets both controllers' gains.

Actual/selected RPM, active controller, fusion zone (BANG-BANG or FEEDFORWARD),
commanded power and error in ticks/s, lowest RPM in the last/current feed, and
elapsed/last return to RPM tolerance (seconds) are displayed. Return time also
includes spin-up, and ends at first entry into the RPM tolerance, before the
separate stable-readiness check. Compare controllers at the same target, battery
condition and ball supply. Feed/recovery timing and RPM interlocks are identical
for both controllers. Full power cannot compensate for insufficient motor power
under sustained load. If the zone keeps flipping between BANG-BANG and FEEDFORWARD
while holding a target, the feedforward is too far off to stay inside the band:
correct `kV` (or `kS`) before raising `kP`.

The installed hood uses increasing PWM to lower away from its upper stop: home
is 0.559155 and full downward travel is approximately 0.728169. D-pad Up increases
travel; Down returns toward home and does nothing if already at home. Both angle
commands and raw servo commands are bounded to the permitted side of home. Verify
with one Up press and then one Down press before attempting the full range.

The goBILDA 5203 Yellow Jacket 6000 RPM motor drives the flywheel at 1:1.
Motor and flywheel RPM are equal; the configured free-speed estimate and target cap
are 6000 RPM. The motor encoder remains 28 counts/rev. The bench has no idle (the
competition TeleOp's 3000 RPM idle is competition-only). The RT selected target starts
at the competition manual RPM, 3000 (`MecanumTeleOp.FLYWHEEL_IDLE_RPM`; it was 2800,
then briefly 3500, on November 19, 2026). Calibration displays flywheel output RPM. It does not save session files.

Turret PWM direction is reversed for the reported mounting so the right stick
moves the turret in the corresponding direction. The external encoder correction
is OFF by default to prevent an extra software trim after stopping. The Axon
servos still hold position using their internal controllers, and the Through-Bore
encoder still reports angle/rate and qualifies initial feed readiness. Check that
physical clockwise motion increases the displayed robot heading. If it does not,
verify encoder sign before enabling external correction. Only `Test: Turret` uses
B to toggle optional correction; B in this combined test remains the motor stop.

If RT spins the flywheel but RT + LT does not feed, read **FEED STATUS** at the top
of telemetry. It identifies missing trigger input, a held GP2 LT inhibit, required homing,
flywheel RPM error, turret settling or hood travel. Actual/selected RPM and yaw
error/rate appear directly below it. A spinning flywheel alone does not establish
RPM readiness: its encoder cable must be connected to encoder port 0. If home is
required, release the motor controls, press X, and wait for homing to finish before
trying again. During an active feed, FEED STATUS shows AUTO FEED or MANUAL FEED, and
recovery/readiness reasons between attempts.

**Transfer investigation (October 31, 2026):** The reported symptoms are no stage-1
motion with RB and no two-stage feed with RT + LT. All transfer callers were traced:
this bench test, the shared red/blue competition loop, transfer-only test, guided
shot calibration and autonomous output coordinator. Each applies transfer outputs
in its active loop; no missing update call was found. The actual bench OpMode also
loads and feeds in SimLoop with an empty sensor signal and modeled turret/flywheel
feedback. This does not reproduce or resolve the reported physical failure.

The **TRANSFER STATUS** line shows this loop's commanded stage-1 power (one command for
both linked transfer motors since 2026-11-18),
loading/feed/full-hold reason and digital sensor state before verbose diagnostics.
**Transfer Stage 1 Devices** shows both configured motor names and SDK connection
information; compare those with Expansion Hub ports 2/3.
With controls armed, RB alone and a LOW sensor should show S1 1.00 (both motors)
and LOADING; qualified feed also shows 1.00. These are commands,
not proof of shaft motion. A 1.00 command with no motion requires checking the active
configuration/controller/motor path on the robot. A 0.00 command requires checking
the displayed full confirmation, OpMode pause or FEED STATUS first. Correct motor cabling
does not establish the full sensor's digital state or flywheel/yaw readiness.

HIGH continuously for 0.5 seconds confirms full even if the physical queue is empty;
LOW immediately clears full and resumes intake/index loading. Check sensor placement,
threshold and raw state if an empty queue reports HIGH/FULL. Polarity, full hold,
motor directions and shooting interlocks were preserved pending physical evidence.
Simulation was chosen to check actual bench loading/reverse/conflicting bumpers,
full hold, gate-delayed qualified feed, release and reload, alongside both actual
competition wrappers. It does not validate electrical delivery, sensor calibration,
Lynx timing or loaded ball movement.

Verification of this investigation: `verify-teleop.ps1 -Sim -Scenario
Mechanisms,Sensors,Competition` passed 185 ordinary tests and 15 simulation tests,
debug APK packaging and lint (0 errors, 14 existing warnings). The competition
manual-feed scenario exercised GP1 LT and GP2 RT separately for both alliances.
Log: `build/transfer-verification.log`. No robot deployment or physical fix was verified.

Start empty: check directions and clearance, then spin the flywheel and exercise
gate/feed into a clear catch area. Next hold RB to collect one ball, release RB,
hold RT until speed is steady, and add LT to feed. Release LT after the ball exits.
Add more balls only after that works. The transfer (both linked motors) loads, and feeds forward during qualified feeding.
Before the first full-power run after the 2026-11-18 linkage change, confirm both motors push the
transfer the same way (both FORWARD); motors fighting each other stall and overheat.
The full sensor must remain HIGH for 0.5 seconds to stop the intake (it does not
reverse). Stage 1 runs on another 0.5 seconds to seat the fourth ball, then holds. Pause, feed, or shutdown cancels the run-on. LOW resumes loading
immediately; a new continuous 0.5 s HIGH starts a new run-on. Check sensor
placement and that this delay does not cause a jam. Full does not prevent shooting.

B removes motor commands; the flywheel coasts and a ball already in the shooter
can still exit. Stop the OpMode and wait for motion to stop before clearing jams.
STOP or a loop exception attempts to stop every motor and close the gate.

See [shared motor baseline, PIDF tuning and hood reference](ShooterVelocityTuning.md) for the 6000 RPM target cap, GP2 live tuning controls, and calibration compatibility.
