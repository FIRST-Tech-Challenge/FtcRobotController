# Shooter speed and loaded tuning

The shooter is a [goBILDA 5203 Yellow Jacket 6000 RPM motor](https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/)
with 1:1 motor-to-flywheel drive. `ShooterSpeed` configures 6000 RPM at 12 V and
28 encoder counts/revolution. Motor RPM and flywheel output RPM are equal. The
6000 RPM catalog no-load speed is a configuration estimate, not a loaded measurement.
`BareMotorSpecifications` holds the separate goBILDA 5000 drivetrain baseline of
5800 RPM; its [20:80 gear and 30:91 belt reduction](Drivetrain.md) applies only to
the wheels. Shooter speed and encoder conversion use their own 1:1 configuration.

Shooter commands, profiles and calibration allow up to 6000 flywheel RPM, with no
software headroom reserve. The combined bench test keeps its 2800 flywheel RPM
starting target; competition TeleOp keeps 3500 flywheel RPM. Readiness tolerances
remain expressed in output RPM. A low battery or continuous ball load can reduce
achievable speed. Do not widen readiness tolerance to hide that. Re-measure shot
rows and loaded PIDF response after this mechanical change; production profiles
remain uncalibrated.

Both competition TeleOps (`MecanumTeleOp.FLYWHEEL_CONTROLLER`, November 15, 2026)
regulate the flywheel with the bang-bang + feedforward fusion controller described
below. Every other turret-based OpMode, including autonomous, defaults to hub
velocity PIDF in `RUN_USING_ENCODER`. Starting gains are P=40, I=0, D=0. P was doubled from 20 for
stronger response to shot-induced speed error. Integral remains off initially to
avoid windup near maximum speed, but can be tuned during the bench test. Feedforward at 12 V is
`32767 / (5400 / 60 * 28) * (14 / 12)`, approximately 15.170 at 12 V
(13.003 at 14 V). This uses the reported installed speed; see the observation below. Every 0.2 seconds its F coefficient
is multiplied by `12 / batteryVolts`, using the lowest valid hub voltage reading.
Invalid or unavailable voltage uses the 12 V reference. These are initial gains,
not gains validated on this flywheel. There is no second software velocity PID
loop. [FTC PIDF documentation](https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/pidf_coefficients/pidf-coefficients.html)
describes the per-port controller settings.

In **Test: Intake + Index + Shooter**, Gamepad 1 retains the shooting controls.
Gamepad 2 can adjust gains while testing:

| GP2 control | Change |
| --- | --- |
| B | Toggle PIDF / bang-bang + feedforward fusion in this bench test; stops motors and requires GP1 controls released to re-arm |
| D-pad Up / Down | P +1 / -1, bounded 0..100 |
| D-pad Right / Left | Feedforward multiplier +0.05 / -0.05, bounded 0.5..1.5 |
| Y / A | I +0.05 / -0.05, bounded 0..5 |
| X | Reset P, I, D, feedforward multiplier and the fusion kS/kV/kP to the initial settings |

Bench settings are shared with FTC Dashboard and remain in memory across bench restarts
until the Robot Controller app restarts. Record validated values in
`ShooterVelocityTuning` to apply them to every turret-based OpMode. Current P/I/D/F,
feedforward multiplier, actual/target RPM, battery voltage, motor current, and the
lowest RPM observed during the last/current feed are displayed. The speed-headroom
percentage is an estimate from voltage and free speed, not measured motor duty.

## Live FTC Dashboard tuning

FTC Dashboard 0.6.0 is installed in TeamCode, with its transitive FTC SDK dependencies
excluded so the project retains SDK 12.0.0. See the official
[setup](https://acmerobotics.github.io/ftc-dashboard/gettingstarted) and
[configuration/telemetry reference](https://acmerobotics.github.io/ftc-dashboard/features).

1. Build and deploy the Robot Controller app once. Connect the laptop to the Control
   Hub's Wi-Fi and open [Dashboard](http://192.168.43.1:8080/dash).
2. Select **Test: Intake + Index + Shooter** on the Driver Station. In Dashboard's
   Config view, expand **IntakeShooterTestOpMode**. Editable fields are `P`, `I`, `D`,
   `F_SCALE` and `TARGET_RPM` (flywheel output RPM, not motor RPM), plus the fusion
   controller's `FUSION_KS`, `FUSION_KV` and `FUSION_KP`.
3. Start, then release the bumpers/triggers to arm. Hold GP1 RT to spin. GP1 Y/A
   adjusts the same target as Dashboard. Hold RT + LT for the existing timed,
   readiness-controlled feed/recovery cycle; GP1 B stops motors and closes the gate.
4. Apply one field change at a time in Config. Gains are read each active loop and
   sent to the hub only when changed. Gamepad gain edits modify those same fields;
   refresh Config to see gamepad edits. GP2 X resets gains, including D, but not RPM.
5. Add `Target RPM` and `Measured RPM` to a graph. Add `Error RPM` and `Feeding (0/1)`
   to observe sag and recovery. Packets are sampled at up to 20 Hz; electrical readings
   retain their 5 Hz sample rate. `Selected RPM` is the operator setting even while
   stopped; `Target RPM` is the actual command and becomes zero when spin is released.

Applied P/D are bounded to 0..100, I to 0..5, F scale to 0.5..1.5 and target to
0..3700 flywheel RPM. Non-finite gains retain the prior valid setting; a non-finite
RPM selection commands zero. Graph/Driver Station gains show the bounded values
actually selected; Config can still display an out-of-range requested value.
The hub F coefficient remains `BASE_F * 12 / batteryVolts * F_SCALE`: edit the
multiplier rather than replacing voltage compensation. PIDF gains do not affect
fusion mode, and the fusion gains (bounded to kS 0..0.3, kV 0..0.002, kP 0..0.02)
do not affect PIDF. Other OpModes keep their compiled defaults; no Dashboard values
automatically transfer into competition. Save winning gains in Java source and
rebuild. Save P/I/D in `INITIAL_P`/`INITIAL_I`/`INITIAL_D` and the F multiplier in
`INITIAL_F_SCALE` in `ShooterVelocityTuning`.

Keep I/D zero initially, tune steady-state F then loaded P, and compare repeated
shots at consistent target, feed pattern and battery voltage. Minimum-feed RPM
only samples while the transfer is commanded on; a later ball impulse may be missed.
Return-to-tolerance time includes spin-up and does not prove sustained stability.
Use the RPM graph to inspect overshoot and dwell; network telemetry may miss short dips.

Simulation decision for this integration: use existing Mechanisms/Competition
regressions for bench interlocks and unchanged competition defaults, plus ordinary
fake-hardware tests for live hub coefficients, bounds, voltage compensation and
unchanged-setting writes. SimLoop has no Android Dashboard/WebSocket service or
calibrated shot-load model; browser editing, graphs and physical recovery require
Control Hub testing. No new OpMode or simulation scenario is introduced.

Verification on November 2, 2026: `tools/verify-teleop.ps1 -Sim -Scenario
Mechanisms,Competition` passed 187 ordinary tests, debug APK packaging, lint
(zero errors, 14 existing warnings), and 12 selected simulation tests. This includes
the actual bench load/feed/STOP scenarios and both competition wrappers. The new
`ShooterLiveTuningTest` checks live coefficient updates without restarting the RPM
command, unchanged-setting write suppression, invalid/bounded gains and retained
voltage compensation. Dashboard/browser operation and loaded physical tuning were
not run on a Control Hub.

## Controller comparison and loaded trials

The bang-bang + feedforward fusion controller (added November 11, 2026, replacing
the plain bang-bang option) is used by both competition TeleOps and is available
through the combined bench test's controller toggle. It switches the motor to RUN_WITHOUT_ENCODER and uses each main
loop's cached encoder sample; the hub PIDF loop is not active in that mode.

- Speed error above 80 motor ticks/s (`FUSION_BANG_BANG_ERROR_TICKS_PER_SECOND`,
  about 171 flywheel RPM): bang-bang. Full power when slow; zero power and a FLOAT
  coast when fast, as the earlier plain bang-bang did. The team chose coasting over
  the reverse-power braking in the source description (November 11, 2026).
- Error within 80 ticks/s: `(kS + kV * target + kP * error) * 12 / batteryVolts`,
  clipped to 0..1 in the commanded direction. Velocities are motor ticks/s; battery
  voltage is the same 5 Hz lowest-valid-hub reading used for PIDF, with the 12 V
  reference as the fallback.
- The command never opposes the spin direction. A zero target (spin released,
  STOP, controller switch) is zero power and a FLOAT coast.

Initial values in `ShooterVelocityTuning`: `INITIAL_FUSION_KS` 0.05 (estimate),
`INITIAL_FUSION_KV` `(14 / 12 - kS) / 2520`, about 0.000443 (so kS + kV reproduces
full power at 5400 RPM and 14 V), and `INITIAL_FUSION_KP` 0.001 (estimate; 0.08
power at the 80 ticks/s edge). None is validated on this flywheel. Suggested order:
set kP to 0 and adjust kV until an unloaded mid-range target holds steady, check a
low and a high target and move kS if the bias differs between them, then restore
and raise kP in small steps for loaded recovery. If the zone alternates between
BANG-BANG and FEEDFORWARD while holding, the feedforward alone settles outside the
band; fix kV/kS first. Watch `Fusion power`, `Fusion bang-bang (0/1)` and
`Error ticks/s` in Dashboard. Because overspeed only coasts, a lowered target is
reached at the wheel's natural coast-down rate.

GP2 gain adjustments (D-pad, Y/A) are retained for switching back to PIDF but do
not affect fusion; fusion gains are edited in Dashboard. The competition TeleOps
always use fusion (no toggle); other OpModes start in PIDF and cannot select fusion. Switching resets minimum-feed-RPM and return-time
measurements. Compare both modes with identical target, battery and loading; last
return-to-tolerance time includes spin-up and is not a measured ball-to-ball
recovery time. These settings cannot be called optimized until loaded trials
confirm sag, overshoot, recovery time and shot distance. Migrate the winning
controller and gains into the competition TeleOps only after that comparison.

Simulation decision for the fusion controller: added the Mechanisms scenario
`benchFusionControllerSpinsUpHoldsAndFeedsThroughActualOpMode`, which runs the real
bench OpMode through the GP2 B switch, spin-up, hold and a staggered feed. The
flywheel fixture responds far faster than the real wheel and has no friction, ball
load or voltage sag, so it checks wiring and interlocks, not tuning. In that
fixture each 20 ms loop moves the wheel by more than the width of the band, so
spin-up alternates full power and coast three times before landing inside it; it
then holds 2836 RPM for a 2800 RPM target (+36 RPM, the frictionless fixture not
needing kS) inside the unchanged +/-100 RPM readiness tolerance. The scenario also
asserts that the commanded power is never negative.

First verify unloaded speed is steady at the intended target. During repeated
loaded trials, adjust P in small steps and compare RPM sag and overshoot. Reduce P
if oscillation increases. Adjust feedforward scale in small steps when the
steady-state speed remains consistently biased. Change one setting at a time.
If speed remains below target under sustained feed despite reasonable P/F, add I
in small steps; reduce it again if recovery overshoots or oscillates. Battery
sampling remains at 5 Hz, while the hub's velocity feedback runs continuously.
Unchanged gains are not rewritten on every battery sample; F is updated when its
computed value changes by at least 0.02 or the operator explicitly adjusts gains.
If the voltage-adjusted speed estimate is near 100%, lowering the target or
reducing the ball feed rate may be necessary; PID gains cannot supply additional
motor power. RT + LT now repeats the shared 0.20-s feed / 0.00-s recovery staggered cycle, with bench readiness required between attempts.

Hood home is shifted 10.5 hood degrees down from the original servo position 0.5,
to approximately 0.559155 (2:1 linkage, 355-degree servo travel). This is the previous
+12.5-degree command position (2.5 degrees below the previous 8-degree-offset home), now designated the team's +15-degree home reference.
The full 30-degree downward range ends near 0.728169. D-pad Up still increases
downward travel; Down returns toward home. This is a commanded angle reference,
not an independent measurement of projectile launch angle. Calibration displays averages without saving files. The unused historical export parser retains version checks for earlier references.

## November 1 robot observation and tuning follow-up

The team corrected the maximum to 5400 RPM (superseding the earlier 4700 RPM report).
The current observation is 5400 RPM maximum at a 6000 RPM command, 14 V, in both PIDF and
bang-bang modes, plus 100–200 RPM underspeed at operating targets blocking shooting.
The selected lower targets, instantaneous bang-bang power and loading conditions
are still pending. This is an installed-speed observation, not a catalog change.
The 6000 RPM command ceiling and 100 RPM readiness tolerance remain unchanged.
A 6000 RPM request with 5400 RPM feedback remains outside readiness.

Use the observation as an initial feedforward estimate: full-scale output divided
by the encoder velocity at 5400 RPM, then scaled by 14 / current battery voltage.
This assumes approximately linear voltage/speed behavior; it requires physical
tracking and overshoot validation at operating targets. It cannot increase a
full-power physical ceiling. Bang-bang continues to request full power whenever
its cached encoder sample is below target; PIDF gains do not affect that mode.

Simulation decision: use the existing bench readiness/feed scenario first with
old feedforward and the earlier, superseded 4700 RPM / 14 V fixture. That historical
baseline is retained in the verification matrix. Re-run Mechanisms and Competition
with the corrected 5400 RPM / 14 V fixture and updated feedforward. The motor response rate is still synthetic,
with no ball impulse, current limit, voltage sag or calibrated drag. The plant
uses the independent reported observation, not controller-desired speed. No
readiness assertion or production calibration flag is relaxed.
