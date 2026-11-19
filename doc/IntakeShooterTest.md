# Intake, index and shooter bench test

November 4 software integration: [final TeleOp audit](TeleOpConsistencyChecklist.md) records collection while aiming, GP2 auto offsets, independent status lights and the retained current controls. Bench tuning/interlocks remain explicitly documented diagnostic policies.

**Test status: complete October 26, 2026.** The operator confirmed all transfer and intake tests complete, and turret rotation, shooting/angle and gate tests good. See [mechanism test sign-off](HardwareReviewImplementationStatus.md#mechanism-tests--complete-october-26-2026). Numerical calibration remains separate.

Select **Test: Intake + Index + Shooter** in the Driver Station's Test group.
It initializes the intake, both transfer motors, full sensor, gate, turret servos,
turret encoder and flywheel. It does not initialize the drivetrain, Pinpoint,
Limelight or depollinator. Use Gamepad 1.

The turret homes during INIT to robot-right (90 degrees), with the hood at its
highest launch elevation (+15 degrees). Clear its travel before INIT. After START,
release B, both bumpers and both triggers once to enable motor controls. The
flywheel requires RT; intake and stage 1 begin automatic loading once armed.

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

| Control | Action |
| --- | --- |
| Bumpers released, armed, no feed request | Automatic intake/index: forward on LOW; intake stops (no reversal) after 0.5 s HIGH, stage 1 runs on 0.5 s then stops; LOW resumes forward/loading |
| Hold RB | Force intake forward regardless of sensor or feed request; index still follows its sensor/feed controls |
| Hold LB | Reverse intake only; transfer stops and gate closes |
| Both bumpers | Stop intake and loading; inhibit feeding |
| Hold RT | Spin flywheel at selected RPM; release stops powering it |
| Hold RT + LT | Staggered: qualify readiness and cycle; continuous: immediately open gate and run both transfers at full power; release either trigger to stop feeding and close gate |
| GP2 RB (tap) | Toggle STAGGERED (default) / CONTINUOUS shooting |
| Hold GP2 LB + D-pad up / down | Increase / decrease recovery pause by 0.05 s (0..5 s) |
| Hold GP2 LB + D-pad right / left | Increase / decrease staggered feed duration by 0.05 s (0.05..5 s) |
| Hold GP2 LB + Y / A | Increase / decrease stable-readiness delay by 0.05 s (0..5 s) |
| Hold GP2 LB + X | Reset feed/recovery/readiness timing to 0.20/0.00/0.15 s; retain shooting mode |
| Tap Y / A | Increase / decrease flywheel output RPM by 100, clamped to 0..6000 RPM; starts at 2800 RPM |
| D-pad up / down | Lower / raise launch elevation by 0.5 degrees per press |
| Right stick X | Rotate turret; release holds its target |
| X | Rehome yaw and hood when bumpers and triggers are released |
| B | Stop intake, transfer and flywheel commands and close gate; release B, bumpers and triggers to resume |

Shooting mode and timing can be changed after START. Each button acts once per press;
holding GP2 LB routes its tuning buttons to shooting timing instead of PIDF tuning.
Any actual mode/timing change stops motors, closes the gate and requires releasing GP1
B, bumpers and triggers before resuming. Telemetry shows the mode and all three times
in seconds. Settings last for this OpMode session only.

STAGGERED shooting pauses loading and requires a homed, settled turret and hood, plus measured
flywheel speed within 100 output RPM of the selected target. Bench yaw settling accepts
up to 4 degrees of error (the retained bench-specific tolerance) and 3 degrees/s
of measured motion; the competition auto-aim's tighter 3-degree requirement does
not apply to this manual test. Release the yaw stick before starting feed. Initial
readiness must hold for the configured 0.15 seconds. Feed then waits for the gate's
0.10-second command-to-feed delay. Both stages then feed together for 0.20 seconds
before the gate closes and stage 2 stops. Default timeline,
relative to the gate-open command (values tuned on the robot):

| Time (s) | Gate | Stage 1 | Stage 2 |
| --- | --- | --- | --- |
| 0.00 | Open command | Off | Off |
| 0.10 | Open | Full forward | Full forward |
| 0.30 | Close command | Full forward | Off |

The default recovery pause is 0.00 seconds. After a completed timed feed the gate is
closed and stage 1 keeps running (even past full) through any recovery pause and the
readiness wait, to advance the next ball while stage 2 stays off. Stage 1 stops again
during the next gate delay and restarts with stage 2. This applies only while
RT + LT are held after a completed timed feed; loss of readiness mid-feed, B, LB, STOP
and trigger release stop it. Holding RT + LT
repeats the cycle after readiness requalifies and the gate reopens. Loss of readiness
closes the gate and enters recovery. Releasing either trigger, pressing B or LB,
losing home, or setting RPM to zero stops feeding. Software counts timed attempts,
not launched balls. These are the default STAGGERED timings; the controls above can
adjust feed duration, recovery pause and stable-readiness delay. Stage 1 always starts
with stage 2 at the start of the feed. The gate delay is the shared 0.10 seconds. Commands run on the next control-loop iteration;
loaded ball travel and servo movement are not measured by these timers.

CONTINUOUS mode immediately commands the gate open and both transfer stages to
full forward power (1.0) while both triggers are held. It bypasses homing/aim/RPM
readiness checks, gate delay, feed duration and recovery pauses. Releasing either
trigger closes the gate and stops both transfers. B, LB/reverse, STOP and settings
changes retain priority; startup and settings changes still require releasing the
motor controls to re-arm. A zero selected RPM prevents feeding. Timing settings
are retained for switching back to STAGGERED. The flywheel still uses the selected
RPM/controller; continuous mode's full-power command applies to the transfers.
Competition red/blue, autonomous, transfer-only and calibration callers now use the same
robot-tuned defaults (0.10 s gate delay, 0.20 s feed with both stages, 0.00 s recovery, stage 1
running with the gate closed), defined once in `ShootingConfiguration`; only the session
adjustments above are bench-specific.

Simulation decision for the revised sequence: run `Mechanisms` and `Competition`.
The existing actual-bench cycle checks the shorter stage-2 gate delay and delayed
stage-1 advancement. `benchContinuousFeedsImmediatelyUntilTriggerReleaseOrStop`
checks the real OpMode's GP2 mode selector, immediate full-power transfer commands,
either-trigger release and stop/re-arm behavior. Both competition wrappers are
included as regression checks for the opt-in helper changes. Deterministic controller
tests check timing boundaries, custom timing/midpoint adjustment, readiness loss in
staggered mode and readiness bypass in continuous mode. Loaded ball spacing, actual
gate clearance and flywheel recovery still require physical testing.

Verification of this bench-only sequence (November 1, 2026):
`verify-teleop.ps1 -Sim -Scenario Mechanisms,Competition` passed 191 unit tests
(19 bench-controller tests), 13 simulation tests, debug APK packaging and lint
(0 errors, 14 existing warnings). No deployment or loaded ball-clearance measurement
was performed. Retained scenario traces are under `TeamCode/build/sim/teleop/`.

The flywheel starts with the FTC SDK's encoder velocity PIDF controller in
`RUN_USING_ENCODER` mode. **Tap GP2 B** to switch between PIDF and the bang-bang +
feedforward fusion controller (this replaced the earlier plain bang-bang option).
Switching stops the motors and closes the gate; release GP1 B, bumpers and triggers
to re-arm, then hold RT again. A held GP2 B toggles only once. The selected RPM and
both controllers' tuning are retained, while comparison measurements reset.

Fusion uses `RUN_WITHOUT_ENCODER` (encoder readings still work) and runs every main
loop, including recovery, using one cached velocity sample per loop:

- **Speed error above 80 motor ticks/s (about 171 flywheel RPM):** bang-bang. Full
  power when slow, zero power (FLOAT coast) when fast. There is no reverse braking.
- **Error within 80 ticks/s:** feedforward, `kS + kV * target + kP * error`, scaled by
  `12 V / battery`. `kS` overcomes static friction, `kV` holds the target speed and
  `kP` is a small correction for the remaining error.

Starting values are `kS` 0.05, `kV` about 0.000443 and `kP` 0.001 (motor power at
12 V; `kV` and `kP` per motor tick/s). `kS` and `kP` are estimates and `kV` is
solved from the 5400 RPM at 14 V observation; none is tuned on this flywheel. Edit
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
are 6000 RPM. The motor encoder remains 28 counts/rev. The starting target remains
2800 flywheel RPM (now also 2800 motor RPM). Calibration displays flywheel output RPM. It does not save session files.

Turret PWM direction is reversed for the reported mounting so the right stick
moves the turret in the corresponding direction. The external encoder correction
is OFF by default to prevent an extra software trim after stopping. The Axon
servos still hold position using their internal controllers, and the Through-Bore
encoder still reports angle/rate and qualifies initial feed readiness. Check that
physical clockwise motion increases the displayed robot heading. If it does not,
verify encoder sign before enabling external correction. Only `Test: Turret` uses
B to toggle optional correction; B in this combined test remains the motor stop.

If RT spins the flywheel but RT + LT does not feed, read **FEED STATUS** at the top
of telemetry. It identifies missing trigger input, a held LB, required homing,
flywheel RPM error, turret settling or hood travel. Actual/selected RPM and yaw
error/rate appear directly below it. A spinning flywheel alone does not establish
RPM readiness: its encoder cable must be connected to encoder port 0. Release LB
while firing. In STAGGERED mode, if home is required, release the motor controls, press X, and wait
for homing to finish before trying again. During an active feed, FEED STATUS shows
STAGGERED FEED or CONTINUOUS FEED while feeding and recovery/readiness reasons between attempts.

**Transfer investigation (October 31, 2026):** The reported symptoms are no stage-1
motion with RB and no two-stage feed with RT + LT. All transfer callers were traced:
this bench test, the shared red/blue competition loop, transfer-only test, guided
shot calibration and autonomous output coordinator. Each applies transfer outputs
in its active loop; no missing update call was found. The actual bench OpMode also
loads and feeds in SimLoop with an empty sensor signal and modeled turret/flywheel
feedback. This does not reproduce or resolve the reported physical failure.

The new **TRANSFER STATUS** line shows this loop's commanded stage-1/stage-2 power,
loading/feed/full-hold reason and digital sensor state before verbose diagnostics.
**Transfer Stage 1 Device** and **Transfer Stage 2 Device** show the configured motor
names and SDK connection information; compare those with Expansion Hub ports 2/3.
With controls armed, RB alone and a LOW sensor should show S1 1.00,
S2 0.00 and LOADING. During qualified feed, both should show 1.00. These are commands,
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
Add more balls only after that works. Stage 2 stays off during loading; both transfers feed forward during qualified feeding.
The full sensor must remain HIGH for 0.5 seconds to stop the intake (it does not
reverse). Stage 1 runs on another 0.5 seconds to seat the fourth ball, then holds; stage 2 stays off. Pause, feed, or shutdown cancels the run-on. LOW resumes loading
immediately; a new continuous 0.5 s HIGH starts a new run-on. Check sensor
placement and that this delay does not cause a jam. Full does not prevent shooting.

B removes motor commands; the flywheel coasts and a ball already in the shooter
can still exit. Stop the OpMode and wait for motion to stop before clearing jams.
STOP or a loop exception attempts to stop every motor and close the gate.

See [shared motor baseline, PIDF tuning and hood reference](ShooterVelocityTuning.md) for the 6000 RPM target cap, GP2 live tuning controls, and calibration compatibility.
