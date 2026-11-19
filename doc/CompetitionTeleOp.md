# Competition TeleOp decisions - October 27, 2026

November 4 software integration: [final TeleOp audit](TeleOpConsistencyChecklist.md) records collection while aiming, GP2 auto offsets, independent status lights and the retained current controls. Bench tuning/interlocks remain explicitly documented diagnostic policies.

These settings apply to both Red and Blue competition TeleOps. Standalone diagnostics and primitive autonomous examples retain their own procedures unless explicitly listed here.

November 3 consistency review: the team chose to retain the newer working-tree
controls below (GP2 B feed-mode selection, continuous startup, 3500 RPM and explicit
manual feed bypass). This supersedes the earlier proposed B manual-aim toggle,
staggered startup and 3000-RPM standby. See the
[software consistency audit](TeleOpConsistencyChecklist.md)
for the completed integration changes, current controls, light assignments and
remaining calibration inputs. No physical testing is required for this audit.

The shared [drivetrain](Drivetrain.md) uses goBILDA 5000 bare motors with 20T:80T
gears and coaxial 30T:91T belt pulleys driving 104-mm goBILDA 30A mecanum wheels,
for 12.133333 motor rev/wheel rev. Both
wrappers use the same 28-tick motor encoder scale and 5800 RPM catalog-tested
no-load baseline. INIT reports the reduction and derived wheel specs; active
telemetry distinguishes motor ticks/s from wheel RPM.

November 5 drive correction: shared motor directions are now left FORWARD and
right REVERSE, swapping forward/back, both strafes and both turns in both wrappers.
GP1 left stick remains translation; right stick X remains rotation. While GP1 RT auto-aims,
rotation is capped so the turret can keep up (see below).

November 2 field rehome: press **GP2 D-pad Left** with the robot stationary in the
alliance corner opposite its pollen depot, rear and side against the walls and
front pointing inward parallel to the end wall. Blue uses the audience-right
corner, facing -X (180 degrees); Red uses the far-left corner, facing +X (0 degrees).
The nominal center poses are Blue `(1.566319, -1.670450 m, 180 deg)` and Red
`(-1.566319, 1.670450 m, 0 deg)`, using the existing 262.481-mm rear and 158.35-mm
side offsets. Each alliance wrapper defines its own `getFieldRehomePose()`.
This sets localization; it does not drive the robot to the corner. It works during
INIT and ACTIVE, once per press (including a button held across START), without
requiring a visible tag or turret hardware. Like the autonomous pose handoff, the corner pose is written
only after the Pinpoint has reported READY for 0.5 s (`PinpointOdometry.READY_SETTLE_SECONDS`) following
the INIT IMU recalibration (`CornerRehomeRequest`, 2026-11-11 audit): an INIT press during calibration shows
**Corner rehome: WAITING** and applies once settled; START discards a press still waiting; after START an
unsettled Pinpoint shows REFUSED and the press must be repeated (the robot may be moving, so it is never queued). It sets Pinpoint position and heading
without IMU recalibration, clears prior camera correction/capture history through
the existing reference ID, and requests a non-blocking Limelight refinement.
The turret points toward the nearest alliance hive while the camera collects
five distinct frames captured after the press, within a three-second window.
All usable mapped raised tags in each frame contribute, weighted by the existing
range/area quality estimate. Capture-time chassis/turret history compensates
latency. Frames require at least 100 ms of turret history stable within 0.5 degrees,
remaining stable through the latest sample. Tag height/plane, frame age, same-reference history, 150-mm correction
limit and 60-mm tag/frame agreement checks reject incompatible observations.
The result combines the camera estimate with the existing 20-mm corner placement
uncertainty; repeated frames are not treated as independent calibration evidence.
The wall-aligned heading is retained. The refined **Aiming field pose** is used
for shooting, while raw Pinpoint telemetry continues to show the corner-seeded
odometry frame. Normal periodic vision corrections resume afterward.

**Automatic aim and the turret dead zone.** The turret reaches -175 to +175 degrees; the rear 10 degrees is a
dead zone. While the goal bearing is inside it, a turret already in its rear half (beyond +/-90 degrees) holds the
limit on its current side; from the front half (for example just homed, or mid-unwind) it takes the limit nearest
the bearing (2026-11-10), so it ends closest to the goal and never reverses an unwind. Once the bearing
is 3 degrees past the zone on the other side, the turret unwinds the long way through home to keep aiming, so a
spinning robot never drives the turret through the dead zone and edge noise cannot cause repeated spins.
A full chassis turn still forces one long-way unwind; the turret cannot aim (and auto feed waits) during it.
The side decision uses the previous yaw *target* as its memory, not the measured yaw (2026-11-17). With the
measured yaw, a bearing jittering across the back flipped the target between +175 and -175 every loop while the
turret was still in its front half, and noise at the 3-degree edge reversed an unwind mid-travel, so the turret
jerked back and forth. Releasing auto-aim or the stick also keeps any learned encoder offset (2026-11-17); zeroing
it moved the turret by the offset and the correction moved it back 0.25 s later.

**Auto-aim edge-case fixes (2026-11-10).** From the auto-aim audit, shared by both competition TeleOps and Test: Auto Aim:
- *Cell switching hysteresis.* The selected cell changes only 5 cm past the field midline
  (`TeleOpFieldReference.SIDE_SWITCH_HYSTERESIS_METERS`). Before, a few millimeters across Y = 0 swung the aim
  25-65 degrees and restarted feed readiness, and weaving along the midline could repeat that every loop.
- *Encoder correction kept while tracking.* A learned external yaw correction now survives continuous target
  motion; only a target jump over 10 degrees in one update (cell switch, unwind, new target) or a lost home
  resets it. Before, it was dropped every 3 degrees of target motion, which (for a servo settling 4 degrees short)
  left auto aim outside the 3-degree shot tolerance 17% of the time at 2 deg/s and 90% at 5 deg/s.
- *Nearer dead-zone limit from the front half* (see above).
- *GP2 hood/RPM buttons while RT is held.* D-pad Up/Down and Y/A trim the automatic hood/RPM only when auto aim
  actually sets them (a calibrated shot solution, judged before trims so an out-of-range trim can still be
  undone). With the current uncalibrated profile they keep stepping the manual hood/RPM the shot is using,
  instead of changing hidden trims that would suddenly apply once a profile is installed. GP2 RS-X still trims
  yaw whenever RT is held.

**Automatic aim while turning (2026-11-06).** Three changes keep the turret on the goal while the chassis turns:
- *Same-loop servo write.* `TurretSubsystem.update()` now only reads sensors; every OpMode calls
  `applyYawCommand()` after auto aim / manual yaw set the target. Previously the servo received the previous
  loop's target, a full loop late.
- *Aim lead.* The servo command uses the pose predicted `TurretControl.AIM_LEAD_SECONDS` ahead (Pinpoint
  heading rate and fixed-frame velocity). Readiness and telemetry still use the bearing now, because the ball
  leaves now. This is actuation-delay compensation only; projectile lead remains removed.
- *Turn cap.* While RT is held, `AutoAimSpinLimiter` limits the GP1 turn command so the measured chassis rate
  stays at `MAX_CHASSIS_RATE_DEG_PER_SECOND` (driver view shows **Turn speed LIMITED**). It learns the
  stick-to-rate gain from Pinpoint, so battery level does not matter. Releasing RT restores full rotation.

Both constants are UNMEASURED starting values (25 ms, 300 deg/s). To measure them on the robot:
1. **Test: Turret**, hold full RS-X both ways: **Turret peak measured rate** is the loaded turret speed. Set the
   cap below it with margin (for example 80%).
2. **Test: Auto Aim**, localize, hold RT and turn the chassis by hand at a steady rate: **Turret lag (ms)**
   reports the residual lag and a **suggested lead**. Put that in `AIM_LEAD_SECONDS`; the residual should then
   read near 0 ms. The same lines appear on the GP2 Back diagnostics page in both competition TeleOps.
After the first field reference, odometry owns the pose and the Limelight only corrects consistent drift
(more than 4 cm or 1.5 degrees). A disagreement over 0.5 m or 8 degrees re-localizes the pose only after 0.5 s of
consistent multi-tag frames (2026-11-10); the driver view then shows **Camera pose check**. See
[Limelight diagnostics](LimelightGoalStateDiagnostics.md).

**Odometry pod directions (2026-11-13).** Auto aim trusts Pinpoint translation, so a pod counting the wrong way
mirrors that robot axis. On the robot this showed as the turret turning the *opposite* way while driving across the
hive (losing the goal for good once the camera looked away) and overshooting during fast in-place spins until the
camera drift correction pulled it back. A hand-push check found the X (forward) pod reversed; `X_POD_DIRECTION` is
now FORWARD (Y stays REVERSED). After any pod rework run **Test: Pinpoint Odometry Only**: GP1 A zeroes, then a
forward push must read +X, a left push +Y, and a CCW spin +heading with X/Y staying within a few cm.

Keep the chassis stationary until **Field rehome refinement** reports
**VISION REFINED** or **CORNER ONLY**. Chassis speed above 0.05 m/s or rotation
above 5 deg/s, manual GP2 turret steering, or GP2 X cancels refinement. Automatic
feeding waits while refinement is pending; manual feed/inhibit priorities remain.
With the measured 55-degree Limelight, the raised tags are below the image from
the corner, so expect **CORNER ONLY** after the three-second timeout.
Missing, inconsistent or rejected camera data times out to the known corner
reference, with an explicit telemetry status. Driving remains responsive.
These quality/uncertainty settings are software heuristics; the installed camera
convention, field map and mount must be physically validated to establish accuracy.
GP2 X still homes only the turret. Startup still supports tag acquisition anywhere;
no corner pose is assumed until the operator presses the button.

November 5 gate update: Both final Red/Blue TeleOps start CONTINUOUS: once a feed is authorized, the gate-open command and both full-power transfer commands occur together, without a gate delay. Automatic readiness and GP2 LT inhibit still apply; manual feed bypasses readiness. STAGGERED opens the gate, starts both stages together at 0.10 s (stage 1 used to wait for the 0.15 s gate-open midpoint; changed 2026-11-18), and closes the gate after a 0.20 s feed (0.30 s total open window). Recovery is 0.00 s. With the gate closed after a completed timed feed, stage 1 keeps running (past full) through the readiness wait while stage 2 is off; at the next opening stage 1 stops until both stages start the next feed together. Release, inhibit, invalid automatic solution, tracking loss or STOP cancels transfer feeding; tracking-loss recovery does not continue closed-gate feed. Mode changes reset the sequence. Updated 2026-11-15 with the on-robot tuned timing (0.10 s gate delay / 0.20 s feed / 0.00 s recovery), now shared by competition, autonomous, the bench test, transfer-only and shot-calibration OpModes through `ShootingConfiguration`; stage 1 timing and the closed-gate run use `StaggeredFeedTiming`.

The gate servo is wired directly to Expansion Hub servo port 3, bypassing the 6 V power injector. It receives immediate full-position targets without software ramping for maximum available speed. The 0.043 s/60-degree rating at 6 V is catalog reference only; actual direct-hub loaded speed is unmeasured.

November 2 collection overrides (both final alliance TeleOps): hold RB on either
gamepad to force the intake and transfer stage 1 forward, bypassing the full sensor
for loading. Hold LB to reverse only the intake and pause stage-1 loading; opposing
bumpers, including across controllers, stop collection. Release bumpers to resume
automatic sensor-based collection. Stage 2, gate commands, firing timing and shot
readiness are unchanged. Shooting requests, gate travel, recovery feeding and GP2 LT
retain priority over stage-1 loading overrides. No separate transfer controls were added.

October 27 localization verification passed all 159 unit tests (zero failures, errors or skipped tests), `:TeamCode:assembleDebug` and `:TeamCode:lintDebug` (0 errors, 14 existing warnings). New coverage includes acquisition from all 16 mapped tags at arbitrary headings, capture-time motion compensation, field-axis velocity rotation, rejected/duplicate observations, field-reference resets and continued tracking without camera visibility. This is desktop validation; no deployment or robot measurement was performed.

Desktop verification on October 26 passed all 153 tests (zero failures, errors or skipped tests), `:TeamCode:assembleDebug` and `:TeamCode:lintDebug` (0 errors, 14 existing warnings). Coverage includes correction hysteresis and persistent bounds, current-pose aiming during motion, 3-degree readiness, stage-2-off loading, 0.20 s gate-command delay and longer feed timing, all 16 tag coordinates and their fusion, and a 180-second no-camera run without an automatic-feeding timeout. This does not measure loaded servo stopping behavior or shooting accuracy. No deployment was performed.

| Item | Competition behavior | Intake + index + shooter test |
| --- | --- | --- |
| Flywheel | Starts at 3500 output RPM; Y/A steps 50 RPM | RT hold, initially 2800 RPM; Y/A steps 100 RPM; B stops motors |
| Full queue | After 0.5 s HIGH, intake stops (no reversal); stage 1 runs on 0.5 s, then holds until LOW | Same full-queue policy |
| Collection override | RB: intake + stage 1 forward past full; LB: intake reverse only, pause loading; both: stop collection | Existing bench controls and interlocks |
| Transfer loading | Stage 1 loads; stage 2 stays off until gate-open feeding | Same stage-2 rule |
| Feeding mode | GP2 B toggles continuous/staggered; starts continuous | GP2 RB selects bench continuous/staggered |
| Manual feeding | GP1 A or GP2 RT bypasses readiness; follows selected mode; continuous feeds immediately; GP2 LT inhibits | RT + LT uses selected bench mode |
| Staggered feeding | Automatic readiness, 0.10 s gate delay, both stages at 0.10 s, 0.20 s feed, 0.00 s recovery, stage 1 running while the gate is closed | Same shared timing; bench yaw checks |
| Turret release | Capture measured yaw once on stick/auto-aim release, discarding queued travel | Preserve diagnostic commanded-target hold |
| External yaw correction | Enabled: start above 3 degrees after 250 ms at <=3 deg/s; stop adjusting below 1.5 degrees; kept while tracking, reset by a >10-degree target jump | Disabled by default |

The staggered gate-open window is 0.30 s. Continuous commands both transfers immediately.
See [shared transfer sequence and diagnostic controls](TransferAndShooting.md).

GP1 RT prepares odometry-based yaw aiming and calibrated hood/RPM selection. Hold RT+LT to request automatic shooting; RT alone keeps the gate closed and transfer feeding off. Releasing either trigger cancels automatic shooting. GP1 LT alone does not feed; hold GP1 A for the manual readiness override. The nearest cell of the selected alliance is chosen continuously: the far cell once field Y exceeds +5 cm, the audience cell once Y is below -5 cm, and the previous selection within that midline band. Switching cells resets automatic feed readiness. D-pad left/right no longer selects a competition target. The robot does not classify hive state in this mode; drivers decide whether the selected cell is raised and when to request shooting. Manual yaw remains available when RT is released.

The field origin is its center; +X points toward blue and +Y toward the far/scoring-table end. Both competition TeleOps can initialize at an arbitrary autonomous finish position and heading. They do not automatically assign a corner pose; GP2 D-pad Left explicitly sets the alliance rehome pose. Pinpoint resets/calibrates during INIT while stationary, providing local motion until a field reference exists: the autonomous pose handoff (below), a corner rehome, or Limelight acquisition.

**Autonomous pose handoff (2026-11-10).** Competition autonomous (`BioBuzzRobot`, both alliances) saves its READY, field-referenced Pinpoint pose every loop in `AutoPoseHandoff` (RAM only, this Robot Controller app session). At TeleOp INIT the stored pose is consumed and, if it is from the same alliance and at most 60 s old (`MAX_AGE_SECONDS`), written to the Pinpoint once it has reported READY for 0.5 s after its INIT IMU recalibration. The driver view shows **Auto pose handoff**: APPLIED (with the pose), Waiting, Ignored (other alliance / too old) or None. Auto aim then works from the first ACTIVE loop without a visible tag, and Limelight drift checks continue as usual. It is never applied after START (the robot may be moving). A new autonomous INIT or an autonomous localization fault clears it; an app restart loses it. Match rules keep the robot still between periods, but practice does not: if the robot was moved, press GP2 D-pad Left (corner rehome), which also discards a pose not yet applied. If the camera can see two or more raised hive tags, a pose that wrong is also re-localized from the camera after 0.5 s of consistent frames (**Camera pose check : RE-LOCALIZED**); without a view of the tags, the corner rehome is the fix. Acquisition runs during INIT and continues after an early START, whether or not RT is held. The homed turret and three distinct, consistent frames of a nominal raised hive tag are required. Tags 30-45 from either alliance/cell can localize; the firing target remains the selected alliance's nearest cell. The existing corner dimensions also define the explicit alliance rehome poses above. Pinpoint pod offsets remain +32/-74 mm.

Opening X is +/-0.32385 m (blue/red). Drawing-derived nominal aim height is 1.51257 m (opening mid-height), tag height 1.2676124 m, opening Y magnitude 0.38322519 m and tag-row Y magnitude 0.31383184 m. `FieldGeometry` and `HiveTagGeometry` share the TU06 dimensions. The Y values retain the approved half-length projection, whose exact pivot-to-surface transform still needs CAD/field verification. See [field reference](FieldReference.md).

At acquisition, the mapped tag's outward normal establishes chassis field heading, and its metric position establishes chassis field position, accounting for the measured camera pitch/offset, turret encoder angle and capture-time odometry history. Three observations must agree on the local-to-field transform within 0.15 m and 8 degrees; stale/missing capture history, poses outside the field, incompatible tag height/plane and unusable tag poses are rejected. The initial solution is absolute, so the subsequent 0.5 m correction limit does not prevent acquisition far from local zero. Position and velocity vectors are rotated into field axes together; corrections never create velocity spikes by differentiating a reset.

After acquisition or explicit corner rehome, Pinpoint tracks relative movement and heading and Limelight supplies periodic translation corrections (minimum 0.5 s between accepted corrections, 35% correction gain). These use either alliance/cell's mapped raised tags, reject position jumps over 0.5 m and preserve the heading alignment. Pinpoint is only reseeded when the operator requests corner rehome. Changing the nearest shooting cell or rehoming the turret preserves the field transform. An odometry reset discards it; local zero requires tag acquisition again, while corner rehome supplies an explicit field reference. Vision remains independent of RT and never directly centers the turret or approves hive state for firing.

Before tag acquisition or explicit corner rehome, telemetry reports WAITING and automatic field aiming/feed remain inhibited; robot-centric driving, manual turret control and manual feeding remain available. Point the camera at a raised hive using the chassis or GP2 yaw control if no suitable tag is visible. Once localized, loss of camera visibility leaves odometry tracking active. The team requests a no-drift assumption: no elapsed-time uncertainty growth, camera-loss penalty or uncertainty cutoff blocks feeding. Uncertainty remains telemetry. Stale/invalid odometry still rejects automatic aiming/feed. No cross-OpMode saved pose is assumed.

Yaw tracking works from odometry even before the shot table is entered. Distance-based hood/RPM needs the empirical profile; release timing does not gate automatic feeding (current values are used as-is). Send distance, average flywheel-output RPM and average launch angle (or clearly labeled hood travel). Linear interpolation is already implemented between rows; out-of-range requests are rejected. Projectile lead, flight-time data and dedicated route firing flags have been removed. Range, yaw, hood and RPM still update from the current pose while moving. Calibration only displays session averages on the Driver Station; stopping discards the in-memory session and no files are saved.

The Axon MAX+ positional controllers perform normal motion. Full manual stick requests the rated 6 V shaft speed, 60/0.115 = 521.74 deg/s at 1:1; steady speed is linear outside the 0.08 deadband. Two short 10-ms velocity filters ease command acceleration changes; a 5-ms position-command filter softens destination changes and stops. Release captures the encoder once, not continuously. External correction applies bounded positional adjustments, allowing a 250-ms settled interval between them, until error drops below 1.5 degrees. The successful offset is retained to hold position and is capped at 25 degrees. It does not run a continuous PID. Target changes above 10 degrees in one update reset qualification; smaller odometry changes do not constantly restart it. Actual loaded acceleration, jerk, speed and stopping distance depend on the servos' internal controllers and mechanics. Competition feed readiness accepts yaw error <=3 degrees and the existing hood/RPM checks. There is no chassis-speed, turret-speed, acceleration or stationary qualification gate. The 150-ms angle/RPM readiness window, timed recovery and physical gate travel remain. Drivers decide when to take the risk of firing while moving.

Gate direction, depollinator direction and hood home retain the installed implementation: gate 0.5 closed to 0.2 open with REVERSE direction (90 degrees CW); depollinator 30 to 165 degrees (135 degrees CW); hood home approximately 0.559155 with 30 degrees downward travel to approximately 0.728169. The turret-only test is unchanged. Pollen collection and general autonomous work remain deferred primitive examples. The match-autonomous adapter now uses odometry for both navigation and shooting, retains the route-selected target, and performs no camera correction or direct visual aiming during the route. It continuously updates aim/RPM while navigating, and requests feed only in scoring/preload states without an extra motion gate. No end-of-route relocalization has been added without an approved route and verified landmark map.


## Remaining calibration and intentional differences

The known competition behavior mismatches in this review are resolved, including the former time-based uncertainty cutoff. This is desktop verification, not a claim that the installed robot has no defects. Final staggered feeding timings/tuning are pending from the team.

- Supply at least two measured distances covering the intended range, with average flywheel-output RPM and labeled launch elevation or hood travel. The competition profile is still an uncalibrated example, so automatic calibrated feeding remains inhibited.
- Confirm loaded gate/feed/recovery timing. Current staggered settings (shared by every OpMode, tuned on the robot 2026-11-15) are 0.10 s after the gate command, 0.12 s feed-to-exit estimate, 0.20 s feed with both stages (0.30 s total open), 0.00 s recovery and stage 1 running while the gate is closed. Continuous starts transfers immediately. These are still starting values, but they no longer block automatic firing (the `RELEASE_TIMING_MEASURED` gate was removed 2026-11-15).
- Check that Limelight pipeline 0 uses 36h11, 3.25-in. individual tags and returns valid metric tag poses. Java uses the documented 337-mm-high, 55-degree-upward camera mount (lens axis; front glass team-measured 55 degrees from vertical on 2026-11-05); camera settings were not remotely changed. On the field, INIT each competition TeleOp at several known positions/headings, with the robot level and stationary during Pinpoint calibration. Confirm LOCALIZED and the reported aiming field pose, including heading; repeat with each visible hive and several turret angles. Cover the camera and move/turn the chassis, then uncover it and check that tracking and bounded corrections continue. Verify waiting behavior with no tag, and that lowered/moving cells cannot establish the nominal raised reference. The field map, tag orientation convention and capture timing still need this physical validation.
- Opening/tag Y coordinates remain drawing-model approximations until the pivot-to-surface CAD transform or an installed-field reference is available. Relative tag spacing and the source drawing dimensions are already entered; no need to remeasure sixteen unrelated tags.

The combined intake/shooter bench test intentionally retains 2800-RPM startup, its own RPM adjustment steps, B-to-stop and 4-degree/3-degree-per-second yaw readiness. Competition retains 3500 RPM, no B stop and angle-only 3-degree feeding readiness. Both stop (never reverse) the intake on confirmed full. Turret-only diagnostics, pollen pickup tuning and match routes remain deferred as requested.

## Desktop verification and human review

Both actual alliance wrappers run in the optional local SimLoop suite. Use
`.\tools\verify-teleop.ps1 -Sim -Scenario Competition,Sensors` or drive one with
`.\tools\review-teleop.ps1 -OpMode RedMecanumTeleOp` / `BlueMecanumTeleOp`.
See [record/replay controls](../simloop/README.md) and [the evidence matrix](TeleOpVerificationMatrix.md).
The normalized-camera and synthetic-calibration fixtures are explicit test conditions, not authorization
to change production calibration or evidence that the installed Limelight axes have been validated.

Verification result (November 5, 2026): 40 affected controller tests and 18 existing Competition/Mechanisms simulation tests passed with zero failures/errors. This includes both real alliance wrappers and the isolated gate/transfer/bench checks. `:TeamCode:assembleDebug` passed. `git diff --check` passed. Full unit-suite, lint, FullSim and robot deployment were omitted for this focused change. Physical direct-hub speed and loaded clearance were not verified.

## Driver Station telemetry

Both competition TeleOps show a short driver view by default, most important first:

| Line | Meaning |
|------|---------|
| Shot | OFF, AIMING (hold LT to fire), WAITING FOR AIM / HOOD / RPM, READY, MANUAL FEED or BLOCKED (GP2 LT), plus the firing state; a FAULT or INHIBITED state also shows its reason |
| Flywheel (RPM) | Measured / target output RPM and OFF, SPINNING UP or READY. With RT released the target is the manual RPM. Regulated by bang-bang + feedforward fusion with untuned initial gains (see ShooterVelocityTuning.md) |
| Launch elevation (deg) | Commanded launch angle above horizontal |
| Turret | HOMED with measured heading, or the homing state and reason |
| Battery (V) | Shooter-hub battery voltage |
| Field localization | LOCALIZED, retained/stale, or WAITING with the vision reason |
| Camera pose check | Only around a camera/odometry disagreement over 0.5 m / 8 deg: CAMERA DISAGREES (size and confirming progress, or why it cannot confirm) for 1 s, RE-LOCALIZED (correction size) for 3 s |
| Auto pose handoff | INIT only (diagnostics while active): APPLIED with the pose, Waiting, Ignored with the reason, or None |
| Corner rehome (GP2 D-pad Left) | INIT: WAITING for a settled Pinpoint, then APPLIED. ACTIVE: shown only while a press was refused or discarded at START |
| Field rehome refinement | Shown after a GP2 D-pad Left corner rehome: VISION REFINED or CORNER ONLY with the reason |
| Auto trims | Yaw / hood / RPM offsets, shown while aiming or while any trim is non-zero |
| Pose (in, deg) | Pinpoint X / Y / heading and EXPLICIT FIELD or LOCAL; a Pinpoint warning appears only when it is not READY |
| Transfer, Collection | Transfer state (LOADING, FULL, FEEDING SHOT, ...) and intake override and power |
| Feed mode, Depollinator | CONTINUOUS/STAGGERED (GP2 B) and DEPLOYED/HOME |
| Slide (GP2 LS) | DOWN, MOVING_UP, UP or MOVING_DOWN with position / target (deg), or FAULT with the reason and the direction the next press moves; UNAVAILABLE if not configured |
| Claw (GP1 B) | OPEN or CLOSED; UNAVAILABLE if not configured |
| Loop (Hz) | Control loop rate |

### Linear slide and claw (November 7)

Both competition TeleOps share the slide controls. **GP2 left-stick click** toggles the side
linear slide between fully down and fully up. **GP1 B** toggles the claw open/closed (the claw
starts closed). The claw toggle always commands `claw_servo`; with the shot-blocker end effector
mounted it has no effect. The slide must be **fully down at INIT**: the Axon analog feedback only
covers one turn, so turns are counted from that position. The slide is not powered during INIT.
A stall (under 5 degrees of progress in 0.5 s), wrong-direction movement or overall timeout cuts
slide power and shows FAULT. The button toggles, so the next press reverses the faulted move (a fault while
raising lowers on the next press); telemetry names the direction the next press will move. A missing slide or claw
shows UNAVAILABLE instead of stopping the OpMode. Autonomous does not use the slide.
Travel (`LinearSlideController.MAX_TRAVEL_DEG`, 180-degree placeholder) and claw positions
(`SlideClawSubsystem`, 0.40 open / 0.60 closed placeholders) must be measured with
`Test: Linear Slide + Claw`.

Missing status lights are listed only when one is absent. INIT shows alliance, turret, localization, pose and the GP2 X / D-pad Left / Back reminders. Press GP2 Back (INIT or ACTIVE) to add the full subsystem diagnostics below the driver view; press again to hide them.
