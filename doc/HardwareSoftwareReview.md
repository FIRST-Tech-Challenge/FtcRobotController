# BioBuzz 10100 hardware and software review

> Historical review/plan: October 26 decisions supersede the moving-shot solver, flight-time data, motion-based firing gates, hive-state gating in competition TeleOp and calibration-file saving described below. These features have been removed from the competition workflow. See [current competition behavior](CompetitionTeleOp.md), [configuration](SoftwareConfiguration.md) and [tag map](FieldReference.md) for the implemented behavior.


Historical review: the October 26 [competition decisions](CompetitionTeleOp.md) supersede competition intake, target/state selection, turret release/correction, initial RPM, feed controls and calibration storage described below. Diagnostic behavior remains separate.

Reviewed October 22, 2026, branch `V1`, commit `6e58ddf`. This is a source review and desktop verification. No robot was connected, moved, or deployed to, and physical testing is deferred.

The October 23 [implementation plan](HardwareReviewImplementationPlan.md) covers these findings and the subsequent decision to use independent Limelight 3D tag observations for raised-state detection. Exact field CAD coordinates are not a prerequisite for that detector; the existing field-localized planner still has separate map requirements.

The debug APK builds and all 81 existing tests pass. The review nevertheless identified eight software issues and one documented hardware-port concern. Several automatic features remain deliberately unavailable until configuration is supplied. Robot control source was not changed during the initial review. Subsequent team clarifications supplied the servo injector, complete C270 mount, high-goal center/plane dimensions and Pinpoint connection; the confirmed mount and goal dimensions are now in configuration. The findings below describe the original review baseline; the October 23 software fixes and current verification are recorded in [HardwareReviewImplementationStatus.md](HardwareReviewImplementationStatus.md).

**Verified results**

| Check | Result |
| --- | --- |
| Scope | All 51 TeamCode production Java files; all 12 test/support Java files; mechanism documentation, Gradle configuration, manifests and camera calibration resource. SDK registration/build integration was inspected; this is not an audit of every bundled SDK implementation or disabled sample. |
| `gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` | PASS with Android Studio Java 21: 81 tests, zero failures/errors/skips; debug APK packaged. |
| `gradlew.bat :TeamCode:lintDebug` | 0 errors, 14 warnings: 12 native-library alignment warnings and two unused resources. These do not identify a TeamCode motor-control failure. |
| Additional desktop probes | Seven assertions reproduced the stationary-threshold issue, growing uncertainty despite accepted vision, delayed visual aiming error, and the difference between normal servo limits and raw diagnostic endpoints. |
| Probe source | [ReviewProbes.java](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/build/hardware-review/ReviewProbes.java), under ignored build output. It constructs no hardware and exercises the current compiled classes. |
| Existing test report | [Unit test report](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/build/reports/tests/testDebugUnitTest/index.html) |
| Lint details | [Lint results](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/build/reports/lint-results-debug.txt) |

**Mechanism facts confirmed by the team during this review**

- The first/top ball rests against the gate; balls two and three follow; the fourth is nearest the intake.
- The digital full-queue sensor detects the fourth ball in transfer stage 1.
- A closed gate prevents any ball reaching the flywheel regardless of transfer motion.
- Opening the gate with both transfer stages stopped does not release a ball. Both stages must run to funnel the balls into the flywheel.
- Shooting requires both stages at full power in their configured directions. Ordinary loading runs stage 1 forward with stage 2 stopped until full.
- The depollinator has no interference with the turret or chassis.

The implemented sequence matches these requirements: stage 1 FORWARD and stage 2 REVERSE (flipped 2026-11-18: motors wired backwards), both at power 1 during feeding. The gate opens first; both stages wait for the 0.20-second command-to-feed delay, then feed forward together for 0.50 seconds with the gate still open. Timed attempts then close the gate and stop both stages for 0.35 seconds of recovery; manual override feeds continuously while held after gate travel. During ordinary loading, stage 2 stays off while stage 1 runs. The full sensor confirms after two continuous HIGH seconds; a LOW clears pending confirmation, while a confirmed full queue holds until feeding begins. Operator feed inhibit closes the gate and stops both transfer stages. See [TransferController:8](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TransferController.java:8), [TransferSubsystem:16](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TransferSubsystem.java:16), [FiringCoordinator:48](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/FiringCoordinator.java:48), and [MecanumTeleOp:184](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumTeleOp.java:184).

An open gate during the deliberate pre-feed opening interval is necessary for this sequence. No transfer logic change is required for the team's clarification.

**Software findings and hardware-design concern**

1. **[P1] The all-servo diagnostic bypasses the installed mechanism limits.**  
   [ServoZeroCalibrationOpMode:64](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ServoZeroCalibrationOpMode.java:64) sends 0 or 1 to every configured positional servo when X or Y is pressed. The normal hood range is approximately 0.331–0.5, and normal turret yaw is approximately 0.021–0.979. The diagnostic therefore requests travel outside both bounds, including the turret's excluded sector, and simultaneously moves the gate and depollinator. Its midpoint is also not the depollinator's home. This is a source-proven risk when the OpMode is run with linkages attached; no collision was tested. Restrict the installed-mechanism diagnostic to subsystem limits, or make raw endpoint calibration a separate, clearly identified procedure for a selected disconnected mechanism.

2. **[P1] A missing turret name can select a servo on the wrong hub.**  
   [TurretSubsystem:450](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java:450) falls back to searching all servos by port number, without checking which controller owns the port. If the intended Expansion Hub device is missing and another hub has a single servo on that number, it is silently selected, reconfigured and commanded. The ambiguity exception catches two matches, but not one wrong match. Require the explicit name or validate a specific controller and port. This affects the fallback path, not a correctly named mapping.

3. **[P2] Fallback auto-aim applies delayed camera error to current turret position.**  
   [ShotCoordinator:55](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotCoordinator.java:55) uses the current encoder angle plus a tag angle captured earlier. The calibrated pose path has history compensation, but this fallback does not. It is the path currently used while shot calibration is missing. A deterministic example: a frame captured at yaw 0 sees a target at yaw +10; by processing time the turret has reached +10, yet the helper requests +20. Chassis rotation adds another error. Reconstruct the bearing using capture-time turret/chassis orientation, or constrain fallback tracking to conditions where those changes can be bounded. The desktop probe reproduces the 10-degree overshoot command; actual oscillation was not tested.

4. **[P2] Automatic shooting eventually locks out despite fresh accepted vision.**  
   [PinpointOdometry:96](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PinpointOdometry.java:96) makes base position uncertainty equal to 0.02 + 0.001 × seconds since field reference. [PoseHistory:55](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PoseHistory.java:55) adds observation-age uncertainty, while accepted observations only change translation and their timestamp. They never reduce the base uncertainty. The configured limit is 0.15 m, so after roughly 130 seconds from reference assignment, even continuous accurate vision cannot restore eligibility. INIT waiting time counts. A desktop probe accepted an accurate 50-ms-old observation at 131 seconds and still rejected the shot. Update uncertainty consistently with accepted observations and their quality, without merely removing the validity check. This becomes active after shooting configuration is enabled.

5. **[P2] Stationary-only calibration requires unrealistically exact zero motion.**  
   [ShotPlanner:93](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotPlanner.java:93) treats chassis speed, chassis turn rate, or turret rate above 0.000001 as moving. The calibration workflow produces rows with unknown flight time, so even 1 mm/s of reported chassis motion rejects them with “Moving shot needs measured flight time.” The autonomous coordinator separately considers up to 0.03 m/s and 0.03 rad/s stationary, so the two layers disagree. Use a shared, explicit stationary qualification with suitable filtering/stability, and preserve the requirement for measured flight time outside that stationary envelope. The probe verifies ideal-zero acceptance and 1-mm/s rejection. Actual sensor noise remains unmeasured.

6. **[P2] Transfer tuning constants do not control the transfer controller.**  
   [TransferSubsystem:18](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TransferSubsystem.java:18) exposes `TRANSFER_POWER` and `FULL_CONFIRM_TIME_SECONDS`, but [TransferController:11](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TransferController.java:11) hard-codes 2 seconds and powers 1/0. Changing the public constants will not change behavior; changing the time only changes the displayed denominator. Use one authoritative configuration. The team's requested present behavior is already full power, so current default outputs are correct.

7. **[P2] “Zero encoder here” also starts an unexpected move toward home.**  
   [TurretTestOpMode:74](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretTestOpMode.java:74) calls `zeroEncoderAtCurrentPosition()`, which invalidates homing. On the same loop, [TurretSubsystem:322](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java:322) sends servo position 0.5 for every non-READY state, including UNHOMED. Thus X is not solely a relative-zero operation: away from home, it commands a homeward move, without starting a measured homing cycle. Separate UNHOMED/FAULT behavior from an explicitly requested MOVING_HOME state, or label and implement this action as a rehome. This is distinct from the documented limitation that a relative encoder cannot prove absolute home.

8. **[P3] INIT advertises controls that are not processed until START.**  
   [MecanumTeleOp:79](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumTeleOp.java:79) displays GP2 X for rehome and D-pad left/right for target selection, but the handlers only exist in the active loop at [MecanumTeleOp:162](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumTeleOp.java:162). An INIT homing fault therefore cannot be retried with the displayed control. Process these controls during INIT with edge detection, or remove the misleading INIT instruction.

9. **[P1 hardware-design concern] The high-resolution turret encoder is assigned to a software-counted port.**  
   [TurretSubsystem:27](C:/Users/preco/Documents/github/BioBuzz10100/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TurretSubsystem.java:27) documents the REV Through-Bore V1 on Expansion Hub encoder port 1, with 8192 counts/rev and 2.7× gearing. [FIRST's hub guidance](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-hub-tips/tech-tip-hub-tips.html) recommends ports 0 or 3 for this encoder because ports 1 and 2 are software-counted and can miss high-rate counts. The configured rated turret speed implies about 32,056 counts/s. This identifies a design risk, not proof that this robot has lost counts. A future wiring change must also update the encoder's software channel/alias and preserve open-loop ownership of whichever motor shares it. No wiring change or physical check is requested now.

**Other gaps and limits**

| Area | Review result |
| --- | --- |
| Loop performance | Hub bulk caching is never enabled in TeamCode. The installed SDK bytecode defaults it to OFF. Drivetrain telemetry requests four positions and four velocities each loop, in addition to turret/flywheel/sensor reads. Enable consistent per-loop bulk caching and report actual OpMode loop duration; Pinpoint's device-loop frequency is not the Java control-loop frequency. The achieved hardware loop rate is unknown. |
| Auto-to-TeleOp localization | Pinpoint construction resets pose/IMU, and TeleOp has a single fixed starting pose. No autonomous pose handoff exists. Field localization after any driving still needs a known reference. Match route planning is deferred by the team and is not required for the current pollen detection/navigation scope. |
| Transfer diagnostic | The transfer-only test initializes both stages and the gate, without the shooter. RB requests shared timed gate-open feeding; LB inhibits. Stage 2 stays off throughout loading and gate travel. The former gate-closed stage-2 bypass is removed. |
| Homing | Relative-encoder stability cannot distinguish correct servo home from a stationary stall or missing encoder movement. Existing telemetry/tests acknowledge this. An absolute reference requires additional feedback; software alone cannot certify it. |
| Hood/gate arrival | Readiness is estimated from timing/rated servo speed, not measured position. There is no external feedback for either. |
| Jam detection | Intake and transfer have no current/stall-based jam handling. Their motor encoders are absent or shared, so a jam strategy needs appropriate inputs and team policy. This is a design gap, not evidence of a current jam. |
| Pollen camera model | Assumes upright camera, negligible yaw/roll and a pinhole ray. SDK distortion coefficients are not applied to contour coordinates. Significant mount yaw/roll or lens distortion needs model support. |
| Pollen pickup transition | At the October review, the 195-mm lens height, 10-degree downward pitch and 248-mm forward offset put the nearest unclipped contact at about 550 mm from robot center with the approximate 60-degree horizontal FOV at 320×240, beyond the original 350-mm pickup threshold. The team updated pitch to 15 degrees down on November 5, bringing the approximate boundary to about 500 mm. The original 350-mm transition remains invisible; the current 650-mm transition / 450-mm blind travel examples remain uncalibrated/ineligible. See [current pollen geometry](PollenAuto.md). |
| Full-queue sensor | Represents four-ball occupancy at one location; it cannot count shots, confirm an individual launch, or identify all jam/failure conditions. Reporting timed attempts rather than ball counts is correct. |
| Field data | The supplied Section 9 TU06 PDF was visually reviewed on October 23. It confirms red far 30–33, red audience 34–37, blue audience 38–41, blue far 42–45, and the code's nominal structural dimensions. Tag spacing/order and sticker placement are recorded in [FieldReference.md](FieldReference.md). The raised cell's pivot-to-front/underside transform remains missing; the 17.73-in. opening Y example is not independently verified. |
| Moving goal state | TU06 confirms that tag clusters move with the bistable hive. Tag IDs alone cannot prove that the selected cell is raised. `GoalTarget` rejects unknown/lowered state, but competition configuration has no populated state/map yet. A future enabled implementation must update/invalidate state when the hive tips; a one-time raised configuration cannot establish that it stays raised. |

**Checks that passed source review**

- Subsystems are plain Java; OpModes coordinate them. Mechanism hardware lookups occur during initialization.
- No blocking sleeps or nested mechanism waits were found in TeamCode control loops. Homing, gate feeding, recovery, collection and navigation use bounded state machines/timers.
- Main control paths update Pinpoint once per loop. Separate cached sample/history logic is used for planning.
- Drive and flywheel motor configuration types are cloned before modification. Drive/intake/transfer use BRAKE; flywheel uses FLOAT. The intake does not reset or regulate the turret's shared encoder.
- Drive joystick Y is negated; navigation converts left/CCW math coordinates to right/CW drive commands. Power normalization and navigation speed bounds are present.
- Normal turret yaw and hood commands are clamped. The raw diagnostic bypass is covered above.
- Automatic feeding is inhibited by invalid calibration, pose, target, motion, homing or tracking; feed inhibit overrides manual feeding. Manual feeding intentionally bypasses shot readiness, including flywheel stop.
- Autonomous has match deadlines, state timeouts, clearance checks and stop outputs. Competition shutdown stops motors before camera shutdown.
- Vision handles stale/missing frames, selected alliance/side, duplicate observations, history and implausible corrections. Pollen pickup requires distinct confirming frames and bounded motion through the blind spot.
- Pinpoint offset conventions and the 4-bar scale agree with the [manufacturer guide](https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf): 2000/(32π) ≈ 19.8944 ticks/mm. The installed dimensions/signs are not physically verified here.
- The digital full sensor's HIGH-near behavior matches [goBILDA's sensor description](https://www.gobilda.com/laser-distance-sensor-1m-range-3-3-5v-digital-or-analog-output/). The configured gate range/direction/rated speed match [its specification](https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/). Axon MAX+ rated 6-V speed matches [its specification](https://docs.axon-robotics.com/archive/max). These checks do not prove the installed programming, supply voltage, load response or wiring.

**Team clarifications after the initial review**

- **Servo power resolved:** [goBILDA 6V Servo Power Injector 3125-0001-0001](https://www.gobilda.com/6v-servo-power-injector-6-channel-8-15v-input/), six channels, 8–15 V input and regulated 6 V output rated up to 24 A shared across all channels. This supports the 6 V ratings used by the code; actual load/timing checks stay deferred.
- **C270 mounting resolved:** lens center 195 mm above floor, on centerline and 248 mm forward of robot center; upright and straight forward. The October pitch was 10 degrees below horizontal, superseded by the team's **15-degree downward pitch on November 5, 2026**. Current values are configured, with mount status measured. Intrinsics/color/pickup calibration retain separate incomplete status.
- **Autonomous scope corrected:** pollen detection and navigation only. Match paths, preload/repeat/scoring/parking strategies and route approval are deferred, not missing requirements for this task.
- **High-goal dimensions resolved:** opening center 1.50 m above floor, high-goal AprilTag center 1.25 m above floor, opening plane 60 degrees from horizontal, same dimensions for both alliance sides. These are stored in `ShootingConfiguration`, and the goal-height value replaces the previous example. Only high goals are shootable; low-goal heights are unnecessary. Opening-plane angle is not a launch angle; the empirical planar solver does not model the plane. Field X/Y positions and state remain unconfigured, so automatic shooting is not enabled by these dimensions alone.
- **Pinpoint connection resolved:** Control Hub I2C port 0, hardware-map name `pinpoint`. Java selects the configured name rather than assigning the bus.

**Information still needed from existing notes/CAD, with testing deferred**

1. **C270 intrinsics:** an existing lens calibration at 320×240, if available. No further camera mounting dimensions are requested.
2. **High-goal field geometry:** a CAD-derived raised-cell transform from the hive pivot to the opening/tag-cluster center. The supplied TU06 PDF now resolves individual tag offsets and ordering, so separate dimensions for each tag are unnecessary. The 17.73-in. opening Y example remains unverified. See [FieldReference.md](FieldReference.md) for the dimensions already resolved. The mount, center-height and plane-angle questions are resolved.

Already recorded and not being re-requested: all four drive motor ports and encoder cables; four-bar pod offsets (+32/-74 mm); turret pivot forward offset 50.68184865 mm; camera 99.53073974 mm behind the pivot, 337 mm high (re-measured 2026-11-06), lens axis 55 degrees up (re-measured 2026-11-05; originally 30), zero yaw offset; muzzle centered on the pivot in plan view; 1:1 turret gearing, 108:40 encoder gearing and 2:1 hood linkage; the clarified transfer sequence and lack of depollinator interference.

**Calibration dependencies remain deferred**

`ShootingConfiguration` still has an uncalibrated example shot table, incomplete field X/Y target geometry, unmeasured release/vision timing and no starting field reference. `PollenVision.CONFIG` has a confirmed mount but remains diagnostic-only until the remaining camera/color/pickup dependencies are supplied. Both competition routes are null because match paths are deferred. These are intentional safeguards, not reasons to invent values or mark calibration complete.

Measured shot rows, flight times for moving shots, loaded gate/feed/recovery timing, camera timing, color thresholds, pickup distances and usable motion envelopes will be needed later. They are not tasks to perform now. The original notebook was session-only. It now supports explicit persistent session export, without automatically installing a competition profile; see the implementation status.

