# Implementation plan from the hardware/software review

> Historical review/plan: October 26 decisions supersede the moving-shot solver, flight-time data, motion-based firing gates, hive-state gating in competition TeleOp, calibration-file saving, the earlier 1.50/1.25-m height estimates and time-based uncertainty cutoffs described below. These features have been removed from the competition workflow. See [current competition behavior](CompetitionTeleOp.md), [configuration](SoftwareConfiguration.md) and [tag map](FieldReference.md) for the implemented behavior.


Historical review: the October 26 [competition decisions](CompetitionTeleOp.md) supersede competition intake, target/state selection, turret release/correction, initial RPM, feed controls and calibration storage described below. Diagnostic behavior remains separate.

October 23, 2026. This plan covers the [review findings](HardwareSoftwareReview.md), the team's confirmed mechanism/mounting details, the [TU06 field reference](FieldReference.md), and the proposed Limelight high/low detector. This is the implementation specification. Completed software changes, verification and remaining calibration boundaries are recorded in [HardwareReviewImplementationStatus.md](HardwareReviewImplementationStatus.md).

Scope: repair hardware-control logic, implement observed goal-state detection and integrate it with aiming/feeding, complete pollen detection/navigation logic, and verify on the desktop. Match autonomous paths, preload/repeat/parking strategies, deployment, wiring changes and physical testing remain deferred.

## Baseline already completed

- Preserve all existing local changes. The latest desktop baseline is 81 passing unit tests and a successful debug APK build. Earlier lint reported zero errors and 14 warnings.
- The October baseline C270 mount was 195 mm lens height, 248 mm forward, zero lateral offset, upright/forward, 10 degrees below horizontal. The team updated pitch to **15 degrees below horizontal on November 5, 2026**; current configuration and geometry are in [PollenAuto.md](PollenAuto.md).
- Keep the Limelight mount: 337 mm lens height (team-measured 2026-11-06; was 335), 55 degrees above horizontal (lens axis; front glass team-measured 55 degrees from vertical on 2026-11-05; originally documented as 30), and the previously confirmed rotating pivot/camera offsets.
- Keep high-goal opening/tag-center heights of 1.50/1.25 m and opening-plane angle of 60 degrees. The opening angle is not the hood command.
- Keep the confirmed 6 V servo injector and Control Hub I2C port 0 for Pinpoint.
- Keep the fourth-ball sensor on stage 1. Loading runs stage 1 forward with stage 2 stopped; feeding opens/settles the gate and runs both stages at full power, stage 1 FORWARD and stage 2 REVERSE (flipped 2026-11-18: motors wired backwards). Gate closure blocks passage to the flywheel. Opening alone does not release a ball.
- Depollinator clearance is confirmed. Do not add an invented collision exclusion for it.

## 1. Repair hardware selection, diagnostics and homing

Primary files: `TurretSubsystem`, `TurretHomingController`, `TurretTestOpMode`, `ServoZeroCalibrationOpMode`, `MecanumTeleOp`.

1. Remove port-number-only servo discovery. Resolve explicit supported hardware names, and fail with useful device/controller diagnostics if the intended servo is absent. Do not substitute another hub's servo.
2. Stop the all-servo diagnostic from broadcasting raw endpoints to attached mechanisms. Normal tests use subsystem limits and correct home positions. Preserve a clearly identified raw calibration procedure with deliberate selection for a disconnected servo; treat coupled installed turret servos as one mechanism.
3. Make UNHOMED, active homing, READY and FAULT output behavior explicit. Relative zero invalidates firing readiness without issuing a new home command. Only explicit homing requests start a home move.
4. Process advertised rehome and side-selection controls during INIT with the same edge detection as the active loop. Allow a homing retry before START.

Acceptance: a missing servo cannot command another hub; normal diagnostics respect mechanism bounds; relative zero cannot initiate a homeward command; INIT rehome works; STOP/exception paths remain bounded. Relative-encoder homing is still an estimate, not proof of an absolute home position.

## 2. Implement Limelight goal-state observation

Add a small hardware-independent observation/classification component and a read-only Limelight diagnostic OpMode. Use `LLResultTypes.FiducialResult.getTargetPoseCameraSpace()`; the installed FTC SDK exposes it. See the [Limelight FTC API](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).

- Read fresh per-tag 3D position/orientation with tag ID and capture time. Require the correct alliance/cell, finite nonzero pose, plausible range and usable detection geometry. Do not interpret missing/zero 3D output as a real close target.
- Convert camera coordinates using the known 337-mm lens height and 55-degree pitch. Height estimation is independent of global field X/Y. Account for chassis pitch/roll when available, or explicitly limit validity to the supported level-chassis assumption.
- Compare observed height with the raised-tag expectation of 1.25 m and corroborate with tag-plane orientation. Inspect the measured tag plane, not the opening's 60-degree angle directly.
- Require distinct consistent frames, agreement among visible tags from the same cell when available, and hysteresis. Track RAISED, LOWERED where supported by evidence, and UNKNOWN/TRANSITIONING; only a fresh, stable RAISED observation can qualify automatic feeding.
- A missing target, conflicting poses, a tipping cell, stale frames or a side change removes automatic firing permission immediately. Reacquisition must qualify again.
- Display measured height, plane orientation, observation age, cell identity and rejection reason. Use only confidence information actually exposed by the SDK/payload; do not invent an ambiguity field.
- Record the camera-side requirements: 36h11, 82.55-mm individual tags and enabled 3D processing. Do not claim the installed camera settings have been changed or verified without camera access.

Do not calculate range from an assumed high-tag height and then use that range to prove the same height. That is circular. Exact field coordinates and the missing 17.73-in. opening offset do not block this state detector.

Acceptance: synthetic observations cover both sides/alliances, raised/lowered/transitioning poses, camera pitch, multiple tags, zero/invalid poses, stale and duplicate frames, false single-frame positives, loss and reacquisition. No physical testing is needed to implement these checks; numeric confidence tolerances retain their unvalidated status until later.

## 3. Integrate observation, aiming and firing

Primary files: `ShotCoordinator`, `ShooterMount`, `GoalTarget`, `PoseHistory`, `ShotPlanner`, `FiringCoordinator`, `ShootingConfiguration`.

- Feed the observed state into readiness instead of relying on a fixed configured raised state.
- Separate camera-relative target observations from absolute field localization. Use the measured 3D tag pose and a tag-to-opening transform to construct the observed opening; the PDF supplies tag ordering and local spacing. Verify the local transform's axes, sign and reference points before marking it usable. A complete absolute field map is needed only for the field-localized mode, not for detecting raised state.
- Allow target-relative aiming in a consistent local odometry frame once the opening transform and shot calibration are valid. Do not pretend a local odometry reset establishes an absolute field reference. Keep motion compensation and uncertainty checks explicit in whichever frame is used.
- Correct delayed visual aiming using capture-time turret and chassis orientation, then express the target in the current frame. Do not add an old image error to the current turret angle.
- Make accepted vision reduce uncertainty according to observation quality, with a realistic floor and subsequent drift growth. Preserve rejection of implausible corrections. Remove the accidental roughly 130-second lockout without simply removing the uncertainty limit.
- Share a filtered, time-qualified stationary definition across readiness and planning. A stationary-only shot table must tolerate bounded measurement noise; actual moving shots still require measured flight times.
- Keep automatic goal-state gating separate from explicit manual feed override. Preserve current driver controls and operator feed-inhibit priority. High/low automation does not silently redefine manual feeding.

Acceptance: delayed-frame aiming does not overshoot merely because the turret moved; fresh accepted observations maintain eligibility over a full match; noise within a stable stationary envelope does not demand moving-shot calibration; unknown/moving goal state closes the gate and stops automatic feeding. Reprojection/transform tests must locate the same opening from different tags and camera poses.

## 4. Consolidate transfer behavior and improve diagnostics

Primary files: `TransferController`, `TransferSubsystem`, `FiringCoordinator`, transfer test OpModes.

- Pass the authoritative power and full-confirmation duration into the controller. Telemetry and actual behavior must use the same configuration.
- Preserve full power on both stages for every shot-feed path: manual, automatic and shot calibration. During loading, run stage 1 forward with stage 2 stopped; preserve full confirmation/reset behavior and the gate-opening delay.
- Exercise stage 2 only in a deliberate timed gate-open feed test independent of the shooter/flywheel. RB requests the shared sequence for both stages, LB inhibits, and release/STOP closes the gate and stops stage 2. Stage 2 stays off during loading.
- Add useful throttled current/output telemetry for intake/transfer diagnosis. Do not invent jam thresholds or automatic reverse/retry sequences without evidence; retain manual reverse control.

Acceptance: trace tests cover INIT, loading, sensor chatter, full queue, gate opening, feeding, readiness loss, manual override, operator inhibit, STOP and exceptions. No path automatically feeds through an unsettled/closed gate, and no ball counts are inferred from the fourth-ball sensor.

## 5. Correct pollen approach and blind pickup

Primary files: `PollenConfiguration`, `PollenVision`, `PollenTarget`, `PollenTracker`, `PollenNavigator`, `PollenAuto`.

- Retain the confirmed camera mount and use one authoritative pickup configuration.
- Use active camera intrinsics consistently; undistort contact pixels when valid distortion calibration is available. Keep fallback-FOV estimates explicitly approximate.
- Calculate/validate the visible floor region for the active camera model and contour-clipping policy. Reject an enabled pickup configuration whose confirmation distance lies outside that visible region.
- Replace reliance on the original 350-mm example: with the October 10-degree approximate model, targets disappeared near 550 mm. The November 5 15-degree mount brings that boundary to about 500 mm; 350 mm remains invisible. Confirm a target while it is visible, then use odometry/heading and bounded travel/time for the final blind pickup. Current uncalibrated examples remain 650-mm confirmation and 450-mm blind travel.
- Keep disappearance before confirmed acquisition distinct from the expected loss during committed pickup. Stale camera data, invalid odometry, full queue, STOP and time/travel limits must still stop motion.
- Show the visibility boundary, pickup settings, calibration status and rejection reason in the standalone camera test. Final transition distance/travel depend on real intrinsics and intake geometry; do not label a computed example as physically calibrated.

Acceptance: synthetic camera projections and navigation traces prove that a valid visible transition reaches bounded pickup, and that premature target loss cannot initiate an uncontrolled blind approach. Confirmed mount data alone must not enable uncalibrated color/pickup behavior. No match paths or obstacle-routing features are added.

## 6. Loop performance and calibration records

- Add hub bulk caching with one clearly owned cache clear per INIT/active loop. Standalone tests must also refresh correctly. Keep one Pinpoint update and one telemetry flush per loop.
- Cache hardware samples within each loop, reduce redundant reads and report actual Java loop duration/frequency. Do not report Pinpoint's device frequency as the OpMode rate.
- Add explicit persistent export of shot-calibration samples/rows with units, geometry identity and calibration status. Write on an explicit save/export action, not on each control loop; report failures. Preserve session samples and do not automatically activate an exported table as a competition profile.
- Keep startup/local/field pose provenance explicit. Full match autonomous pose handoff remains deferred with match-path work; local pollen odometry must not be mistaken for a global reference.

Acceptance: independent output/storage checks cover export round trips and failures; loop ownership is inspected across combined and standalone OpModes. Actual on-hardware loop rate remains unmeasured.

## 7. Regression verification and completion

For each behavior change, add targeted hardware-independent regressions rather than tests that merely repeat constant values. Reuse the existing tests and time-injected controllers. Integrate across startup, normal operation, loss/recovery, inhibition and shutdown.

Run from the repository root:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
.\gradlew.bat :TeamCode:lintDebug
```

Review the final diff and update documentation, controls and calibration status. Report the new test total, build/lint result, remaining defects and which automatic modes remain inhibited. Do not deploy or claim physical accuracy from desktop success.

## Deferred physical/configuration dependencies

- Turret encoder port 1 versus a hardware-counted port: document the eventual wiring/software alias change together; do not remap software ahead of wiring.
- Absolute turret home detection and measured hood/gate position require feedback the current hardware does not provide.
- [x] Turret physical checks, including gear/home alignment, servo directions/travel, encoder sign, mechanical limits, hood/flywheel operation and homing response: operator-confirmed complete October 24, 2026 after `Test: Turret`. See [physical sign-off](HardwareReviewImplementationStatus.md#turret-physical-checks--complete-october-24-2026).
- [x] All transfer and intake tests, turret rotation, shooting/angle and gate tests: operator-confirmed complete October 26, 2026. See [mechanism test sign-off](HardwareReviewImplementationStatus.md#mechanism-tests--complete-october-26-2026).
- Numerical turret timing calibration and measured gate/feed/recovery delays remain separate from the completed mechanism tests; other mechanisms' servo programming/loaded travel remains open.
- Limelight pipeline configuration, 3D pose quality, latency and state-classifier tolerances.
- C270 intrinsics/distortion, HSV thresholds, intake capture location, pickup travel and usable drive powers.
- Shot tables, moving-shot flight times and usable motion envelopes.
- Current thresholds and recovery policy for jam handling.

These dependencies are explicitly recorded, not requests to run physical checks now. The result of implementation should be complete, desktop-verified software with clear calibration boundaries; it should not hide unfinished measurements behind permissive defaults.
