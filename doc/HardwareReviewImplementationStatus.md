# Hardware-review implementation status

> **2026-11-18 transfer change:** `transfer_motor_2` is now mechanically linked to stage 1 (both motors FORWARD, one shared command; the back rollers it used to drive were removed), so there is no separate stage 2. Stage-2 descriptions below are historical; see [TransferAndShooting.md](TransferAndShooting.md) for current behavior.

> Historical review/plan: October 26 decisions supersede the moving-shot solver, flight-time data, motion-based firing gates, hive-state gating in competition TeleOp, calibration-file saving, the earlier 1.50/1.25-m height estimates and time-based uncertainty cutoffs described below. These features have been removed from the competition workflow. See [current competition behavior](CompetitionTeleOp.md), [configuration](SoftwareConfiguration.md) and [tag map](FieldReference.md) for the implemented behavior.


Historical review: the October 26 [competition decisions](CompetitionTeleOp.md) supersede competition intake, target/state selection, turret release/correction, initial RPM, feed controls and calibration storage described below. Diagnostic behavior remains separate.

October 23, 2026. Implements the desktop software scope of [HardwareReviewImplementationPlan.md](HardwareReviewImplementationPlan.md). Existing local mount/field/configuration edits were preserved. No deployment, robot movement, camera settings changes or physical accuracy verification was performed. Match routes remain deferred.

## Turret physical checks — complete October 24, 2026

Operator confirmation: the turret is built, `Test: Turret` ran successfully, everything looks fine, and the gears are aligned. All turret assembly/mechanism physical checks are marked complete on that basis:

- [x] Turret assembly and gear alignment.
- [x] Home alignment and turret zero direction.
- [x] Coupled rotation-servo directions, travel and mechanical clearance/limits.
- [x] Through-Bore Encoder operation and sign.
- [x] Homing and settling behavior in the turret test.
- [x] Hood direction/travel and flywheel operation in the turret test.

This records the operator's physical sign-off. No numerical loaded-speed, timing, PID-tuning or shot-calibration measurements were supplied. Those calibration dependencies remain open, along with camera/goal validation. See the October 26 sign-off below for additional completed mechanism tests. Relative homing still has no independent absolute-home sensor.

## Mechanism tests — complete October 26, 2026

Operator confirmation: all transfer and intake tests are complete; turret rotation, shooting/angle and gate tests are good.

- [x] All transfer tests.
- [x] All intake tests.
- [x] Turret rotation tests.
- [x] Shooting and hood/angle tests.
- [x] Gate tests.

These mechanism tests are complete based on the operator's robot testing. Numerical calibration measurements and camera/automatic-aim validation remain separate; this sign-off does not supply new calibration values.

## Changes and acceptance evidence

| Plan area | Implemented behavior | Desktop evidence |
| --- | --- | --- |
| Hardware selection and homing | Turret servos resolve supported names only. Missing hardware reports names, controller connection and port inventory. UNHOMED/FAULT retain the last bounded position; only an explicit home request sends the home target. INIT rehome/selection controls share edges with active controls. | Name-only resolver regression excludes another hub's same-number port; homing traces cover travel, settling, READY, invalidation, fault and retry. |
| Servo diagnostics | Installed mechanism tests use subsystem endpoints and individual homes. Coupled turret servos receive one shared command. Removed the all-servo raw broadcast. Raw programming is a separate selected, armed, disconnected-servo procedure with no INIT/START command. | Source inspection of every servo-output path; existing hood/yaw bounds and conversion tests. Physical disconnection remains the operator prerequisite for raw programming. |
| Observed goal state | Independent per-tag metric 3D height and underside plane, correct alliance/cell, finite nonzero pose, range/area/incidence checks, multiple-tag agreement, three distinct frames spanning 120 ms, hysteresis and immediate loss/conflict revocation. Capture age limit 150 ms. Lowered classification requires a supplied measured low pose; unsupported low poses are TRANSITIONING. | Synthetic tests for every alliance/side, raised/lowered/transitioning, camera pitch, zero/NaN/range/back-facing poses, duplicate/stale frames, tipping, conflicting tags, loss, reacquisition and selection changes. Camera-only `Test: Limelight Goal State 3D` exposes measurements and decisions. |
| Aiming and firing | Metric tag observations are separate from field localization. Verified tag-local transforms can construct openings in local odometry. Delayed tracking uses capture history; metric tracking also accounts for translation. Angle-only fallback rejects appreciable parallax. Fresh stable observed RAISED state and validated detector configuration are additional automatic gates. Manual feed and operator inhibit keep their existing priority. | Reprojection from different tags, camera/chassis poses and rotations; delayed yaw and translation regressions; local/field provenance; integrated observer/planner/feed loss/requalification trace. |
| Uncertainty and stationary shots | Accepted fusion retains observation quality, a 25-mm floor, residual correction uncertainty and subsequent drift growth. Shared filtered stationary envelope qualifies over 150 ms and revokes on raw excursions. Qualified bounded noise is treated as stationary; real motion still requires flight-time rows. | Full 180-second fresh-vision sequence remains below the uncertainty threshold; drift returns after loss; implausible corrections remain rejected. Noise, acceleration noise, moving-shot and reference-reset regressions. |
| Transfer and diagnostics | Authoritative subsystem power/confirmation settings are injected into the controller. Stage 1 loading with stage 2 off, two-stage full-power shot feeding, confirmed-full holding until feeding, and gate travel are shared across OpModes. All runtime feeding adapters also require the gate's 0.20 s command-to-feed delay. Hold RB in the transfer test requests the shared gate-open timed feed for both stages; stage 2 has no gate-closed diagnostic bypass. Intake/transfer current and commanded outputs are reported at 5 Hz. | Configured-time/power regression, full/chatter/loading traces, gate delay, readiness loss, manual override/inhibit and shutdown traces. No counts are inferred from the fourth-ball sensor and no automatic jam policy was invented. |
| Pollen visibility and pickup | One pickup configuration; active intrinsics and valid distortion determine floor rays. Full-contour clipping is retained. Near/far visibility is computed for the pickup cone; enabling an invisible transition is rejected. Default example is 650-mm visible confirmation then bounded 450-mm blind travel / 0.9-s limit. Premature loss stops; confirmed blind pickup still requires fresh camera operation and valid odometry. | Independent projection/undistortion round trips, rejection of the old 350-mm blind transition and invalid enabled models, distinct-frame confirmation, bounded blind travel, early loss, stale camera, invalid pose, full queue and STOP tests. |
| Loop ownership and shutdown | Every team INIT/active loop calls `RobotLoop.begin()` exactly once; no subsystem clears hub caches. One Pinpoint update and one telemetry flush per owning loop. Java period/frequency/work telemetry is distinct from Pinpoint device frequency. Reference generations invalidate history/qualification on resets. Shutdown attempts all devices even after a failure. | Source inspection of all team OpModes and cache/telemetry/update call sites; reset and shutdown-failure regressions. SDK 12 Lynx bytecode confirms cache mode resets to OFF between OpModes. Actual robot loop frequency remains unmeasured. |
| Calibration storage | Explicit REVIEW + right-stick click exports hit/miss samples and averaged rows, units, geometry/mount identity and unvalidated stationary-only status. Temporary output is parsed before finalization; failures preserve the session. Files never activate a competition profile. | Reader/writer round trips, persistent file readback, independent exports, bad units/counts/data and simulated writer/filesystem failures. |

## Controls changed or clarified

- Competition INIT: GP2 X rehomes; GP2 D-pad left/right selects audience/far. Holding across START does not repeat the edge.
- Turret test: A or Y explicitly requests bounded home; X sets a relative encoder zero and invalidates readiness without issuing a new yaw move. A works during INIT. B toggles trim. Y no longer bypasses the homing state machine every loop.
- Auto-aim test: A rehome also works during INIT. Target controls remain B/X and D-pad left/right.
- Transfer test: gate is required. RB requests timed gate-open feeding of both stages; release closes the gate and stops stage 2. LB pauses loading and inhibits feeding. Stage 2 stays off during loading. No shooter is initialized; begin with an empty mechanism.
- `Calibrate: DISCONNECTED Servo RAW`: select one servo with D-pad left/right. After START, both bumpers toggle arming, then RT plus A/X/Y deliberately requests .5/0/1. Disconnect all mechanical coupling first. Installed turret servos are exercised together in the bounded turret test.
- `Test: Limelight Goal State 3D`: camera only; B/X selects red/blue and D-pad left/right selects audience/far in INIT and active operation. Shows every metric tag, selected-cell height/plane/age and rejection reason.
- Shot calibration: BACK enters REVIEW. (Superseded: the file save was later removed; the OpMode now shows session averages on the Driver Hub only. See [ShotCalibration.md](ShotCalibration.md).)

## Calibration boundaries and automatic-mode status

Automatic feeding remains inhibited by the uncalibrated shot profile, unmeasured release/vision timing, `GOAL_OBSERVATION_VALIDATED=false`, and unavailable approved opening geometry. The field mode additionally needs an absolute map/reference. Target-relative mode does **not** need that map, but `ShootingConfiguration.OPENING.verified` remains false until its reference points and signed tag-local Y/Z offsets are approved. The generic transform and cross-tag reconstruction are implemented and tested; zero placeholder offsets are never consumed as real geometry.

Tag-local axes are +X sticker right, +Y sticker bottom/toward the cell front, +Z out of the printed underside. ID order and X offsets (-165.1, -69.85, +69.85, +165.1 mm) are from TU06. Rotation into camera optical axes uses Z-Y-X SDK pose orientation, followed by the 30-degree camera mount and capture-time odometry/turret transform. The missing absolute 17.73-in. offset is unused by the state detector and local-opening path. Exact front/underside-to-aim-point references remain an explicit geometry dependency; they were not guessed from heights or the opening-plane angle. The cached SDK BIOBUZZ cluster metadata also contains candidate local offsets (7.1874/-5.622 in.); these are not promoted to the team's transform without reconciling its cluster axes and chosen aim-point reference.

The state detector assumes a level chassis; no chassis pitch/roll source is supplied. A measured low-state height/plane may be entered separately; both default to NaN. The raised underside-plane expectation is 30 degrees, distinct from the 60-degree opening plane and from hood travel. All classifier tolerances, range/area/incidence thresholds and uncertainty/noise envelopes remain unvalidated physical estimates. Camera pipeline requirements are **36h11, 82.55-mm individual tags and enabled Full 3D**. Java accesses `FiducialResult.getTargetPoseCameraSpace()`; it does not verify or change tag family/size or camera 3D configuration. See [Limelight FTC API](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming) and [coordinate axes](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems).

Pollen autonomous remains inhibited: the mount is confirmed, but color thresholds and pickup geometry are unmeasured, and approximate autonomous intrinsics are not approved. The 650/450-mm values are a computed visibility-compatible example, not an intake calibration. Active SDK/custom intrinsics can change visibility; invalid models keep autonomous unavailable. Blind travel never treats an absent blob before confirmation as acquisition.

Relative homing remains an estimate without an absolute home sensor. Turret mechanism physical checks are complete per the October 24 operator sign-off above. Hood/gate timing remains estimated without position feedback; numerical loaded timing, shot rows, moving flight times, usable powers and jam thresholds remain calibration dependencies. Servo programming and loaded travel checks for other mechanisms remain separate. Turret encoder wiring stays on port 1; any future wiring/alias change must be made together. Confirmed 6-V injector, mounts, pod offsets, stage directions and depollinator clearance are preserved.

## Verification

Using Android Studio's Java 21 and the existing Android SDK, from the repository root:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
.\gradlew.bat :TeamCode:lintDebug
```

Final result: **118 tests, zero failures/errors/skips**, debug APK packaged; **lint 0 errors and 14 warnings** (the existing 12 native-library alignment warnings and two unused resources). Java 8 source/target and Gradle deprecation notices remain. No additional software defect is known from these checks; desktop verification does not certify hardware behavior or close the calibration dependencies above.

Reports: `TeamCode/build/reports/tests/testDebugUnitTest/index.html`, `TeamCode/build/reports/lint-results-debug.html`; APK: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.
