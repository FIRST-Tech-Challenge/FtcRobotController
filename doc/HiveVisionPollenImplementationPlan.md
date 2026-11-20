# Hive Vision migration plan for pollen navigation

> **2026-11-18 transfer change:** `transfer_motor_2` is now mechanically linked to stage 1 (both motors FORWARD, one shared command; the back rollers it used to drive were removed), so there is no separate stage 2. Stage-2 descriptions below are historical; see [TransferAndShooting.md](TransferAndShooting.md) for current behavior.

Created November 5, 2026. Status: **planned; implementation and robot validation have not started**.

Migrate the C270 pollen detector to Hive Vision's webcam HSV implementation while retaining the existing Pedro/Ivy navigation and collection architecture. The team selected the C270 path in this chat. Hive's neural detector is a separate Limelight path; this plan uses the Control Hub webcam processor described in the [upstream webcam installation guide](https://hivevisionftc.github.io/HiveVisionDocs/installation/webcam/).

The reviewed local baseline is `8e5a77583e9f5752a597fbbaa6d965769e6efbc4` **plus the existing staged, unstaged and untracked work**. Implementation must capture its own starting diff and preserve that work. This document is the only file added for this planning task.

**Current behavior and migration boundary**

`PollenAuto` is **Test: Pollen Navigation (Pedro + Ivy)**, loading `autos/pollen-navigation-test.auto.json` through `BioBuzzAutoFromFile`. Its `POLLEN_TEST` profile initializes drive/Pinpoint, intake, transfer, gate and the C270. The match autonomous runtime can also use the same camera and named collection commands. Both competition TeleOps currently use the Limelight for their separate AprilTag/localization function, rather than `PollenVision`.

| Existing component | Migration responsibility |
| --- | --- |
| `PollenVision.FrameProcessor` | Replace its hardwired SDK blob detector with a selectable detector; retain camera ownership, calibration and immutable frame publication. |
| `PollenTracker` | Keep filtering, largest-valid acquisition, 65-px continuity gate and 0.4-s lock expiry. Feed it complete per-frame candidate lists. |
| `PollenCameraModel` / `PollenTarget` | Keep undistortion, contour-contact floor projection, robot-center offsets and right-positive bearing. |
| `PollenNavigator` / `PollenPathFollower` | Retain the non-blocking state machine, pose goals, bounded Pedro paths/holds and immediate stop behavior. |
| `BioBuzzRobot` / `AutonomousCommands` | Keep INIT camera creation, one cached snapshot per loop, Ivy resource ownership and named command/condition semantics. |
| `PollenVisionTestOpMode` | Extend the existing camera-only diagnostic for backend comparison and calibration. |

Keep `webcam_1`, 320 x 240 MJPEG, and the confirmed C270 mount: height **195 mm**, forward **248 mm**, right **0 mm**, pitch **15 degrees down** (team update November 5, 2026, already applied to the shared configuration). The approximate near floor boundary is now 498.6 mm from robot center, about 500 mm; the existing 650-mm pickup transition remains visible and uncalibrated. A resolution or mount change requires separate calibration and filter review. The current 60-degree fallback FOV is approximate. Color, pickup and explicit intrinsics remain unmeasured; Pedro tuning is also gated separately. A detector change supplies none of those measurements.

**Upstream source assessment**

Reviewed upstream revision: [`3e96153422e73b8f254176d04708c97766ec9069`](https://github.com/hivevisionftc/Hive-Vision/commit/3e96153422e73b8f254176d04708c97766ec9069). Pin the imported files to this revision and review later updates deliberately.

The [detector source](https://github.com/hivevisionftc/Hive-Vision/blob/3e96153422e73b8f254176d04708c97766ec9069/cv/TeamCode/BallDetectorPipeline.java) thresholds yellow/red/blue and contains fixed HSV, area, aspect and fill settings. Its 900–12000-px area gate and 3500-px expected yellow area are not calibrated for this C270 configuration. It performs an RGB-to-HSV conversion per color. Its source comments report limited yellow precision on development footage; that is a reason to measure local performance, not evidence of an improvement on this robot.

The [shared pipeline source](https://github.com/hivevisionftc/Hive-Vision/blob/3e96153422e73b8f254176d04708c97766ec9069/cv/TeamCode/BallBlobPipeline.java) publishes boxes, contour area and normalized centers, but discards contour points and capture timestamps. Its candidate objects include mutable fields. `bestOf()` ranks by expected area/fill, which differs from our continuity selection. `suspend()` retains the previous detection list. It returns a cloned overlay Mat while its Canvas callback is empty. Adapt these contracts for our frame publication, geometry, drawing and resource ownership.

This is an HSV-to-HSV migration. Its benefit must be established with recorded C270 scenes and robot measurements; neural-model accuracy and Limelight inference speed do not establish webcam performance. Preserve the [upstream MIT notice](https://github.com/hivevisionftc/Hive-Vision/blob/3e96153422e73b8f254176d04708c97766ec9069/LICENSE) with imported source. Import only the two required Java sources and their notice.

**Implementation sequence**

1. **Record a baseline and establish the comparison.**

   Save the starting revision/diff and current camera settings. Run the focused existing pollen and autonomous integration tests listed below; distinguish existing failures from migration failures. With the robot available, capture representative C270 frames and logs using the current detector: single pollen at measured ranges, several pieces, image edges, shadows, glare, yellow field/robot surfaces, red/blue nectar and empty floor. Record lighting, exposure, white balance, stream format, dimensions, active intrinsics and ground truth. Use the same labeled scenes for both detectors. Until hardware captures exist, mark detector-quality comparison pending.

   Deliverable: retained baseline results and a reproducible frame set with expected candidates/contact points. This is the reference for subsequent changes.

2. **Vendor the source and make configuration explicit.**

   Place adapted `BallBlobPipeline.java` and `BallDetectorPipeline.java` under `org.firstinspires.ftc.teamcode.vision.hive`, with an upstream revision/patch record and MIT notice. Keep them as plain processor classes with no OpMode annotation or drive hardware. Use the existing FTC SDK 12.0.0 Vision/OpenCV dependencies; compile to confirm compatibility before adding any dependency. Avoid a floating JitPack dependency or unrelated Pedro/Zenith upgrades.

   Add an INIT-selected `PollenVisionBackend` with `SDK_COLOR_BLOB` and `HIVE_HSV`. Initially retain the SDK default. Give Hive a separate validated detector configuration covering enabled colors, full HSV lower/upper bounds, area, shape and morphology settings at a declared resolution. Enable yellow only for collection. Convert to HSV once per frame and reuse owned buffers. Express area gates as pixel-area fractions or explicitly resolution-bound values; treat morphology dimensions and continuity distances as resolution-dependent too. Keep the production stream at 320 x 240 for this migration.

   Keep Hive settings separate from the SDK HSV profile. Bind color-calibration evidence to the selected backend/configuration/resolution so an SDK calibration flag cannot qualify unmeasured Hive settings. A changed HSV upper value or morphology operation changes the contour and requires renewed contact/pickup checks. Record tuning choices rather than silently copying the upstream defaults.

   Deliverable: packaged source compiles, provenance is reviewable, invalid settings fail during INIT, and both backend choices are constructible without changing the runtime's camera ownership.

3. **Restore the contour data needed for navigation.**

   Before Hive releases each contour, extract the lowest contour point, full-contour clipping status and the shape measurements required by `PollenTracker.Detection`. Preserve contour area units and use a consistent box-center definition for continuity. Audit SDK density/aspect/circularity definitions against the installed SDK before porting them; Hive's bounding-box fill is a different measurement and must not be substituted under an existing field name. Compute SDK-compatible values or explicitly version and validate any changed gate semantics. Do not supply constant passing values.

   Add a narrow detector adapter, proposed `HivePollenDetector`, that returns all yellow candidates to the existing tracker. Keep `bestOf()` out of navigation selection. Copy all candidate fields into immutable team-owned values on the camera thread. Do not retain mutable `Rect`, `Mat`, contour objects or upstream candidate references in a control-loop snapshot.

   Continue to project the actual lowest contour point through `PollenCameraModel`; a box-bottom midpoint or normalized blob center is not a replacement floor-contact observation. Reject clipped contours, malformed geometry and invalid/above-horizon rays. Recheck the near visible pickup boundary after segmentation changes. Preserve the existing 2200-mm maximum target range unless physical evidence justifies a separately reviewed change.

   Deliverable: fixture comparisons show correct pixel units, clipping, candidate identity and projected range/bearing; deliberate geometry failures remain rejected.

4. **Integrate frame publication and lifecycle inside `PollenVision`.**

   Keep one `VisionPortal` and the current `PollenVision` public facade. Its camera-thread `FrameProcessor` initializes active intrinsics/model, invokes the chosen detector, selects through `PollenTracker`, and atomically publishes one immutable observation containing the original SDK `captureTimeNanos` and target or null. Optionally include immutable backend/configuration identity and rejection diagnostics in that observation. The main loop continues to read `snapshot(nowNanos)` once through `BioBuzzRobot.readSensors()`.

   Publish an observation for every successfully processed frame, including frames containing no eligible blobs. A fresh empty frame means camera-ready with no target; a stalled/disconnected camera means not ready. Never refresh a target's capture timestamp when polling it, and never combine candidates and time from different frames. Preserve the 0.2-s freshness bound and duplicate/out-of-order rejection.

   On disable, invalidate published eligibility/target and the runtime's cached snapshot, cancel active collection through its resource ownership, and arrange camera-thread tracker reset. On enable, require a new post-enable frame and cleared continuity/pickup-confirmation history; handle generation changes explicitly even if no loop sampled the disabled interval. Use a generation token or equivalent to discard in-flight pre-disable work. Change backend only during a new INIT session. Treat processor exceptions as unavailable vision; stop publishing successful frames and surface the error. Camera freshness must fail rather than silently falling back while moving.

   Have `PollenVision` own processor buffer cleanup after callbacks have ceased. Release temporary native objects and persistent buffers on shutdown/initialization failure. Replace the unowned overlay clone with immutable drawing context and `onDrawFrame(Canvas, ...)` rectangles/contact markers. Verify the Driver Station stream physically; returning a Mat as user context alone is not overlay acceptance evidence.

   Deliverable: frame/lifecycle tests pass, telemetry identifies the active backend and measured status, and repeated INIT/STOP sessions release the camera and native buffers.

5. **Apply the integration consistently to all consumers.**

   In `AutonomousCommands.collect()`, construct `PollenNavigator` from the active vision instance's `configuration()` rather than independently reading static `PollenVision.CONFIG`. Use the same validated pickup/camera configuration in readiness checks and planning. Keep `collectPollen`, `pollenVisionEnable`, `pollenVisionDisable`, `pollenVisible` and `pollenCameraReady` names, requirements and cached-condition meanings.

   Check `BioBuzzRobot`'s isolated `POLLEN_TEST` and optional-camera `FULL_ROBOT` paths, including missing-camera INIT failures for camera-dependent commands and successful initialization of camera-independent paths. Cover the actual `PollenAuto` wrapper/file builder and the common runtime used by red/blue/generated autonomous entry points. Keep one Pinpoint read and one telemetry flush per loop; initialize all hardware before START. Ensure terminal motor/feed shutdown runs before potentially slow camera shutdown, including exceptions and STOP during INIT.

   Review `RedMecanumTeleOp`, `BlueMecanumTeleOp`, shared TeleOp and Limelight diagnostics for affected dependencies. Their current camera path has no pollen consumer, so record that disposition. If implementation expands into shared drive/intake/transfer/turret behavior, extend changes and verification to both actual competition wrappers and the affected isolated test.

   Deliverable: all affected camera consumers use the same backend/configuration and retain their documented lifecycle, hardware isolation and command semantics.

6. **Extend the camera-only diagnostic and measure on the robot.**

   Reuse **Test: Pollen Camera Calibration**. Select the backend before constructing the portal, through explicit configuration or a non-blocking INIT selection state. Show backend/revision, dimensions, FPS, frame age in ms, processing time in ms, eligible candidate count, rejection reason, color/morphology settings, contact pixels, bearing in degrees, forward/right range in mm, intrinsics source and autonomous eligibility. Overlay the selected contour/contact distinctly from other candidates. Flush telemetry once at the end of each loop.

   Run the same labeled frame set and physical placements through both detectors. Measure precision/recall, false-target approaches, lock switches, contact-pixel error, range error, FPS and frame-age distribution, separating processing rate from control-loop rate. Set and record the team's acceptable range/bearing error budget before driving, based on pickup clearance. At minimum require correct steering signs, valid and unclipped pickup transition, no observed false pickup qualification on negative scenes, and no degradation against the SDK baseline on the agreed field set. Fixed image fixtures alone do not certify field reliability.

   Keep mount, intrinsics, color and pickup provenance independent. Only mark Hive color calibration measured after collecting its evidence. Revalidate the 650-mm transition / 450-mm blind-travel examples against the changed segmentation and measured intake geometry; these are currently uncalibrated examples. Keep approximate intrinsics ineligible unless explicitly allowed under the existing policy. Verify Pedro dynamics before setting its tuning flag.

   Deliverable: camera comparison report and calibration evidence, including measurements, configuration identity, lighting, reviewer/date and any failed or pending criteria.

7. **Validate bounded collection, then select Hive as the default.**

   Run the existing pollen navigation experiment in a cleared area at its current bounded speed, starting with one stationary pollen piece. Progress to several candidates, target occlusion/loss, empty scenes, camera outage, Pinpoint not ready, queue-full confirmation, operator STOP and restart. Confirm targets on the right produce the intended clockwise correction, the final pickup requires distinct close frames, and the camera can remain fresh while the confirmed piece leaves view during the bounded blind pickup.

   Preserve current limits: 4-s hardware wait; 90-degree / 4-s search; 7-s approach; 0.35-s lost-target grace with immediate movement stop; 3 distinct close frames spanning at least 0.12 s; 4-degree pickup alignment; configured pickup distance and 0.9-s pickup deadline; 0.4-s settle; 28-s total duration; 2400-mm accumulated travel. Retain the existing Pedro speed/power caps, transfer stage-1 loading, stage-2-off collection policy, closed gate and 0.5-s queue-full confirmation. Pickup attempts remain distinct from sensor-confirmed pickups.

   Switch the shared default to `HIVE_HSV` only after software checks and the recorded physical acceptance criteria pass. Retain an explicit INIT-only SDK selection for rollback, with its own calibration evidence. A Hive failure during an active routine stops collection; restarting with the SDK backend is a deliberate next-session choice. Record the default change and its evidence in the documentation.

   Deliverable: accepted detector migration with a documented rollback setting, or a clearly recorded pending physical gate while the SDK default remains active.

**Files to add or update during implementation**

All Java paths below are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` unless noted.

| Files | Planned change |
| --- | --- |
| New `vision/hive/BallBlobPipeline.java`, `vision/hive/BallDetectorPipeline.java` | Pinned, attributed local fork with contour metadata, configurable yellow processing, explicit resource ownership and Canvas drawing support. |
| New `vision/hive/UPSTREAM.md` and `LICENSE` | Source revision, local patch ledger, license and upstream update procedure. |
| New `PollenVisionBackend.java`, `HivePollenDetector.java` and detector configuration/helper as needed | Small backend selection/adapter boundary and validated, resolution-bound Hive settings. Extract the SDK adapter only as needed for comparison; avoid a general vision framework. |
| `PollenVision.java`, `PollenConfiguration.java` | Backend selection, immutable per-frame publication, calibration identity, lifecycle invalidation, telemetry and cleanup. |
| `PollenTracker.java`, `PollenTarget.java`, `PollenCameraModel.java` | Reuse current geometry/filter contracts; change only where the adapter needs an explicit shared contract, with corresponding regression evidence. |
| `PollenVisionTestOpMode.java` | Isolated backend diagnostic and comparison telemetry. |
| `AutonomousCommands.java`, `BioBuzzRobot.java` | Active-instance configuration and integration/test seams where necessary. |
| `PollenAuto.java`, `AutonomousProfile.java`, competition/generated autonomous wrappers | Review common behavior and hardware contracts; source changes only if needed for the integration. |
| `TeamCode/src/test/java/.../Pollen*Test.java`, new Hive/frame-contract tests, `AutonomousIntegrationTest.java`, `CompetitionAutonomousTest.java` | Adapter, frame/lifecycle, navigation and registry regression coverage. |
| `doc/PollenAuto.md`, `doc/SoftwareConfiguration.md`, `doc/Autonomous.md`, root/team-code readmes | Backend setup, tuning/provenance, physical acceptance status and rollback. |
| `doc/TeleOpVerificationMatrix.md`, `simloop/opmode-coverage.json` | Record evidence and unsupported native camera boundary accurately. Keep the existing diagnostic's unsupported classification. |
| `tools/teleop-test-baseline.json`, `doc/SimLoopRobotModel.md`, `simloop/README.md` | Update if adding/renaming scenarios or changing modeled behavior; preserve baseline tests and measurement provenance. |

No autonomous JSON schema or named-command catalog change is expected. If later edits affect canonical `autos/`, robot command metadata or a deployment-tracked wrapper, use the existing Zenith authoring/deployment workflow; regenerate managed APK assets and their manifest rather than hand-editing outputs. Check deployment freshness before packaging. Reuse the existing test OpMode; any new annotated OpMode must also receive an inventory entry and an honest evidence classification.

**Verification and evidence**

| Layer | Required observations/assertions |
| --- | --- |
| Adapter/filter | Yellow-only collection; all candidates reach continuity selection; SDK-compatible metric semantics; no fabricated confidence or passing shape values; area/morphology resolution handling; clipping/contact extraction before contour release. |
| Frame/lifecycle | Atomic target/time publication; original capture times; fresh empty frames; missing/stale/out-of-order/duplicate frames; disable/re-enable and in-flight frame races; processing failure; repeated close/INIT failure cleanup. |
| Geometry | Left/right signs, offsets, measured/SDK/approximate intrinsics, distortion, clipping, invalid floor rays, near visibility and changed contour/contact position. |
| Navigation | Existing `PollenNavigatorTest`, `PollenPathFollowerTest` and `PollenVisibilityTest` safeguards, including duplicate-frame pickup rejection and bounded blind travel with fresh camera health. |
| Runtime | Real file builder/command registry, both hardware profiles, active-instance configuration, optional/missing camera, cached conditions, resource cancellation and motor-first shutdown. Separate registry/fake-observation tests from native camera evidence. |
| Native image processing | Run recorded/synthetic image fixtures in an Android/OpenCV-capable test environment or on the Control Hub. Ordinary JVM tests exercise immutable candidate/frame logic; SDK default-return stubs do not verify native HSV/contour processing. |
| Hardware | C270 streaming/overlay, contact/range calibration, comparative detections, Control Hub loop-period p50/p95/p99/max, frame age, USB coexistence with Limelight and sustained native-memory behavior over repeated sessions. |

During iteration, run focused ordinary tests from the repository root:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests '*Pollen*Test' --tests '*HivePollen*Test' --tests '*AutonomousIntegrationTest' --tests '*CompetitionAutonomousTest'
```

Use the final implemented class names in filters and confirm the expected tests were discovered. At the integration milestone, this shared vision behavior change warrants the broader ordinary tests, debug APK packaging and lint:

```powershell
.\tools\verify-teleop.ps1
```

The repository requires `.\gradlew.bat :TeamCode:assembleDebug` before committing or deploying robot code; the verification script includes packaging. If deployment-tracked sources change, run `npm run zenith:validate`, regenerate with `npm run zenith:deploy`, then `npm run zenith:check` before packaging. A freshness check failure from the starting dirty tree must be recorded and resolved without discarding unrelated work. Close any live reviewer before Gradle and run builds serially.

**Simulation decision:** omit end-to-end C270/Hive SimLoop testing because the current harness does not model VisionPortal, USB capture or native webcam processing. Use deterministic ordinary tests for candidate/frame delivery through the real tracker/navigator and retained Pedro adapter tests; label synthetic inputs explicitly. Use Android/Control Hub image tests and physical navigation for the unsupported boundary. The [integration audit](SimLoopIntegrationAudit.md), [simulation plan](SimLoopTeleOpImplementationPlan.md), [verification matrix](TeleOpVerificationMatrix.md), [model provenance](SimLoopRobotModel.md) and [local usage](../simloop/README.md) establish these limits.

If implementation changes a shared mechanism or drive behavior, run relevant supported `Mechanisms`, `Drivetrain`, `Sensors` and/or `Competition` scenarios, including both actual red/blue wrappers and the affected isolated OpMode. Record baseline, assertions, selected results and model limits; update the scenario manifest/matrix when adding coverage. Such runs are regression evidence for those mechanisms, not Hive camera simulation. Preserve the existing raw/normalized Limelight distinction and production calibration flags.

Completion requires a reviewable source diff, source/license provenance, discovered test results, current debug packaging, accurate deployment artifacts where applicable, recorded simulation decision, and physical camera/navigation acceptance or an explicit pending status. Keep logs and camera measurements with revision/configuration identity and report exactly what ran.

The **original plan creation** was document-only: source/document review and document diff/format checks, with no Gradle tests, APK build, simulation, deployment or physical camera checks. The subsequent 15-degree camera-pitch configuration update is separate from the planned Hive migration; its current geometry and remaining calibration requirements are recorded in [PollenAuto.md](PollenAuto.md).
