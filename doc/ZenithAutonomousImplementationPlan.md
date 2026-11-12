# Complete Zenith autonomous integration — implementation plan

Date: 2026-10-28. Implementation/audit update: 2026-10-31. Software commands, coordinator, canonical project, team deployment, examples and automated checks are implemented; see [the phase-by-phase audit](ZenithIntegrationAudit.md), [coverage/evidence](ZenithFunctionCoverage.md), [authoring](ZenithAuthoringGuide.md) and [verification](ZenithVerification.md). The final tree passes 185 ordinary tests, 12 deployment tests and all 24 current simulations, with APK/lint verification. The [core local UI round trip](ZenithUiAcceptance.md) is verified, with its remaining checks explicitly listed. Robot commissioning remains pending. This document retains the original requirements and historical baseline below; checked software/UI items do not establish hardware acceptance.

## Intended result

Autonomous programmers can open this repository in Zenith, find the pollen experiment and authored match routines, edit paths and every supported robot action, select arguments and conditions, and produce runnable FTC OpModes without hand-editing generated Java. Every existing robot function has an explicit mapping to a routine action, a condition/telemetry value, initialization, or an existing maintenance diagnostic.

Routine authoring and mechanism implementation have different responsibilities. Programmers compose supported actions in Zenith; adding new hardware, changing a mechanism's control law, or changing its calibration still requires the Java/configuration implementation. Saving an editor file does not update the Control Hub: assets and OpModes must be deployed into the repository, packaged, and installed. Editor estimates do not establish physical calibration or demonstrate that a mechanism was simulated.

Completion has three evidence levels: software integration verified, Zenith UI workflow verified, and robot motion/firing verified. Report each separately. Match strategy design is independent of finishing the integration.

## Observed baseline

The inventory was read from HEAD `d020024b94b14b5e756861d6fc297319705688ad` plus the current uncommitted working tree. Preserve the existing TeleOp/localization changes and the separate SimLoop TeleOp plan.

| Area | Current state |
| --- | --- |
| Runtime | Pedro core `3.0.0-20260828.185437-17`, Ivy `1.1.1`, Zenith runtime `v0.1.2`; Zenith CLI `0.1.2`. Keep these pins during this work unless compatibility evidence requires a documented change. |
| Autonomous entry points | Red/blue wrappers and the pollen test extend Zenith's Ivy `AutoFromFile`. Pedro owns autonomous drive outputs. |
| Commands | Eight: `intakeOn`, `intakeOff`, `loadOn`, `loadOff`, `aim`, `shoot`, `collectPollen`, `stopAll`. |
| Conditions | Four: `queueFull`, `pollenVisible`, `localizationReady`, `turretHomed`. |
| Missing mechanism | `BioBuzzRobot` does not initialize/register the depollinator. Independent yaw, hood, flywheel, reverse-intake, gate and explicit manual-feed actions are missing from the registry. |
| State coordination | Persistent aiming can overwrite later manual setpoints. Current `stopAll` calls terminal turret shutdown, which invalidates homing. Transfer hard stops reset full confirmation; LOW clears full immediately. These behaviors must be handled deliberately before exposing more controls. |
| Source organization | `autos/` has robot/field/waypoint metadata and no match routine files. The pollen routine is maintained directly in APK assets, outside the UI project's routine directory. |
| Generation | Pinned Zenith generates a stub for each source auto and assigns group `Generated`. Moving pollen into `autos/` without a generation policy would create a second entry point using the default full robot. |
| Calibration | Pedro tuning is unverified; robot dimensions/dynamics include labeled examples. Pollen and shooting calibration requirements remain active. |
| Verification | The earlier implementation reported 154 passing tests and a successful debug APK. Those results are historical until rerun in Phase 0. During this planning turn, `npm run zenith:validate` passed, with a cleared-test-area season-plugin notice and `TIMEOUT_TIGHT` information. No actual Zenith UI round trip or physical robot verification was performed. |

Relevant production sources are `BioBuzzRobot`, `PedroDrive`, `MecanumDrivetrain`, `PinpointOdometry`, `IntakeSubsystem`, `TransferSubsystem`/`TransferController`, `GateSubsystem`, `DepollinatorSubsystem`, `TurretSubsystem`, `ShotCoordinator`/`FiringCoordinator`, `PollenVision`/`PollenNavigator`/`PollenPathFollower`, and the current combined/test OpModes. Existing behavior in those sources takes precedence over older implementation-plan prose.

## Scope and engineering rules

- Keep the subsystem/test-OpMode/combined-OpMode architecture in AGENTS.md. Keep hardware mapping and I2C initialization in INIT. Retain standalone mechanism tests.
- Use Pedro for all autonomous motion and Ivy/Zenith for sequencing. Do not add another follower or a second autonomous shooting controller.
- Preserve hardware aliases, motor directions, cloned motor configurations, units, mechanical bounds, and configured current behavior. Autonomous drive remains open-loop power with BRAKE; flywheel remains FLOAT. Preserve unrelated TeleOp behavior.
- Keep one Pinpoint bulk update, one follower update, and one telemetry flush per active loop. Command/condition evaluation uses cached samples. No blocking sleeps or nested control loops.
- Declare every command's arguments, requirements, timing semantics, interruption cleanup, hardware dependencies, and estimated/measured status. Reject invalid arguments during INIT rather than silently clipping editor mistakes.
- Preserve automatic shooting's calibration/readiness checks. Explicit manual feeding remains a separately named, deliberate operation, matching the robot's existing override function; it must never be an automatic fallback for failed shooting readiness.
- Maintain autonomous field-odometry aiming. A visible Limelight tag is not an added prerequisite for odometry-based firing. Do not add chassis/turret speed cutoffs to the current competition feeding policy.
- Keep units explicit: Zenith/Pedro path coordinates in inches and CCW radians; team motion samples in meters and CCW radians; turret yaw in internal CCW degrees from its right-facing home; hood travel in downward degrees; flywheel targets in output-shaft RPM.

## Function-to-Zenith contract

The following are proposed public contracts, not already registered commands. Preserve existing names and defaults as compatible aliases. Prefer explicit setpoints to toggles so the same routine has the same meaning after interruption or restart.

### Chassis and localization

| Robot function | UI representation / proposed action | Required behavior and completion |
| --- | --- | --- |
| Forward, reverse, strafe, diagonal motion; lines and curves | Native Zenith path steps | Pedro follows the geometry/heading; speed fraction is applied. Requires `drive`. Timeouts/end conditions interrupt the path. |
| Chassis heading, reversing travel, facing a point, turning in place | Native heading modes and short turn steps | Preserve constant/linear/tangent/reversed/face-point/piecewise semantics supported by the pinned runtime. Test heading wrap and both alliances. |
| Hold a pose | `driveHold(xIn, yIn, headingRad, seconds)` | Pedro holds a bounded-duration target; cleanup stops drive. A hold is not a new waypoint follower. |
| Stop chassis while another mechanism continues | `driveStop` | Immediate zero drive output and cancellation of active following/hold ownership; must remain stopped on the following update. |
| Robot-centric power and individual-wheel testing | Existing Drivetrain/Pedro test OpModes | Keep direct wheel/power diagnostics out of ordinary path authoring; these are documented maintenance functions, with the original tests retained. |
| Set autonomous start pose | Native routine start pose and waypoints | Apply the alliance-correct pose after Pinpoint reports READY. Distinguish an explicit field reference from the local zero used by a cleared-area test. |
| Change pose at a known reference during a routine | `setFieldPose(xIn, yIn, headingRad)` | Requires `drive`, `gate`, `transfer`; stop/cancel following and feeding first, invalidate aiming history/readiness and reference generation, then explicitly seed the known pose. Never silently continue a stale path. |
| Local zero, IMU reset/calibration, pod/motor configuration | INIT and existing Pinpoint/drive diagnostics | IMU calibration stays stationary in INIT. Zeroing odometry must not imply acquisition of an absolute field reference. Hardware configuration is not an arbitrary mid-path UI action. |
| Pose/velocity/status, progress and arrival | Cached conditions and telemetry | Report units and sample age, path/step ID, follower mode and reference validity. Do not re-read Pinpoint from a condition. |

### Intake, transfer, gate and depollinator

| Robot function | UI representation / proposed action | Required behavior and completion |
| --- | --- | --- |
| Intake forward/off | Keep `intakeOn` / `intakeOff` | Existing names continue to work. Forward operation respects the confirmed-full policy. |
| Intake reverse and variable power | `intakeReverse`; `intakeSet(power)` | Validate finite power in [-1, 1]. Confirmed HIGH stops positive loading; LOW restores forward. Explicit reverse remains allowed. Define these as latched setpoints. Requires `intake`. |
| Timed intake/ejection | `intakeRun(power, seconds)` | Run for a bounded duration, stop on interruption/completion, and make cleanup unable to overwrite a replacement owner. |
| Automatic stage-1 loading | Keep `loadOn` / `loadOff` | Stage 1 loads while space exists; stage 2 stays off. Pausing/resuming preserves confirmed-full information. Requires `transfer`. |
| Stop transfer while preserving queue knowledge | `transferStop` | Stop both outputs and clear feed requests without treating the queue as empty. Reserve full controller resets for an intentional lifecycle/reset operation. |
| Coordinated collection until full | `loadUntilFull(seconds)` | Own intake/transfer, end on confirmed full or timeout, then stop loading. Readiness/full confirmation still uses the shared 0.2-second HIGH policy. |
| Readiness-gated two-stage feeding | `shoot` / `shootPreset`, described below | Gate delay precedes both feed motors; full does not block feeding. Requires `gate` and `transfer`, plus the shot resources. |
| Existing explicit manual feed override | `feedManual(seconds)` | The UI description states that shot readiness is bypassed. Preserve gate delay and feed-inhibit priority, bound duration, stop both stages and close gate on end. Never infer piece count. Can be tested with gate/transfer hardware alone. |
| Open gate without feeding | `gateOpen(seconds)` | Bounded open command; transfers remain stopped. Close at end/interruption. It does not grant permission to feed. A latched feed inhibit wins. |
| Close gate and prevent unintended reopening | `gateClose`; `feedInhibit` / `feedResume` | Closing/inhibiting first removes feed requests. Inhibit is latched and dominates all feeding modes. Resume clears the inhibit but does not itself start feeding. |
| Confirm full, observe sensor and feed state | Conditions/telemetry | Distinguish raw sensor HIGH, confirming full, confirmed/held full, loading, and feeding. The sensor cannot count pollen or certify that the queue is empty. |
| Deploy/home depollinator | `depollinatorDeploy`; `depollinatorHome` | Initialize the existing subsystem; use its 30-degree home and 135-degree CW deploy travel. Requires `depollinator`. Add bounded settle estimates without claiming servo position feedback. |
| Wait for depollinator posture | Estimated-settle conditions or bounded action option | Expose commanded posture separately from estimated completion. Use measured travel timing when available. Until posture-aware path clearance is verified, plan with the conservative expanded footprint. |

### Turret, hood, flywheel and shooting

| Robot function | UI representation / proposed action | Required behavior and completion |
| --- | --- | --- |
| Home coupled yaw servos and establish encoder reference | `turretHome(timeoutSeconds)` | Begin the existing non-blocking home procedure, inhibit feeding, and finish only on homing success or an explicit timeout/fault outcome. Do not fabricate a limit-switch measurement. |
| Absolute yaw target | `turretSetYaw(yawDeg)` | Internal CCW degrees from right/home, currently [-175, 175]. Show orientation help in the UI. Requires `turretYaw`. Reject unreachable requests. |
| Relative yaw adjustment | `turretAdjustYaw(deltaDeg)` | Resolve once at command start; validate the resulting bounded target. Do not repeatedly add the delta every loop. |
| Hold measured yaw / end manual yaw travel | `turretHold` | Capture measured yaw once; do not invalidate homing or chase encoder noise. |
| Wait for yaw | `waitTurretReady(timeoutSeconds)` | Use cached homing/error information and declared tolerance. Distinguish a mechanism-settle wait from the automatic-shot angle predicate. |
| Hood setpoint and adjustment | `hoodSet(travelDeg)` / `hoodAdjust(deltaDeg)` | Current travel is [0, 30] degrees down from home; geometric launch elevation is +15 minus travel. Keep those quantities distinct. Requires `hood`. |
| Hood home and estimated tracking wait | `hoodHome`; `waitHoodReady(timeoutSeconds)` | Preserve existing home offset, 2:1 linkage and estimated tracking model. Label completion as estimated because there is no position sensor. |
| Independent flywheel RPM | `flywheelSetRpm(rpm)` | Output-shaft RPM; use the configured cap (currently 6000). Normal spin is [0, cap]. Manual RPM mode must not be overwritten by persistent automatic aiming. Requires `flywheel`. |
| Pre-spin and wait for speed | `flywheelSpinUp(rpm, timeoutSeconds)` | Set RPM, wait for cached speed qualification, and never feed. Define success/timeout and end behavior explicitly; natural success retains the requested RPM for the following shot, interruption stops it. |
| Stop flywheel independently | `flywheelStop` | Immediately inhibit feed and latch flywheel OFF while permitting separately requested yaw/hood tracking. A later background aim update must not restart it. |
| Select alliance/hive side | Routine alliance plus `selectTarget(side, frame)` | Use explicit AUDIENCE/FAR choice and FIELD/AUTHORED frame. Alliance comes from the selected routine/mirror, not a hidden toggle. Selecting a new target invalidates previous shot readiness. |
| Automatic odometry aiming | Keep `aim(side)`; add `aimOff` and a documented spin option | Keep the existing default for old files. Expose yaw/hood tracking with or without automatic flywheel control. `aimOff` holds measured yaw/current hood, clears feeding, and stops automatic flywheel output. |
| Readiness-gated automatic shooting | Keep `shoot(seconds)` | Preserve field-target/profile/homing/hood/RPM/timing checks and the existing yaw policy. Duration bounds requests, not measured ammunition. Explicitly define whether aiming/pre-spin continues afterward; preserve the old contract by default. |
| Fixed yaw/hood/RPM shot routine | `prepareShotPreset(preset)` / `shootPreset(preset, seconds)` | Back presets with validated configuration containing units, target/setpoints and measured status. Require homing, verified preset/release timing, hood/RPM/yaw readiness and the shared firing coordinator. Unknown/unverified presets cannot feed automatically. No invented preset measurements. |
| Stop feeding without losing prepared aim | `feedStop` | Clear automatic/manual feed requests, close gate, stop feeding, preserve valid homing and explicit aim/pre-spin modes. Full-queue information is retained. |
| Inspect readiness and attempts | Conditions/telemetry | Show rejection reason and relevant target/reference generation. Report timed feed attempts, not launched-ball counts. |

### Vision, configuration and maintenance

| Existing function | Planned exposure | Boundary |
| --- | --- | --- |
| C270 detection and bounded navigation | Keep `collectPollen(seconds)`; expose calibrated configuration/status | Pollen remains a test experiment, with STOP/HOLD/FOLLOW handled by Pedro. Preserve distinct-frame confirmation, loss stopping, blind-pickup bounds, queue/time/travel limits. |
| Enable/disable pollen processing | `pollenVisionEnable` / `pollenVisionDisable`, if implemented through VisionPortal processor control | Camera construction stays in INIT; re-enabling never constructs hardware mid-loop. Invalid/stale observations are discarded. Disabling processing interrupts dependent collection. |
| Limelight results and pipeline selection | `limelightSetPipeline(pipeline)` plus cached freshness/tag conditions | Validate against documented configured pipelines. Invalidate affected observation history; keep autonomous Pinpoint-following pose unchanged unless an explicitly verified correction feature exists. |
| Camera mount, intrinsics, HSV, profile and release-timing calibration | Existing calibration OpModes and configuration files; UI displays status/provenance | These are setup data, not unverified measurements writable into existence by a command. Existing measured flags remain authoritative. |
| Limelight-based field pose correction | Explicitly documented future option; current autonomous behavior remains odometry-only | Do not enable TeleOp's vision correction policy during autonomous routes merely because pipeline/vision actions are exposed. |
| Turret encoder-correction enable/disable | Named configuration in a diagnostic preset; fixed, verified competition setup | Preserve bounded correction logic and diagnostic defaults. Correction qualification is not a new shot-feed speed cutoff. |
| Encoder zero at current angle | Existing Turret test | Preserve invalidation of homing/automatic firing; it is not a shortcut around the homing procedure. |
| Raw yaw/hood servo positioning and home-all convenience | Existing bounded Turret test; expose normal home/yaw/hood actions above | Keep paired yaw servos coupled and within mechanical limits. Map the misleadingly named `setAllServosToZeroPosition` to bounded home semantics, never PWM zero. |
| Raw disconnected-servo programmer | Existing `ServoZeroCalibrationOpMode` only | Preserve its disconnected-servo workflow and arming behavior. It is accounted for as maintenance, not an ordinary autonomous command. |
| Flywheel raw power/reverse, PIDF/bang-bang-fusion selection and tuning reset/adjust | Existing shooter diagnostics; add named diagnostic presets only when needed for UI-authored bench routines | Preserve controller-switch stopping and output RPM units. Do not let a routine silently rewrite tuning or select a different controller. |
| Motor specs, pod directions/offsets, camera hardware startup/close, telemetry/cache updates | Initialization/runtime infrastructure | These are not user routine steps. They remain available through configuration and diagnostics. |
| Global stop and terminal shutdown | Keep `stopAll`; separate terminal `onStop` cleanup | An in-routine stop clears active outputs and holds positional mechanisms without invalidating reference unnecessarily. Terminal STOP/exception shutdown closes resources. Depollinator STOP does not introduce an unrequested retraction. |

This covers the current operational APIs, TeleOp actions and diagnostic-only mutators. It does not claim that every SDK getter, hardware-map accessor or mathematical helper needs a UI command. Any implementation-time discovery of an additional operator function must be added to this matrix before command-coverage acceptance.

## State ownership, conditions and command semantics

Introduce a small autonomous output coordinator with explicit desired modes for intake, transfer/feed, turret yaw, hood and flywheel. Reuse `ShotCoordinator`, `FiringCoordinator`, `TransferController` and subsystem control laws; do not duplicate them. `BioBuzzRobot` remains lifecycle/hardware wiring, and a focused `AutonomousCommands` class holds validated command factories and registration.

Use matching requirement keys in Java and robot.json: `drive`, `intake`, `transfer`, `gate`, `depollinator`, `turretYaw`, `hood`, `flywheel`, and camera controls where applicable. A composite shot claims every channel it controls. Shared physical hardware is represented once even though `TurretSubsystem` contains several actuators. Conditions read snapshots and do not acquire writable outputs.

Latched setters are intentionally instant: their modes persist until replaced/stopped. Timed/waiting commands retain ownership until completion. Requirement declarations alone do not give persistent background modes a continuing Ivy lock; the coordinator must resolve those modes explicitly. Manual setpoints disable automatic ownership of the affected channel. Explicit flywheel stop, feed inhibit and global stop dominate background aim requests. Define natural completion, cancellation, timeout, fault and replacement behavior for every command. Test cleanup exactly once and ensure cleanup of a replaced action cannot stop its successor.

Named-command arguments are not automatically mirrored by native path conversion. New absolute pose commands use the authored file frame and apply the context's alliance transform exactly once; live/current-pose targets are already in the running frame. Relative turret yaw/hood/RPM arguments are robot-local and do not receive a field transform. Keep existing `aim(side)` field-side behavior as its compatible default, and add an explicit FIELD/AUTHORED target-frame option for mirrored routines. In AUTHORED mode, point symmetry also maps AUDIENCE to FAR and vice versa; changing alliance alone is insufficient. Document the defaults in the UI and test the complete path-plus-target routine on both alliances.

Provide cached conditions for `queueFull`, `queueSensorHigh`, `loadingActive`, `feedingActive`, `gateFeedDelayElapsed`, `feedInhibited`, `localizationReady`, `driveIdle`, `turretHomed`, `turretAtTarget`, `hoodTrackingEstimated`, `flywheelAtSpeed`, `aimReady`, `shotReady`, `pollenVisible`, `pollenCameraReady`, `limelightFresh`, `selectedGoalTagVisible`, `depollinatorDeployedCommanded`, `depollinatorHomeCommanded`, `depollinatorSettledEstimated`, `commandFailed`, and `matchTimeLow`. Keep existing names compatible. Parameterized distance/heading/time predicates require supported schema/runtime handling; do not invent arguments for Zenith's currently named BooleanSupplier conditions. Fixed conditions such as `matchTimeLow` use documented team configuration until that support exists.

Each wait is bounded by a timeout in its named action or native Zenith wait step. Publish the last meaningful outcome (`SUCCESS`, `TIMEOUT`, `INHIBITED`, `FAULT`, `CANCELLED`) with reason, step ID and timestamp. Do not report an interrupted command as successful. An invalid localization sample/global deadline causes terminal stop; ordinary failed preparation can be handled by an authored branch if continuing is safe. Outcomes must remain observable long enough for that branch, rather than being overwritten by a following instant step.

All registry entries include argument type/default/range/unit help, actual requirements, stationary/movesRobot behavior, honest estimates and provenance. Use `unknown` where timing is unmeasured. A camera-guided drive command's duration is a bound, not a known path or guaranteed trajectory. Do not write unsupported schema properties expecting the pinned editor to display them; use supported summaries/parameter descriptions and a companion catalog where necessary.

## Delivery phases

### Phase 0 — establish baseline and prove deployment policy

1. Record the current diff and rerun unit tests, `:TeamCode:assembleDebug`, registry validation and deploy dry-run. Preserve unrelated changes. Record actual counts/failures; do not assume the historical 154 count remains fixed.
2. Inspect the installed/pinned Zenith editor/CLI and runtime hooks. Verify command parameter controls, condition selection, path/marker/parallel support, project file discovery and generation behavior with a small temporary fixture.
3. Confirm the deployment policy below against an actual UI-created routine. The pinned CLI's fixed `Generated` group and one-stub-per-auto behavior are known constraints, not configurable group/profile features.
4. Create `doc/ZenithFunctionCoverage.md` from the matrices above, with implementation owner, UI command/condition, hardware profile, tests, UI evidence and hardware evidence. Track the three evidence levels separately.

Acceptance: reproducible baseline, a complete coverage ledger, and a demonstrated route from UI-created file to correctly classified/profiled OpMode. No production route coordinates are fabricated for this proof.

### Phase 1 — canonical UI project and reproducible generation

1. Move the authoritative pollen routine to `autos/pollen-navigation-test.auto.json`. Make `autos/` the canonical source for all editor routines. Treat APK assets as deployed output. Keep the cleared test-area field distinct from the competition field.
2. Add repository-owned routine metadata (proposed `autos/routines.json`) for reserved test wrappers/profiles. This is team metadata, not a claimed extension of Zenith's auto schema. The known pollen routine maps to the existing Test-group wrapper and `POLLEN_TEST`; newly created match routines default to `FULL_ROBOT`.
3. Extend `scripts/validate-autos.cjs` to validate every canonical routine, the metadata, command/condition parity, versions, duplicate names/IDs, profile/capability compatibility, matching file stems and deployed-output freshness. Remove the hard-coded asset-directory pollen exception.
4. Add `scripts/deploy-autos.cjs` as the supported repository deployment entry point. Consume pinned Zenith's deployment preview/output and generate or retain exactly one wrapper per routine. Retain the hand-written pollen Test wrapper, omit its redundant generated counterpart, and resolve redundant red/blue wrappers explicitly. Do not use broad regex rewrites of generated Java or edit installed npm package code.
5. Manage only declared/generated paths, validate resolved paths stay inside the project, and remove only obsolete managed outputs. Include test field/metadata deployment. Repeated deployment must be idempotent; renamed/deleted routines must not leave selectable stale OpModes or assets.
6. Verify how the native editor Deploy button interacts with this policy. If it cannot run the team generation policy, document the exact workflow as **UI edit/save → `npm run zenith:deploy` → build/install**, and make a generation check reject raw-CLI duplicate/wrong-profile output. Do not advertise the native button as completing team deployment until verified. No Java editing is required of routine authors.
7. Keep schema/runtime/CLI and exact Pedro pins consistent, along with lockfiles and source/deployed robot metadata. Preserve the current numerical hold guard until the chosen Pedro build no longer needs it.

Acceptance: pollen appears in the editor routine list; its deployed entry remains in Test with no turret/Limelight/depollinator dependency. A newly created match routine generates a selectable full-robot OpMode. Rename/delete/deploy twice produces no duplicates or stale assets. UI save preserves recognized data and source remains authoritative.

### Phase 2 — lifecycle, hardware capabilities and output ownership

1. Add explicit `FULL_ROBOT`, `POLLEN_TEST`, and existing isolated diagnostic capability definitions. Full robot includes the depollinator; pollen retains only drive/Pinpoint, intake, transfer, gate and C270. Initialize the required hardware in INIT. An unsupported command fails at INIT with its command/step and missing capability, not a null pointer at START.
2. Introduce the output coordinator and requirement mapping above. Keep one hardware/output owner. Snapshot sensor data once, preserve the existing native follower/scheduler lifecycle, and apply resolved mechanism requests once per loop. Avoid a new general-purpose scheduler.
3. Separate an in-routine all-output stop from terminal turret/camera shutdown. Keep a valid home/reference across stop-and-resume actions. Define terminal cleanup for partial INIT failure and exceptions, including camera cleanup after motors stop.
4. Preserve HIGH confirmation across output-only pauses; LOW must immediately clear full and resume requested loading. A fresh continuous 0.2-second HIGH confirms full again. Confirmation stops forward intake (no reversal); stage 1 runs on 0.5 s to seat the fourth ball. Feeding and pauses cancel the run-on.
5. Use one robot-clock contract for command waits and gate/feed/confirmation timing used in autonomous tests. Add narrow optional clock injection where required, retaining default real-time constructors/TeleOp behavior. Do not mix a stepped command clock with wall-clock mechanism delays and call the result deterministic.

Acceptance: replacement/manual commands are not overwritten by automatic aiming; stop-all can be followed by a valid new action; STOP/deadline/pose loss always removes outputs; profile tests prove no unrelated hardware lookup and no active-loop hardware initialization.

### Phase 3 — intake, transfer, gate and depollinator commands

Implement the material-handling rows, metadata and conditions. Reuse existing loading/feed controllers. Add truthful getters/snapshots for raw/confirmed full state and commanded depollinator posture as needed. Preserve the existing mechanical bounds and test OpModes.

Acceptance: UI-authored actions demonstrate forward/reverse/stop, confirmed-HIGH reversal, immediate LOW recovery, pause/resume with live sensor confirmation, manual feed with gate delay, inhibit priority, gate-only open/close, deploy/home, and interruption in every timed operation. Mechanisms can run alongside a path when resources do not overlap.

### Phase 4 — turret, hood, flywheel and shooting commands

Implement the aiming/shooting rows, validated preset structure and wait predicates. Keep automatic and explicit manual modes distinct in state. Preserve current homing, correction bounds, hood estimation, output RPM gearing/cap, moving-shot policy, target selection and firing recovery. Reuse the shot coordinator for automatic planning and the firing coordinator for the gate/feed sequence.

Acceptance: fixed yaw/hood/RPM survive subsequent loop updates; aim/pre-spin can run before shooting; flywheel stop is respected while yaw tracking continues; target changes requalify readiness; home/adjust/hold are bounded; uncalibrated/invalid automatic shots never feed; manual feed is explicit and never chosen as fallback. Preset rows carry actual measured status rather than example values marked valid.

### Phase 5 — vision, localization actions and author feedback

1. Complete pollen/Limelight controls and cached conditions with INIT-only hardware construction. Preserve camera freshness and collection eligibility. Represent camera failure and collection timeout/fault as observable outcomes.
2. Implement explicit known-pose seeding/drive hold/drive stop and test reference invalidation. Leave ordinary odometry calibration in INIT and preserve autonomous odometry-only aiming by default.
3. Show current routine and step ID, requested/active output modes, parameters with units, command outcomes, readiness/inhibit reasons, remaining match time, robot profile, calibration status and relevant sample ages. Include these in optional trace output through Zenith's existing hooks.
4. Fill editor estimates, footprint/mouth/shooter metadata and provenance with measurements when available. Use conservative expanded geometry for depollinator operation until dynamic footprint transitions are represented correctly. Do not treat planner-only transfer/ledger records as hardware commands.

Acceptance: branch/wait conditions are fresh and side-effect free; camera disable/loss cannot leave collection driving; reseeding cannot leave a stale follower/aim history; telemetry explains why a routine did or did not act. UI preview labels timing/physical-model limitations honestly.

### Phase 6 — automated verification and packaged artifacts

Extend existing JUnit 4 tests and add focused command/coordinator/deployment tests. Avoid tests that merely repeat setter assignments. Use independently evolving feedback where motion/settling is asserted. Share any later SimLoop infrastructure with the separate TeleOp plan rather than making that unfinished project a hidden prerequisite.

| Verification area | Required scenarios |
| --- | --- |
| Registry and arguments | Every matrix action and condition mapped; defaults/aliases; NaN/infinity/out-of-range arguments; unknown profiles/commands/presets; unsupported hardware rejected during INIT; Java/robot metadata requirements agree. |
| Ownership | Background aim versus manual yaw/hood/RPM, flywheel stop versus aim, feed inhibit versus all feed modes, command replacement and marker conflicts, unrelated actions in parallel, cleanup exactly once, persistent modes after instant steps. |
| Material handling | 0.2-second confirmation and 0.5-second run-on boundaries, LOW recovery, confirmed-HIGH intake stop, loading pause, feed release, gate delay, cancellation, commanded/estimated depollinator posture and footprint declaration. |
| Turret/shooting | Home/stall/timeout/fault, reference invalidation, bounds/signs/heading wrap, estimated hood tracking, output RPM conversion/cap, pre-spin and recovery, fixed versus auto ownership, unavailable calibration, target switch, explicit manual override. |
| Pedro/runtime | Actual hold/path-speed behavior and numerical guard; one Pinpoint read/follower update; short turns, curves/heading modes, endpoint/distance markers, all/race/deadline, bounded waits/branches, deferred starts, both alliances/mirrors including named pose/target arguments and literal versus authored hive side, STOP before START and during each action. |
| Fault/deadline | Pose loss/staleness, camera loss, failed initialization, exceptions during execute/end/shutdown, global deadline, safe park/stop branches, preserved original error plus attempted cleanup of all outputs. |
| Files/generation | Canonical pollen file, field/robot/waypoint/profile assets, idempotent deploy, add/rename/delete, malformed file, unsupported format, duplicate OpModes, reserved test wrapper, test hardware isolation, no unintended dependency changes. |
| End-to-end artifact | UI-created source parses through real Zenith AutoBuilder/Ivy; generated Java compiles; APK contains the deployed files/version metadata; the scheduled command tree corresponds to the edited source. |

Required commands: `npm ci --ignore-scripts`, `npm run zenith:validate`, team deploy dry-run and deployment, then `.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`. Inspect APK assets. Verify a no-change redeploy produces no source/output changes. Re-run the meaningful affected checks after fixes; do not claim hardware behavior from stubbed Android defaults.

Acceptance: all relevant existing tests still execute, new behavior tests pass, no unexpected skips, registry/deployment checks pass and a cleanly packaged APK contains the intended routines. Record precise counts and remaining warnings at implementation time.

### Phase 7 — actual Zenith UI round trip and coder handoff

1. Open the real repository in the selected pinned-compatible Zenith UI. Record editor build/version and a reproducible procedure; CLI-only evidence is insufficient for this phase.
2. Confirm the pollen test is visible and every normal command/condition has usable labels, controls, defaults and units. Confirm diagnostic-only functions are discoverable through the documented maintenance path.
3. Edit a start/waypoint and curved path; add a marker, wait/condition, parallel action, branch and representative actions for every subsystem. Save, close/reopen, and verify values and semantics survive.
4. Create a new routine, rename it and delete a temporary routine. Run the supported deployment workflow; verify the exact generated Driver Station entries and correct profiles. No hand-edited Java or silent substitution of unsupported features is accepted.
5. Exercise a deliberately invalid argument/unregistered action/capability mismatch and confirm validation reports actionable file/step errors. Confirm the corrected routine deploys.
6. Add `doc/ZenithAuthoringGuide.md` and runnable editor examples with source paths, units, persistent versus timed semantics, readiness/manual feeding distinctions, resource conflicts, outcomes, deployment, rollback through source control and calibration boundaries.

Acceptance: the UI evidence and source/deployed diff demonstrate that an autonomous programmer can complete the workflow independently. Do not mark this phase complete merely because CLI parsing or generated Java compilation passes.

### Phase 8 — robot commissioning and match-author readiness

1. Run existing isolated diagnostics and the Pedro drive check. Measure robot dynamics, wheel-power behavior, dimensions/clearances, pod signs, in-place turns and braking. Update Java and robot.json together; retain provenance and the verified flag boundary.
2. Verify depollinator posture/travel timing, gate delay, transfer full confirmation, turret home/sign/bounds, hood motion, flywheel RPM/recovery and all stop paths with actual hardware. Positional servos hold their last bounded command on terminal stop unless an explicit, tested mechanical policy requires otherwise.
3. Finish pollen mount/intrinsics/color/pickup verification before collection driving. Finish verified shot profiles/presets and release timing before automatic feeding. Do not mark example values measured to clear an inhibit.
4. Run a UI-authored integration test in a cleared area at deliberately bounded speed, then a representative path-plus-mechanism routine. Verify Driver Station STOP and fault recovery on the real robot.
5. Authors then create actual red/blue match routines using the real placement/strategy. Include per-step timeouts, readiness/outcome branches and an explicit park/stop policy within the 29-second runtime bound. The integration plan does not invent those coordinates or imply the old automatic park fallback still exists.

Acceptance: record hardware evidence/date/configuration for each relevant coverage row. Enable only verified capabilities. A completed software/UI phase does not automatically clear physical calibration gates.

## Planned file changes and ownership

All paths below are relative to the repository root; names for new classes are proposed implementation targets.

| Files | Responsibility |
| --- | --- |
| `TeamCode/.../BioBuzzRobot.java` | Lifecycle, INIT capability/profile wiring, shared clock/snapshots and terminal cleanup; remove inline command/state growth as command factories move out. |
| New `AutonomousCommands.java` and `AutonomousOutputCoordinator.java` | Command contracts/registration and explicit per-output modes/arbitration; reuse existing controllers. |
| New routine-profile/preset configuration classes | Reserved test capability mapping and verified shot presets; no hardware lookup from command execution. |
| `DepollinatorSubsystem`, `TransferSubsystem`, `GateSubsystem`, `TurretSubsystem`, `ShotCoordinator`, relevant controllers | Narrow additions for cached status, clock seams, preserving queue state and separating normal hold/stop from terminal shutdown. Keep current TeleOp APIs/defaults compatible. |
| `PedroDrive`, `PollenNavigator`, `PollenPathFollower` | Add only the required motion/reference/outcome hooks; retain Pedro control and the existing collection bounds. |
| `zenith.json`, `autos/robot.json`, `autos/waypoints.json`, fields, proposed `autos/routines.json` | Canonical project, complete supported command/condition metadata, validated units/provenance and routine classification. |
| `autos/pollen-navigation-test.auto.json`, examples | UI-visible test source and clearly labeled integration examples. Example coordinates are not competition route data. |
| `TeamCode/src/main/assets/autos/` and generated wrappers | Managed deployment output, with a manifest/version/source digest sufficient to detect staleness. |
| `scripts/validate-autos.cjs`, new `scripts/deploy-autos.cjs`, `package.json` | One reproducible author/deploy/check workflow, test-wrapper preservation, safe managed-file cleanup and deterministic generation. |
| `TeamCode/src/test/...` | Focused coordinator/command/runtime/profile/packaging regressions; preserve existing tests. |
| `doc/Autonomous.md`, `doc/PollenAuto.md`, new coverage ledger/authoring guide | Current supported behavior, UI proof, physical verification boundaries and handoff instructions. |

## Definition of done

- [x] Every current robot function has a reviewed entry in the coverage ledger, including maintenance-only functions and read-only sensor information.
- [ ] Every supported UI command has implemented behavior, validated arguments, correct resources, documented persistence/completion, and tested interruption/shutdown semantics.
- [ ] The depollinator and independent intake/turret/hood/flywheel/gate/feed functions are usable as declared; automatic modes do not overwrite later explicit commands.
- [x] Pollen is visible and opens in the actual canonical editor project, remains a separate test, and initializes only its declared hardware in the profile regression.
- [x] Every routine has exactly one intended Driver Station entry in generated/handwritten registration code; add/rename/delete/redeploy checks reject stale/duplicate outputs. Physical Driver Station selection remains part of commissioning.
- [x] The core actual UI edit/save/reopen plus supported team deploy/build workflow has evidence, and routine authors need no Java changes for supported actions. Extra UI checks and source-maintenance boundaries are listed in ZenithUiAcceptance.md.
- [x] Native Zenith geometry/markers/parallel/branch/wait semantics are preserved in the automated fixtures, with clear failures for unsupported inputs. Preview/outcome limits are documented in the audit.
- [x] Unit/runtime/deployment tests and the required TeamCode APK build pass; packaged assets match source.
- [x] Current software/UI evidence and unverified hardware/calibration items are reported separately.
- [ ] Physical tuning/calibration and actual match-route validation are recorded before claiming competition readiness.

## Source references

The baseline is grounded in the working-tree sources listed above and in the pinned upstream source already inspected during integration. Upstream behavior to retain/reverify:

- [Zenith v0.1.2 robot runtime and dependency contract](https://github.com/Horizon-36596/zenith/blob/v0.1.2/robot/README.md).
- [Pinned deployment implementation](https://github.com/Horizon-36596/zenith/blob/v0.1.2/packages/cli/src/commands/deploy.ts): generation, fixed Generated group, opModeBase and deploy preview.
- [Pinned link schema](https://github.com/Horizon-36596/zenith/blob/v0.1.2/packages/schema/src/link.ts): supported project/deploy/codegen/simulation fields; no assumed custom profile/group UI controls.
- [Pinned Ivy runtime](https://github.com/Horizon-36596/zenith/tree/v0.1.2/robot/ivy-runtime/src/main/java/org/horizon36596/zenith/ivy): lifecycle, requirements, native command-tree construction and robot-clock timing.
- Repository `AGENTS.md`, current `doc/Autonomous.md`, and `doc/SimLoopTeleOpImplementationPlan.md` for architecture, the implemented baseline and coordination with the separate test-infrastructure work.
