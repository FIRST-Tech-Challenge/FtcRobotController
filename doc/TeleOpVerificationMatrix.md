# TeleOp verification matrix

## November 6, 2026 physical verification status - scope decision

On November 6 the team decided that simple, low-precision features which have been
run on the robot and seen working are complete and verified by functional check;
they need no logged measurements. Autonomous is not in use and is out of scope.
Physical testing remains only for the precision features (tasks 8, 9, 10 and 12),
which depend on real geometry, physics or timing.

| Task | Verification scope | Physical acceptance status |
| --- | --- | --- |
| 1 | Pinpoint X correction and direction verification | COMPLETE / VERIFIED |
| 2 | Turret and hood | COMPLETE / VERIFIED |
| 3 | Flywheel | COMPLETE / VERIFIED |
| 4 | Loaded transfer/feed sequence | COMPLETE / VERIFIED |
| 5 | Status lights | COMPLETE / VERIFIED |
| 6 | Depollinator integration | COMPLETE / VERIFIED |
| 7 | Limelight and field geometry | COMPLETE / VERIFIED - 35 ms button-to-detection |
| 8 | Both-alliance localization and auto aim | FIXES APPLIED / PHYSICAL RETEST PENDING - tag +Z convention, 55 deg camera pitch, 30 deg tag plane, multi-tag fusion and drift-only corrections (see [diagnostics](LimelightGoalStateDiagnostics.md)) |
| 9 | Shot calibration and measured release timing | PENDING / NOT TESTED - production profile is `UNMEASURED`; `RELEASE_TIMING_MEASURED = false` |
| 10 | Both-alliance combined competition controls and automatic firing | PENDING / NOT TESTED - automatic firing stays blocked until task 9 |
| 11 | Autonomous routes, Pedro path tuning, pollen vision | COMPLETE / NOT REQUIRED - autonomous not in use |
| 12 | Full loaded TeleOp match rehearsal (both alliances, partly drained battery) | PENDING / NOT TESTED - autonomous-to-TeleOp transition not required |

Also COMPLETE / VERIFIED by functional check: build/configuration, drivetrain,
intake, intake jam recovery, queue sensor, gate, transfer-only test, servo zero
calibration, Goal State diagnostic and the camera-only tests. Completed status
records operator confirmation, not logged measurements.

Latest physical observation: after correcting a flipped Red hive, the supplied
`Test: Limelight Goal State 3D` photo shows selected Red/Far and tags 30–33,
approximately 1.11 m forward, areas 0.006–0.007%, all `usable false`. Height/plane
are NaN and no classification pose has been accepted. The old diagnostic replaced
the actual rejection with a generic lost/stale message. Raw rotation/facing
readings remain pending; a physical normal-axis change is not established.
The 35 ms result is button-to-detection, not calibrated camera latency.

Shared rejection diagnostics now cover camera-only tests, vision-enabled turret
telemetry, Auto Aim and both competition TeleOps. Pinpoint's raw local reference
is distinguished from the coordinator's fused field reference. See
[readings and physical follow-up](LimelightGoalStateDiagnostics.md).

Simulation decision for this diagnostic propagation: run Sensors and Competition,
including `cameraRejectionDiagnosticsAgreeAcrossCameraTestsAutoAimAndBothTeleOps`,
existing raw/normalized camera cases, corner rehome and camera loss. The new case
checks readable rejections without localization/feeding; it does not certify
physical pose conventions or promote lifecycle tests to full behavior coverage.
Verification: 219 ordinary unit tests passed, debug APK assembled and lint passed.
All 16 selected Sensors/Competition/runtime simulation tests passed with zero
failures/errors/skips and their expected manifest names present. The new
camera-only setup initially slewed unused fake yaw servos from their default
command; its trace identified the setup error. Initial mount commands were set
to the known stationary home angle, retaining camera geometry, raw axes, finite
physical turret feedback and the rejection assertions. No physical retest or
deployment is claimed.

Shot-profile calibration, release timing, localization/auto aim, integrated
controls, autonomous acceptance and match rehearsal remain open. Production
calibration data and flags were not changed by this documentation correction.
Historical desktop verification below retains its original scope and limits.

Updated November 2, 2026. See [commands and human review](../simloop/README.md),
[model provenance](SimLoopRobotModel.md), [audit and remaining boundaries](SimLoopIntegrationAudit.md).
Simulation remains optional for major changes; minor edits do not require it. Local checks first, CI deferred.

## November 5 drive-direction correction

The team reported that forward/backward, both strafes and both turns all moved
opposite the requested direction. Baseline revision remains
`8e5a77583e9f5752a597fbbaa6d965769e6efbc4` plus the existing dirty working tree.
Reverse all four SDK directions in the shared `MecanumDrivetrain`: left FORWARD,
right REVERSE. Both competition wrappers, the isolated wheel test and every
Pedro/autonomous caller use this one configuration, including direct wheel power.

Simulation decision: run Drivetrain, Competition and Sensors using
`verify-teleop.ps1 -Sim -Scenario Drivetrain,Competition,Sensors`. Explicitly replace
the old assumed physical mounting signs -,+,-,+ with +,-,+,- based on the team's
report, independent of production direction constants. Before the production fix,
the existing transmission regression failed its forward-motion assertion with
the updated reported orientation. At ACTIVE tick 21 the standalone forward-stick
trace had X = -21.266332 in. Baseline log: `build/drive-direction-baseline.log`;
trace: `drive-transmission-DrivetrainTestOpMode-4178037328472642719`.

The new `allDriveOpModesFollowAllSixStickDirections` regression runs 24 deterministic
cases: forward/back, left/right and CCW/CW in the actual isolated drive OpMode,
actual Red/Blue wrappers and actual Pedro diagnostic. Assertions require motion
in the requested coordinate direction, zero unintended translation/rotation,
normal lifecycle completion and zero drive output at STOP. Existing motor-shaft
sign expectations now match the reversed SDK directions; wheel mixing,
normalization, odometry calibration and camera conventions retain their values.
The named-test manifest and Pedro coverage record include the new regression.

Final verification passed **216 ordinary tests and 23 selected simulation tests**
(8 Drivetrain, 9 Competition, 5 Sensors, 1 runtime), zero failures/errors/skips.
The new direction test completed all 24 OpMode/direction cases. Debug APK packaging,
lint (0 errors, 14 existing warnings) and diff whitespace checks passed. Log:
`build/drive-direction-verification.log`; fresh JUnit XML and HTML are under
`TeamCode/build/test-results/` and `TeamCode/build/reports/tests/`. Retained traces
are under `TeamCode/build/sim/teleop/drive-direction-*`; the corrected standalone
transmission trace is `drive-transmission-DrivetrainTestOpMode-13435697449263068262`.

The fixture orientation is inferred from the operator's report, not new shaft
measurements. These simulations check software direction consistency and
integration under that reported orientation, not loaded traction, autonomous path
accuracy or physical acceptance. Full simulation, live human review and physical
robot checks are outside this selected run; no deployment is performed.

## November 2 drivetrain transmission correction

Baseline: revision `8e5a77583e9f5752a597fbbaa6d965769e6efbc4` plus the existing
dirty tree. Six existing drivetrain tests and the existing both-alliance manual
drive/queue/feed scenario passed before the correction. The new
`allDriveOpModesConvertMotorFeedbackThroughBothReductionStages` regression then
failed against the old arbitrary 64 motor ticks/in: the first standalone sample
produced 19.979093 in/s instead of 47.296330 in/s from the independently specified
20:80 gears, 30:91 belt, 28 motor ticks/rev and synthetic 2-in wheel radius. Baseline
trace: `drive-transmission-DrivetrainTestOpMode-8980967890663460574`.
The team subsequently specified 104-mm goBILDA mecanums with 30A rollers; the final
model and assertion use that nominal diameter rather than the baseline's 2-in radius.

The [current hardware and units](Drivetrain.md) now give 12.133333 motor rev/wheel
rev and 339.733333 motor ticks/wheel rev. SDK motor configuration stays at the bare
shaft, with 5800 RPM catalog-tested no-load speed at 12 V. Telemetry distinguishes
motor ticks/s and wheel RPM. Pinpoint pod calibration and unmeasured Pedro dynamics
remain separate.
The nominal rolling conversion is 1.039812 motor ticks/mm (26.411232 motor ticks/in).

Simulation decision: run `verify-teleop.ps1 -Sim -Scenario Drivetrain,Competition,Sensors`.
The new named scenario runs `DrivetrainTestOpMode`, actual Red/Blue competition
wrappers and `PedroDriveTestOpMode` with deterministic manual input. It compares
observed chassis motion against independent motor-feedback conversion and requires
motion, lifecycle completion and zero drive power at STOP. Ordinary regressions
check complete reduction and signed wheel-RPM conversion. The manifest and Pedro
inventory now include this limited manual-drive behavior coverage; no path-accuracy
coverage is claimed. Existing wheel signs, diagonal normalization, aliases, 10/20-ms
ticks, neutral, STOP, repeatability, competition integration and sensor scenarios
remain checked. Nominal wheel diameter is reported hardware; loaded effective
radius, traction, speed and braking are unmeasured.

Final 104-mm configuration verification passed **213 ordinary tests and 20 selected
simulations**, with zero failures/errors/skips, debug APK packaging and lint
(0 errors, 14 existing warnings). Log: `build/drivetrain-104mm-verification.log`;
JUnit XML/HTML: `TeamCode/build/test-results/` and `TeamCode/build/reports/tests/`.
Final transmission traces under `TeamCode/build/sim/teleop/`:

- `drive-transmission-DrivetrainTestOpMode-10446950534902319051`
- `drive-transmission-RedMecanumTeleOp-1579831162864233093`
- `drive-transmission-BlueMecanumTeleOp-3336370818633142822`
- `drive-transmission-PedroDriveTestOpMode-13372907103184927070`

`zenith:validate` passed with zero errors and existing scanner/field warnings;
`zenith:check` reports no stale outputs after local asset/manifest regeneration.
Source and packaged robot JSON match, and every deployment manifest hash matches
its current file. No Control Hub deployment or physical drive measurement was run.
Other simulation selectors and human keyboard review were omitted because this
change affects the drive units/model, not their independent mechanism behavior.

## November 2 Limelight refinement of alliance corner rehome

The corner button now requests a bounded camera refinement in both real alliance
wrappers. It aims the turret toward the nearest alliance hive, waits for 100 ms
of stable measured turret history (within 0.5 degrees), and combines five distinct
post-request frames with the corner prior. All compatible visible tags contribute
with range/area weighting; correlated observations do not artificially divide
uncertainty by sqrt(N). Wall-aligned heading, odometry derivatives and the existing
normal periodic correction behavior are retained. Refined translation lives in
the shared aiming field transform; raw Pinpoint retains its explicit corner frame.
Automatic feeding waits, and camera refinement owns yaw until it finishes.
Manual turret input/home and chassis motion cancel it. Missing/rejected vision
times out in three seconds with CORNER ONLY telemetry; driving never blocks.

Simulation decision: extend Sensors and run Competition, including actual Red/Blue
wrappers and the existing isolated Pinpoint/vision diagnostics. The new
`bothAllianceCornerRehomesRefineFromFreshVisionAndInvertedConventionFallsBack` scenario
uses both INIT/START holds and ACTIVE presses with independent 25/20-mm placement
error, compares the refined position to plant ground truth, checks raw-convention
fallback and continued localization after camera loss. The explicit alternate
fixture raises red far and blue audience tags; existing fixtures are unchanged.
`syntheticCornerRefinementHoldsAutomaticFeedUntilFreshVisionCompletes` uses labeled
synthetic shot calibration in both inherited alliance controllers, holds RT+LT,
asserts no gate/feed during refinement, then requires automatic feeding afterward.
Production calibration flags and camera normal conventions are unchanged.

The initial accuracy assertion caught a wrong-direction correction while the
turret was turning (Red trace `Red-corner-vision-true-5723525427595290814`). Added
the production turret-settling requirement, preserving the assertion. The new
synthetic-feed scenario exposed that the synthetic Blue fixture inherited Red's
new rehome pose (Blue trace `Blue-SYNTHETIC-corner-feed-2228128345536443155`). It
now inherits the actual Blue wrapper and shares only synthetic shot measurements.
Refinement also explicitly owns yaw while pending so held RT cannot override its
camera-pointing command. No perfect feedback, model tuning, relaxed
accuracy assertion or production calibration change was used. The float32 fake
Pinpoint heading comparison uses 1e-6 rad numeric tolerance; unit-level fused
heading preservation is checked at 1e-9 rad.

Final verification: 21 focused localization/geometry unit tests and 12 selected
Competition/Sensors simulation tests passed with zero failures/errors/skips;
`:TeamCode:assembleDebug` and diff whitespace checks passed. The new camera
scenario includes eight real-wrapper runs (both alliances, INIT/ACTIVE, raw and
normalized), plus two synthetic automatic-feed runs. Evidence: final
`build/field-rehome-vision-simulation.log`, retained trace directories and JUnit
reports under `TeamCode/build/test-results/`. Full unit suite, lint, full simulation
and physical testing were omitted; selected scenarios cover the changed coordination.
No deployment.

Unit coverage exercises multi-tag/five-frame fusion, prior weighting, preserved
heading/velocity, post-reset history, old/duplicate/unmapped/lowered/jumping
observations, turret motion, chassis motion, timeout and reference changes.
Physical camera axes, field-map accuracy, wall contact and loaded mechanisms
remain unverified. Desktop accuracy improvement is conditional on the explicitly
normalized geometric camera fixture, not a measured robot accuracy guarantee.

## November 2 alliance corner field rehome — initial implementation

The corner-only report below predates the Limelight refinement above.

GP2 D-pad Left was previously unassigned. Both competition wrappers now provide
their own corner pose opposite the alliance pollen depot: Blue audience-right,
180 degrees; Red far-left, 0 degrees. Existing rear/side dimensions determine the
robot-center offset. The operator seats the robot against the walls; this button
does not navigate or recalibrate the IMU. Autonomous and isolated diagnostics
retain their existing pose controls and starting poses.

Simulation decision: run the focused
`CompetitionSimTest#bothAlliancesFieldRehomeInInitAndActiveUsesPressEdgesWithoutCamera`
scenario against actual Red/Blue wrappers, with both INIT-to-START holds and
ACTIVE presses. All four runs pass: no-camera explicit reference, independent
corner/heading values, continued movement while held, release/repress, and
localization after the next valid acceleration sample. The initial scenario
incorrectly expected automatic aiming on the reset sample; Pinpoint deliberately
invalidates that derivative sample. The final assertions require that rejection
and localization on the following loop. No production validity checks were changed.
`TeleOpLocalizationTest#explicitCornerRehomeDiscardsPreviousVisionTransformAndCaptureHistory`
also verifies removal of an acquired camera transform and rejection of old captures.

Verification: 9 focused localization/field-reference unit tests and the new
simulation scenario passed (4 actual-wrapper runs), zero failures/errors/skips;
`:TeamCode:assembleDebug` passed. Diff whitespace check passed. Evidence:
`build/field-rehome-verification.log` and JUnit reports in `TeamCode/build/test-results/`.
Full unit suite, lint, full simulation and physical testing were omitted for this
localized control addition. Simulation verifies pose reseeding and coordination;
installed wall contact, robot dimensions and Pinpoint frame alignment still need
physical confirmation. No deployment.

Final TeleOp collection overrides (November 2): simulation selected using the
existing `bothAlliancesManualDriveQueueFeedAndInhibit` scenario, extended for
RB loading stage 1 past confirmed full, release back to full hold, LB intake-only
reverse with space available, opposing bumpers across controllers, gate delay
and GP2 LT priority. Both actual Red/Blue wrappers and both manual-feed mappings
are exercised. The existing isolated transfer scenario checks unchanged ordinary
loading/feed behavior; a focused controller regression covers forced loading,
full-state preservation and shot/pause priority. No new scenario or OpMode.
Command and timer assertions do not prove physical ball flow or gate clearance.
Verification: all 7 focused `TransferControllerTest` cases and both selected
simulation tests passed with zero failures/errors/skips; `:TeamCode:assembleDebug`
and `git diff --check` passed. The competition test runs Red and Blue with each
manual-feed mapping. Full unit/simulation suites, lint and physical testing were
omitted for this localized control change.

Conflict-resolution verification (November 2, 2026): retained Dashboard live gains,
bench jam recovery, automatic intake/loading, selectable shooting modes and timing,
and turret feedback on transfer motor 1. Simulation was selected to check their
combined behavior: 202 ordinary tests, debug APK packaging and lint passed; after
reconciling the isolated turret scenario with the already-configured 8-degree hood
home offset, all 13 Mechanisms/Competition/runtime simulations and manifest entries
passed. Jam-recovery tests now distinguish automatic loading from stopped/inhibited
requests and verify that releasing RB preserves recovery when direction is unchanged.
No production calibration or simulation assertions were relaxed. Full simulation and
physical robot testing were not run; ball contact and loaded clearance remain unmodeled.

Full-queue intake stop and 0.2 s confirmation (November 12, 2026): this supersedes the
intake reversal and 0.5 s confirmation described below. Continuous HIGH for 0.2 s
(was 0.5 s) now confirms full and stops automatic intake (no reversal) in both
competition wrappers, the bench test and autonomous forward collection. The 0.5 s
stage-1 run-on that seats the fourth ball is kept, so stage 1 stops about 0.7 s after
HIGH begins. LOW immediately clears full and resumes forward intake and loading.
RB/LB overrides and deliberate autonomous reverse are unchanged. Simulation was
selected for this shared behavior change: `simTeleOpTest` with `SimRuntimeTest`,
`MechanismsSimTest` and `CompetitionSimTest` passed (28 scenarios, both alliances),
the full `testDebugUnitTest` suite passed (268 tests) and `assembleDebug` succeeded.
Scenario assertions were retimed to the 0.2 s confirmation and expect 0 intake power
during the run-on; no scenario or OpMode was added. These are command/timing checks,
not physical ball seating, sensor-placement or loaded-jam validation.

Sensor recovery and bumper overrides (November 1, 2026): this supersedes the earlier
full-latch/intake-stop policy below. Continuous HIGH for 0.5 s reverses automatic
intake; stage 1 runs on for 0.5 s and then stops while HIGH persists. LOW immediately
restores forward intake and loading. RB forces intake forward, LB backward, both
stop, and releasing bumpers restores automatic control. Bench automatic loading
starts after its release-to-arm interlock; stop and shooting pauses remain effective.
Simulation was selected for this shared behavior change:
`verify-teleop.ps1 -Sim -Scenario Mechanisms,Competition` passed 191 unit tests,
13 simulations, debug packaging and lint. Existing bench and actual red/blue scenarios
assert sensor recovery and overrides alongside isolated transfer timing; no new
scenario or OpMode was added. These are command/timing checks, not physical ball,
electrical delivery, or sensor-placement validation.

Bench-only shooting sequence (November 1, 2026): simulation was selected for the
0.10 s stage-2 gate delay, 0.40 s stage-2 feed window, stage-1 gate-open midpoint
start and immediate continuous feed. `verify-teleop.ps1 -Sim -Scenario
Mechanisms,Competition` passed 191 unit tests, 13 simulations, debug APK packaging
and lint (0 errors, 14 existing warnings). The actual bench cycle exercises the
stage ordering; the added `benchContinuousFeedsImmediatelyUntilTriggerReleaseOrStop`
scenario exercises GP2 selection, immediate outputs before flywheel qualification,
held feed, either-trigger release and B/re-arm. Both actual competition wrappers
retain their defaults. Timers and commanded outputs do not prove one-ball
separation, physical gate clearance or loaded recovery.

Stage-1 run-on verification (November 1, 2026): simulation was selected for this shared
sequence change. `verify-teleop.ps1 -Sim -Scenario Mechanisms,Competition` passed
189 unit tests and 12 simulations. Existing scenarios assert bench intake reversal
with stage 1 still forward and stage 2 off, the later stage-1 stop, isolated transfer
timing, and both actual competition wrappers retaining their intake-stop policy.
Controller regressions cover exact 0.5-second boundaries, LOW reset, latched chatter,
and pause cancellation. No new scenario or OpMode was added. These checks validate
commands and timing, not loaded ball travel, sensor placement, or electrical delivery.

## Current automated coverage

All rows retain their existing pure-controller JUnit 4 regressions. A lifecycle smoke test only proves
initialization/loop/STOP under its stated fixture; it does not certify every control or fault branch.

| Capability | Actual OpMode simulation | Assertions / remaining boundary |
| --- | --- | --- |
| Drivetrain | DrivetrainTestOpMode, both competition wrappers, Pedro manual diagnostic | Two-stage motor-feedback-to-distance conversion with reported 104-mm wheels; forward/reverse/strafe/yaw, normalization, wheel selection, neutral/STOP, alias/missing hardware, 10/20ms, repeat decoded traces; chassis geometry/dynamics synthetic, loaded compression/slip unmeasured |
| Gate | GateTestOpMode, TransferTestOpMode, both wrappers | Logical/physical direction, PWM, delayed feed, held/reopened gate, trigger priority, shutdown, missing/PWM-incapable device |
| Intake | IntakeTestOpMode, bench, both wrappers | Forward/reverse/both/neutral, confirmed-HIGH stop (no reversal), LOW recovery, bumper overrides, independent shaft encoder, shutdown; shared 0.5-second jam recovery has controller and stalled-encoder tests; no ball-contact simulation |
| Transfer | TransferTestOpMode, bench, both wrappers | 0.2-second HIGH dwell, 0.5-second stage-1 run-on, LOW resume, stage1-only loading, timed gate/both-stage feed, inhibit priority, STOP |
| Depollinator | DepollinatorTestOpMode, both wrappers | Deploy/home edges, home priority, PWM range/endpoints; holds commanded position at STOP |
| Linear slide / claw | LinearSlideTestOpMode, both wrappers | GP2 LS-click toggle raises across the analog 360/0 wrap and lowers to zero, no power in INIT, jam -> FAULT with zero power, GP1 B claw toggle, zero power at STOP. Scenarios: `linearSlideTogglesAcrossAnalogWrapAndClawToggles`, `jammedLinearSlideFaultsWithZeroPower`, `bothAlliancesToggleSlideOnGp2AndClawOnGp1`. SYNTHETIC unloaded slide (rated Axon speed, no gravity/lag); travel and claw positions are placeholders until measured |
| Turret / hood / flywheel | TurretTestOpMode, bench, both wrappers | Independent finite-rate yaw on transfer motor 1's encoder input, SDK REVERSE sign normalization, paired servo commands, hood held-button edge, shaft response, startup and STOP; loaded response unmeasured |
| Pinpoint | PinpointTestOpMode smoke, full competition fixture | SDK fake calibration/pod configuration and chassis coupling; NOT_READY fault blocks synthetic automatic shots; no physical pod calibration certified |
| Limelight / goal state | LimelightTagTestOpMode, GoalStateTestOpMode smoke | Geometric camera; physical tag +Z-into-print convention resolved 2026-11-05 and converted once in `LimelightObservation`; inverted-camera rejection retained; physical height/plane accuracy unvalidated |
| Localization | Both actual competition wrappers with raw (physical-convention) camera | Mapped-tag acquisition from unknown start and retention after camera loss; inverted convention rejection retained; production automatic feed remains blocked |
| Automatic shot success | Test-only SyntheticRedTeleOp / SyntheticBlueTeleOp inheriting competition loop | Artificial calibrated rows exercise feed then Pinpoint-fault blocking; no production measured flags altered, no ball scoring |
| Competition integration | RedMecanumTeleOp / BlueMecanumTeleOp | Manual drive before reference, queue/feed/inhibit, early START, STOP before START, partial-init cleanup, 180 simulated seconds each |
| Bench | IntakeShooterTestOpMode | Actual intake/stage-1 loading, LB reverse and both-bumper stop, confirmed-HIGH intake stop with stage-1 run-on and immediate LOW recovery; staggered stage-2 feed after 0.10 s gate delay with stage 1 starting at the gate-open midpoint; GP2 continuous selection, immediate both-stage full-power feed, either-trigger release and B/re-arm; RB reload/STOP. Scenarios: `benchLoadsHoldsFullFeedsAndReloadsThroughActualMotorOutputs`, `benchContinuousFeedsImmediatelyUntilTriggerReleaseOrStop`, `benchFusionControllerSpinsUpHoldsAndFeedsThroughActualOpMode` (GP2 B fusion selection, full-power spin-up, feedforward hold inside readiness, staggered feed). Modeled feedback, no physical clearance, ball-spacing or flywheel-tuning claim |
| Aiming / calibration diagnostics | AutoAimTestOpMode, ShotCalibrationOpMode | Real lifecycle smoke; detailed planner/calibration logic remains covered by ordinary tests; calibration does not create real measured rows in simulation |
| Pedro diagnostic | PedroDriveTestOpMode | Manual forward drive with both transmission stages, independent shaft-feedback-to-distance assertion and STOP in Drivetrain; idle lifecycle smoke in Sensors. Pedro clock instrumented; no autonomous path-accuracy claim |
| Raw disconnected-servo programmer | Not simulated | Upstream FakeServo has no ServoController metadata; use actual disconnected-servo procedure |
| Pollen / webcam | Not simulated | VisionPortal/USB/webcam processing unsupported by this harness; existing controller tests and hardware evidence only |
| Zenith UI-authored autonomous tree | Ordinary native-runtime regressions, outside `simTeleOpTest` | Actual saved UI JSON exercises curve/speed/marker, parameterized actions, bounded condition wait and stop through native AutoBuilder/Ivy. See [UI record](ZenithUiAcceptance.md); this is not autonomous chassis physics or camera capture coverage. |

## Physical evidence retained

November 2 encoder-wiring verification: `verify-teleop.ps1 -Sim -Scenario
Mechanisms,Sensors,Competition` passed 195 ordinary tests and 15 existing simulations,
debug APK packaging and lint. Fixtures now put turret feedback on transfer motor 1
and intake feedback on its own shaft. At that review jam recovery remained bench-only (shared as of November 4); its timing
and cancellation have controller tests, with physical jam clearing unverified.

The [hardware-review record](HardwareReviewImplementationStatus.md) records intake/transfer completion
October 26, turret rotation/hood/shooting checks October 24/26 and gate mechanism good October 26. This audit
does not revoke unchanged sign-offs or turn them into numerical calibration. New mechanics, wiring,
direction, limits or timing changes invalidate the corresponding evidence.

Still required on the robot: camera target-axis sample and known-pose acquisition; measured shot table
and release timing; loaded gate clearance/feed timing; drive/turret/flywheel dynamic measurements;
Control Hub loop-period p50/p95/p99/max under simultaneous load. Fixed sim ticks cannot benchmark I2C,
USB, Lynx bulk caching, scheduling, voltage sag or thermal behavior.

## Evidence and review policy

JUnit XML/HTML records assertions. Scenario TSV records inputs, commands, physical model states and
telemetry; runner-outcome.txt is not a behavioral verdict. Human recordings include environment events,
profile and STOP tick and can be replayed with exact decoded-output comparison. Record source revision
and dirty changes alongside reviewer notes. The baseline manifest prevents silent test-discovery loss.

Competition and bench policies intentionally differ: competition starts at 3500 flywheel output RPM,
uses GP1 A / GP2 RT manual feed and GP2 LT inhibit; bench uses its dedicated readiness/release controls.
Keep those policies distinct when promoting a subsystem change. Production shot rows remain examples;
synthetic success tests do not authorize automatic shots on the robot.

## November 1, 2026 flywheel hardware update

The reported shooter is a goBILDA 5203 Yellow Jacket 6000 RPM motor with 1:1
motor-to-flywheel drive. This replaces the previous 2:3 conversion and 3700 RPM
ceiling. All shooter commands may request 6000 RPM; no lower measured-speed cap
or recovery reserve is imposed. Existing bench/competition output-RPM presets
and readiness tolerances are retained. Actual-versus-target RPM telemetry is the
basis for the team's upcoming physical maximum-speed measurement.

Simulation decision: run the existing Mechanisms and Competition scenarios via
`tools/verify-teleop.ps1 -Sim -Scenario Mechanisms,Competition`, including the
isolated TurretTestOpMode, combined bench test and both actual alliance wrappers.
The independent fake flywheel now uses the catalog 6000 RPM / 28 ticks-per-rev
plant; the drivetrain at that time used its separate 5550 RPM baseline (superseded
by the November 2 drivetrain correction above). Ordinary tests
check 1:1 encoder conversion, full 6000 RPM command/profile/calibration acceptance,
out-of-range clipping and feedforward. No new scenarios or OpModes were added.
These checks do not measure loaded recovery, reachable RPM, ball flight or a new
shot table. Production calibration flags remain unchanged.

Verification result: 185 ordinary tests and 12 selected simulations passed with
zero failures, errors or skips; debug packaging and lint passed. Evidence:
`build/flywheel-1to1-verification.log` and JUnit reports under
`TeamCode/build/test-results/`. Existing Java/source compatibility and Gradle
deprecation warnings remain. `git diff --check` and matching autonomous registry
copies were also checked. Physical maximum RPM is pending team telemetry data.

Follow-up physical evidence: team reports 4700 RPM maximum at a 6000 RPM command,
plus 100–200 RPM underspeed at operating targets, blocking the existing 100 RPM
readiness check. Controller mode, battery voltage and loading are pending. This
is an observation, not a voltage-normalized free-speed calibration. Added explicit
error/tolerance/readiness telemetry; command limits and gains remain unchanged.
Simulation omitted for observation/telemetry-only changes; regulation changes await
the missing measurement conditions. See ShooterVelocityTuning.md.

Follow-up verification: 185 unit tests, debug packaging and lint passed. Evidence: build/flywheel-speed-observation-verification.log. Simulation was not run for these telemetry-only edits.

## November 1 measured-speed feedforward follow-up (superseded observation)

The team clarified the observation: 4700 RPM maximum at 14 V in both PIDF and
bang-bang modes. Updated PIDF feedforward and speed-usage telemetry to reference
that installed-speed observation, with linear voltage scaling; command ceiling
remains 6000 RPM and readiness remains +/-100 RPM. P/I/D and bang-bang law are
unchanged. The selected lower-target bang-bang trace and instantaneous power are
still needed to distinguish saturation from sampled cycling.

Simulation decision: reproduce with the existing bench transfer-cycle scenario,
then run Mechanisms and Competition (isolated turret, bench, actual Red/Blue).
The independent fixture now uses the reported 4700 RPM at 14 V rather than the
catalog 6000 RPM estimate. Response/drag/ball-load dynamics remain synthetic.
Before changing feedforward, the bench scenario failed its existing feed assertion
at MechanismsSimTest.java:105: physical feedback settled around 2550 RPM for a
2800 RPM command and feeding stayed blocked. Evidence retained in
`build/flywheel-measured-baseline.log`, `build/flywheel-measured-baseline-test.xml`
and `TeamCode/build/sim/teleop/bench-transfer-cycle-8605208065857636247/`.
No assertions, readiness tolerances or production calibration flags were relaxed.

After correction: 185 unit tests and 12 selected simulations passed with zero
failures/errors/skips; debug packaging and lint passed. The unchanged bench feed
assertion now passes. Final evidence: build/flywheel-measured-tuning-verification.log,
JUnit XML/HTML, and TeamCode/build/sim/teleop/bench-transfer-cycle-8177026944439584903/.
A first verification run caught the headroom telemetry formula still using the
catalog reference; corrected it to the observed reference without changing the
expected 4700 RPM at 14 V assertion. Physical tracking/overshoot and the bang-bang
underspeed report still require a robot trace at the selected operating target.

## November 1 maximum-speed correction: 5400 RPM

The team corrected the maximum to **5400 RPM**, superseding the earlier 4700 RPM
report. Retain the previously supplied 14 V condition. Earlier entries above are
historical test evidence, not current calibration. Updated the shared observation,
PIDF feedforward, speed-usage estimate, independent flywheel fixture, tests and
current tuning/model documentation. Command ceiling stays 6000 RPM, motor-to-
flywheel drive stays 1:1, and readiness tolerance stays +/-100 RPM. Feedforward is
approximately 13.003 at 14 V (15.170 normalized to 12 V), still a provisional
linear voltage/speed fit given the reported intermittent startup/encoder behavior.

Simulation decision: rerun existing Mechanisms and Competition scenarios against
the corrected independent 5400 RPM at 14 V fixture, including isolated turret,
bench feeding and actual Red/Blue competition wrappers. No new scenario or OpMode,
no assertion relaxation, and no production calibration flags changed. No claim
that this parameter correction fixes the intermittent physical startup issue.

5400 RPM correction verification: 185 unit tests and 12 Mechanisms/Competition simulations passed, zero failures/errors/skips; debug packaging and lint passed. Evidence: build/flywheel-5400-verification.log and JUnit reports. Existing Java/Gradle warnings remain. Physical speed regulation and startup repeatability remain unverified.

## November 2 competition feeding mode toggle

Both production alliance wrappers share `CompetitionFeedController`: GP2 B
selects continuous/staggered (starts continuous) for manual and automatic feed.
The prior baseline had timed automatic feed, continuous manual feed and no mode
binding. Staggered mode now also delays stage 1 to the gate-open midpoint.

Simulation decision: run Mechanisms and Competition, retaining the bench and
transfer-only defaults and exercising both real Red/Blue wrappers. The new
`bothAlliancesToggleContinuousAndStaggeredFeedsOnEitherManualControl` scenario
holds each manual control in turn, holds/releases B to check rising edges,
switches modes during feeding, checks stage-2 lead and recovery, asserts GP2 LT
priority and release/restart, and keeps driving throughout. Controller tests
also assert automatic validity/readiness and loss-of-RPM behavior in both modes.
The existing synthetic-calibration scenario covers actual automatic coordination;
production calibration flags and camera conventions are unchanged. These checks
verify motor/gate commands, not loaded ball spacing or physical travel. Final
staggered timing measurements remain pending from the team.

Verification: 205 ordinary tests and 14 Mechanisms/Competition/runtime simulation
tests passed with zero failures/errors/skips. Debug APK packaging and lint passed
(0 errors, 14 existing warnings). Evidence:
`build/competition-feed-toggle-verification.log` and JUnit reports under
`TeamCode/build/test-results/`. Full simulation and physical testing were omitted;
the selected scenarios cover the affected coordination. No robot deployment.

## November 2 competition prepare/fire controls

GP1 RT now prepares yaw, hood and flywheel RPM; automatic feed requires RT+LT.
Releasing either trigger cancels automatic feed. GP1 A replaces LT as the manual
readiness bypass; GP2 RT, GP2 LT inhibit and the feed-mode toggle retain their roles.
This is competition input mapping: autonomous and isolated diagnostic controls
were reviewed and retain their separate procedures. The existing `ShotCoordinator`
overload already separates aiming from automatic feed requests.

Simulation decision: extend and run the existing Competition scenarios for both
real alliance wrappers and the explicitly synthetic-calibration subclasses in
continuous and staggered modes. Assert RT-only preparation commands hood/RPM
and moves physical yaw from an off-center starting pose while keeping the gate
closed; RT+LT feeds; releasing either trigger stops; LT alone does not feed; and
invalid odometry still blocks RT+LT. Existing manual-feed scenarios use GP1 A and
GP2 RT and retain feed-mode and inhibit assertions. The renamed manual-control
scenario is reflected in the manifest; no new OpMode or model calibration.

The first run caught an incorrect new yaw-motion assertion: the original fixture
was already aligned with the hive. Retained that trace under
`TeamCode/build/sim/teleop/Red-SYNTHETIC-calibration-false-18367486020751945435/`
and the failure in `build/competition-trigger-controls-verification.log`.
The revised scenario starts 8 inches off the hive centerline to require yaw
movement. Production calibration flags, camera convention, readiness checks and
simulated physical feedback were preserved. Ball flow, loaded clearance and
shooting accuracy remain outside this fixture's evidence.

Verification: all 8 focused firing/controller tests and all 6 Competition
simulation tests passed (zero failures/errors/skips); the scenarios include both
alliance wrappers, both feed modes and both manual controls. Debug APK packaging,
the selected simulation manifest check and `git diff --check` passed. Final evidence:
`build/competition-trigger-controls-simulation.log` and current JUnit reports.
Full unit tests, lint, other simulation groups and physical tests were omitted
for this localized mapping change. No robot deployment.

## November 4 final TeleOp software consistency pass

Scope and source-by-source disposition: [final TeleOp audit](TeleOpConsistencyChecklist.md).
Both wrappers retain continuous startup, 3500 RPM and GP2 B feed-mode selection.
Software integration adds shared intake recovery, collection during aim-only,
collection pause during firing, bounded auto offsets and independent status lights.
AutoAimTestOpMode now uses TELEOP_ODOMETRY; its current simulation evidence is still
lifecycle-only and must not be described as full aiming coverage.

Baseline: reviewed the pre-existing working tree and retained its tests/control
policies. A default-Java baseline launch stalled and was canceled without results.
The explicit-JDK compile passed. Per the user's request, the revised method uses
JDK 25 directly, offline dependencies, focused test filters and a 120-second process
cutoff. The completed run took 64 seconds and did not hit the cutoff.

| Evidence run | Result / limits |
| --- | --- |
| TeleOpIntegrationPolicyTest, IntakeShooterTestControllerTest, CompetitionFeedControllerTest, FiringCoordinatorTest, TransferControllerTest | 43 tests, zero failures/errors/skips |
| Both actual Red/Blue wrappers: aim-only collection, pause for requested fire, independent lights, simulated intake encoder stall, STOP | Passed; uncalibrated production profile remains blocked |
| Both explicit synthetic-profile subclasses: GP2 auto offsets, held-button edges, clear and manual RPM fallback | Passed; profile and normalized camera basis are test-only |
| Both actual wrappers: preserved manual feed/inhibit/bumper priorities and continuous/staggered selection | Passed for both manual inputs |
| Isolated intake and LightsTestOpMode | Stall timing/release cancellation, independent channel selection, 1500/1100/1700-us colors and 600-us STOP commands passed |
| Sensors diagnostic lifecycle scenario | Passed; includes AutoAimTestOpMode but does not certify its localization/aim performance |

The seven selected simulation methods passed with zero failures/errors/skips;
methods parameterize both alliances and, where relevant, both manual inputs.
Production source compilation and test compilation succeeded. No calibration flags
or feedback models were relaxed. No ball contact, loaded feeder clearance,
actual launch, optical color, or physical jam clearing is claimed.

Logs: `build/teleop-audit-focused.log`, `build/teleop-audit-focused-errors.log`.
JUnit XML: `TeamCode/build/test-results/testDebugUnitTest` and
`TeamCode/build/test-results/simTeleOpTest`; detailed retained simulation traces
are under `TeamCode/build/sim/teleop`. New scenario IDs and LightsTestOpMode are
registered in the test baseline and OpMode inventory.

Full ordinary suite, lint, full simulation, APK packaging and physical tests were
omitted for this bounded pass; no commit or deployment was performed. Packaging
remains required before a later commit/deployment. Diff/manifest review completes
this software-only audit.


## November 5 competition gate update

Baseline: competition previously waited 0.20 s, fed 0.50 s and stopped both stages during 0.35 s recovery. New policy: 0.10 s delay, 0.30 s feed, 0.40 s total gate open; stage 1 starts at 0.20 s. Both transfers continue through the closed-gate 0.10 s recovery and readiness wait, then stop for the next opening delay. Continuous feed starts both stages with the open command, retaining automatic readiness/inhibit. Gate power is direct Expansion Hub port 3, without 6 V injection; immediate full target commands retain maximum available speed.

Simulation decision: update and run the existing `bothAlliancesToggleContinuousAndStaggeredFeedsOnEitherManualControl` scenario against both actual competition wrappers, plus existing competition and isolated gate/mechanism checks as appropriate. No new scenario or OpMode is introduced. Controller tests cover recovery persistence, repeated staggering and cancellation. The model asserts commanded outputs only; direct-hub electrical speed, ball pressure and loaded clearance are unsupported and unmeasured. Calibration flags and raw/normalized camera conventions are preserved. Results are recorded below after execution.

Verification result (November 5, 2026): 40 affected controller tests and 18 existing Competition/Mechanisms simulation tests passed with zero failures/errors. This includes both real alliance wrappers and the isolated gate/transfer/bench checks. `:TeamCode:assembleDebug` passed. `git diff --check` passed. Full unit-suite, lint, FullSim and robot deployment were omitted for this focused change. Physical direct-hub speed and loaded clearance were not verified.

Recovery tuning follow-up (November 5, 2026): competition closed-gate recovery is now 0.10 s, with both transfers continuing through recovery and readiness waiting. The focused CompetitionFeedControllerTest passed, including checks before/after the new recovery boundary and unchanged 0.35 s diagnostic/autonomous defaults. Diff formatting check passed. This minor value adjustment did not rerun simulation, full-suite/lint checks or debug packaging; no deployment was performed. Earlier verification above refers to the prior recovery value.

## Physical Limelight tag-normal convention (2026-11-05)

Change: a Driver Station frame from Test: Limelight Goal State 3D (Red / Far, raised tags 30-33,
about 1.12 m, stationary) showed every tag rejected only by the facing check with raw scores of
-0.96 to -0.99. The installed Limelight reports tag +Z into the print. `LimelightObservation`
now applies Rx(pi) (SDK roll + pi) once at the SDK boundary, so both Limelight tests, Auto Aim,
corner rehome and both competition TeleOps share it. Thresholds, axes downstream, shot calibration
flags and driver controls are unchanged.

Simulation decision: run because shared localization input changed. The raw SimLoop camera
(same convention) is now the default fixture; the former normalizing adapter is the inverted-camera
rejection fixture. Renamed cases: `physicalTagConventionAcquiresAndSurvivesCameraLoss`,
`invertedTagConventionNeverLocalizesOrFeeds`,
`bothAllianceCornerRehomesRefineFromFreshVisionAndInvertedConventionFallsBack`. The cross-OpMode
rejection-diagnostics case now uses the inverted camera. New unit regression
`PhysicalLimelightFrameTest` keeps the photographed values.

Limits: the same physical readings measure tag height about 0.91 m (mapped 1.268 m +/- 0.10) and
single-tag plane angles of about 49-72 deg (60 +/- 10). Those are physical geometry questions and
remain unresolved; simulation cannot answer them.

## Limelight tilt 55 degrees and 30-degree tag plane (2026-11-05)

Change: the team measured the Limelight front glass 55 degrees from vertical (lens axis 55 degrees
above horizontal; an interim 35-degree setting from the same session was the glass angle from
horizontal) and the raised tag stickers 60 degrees from vertical (30 degrees from horizontal).
`ShootingConfiguration.CAMERA_PITCH_UP_DEG` is now 55; the shot-calibration export and SimLoop camera
mount read it. Field acquisition (`PoseHistory`) and the shared raised-tag check
(`TeleOpFieldReference`, used by corner refinement) now compare tag plane against
`HiveTagGeometry.RAISED_TAG_PLANE_DEG` = 30 instead of the 60-degree opening plane; the goal-state
classifier already used 30. Lens height stays 335 mm; tag size 3.25 in was confirmed.

Simulation decision: run the full simulation set because camera pitch and tag-plane acceptance feed
localization, goal-state and aiming in both competition TeleOps, Auto Aim and both Limelight tests.
Fixture updates follow the measurements: SimLoop tags now tilt their normal 60 degrees down
(`RAISED_TAG_TILT_UP_RAD`), and raised-tag scenarios start 40 in from field center
(`RAISED_TAG_VIEW_Y_INCHES`) because a probe showed the 55-degree camera loses the tags beyond about
52 in. From the corner-rehome pose the tags are below the image, so the corner scenarios were
renamed and assert the CORNER ONLY fallback:
`bothAllianceCornerRehomesFallBackToCornerWhenRaisedTagsAreBelowTheImage` and
`syntheticCornerRefinementHoldsAutomaticFeedUntilCornerFallbackCompletes`. Vision-refined corner
fusion remains covered by `TeleOpLocalizationTest` unit cases.

Results: 221 unit tests, lint, debug APK, and all 37 simulations passed.
Limits: the simulated camera follows the same constants, so simulation checks consistency, not the
physical values. Photographed single-tag plane angles (about 36-46 degrees) are marginal against
30 +/- 10 and need a fresh robot frame.

## Hive tag left/right order (2026-11-05)

Symptom: after localizing on the red far tags, RT tracked a fixed point to the right of the goal.
Cause: the field map placed each cell's four tag IDs in mirror order. Photographed red far tags
read camera-right 0.232/0.139/0.000/-0.142 m for IDs 30-33, while the map put tag 30 at the
robot's left; a single-tag fix then shifted the believed pose sideways by up to 13 in. The
stickers face down with their bottom edge toward field center, so the robot sees them upside down.
`HiveTagGeometry.nominalRaisedTags` now puts far IDs increasing with field X and audience IDs
decreasing with X (red far observed; other cells follow the same mounting rule, not yet observed).
The photographed tags' own axes and offsets agree on one cluster center within 3 cm.

Simulation decision: run all simulations because localization feeds aiming in both competition
TeleOps, Auto Aim and both Limelight tests. SimLoop tag layouts were flipped to the same physical
order. New unit regression: `PhysicalLimelightFrameTest#redFarTagOrderAndClusterCenterMatchThePhysicalFrame`.
Limits: simulation shares the map, so it checks consistency; aim accuracy needs a robot test.

## Multi-tag field fusion and settle gating (2026-11-05)

Symptom: with the robot still, RT made the turret jerk 3-4 times, then track with drift.
Change (shared `PoseHistory`, used by Auto Aim and both competition TeleOps through `ShotCoordinator`):
fuse all accepted tags per frame; derive heading from each tag's horizontal +X edge instead of the steep
outward normal (physical frame spread 5 deg vs 12 deg); average acquisition over tags and frames; apply
35% heading corrections about the capture position (reject > 8 deg) alongside position; skip frames when
turret or chassis turned faster than 20 deg/s around capture. Corner refinement keeps its stricter check.

Simulation decision: run all simulations because localization and aiming are shared by both competition
wrappers, Auto Aim and both Limelight tests. Probing showed SimLoop draws stickers upright (tag +X
camera-right) while the physical frame has them upside down (tag +X camera-left), so the sim camera adapter
now applies Rz(pi) in target coordinates for every camera; the inverted-normal case adds Rx(pi).
`physicalTagConventionAcquiresAndSurvivesCameraLoss` now also asserts the localized pose within 3 cm and
1.5 deg of the plant and turret yaw steady within 0.5 deg while stationary with RT held. New unit cases:
`turretSwingAtCaptureDefersFieldCorrectionWithoutMovingThePose`,
`laterFramesCorrectHeadingGraduallyAboutTheCapturePosition`, `perTagEdgeNoiseAveragesWithinOneFrame`.
Unit fixtures now build full physical tag orientation (`TagFixtures`) and seed 0.1 s of settled history.
Limits: real Limelight latency and orientation noise are not modeled; robot verification still required.

## Drift-only camera corrections and turret dead-zone wrap (2026-11-06)

Change: in `TELEOP_ODOMETRY` (both competition TeleOps and Auto Aim) the Limelight sets the first field pose,
then `PoseHistory.checkDrift3d` only compares settled multi-tag frames with odometry: five consistent frames
averaging over 4 cm / 1.5 deg are blended in over 0.4 s; smaller differences change nothing; over 0.5 m / 8 deg
are ignored. The 0.5-s periodic 35% correction no longer runs in TeleOp (the diagnostic path keeps it).
`TurretSubsystem.reachableYawDegrees` is now the single automatic-aim yaw rule (planner, odometry fallback,
visual tracking, corner refinement): hold the current side's limit inside the rear +/-5-deg dead zone and unwind
the long way only after the bearing is 3 deg past the zone. Limelight UI orientation (upside-down) documented.

Simulation decision: run all simulations because the change affects shared aiming and localization in both
competition wrappers, Auto Aim and both Limelight tests. New scenario
`bothAlliancesSpinMoreThanOneTurnWhileAutoAimRespectsDeadZoneAndUnwinds`: real Red/Blue TeleOps localize, hold RT
and spin more than one turn in opposite directions; asserts measured turret yaw never enters the dead zone, the
turret swings back through home after reaching a limit, and ends within 2 deg of the odometry bearing.
New unit cases cover deadband, blended drift correction (per-loop step under 1.2 cm / 0.25 deg) and ignored
large disagreement, plus the wrap rule. Limits: the turret servo and Limelight latency are modeled, not measured;
thresholds are starting values for field tuning.

## Lens height 337 mm and field-guide confirmation (2026-11-06)

Change: `ShootingConfiguration.CAMERA_HEIGHT_MM` 335 -> 337 (team re-measure). `TurretSubsystem.LIMELIGHT_LENS_HEIGHT_MM`,
`ShotCalibrationExport`, the SimLoop camera mount and `GoalObservationTest` now read that one constant instead of
their own 335 copies. Lens axis stays 55 deg up: the team's "35 deg from horizontal" is the front glass/face angle (team-confirmed), and the
2026-11-05 photo tags compute to 1.26 m with 55 deg (0.99 m with 35 deg) against the TU06 nominal 1.268 m.
TU06 Section 9 nominal values (tag 3.25 in, opening 53.5-65.6 in, face 30 deg from vertical, tag underside 30 deg
from horizontal) were checked against `FieldGeometry`/`HiveTagGeometry`; they already matched, so no map change.

Simulation decision: minor calibration value change; ran unit tests, packaging and the Sensors scenario because
the sim camera now reads the shared height. No assertion or tolerance changed.

## Auto aim while turning: same-loop servo write, aim lead, turn cap (2026-11-06)

Change: `TurretSubsystem.update()` now only reads sensors (and advances the hood estimate); the new
`applyYawCommand()` writes the yaw servos after this loop's target, in both competition TeleOps, Auto Aim, the
Turret, Shot Calibration and Intake/Shooter tests, and autonomous (`AutonomousOutputCoordinator.apply`,
`BioBuzzRobot` INIT / not-running loops). Before, the servo received the previous loop's target. The
auto-aim servo command (`ShotPlanner.Solution.commandYaw`, odometry fallback, corner refinement, diagnostic
visual path) uses the pose led by `TurretControl.AIM_LEAD_SECONDS` (25 ms, UNMEASURED); `Solution.yaw` and
the 3-degree readiness check keep the bearing now. `AutoAimSpinLimiter` caps GP1 turning at 300 deg/s
(UNMEASURED) while RT is held, learning the stick-to-rate gain from Pinpoint. `TurretLagEstimator` and a
turret peak-rate readout provide the measurements for both constants (see CompetitionTeleOp.md).

Simulation decision: shared aiming in both competition wrappers, Auto Aim and autonomous, so the full suite ran
(40 scenarios, all pass) plus ordinary unit tests (237, all pass), `assembleDebug` and `lintDebug`.
Baseline (HEAD 6cc3023, new scenarios only): `bothAlliancesTrackTheGoalWithinShotToleranceWhileSpinning`
failed with Red mean 5.34 / max 5.52 deg at a 126 deg/s spin; `bothAlliancesCapChassisTurnRateOnlyWhileAutoAiming`
failed with 419 deg/s while aiming. After: Red mean 0.61 / max 0.63 deg, Blue 0.55 / 0.58 deg (assert max <= 3,
mean <= 1); turn rate 300 deg/s aiming vs 419 free. The tracking-sample sanity count is > 100 (baseline had 149;
unwinds take much of each turn); error thresholds were set before the baseline run and not changed.
Limits: the sim turret is a 180 deg/s slew with no servo lag, so it checks ordering and lead sign, not the robot's
lead; sim drive gain is synthetic. Rear dead-zone unwinds still leave the turret off target once per chassis turn.

## Auto-aim audit edge-case fixes (2026-11-10)

Change (shared by both competition TeleOps and Test: Auto Aim; see CompetitionTeleOp.md):
- `TeleOpFieldReference.nearestSide` switches cells only 5 cm past Y = 0 (`SIDE_SWITCH_HYSTERESIS_METERS`).
  Measured before: the cell flipped at exactly Y = 0, swinging the aim 63 / 38 / 27 deg at 1.0 / 1.5 / 2.0 m.
- `TurretControl` keeps a learned encoder correction through continuous target motion; a jump over
  `ENCODER_CORRECTION_RESET_STEP_DEG` (10 deg) in one update or lost home resets it. Measured before (probe, servo
  settling 4 deg short): outside the 3-deg shot tolerance 17% of the time at 2 deg/s and 90% at 5 deg/s.
- `TurretSubsystem.reachableYawDegrees`: from the front half (|yaw| <= 90 deg, including mid-unwind) a dead-zone
  bearing takes the nearer limit; the rear-half hold and 3-deg unwind hysteresis are unchanged.
- `MecanumTeleOp`: with RT held, GP2 D-pad / Y / A trim hood/RPM only when `ShotCoordinator.automaticHoodRpmAvailable()`
  (calibrated, aimable solution before trims); otherwise they step the manual hood/RPM the shot is using.

Simulation decision: shared aiming behavior in both competition wrappers, so ran `verify-teleop.ps1 -Sim -Scenario
Competition,Sensors`: 251 unit tests, 21 scenarios, `assembleDebug` and `lintDebug` (0 errors, 14 existing warnings),
all pass. New scenario `bothAlliancesAdjustManualHoodAndRpmWhileAimingWithoutACalibratedProfile` (real Red/Blue
TeleOps, installed uncalibrated profile, RT held, one GP2 D-pad Up + Y press): baseline HEAD 27fe90b fails (hood
stayed at home 0.54507 instead of the +0.5-deg 0.54789); after, hood moves 0.5 deg, target RPM 3550, trims stay
0 / 0 / 0. The synthetic calibrated trims scenario still passes unchanged. Spin tracking unchanged: Red mean 0.61 /
max 0.63 deg, Blue 0.55 / 0.58 deg. New unit cases: midline band and weave (pure and through `ShotCoordinator`),
nearer dead-zone limit and no unwind reversal, correction kept at 0 / +/-5 / 2 / 20 deg/s and reset by an 11-deg jump
or lost home, and hood/RPM trim availability including an out-of-range trim.
Limits: the 4-deg servo shortfall and 30-ms response are synthetic; the robot's real servo error, backlash and
direction-reversal behavior are unmeasured (a large backlash would need re-learning after a reversal). The 5-cm
band and 10-deg step are starting values.

## Autonomous-to-TeleOp pose handoff (2026-11-10)

Change: new `AutoPoseHandoff` (RAM, one app session). `BioBuzzRobot` (both alliance autonomous wrappers) clears it
at INIT and on a Pinpoint localization fault and saves the READY, field-referenced pose every running loop
(cleared-area local starts are ignored). `MecanumTeleOp` (both competition wrappers) consumes it at INIT; a
same-alliance pose at most 60 s old is written to the Pinpoint after it has reported READY for 0.5 s following the
INIT IMU recalibration, before that loop's Pinpoint read. GP2 D-pad Left discards a pending pose; START discards one
not yet applied. Driver INIT view shows **Auto pose handoff**; GP2 Back diagnostics show it while active.
Test: Auto Aim, diagnostics and the pollen test are unchanged.

Simulation decision: new TeleOp INIT behavior in both competition wrappers, so ran `verify-teleop.ps1 -Sim -Scenario
Competition,Sensors`. New scenario `bothAlliancesStartTeleOpAimingFromTheHandedOffAutonomousPose` runs the real
Red/Blue TeleOps through test-only `HandoffRed/BlueTeleOp` subclasses that save the autonomous end pose inside the
simulated class loader and clock, with no camera. Asserts LOCAL at INIT tick 5, APPLIED and EXPLICIT FIELD by tick
150 (applied at tick 38, 0.76 s), LOCALIZED in ACTIVE and turret yaw within 2 deg of the plant bearing (87.49 vs
87.5 deg computed by hand, both alliances). Result: 257 unit tests, 22 scenarios, `assembleDebug` and `lintDebug`
(0 errors, 14 existing warnings), all pass. Baseline: HEAD has no handoff, and the existing no-camera scenario
`unknownFieldReferenceInhibitsAutoFireWithoutBlockingDrive` shows TeleOp then stays unlocalized. New unit cases:
apply once after the settle interval, settle restart after a CALIBRATING read, other alliance / over 60 s / clock
going backwards ignored and consumed, only READY field-referenced finite poses saved, corner rehome / early START
discard, and autonomous INIT plus localization fault clearing the stored pose.
Limits: the sim Pinpoint's reset/READY timing is the fake's, not the device's; the 60-s age and 0.5-s settle are
starting values. A wrong pose from a robot moved in practice is not corrected by the camera beyond 0.5 m / 8 deg
(superseded the same day by the confirmed re-localization below, when 2+ tags are visible).

## Confirmed camera re-localization of large odometry errors (2026-11-10)

Change: `PoseHistory.checkDrift3d` (both competition TeleOps and Test: Auto Aim, `TELEOP_ODOMETRY` only) no longer
ignores every frame that disagrees with odometry by more than 0.5 m / 8 deg. Such frames are collected; once at least
10 settled frames spanning 0.5 s within 1 s, each fusing 2+ agreeing tags, imply the same rigid correction (within
the drift agreement 4 cm / 1.5 deg, about their mean odometry position) and an on-field pose, the correction is
blended in over 0.4 s through the existing drift blend. A frame that agrees with odometry, an inconsistent set, a
single-tag frame or an off-field camera pose restarts the count. Small-drift handling, the diagnostic `observe3d`
limits and autonomous (no camera corrections) are unchanged. `ShotCoordinator` shows **Camera pose check**
(CAMERA DISAGREES with progress for 1 s, RE-LOCALIZED for 3 s) in the driver view and diagnostics.

Simulation decision: shared TeleOp localization in both wrappers, so ran `verify-teleop.ps1 -Sim -Scenario
Competition,Sensors`: 259 unit tests, 23 scenarios, `assembleDebug`, `lintDebug` (0 errors, 14 existing warnings),
all pass. New scenario `bothAlliancesCameraReLocalizesAWrongHandedOffPose`: real Red/Blue TeleOps via test-only
`MisplacedHandoffRed/BlueTeleOp` hand off a pose 0.6 m and 12 deg wrong in front of the audience cell, with the
raised-tag camera. Both alliances: disagreement reported at INIT tick 99 (as the turret finished homing),
re-localized at tick 126 (moved 60 cm, turned 12.0 deg), ACTIVE yaw -4.58 deg vs -4.58 deg true bearing (assert
<= 2 deg); the wrong pose would have aimed at 28.6 deg (Red) / -34.2 deg (Blue), asserted > 20 deg off. Baseline:
before this change every such frame returned `Camera disagrees with odometry ... ignored` (existing
`largeCameraDisagreementIsReportedAndIgnored`, which still passes: 8 frames over 0.14 s stay unconfirmed).
Spin tracking unchanged (Red 0.61 / 0.63, Blue 0.55 / 0.58 deg). New unit cases: confirmation timing, blended
convergence with bounded steps (< 6 cm, < 0.7 deg per loop) to the true pose; single-tag, inconsistent and
agreeing-frame-interrupted evidence never re-localize; driver status text.
Limits: a systematic camera/map error seen consistently on 2+ tags would also be trusted; the 10-frame / 0.5-s /
2-tag gates are starting values, and real Limelight noise on the physical robot is unmeasured.

## Bench bang-bang + feedforward fusion flywheel controller (2026-11-11)

Change: in **Test: Intake + Index + Shooter** only, the GP2 B alternative to hub PIDF is now a fusion controller
instead of plain bang-bang. Speed error above 80 motor ticks/s (about 171 flywheel RPM): full power when slow,
zero power (FLOAT coast) when fast. Within 80 ticks/s: `(kS + kV * target + kP * error) * 12 / batteryVolts`.
The command never opposes the spin direction: the first version used full reverse power when fast, per the source
description, and the team chose coasting the same day.
The law is `ShooterVelocityTuning.fusionPower`; `TurretSubsystem.ShooterController.BANG_BANG_FUSION` replaces
`BANG_BANG` and is selected by no other OpMode, so both competition TeleOps, autonomous and the other tests keep
hub PIDF unchanged. Gains are live in Dashboard (`FUSION_KS`, `FUSION_KV`, `FUSION_KP`); initial kS 0.05 and kP 0.001
are estimates and kV is solved from the 5400 RPM at 14 V observation. Readiness tolerance (+/-100 RPM), feed timing,
arming and stop behavior are unchanged. A zero target still coasts in FLOAT, and FLOAT behavior is retained.

Simulation decision: bench-only controller, so ran the Mechanisms class (all four actual-bench scenarios, including
the new `benchFusionControllerSpinsUpHoldsAndFeedsThroughActualOpMode`) rather than Competition; the PIDF path the
competition wrappers use is not modified. The new scenario drives the real OpMode through the GP2 B switch (no spin
from switching alone), full-power spin-up, feedforward hold and a staggered feed. Result: 12 Mechanisms scenarios
pass. Trace: three coast loops alternating with full power on spin-up overshoot, zero negative-power loops, then
a steady 53% command holding 2836 RPM for a 2800 RPM target, 62 stage-2 feed ticks. Focused unit tests:
`ShooterVelocityTuningTest` (7) and `ShooterLiveTuningTest` (4) pass, replacing the two plain bang-bang cases with
fusion threshold, direction, overspeed-coasts, voltage-compensation, invalid-input and gain-bound cases; the
scenario and the fake-flywheel test both assert the command is never negative. `assembleDebug` passes. The full unit suite,
lint and the Competition/Sensors/Drivetrain simulations were not run for this bench-only change.
Limits: the flywheel fixture has synthetic response (0 to 2800 RPM in about 80 ms), no friction, ball load, current
limit or voltage sag, so overshoot, coast-down time after lowering a target, recovery time and all three gains
require tuning on the robot. No assertion, tolerance or calibration flag was relaxed.

## Full-codebase audit: corner rehome settle, slide fault text, INIT-phase sim coverage (2026-11-11)

Change: a review of every subsystem, both competition TeleOps, autonomous and the test OpModes found three issues.
(1) GP2 D-pad Left wrote the corner pose to the Pinpoint immediately, even during the INIT IMU recalibration, while
`AutoPoseHandoff` and `PedroDrive.setFieldPose` both require a READY device. `PinpointOdometry` now tracks how long it
has reported READY since the last recalibration request (`READY_SETTLE_SECONDS` = 0.5 s, `isSettled`), and
`AutoPoseHandoff.PINPOINT_SETTLE_SECONDS` reads that constant. New `CornerRehomeRequest` (both competition wrappers):
an INIT press during calibration shows WAITING and applies once settled; START discards a waiting press; after START
an unsettled press is REFUSED and never queued, because the robot may be moving. A settled press behaves as before.
(2) The slide FAULT text said "press again to retry", but the button toggles, so a fault while raising lowers on the
next press; telemetry now names the direction of the next press. Slide control logic is unchanged.
(3) Test harness: SimLoop runs the INIT loop in phase `INIT_LOOP` (`INIT` is one constructor tick). The INIT presses
in `bothAlliancesFieldRehomeInInitAndActiveUsesPressEdgesWithoutCamera` and
`bothAllianceCornerRehomesFallBackToCornerWhenRaisedTagsAreBelowTheImage` were gated on `INIT && t >= 150/145`, so they
never fired and their "in INIT" variants only pressed at ACTIVE tick 0. The two "slide unpowered during INIT" checks
inspected only that single tick. All four now use `INIT_LOOP`; no assertion or tolerance was changed.

Simulation decision: shared TeleOp localization in both wrappers, so ran `verify-teleop.ps1 -Sim -Scenario
Competition,Sensors,Mechanisms`. New scenario `bothAlliancesCornerRehomeDuringInitCalibrationWaitsForSettledPinpoint`
presses D-pad Left on INIT_LOOP ticks 0-2 (before the first Pinpoint read) in the real Red/Blue TeleOps. Asserts the
Pinpoint pose stays at the origin through tick 30, WAITING at tick 5, the corner pose and heading applied between ticks
30 and 60 and held to tick 99 (applied at tick 37 = 0.25 s fake calibration + 0.5 s settle, both alliances), APPLIED in
INIT, LOCALIZED and no corner-rehome line in ACTIVE. The corrected scenarios now exercise real INIT presses and pass.
Baseline (code inspection; the pre-change scenario was not run): previously the press was applied in the same
loop, during CALIBRATING. The SimLoop fake firmware zeroes
the pose at the reset command and keeps a pose written during calibration, so the possible device-side loss itself is
not reproducible in simulation; this verifies ordering only. New unit cases: `CornerRehomeRequestTest` (4).
Result: 268 unit tests, 36 scenarios (Competition 15, Sensors 8, Mechanisms 12, runtime 1),
`assembleDebug` and `lintDebug` (0 errors, 14 existing warnings), all pass; corner applied at INIT_LOOP tick 37 for both alliances.
Limits: whether the physical Pinpoint discards a pose written during calibration is unmeasured; the 0.5-s settle is the
existing handoff starting value. A refused ACTIVE press must be repeated by the operator.
