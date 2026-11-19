# SimLoop implementation plan for competition TeleOp

Created October 27, 2026. Updated October 31, 2026 after the implementation audit. The dependency and
lifecycle foundation, installed-mechanism fixtures, actual red/blue scenarios and human keyboard
record/replay are implemented. Calibration, physical sign-off and the camera target-normal convention
remain explicit boundaries. See [audit](SimLoopIntegrationAudit.md), [results and limits](TeleOpVerificationMatrix.md)
and [commands](../simloop/README.md). The phases below retain the original acceptance goals; this status
table distinguishes implemented coverage from pending work.

| Phase | Status after audit |
| --- | --- |
| 0 Baseline | Existing named tests retained; current build/test/lint evidence recorded in audit; concurrent autonomous work preserved |
| 1 Harness | Pinned alpha.2, separate serial test/review tasks, real JSON/Vintage, simulated team/SolversLib/Pedro clocks |
| 2 Robot model | All competition hardware represented with explicit synthetic dynamics; independent turret encoder, Pinpoint and rotating camera; physical calibration pending |
| 3 Isolated mechanisms | Behavioral drive/gate/intake/transfer/depollinator/turret tests; bench and remaining diagnostic lifecycle smoke; not exhaustive fault coverage |
| 4 Competition | Actual red/blue manual/queue/inhibit/stop/endurance; conditional normalized-camera acquisition; separate inherited synthetic calibrated-shot fixture |
| 5 Measurements | No new physical measurements, shot rows, release timing or calibration approval invented |
| 6 Optimization | Fixed-step sequencing verified; Control Hub timing distributions and loaded behavior still need physical measurement |
| 7 Routine use | Optional local selectors, inventory/manifest checks, AGENTS workflow, keyboard review with environmental faults and exact replay; CI deferred |

## Outcome and scope

Make changes to robot functions verifiable on a laptop before robot testing and before promotion into `RedMecanumTeleOp` and `BlueMecanumTeleOp`. Exercise actual test OpModes and the actual competition OpModes, as well as their existing controller tests. Keep hardware ownership in plain subsystem classes and competition behavior in the shared `MecanumTeleOp`; the alliance wrappers remain small.

Autonomous authoring, Zenith, Pedro Pathing, autonomous deployment, and changes to autonomous behavior are outside this implementation. Existing autonomous regression tests still run when shared code changes. Webcam/pollen functionality is deferred because the competition TeleOps do not use it; a later change there must explicitly declare the unsupported simulation boundary.

This plan creates a verification workflow, not permission to invent new controls, shot measurements, jam policies, mechanical limits, or calibration sign-offs.

## Confirmed team decisions

The team clarified the following after reviewing the initial plan:

| Decision | Agreed approach |
| --- | --- |
| Required or optional simulation | Optional, targeted to major features added or changed. Minor changes do not require SimLoop. |
| Local versus CI | Local checks first. GitHub Actions is deferred. |
| Current failures and priorities | Most features already work and are implemented on the robot. Focus on tuning and fixing inconsistencies; preserve working behavior and existing physical-test evidence. |

### When to use SimLoop

Consider simulation for major changes such as a new mechanism, a substantial control/state-machine rewrite, changes to interactions among intake/gate/transfer/shooter, or a new localization/aiming approach. It is an optional tool, not a mandatory promotion gate even for those changes. When selected, choose scenarios that exercise the affected behavior and both alliances where relevant.

Minor changes such as telemetry wording, a small control adjustment, or routine tuning within an existing algorithm do not require simulation. Use focused existing tests and appropriate robot measurements instead. Judge scope by behavioral impact rather than line count: one altered priority condition can change several mechanisms. Do not ask for permission merely to classify a change, and do not require a full simulation suite for every small fix.

The phases below describe how to build useful coverage progressively. Their acceptance criteria validate the simulation infrastructure itself; they do not require every already-working feature to be reimplemented or requalified before normal development can continue.

## Repository baseline

- SDK dependencies are FTC 12.0.0. Team code targets Java 8, using Gradle 9.1.0 and Android Gradle Plugin 8.13.2. Confirm dependency resolution against this combination during implementation.
- Existing tests use JUnit 4. They cover controllers, geometry, firing, calibration, and autonomous logic, but do not provide a complete simulated SDK hardware/lifecycle test of the two competition TeleOps.
- `HardwareReviewImplementationStatus.md` records operator completion of intake, transfer, turret rotation, hood/shooting, and gate mechanism tests. Preserve that sign-off; it is not numerical calibration or proof that later changes are validated.
- `ShootingConfiguration.PROFILE` holds the team-measured 12-60 in table (2026-11-15); release timing no longer gates firing (the `RELEASE_TIMING_MEASURED` flag was removed 2026-11-15), so production automatic feeding uses the current timing values.
- At planning time, local modifications exist in `MecanumTeleOp`, `PoseHistory`, `ShotCoordinator`, and `TeleOpFieldReference`, plus a new `TeleOpLocalizationTest`. Do not overwrite or revert these changes.
- Those local changes introduce tag-based field acquisition from an arbitrary starting pose. Older `CompetitionTeleOp.md` text assumes a fixed blue-corner start. Reconcile documentation and intended behavior before freezing the regression baseline. Never restore the older assumption merely to make a test pass.
- No `.github/workflows` directory exists at planning time.

## Phase 0 — establish behavior and evidence

1. Re-read the working tree and applicable instructions; record the revision and local diff used for the baseline.
2. Run the existing unit suite, debug build, and lint. Record discovered test names/counts, failures, skips, and existing warnings. Historical reports are not a fresh baseline.
3. Create `doc/TeleOpVerificationMatrix.md`: function, production owner, test OpMode, desktop coverage, hardware evidence/date, unresolved measurements, and competition integration status.
4. Record deliberate differences between diagnostics and competition. For example, the intake/shooter bench test has different starting RPM, adjustment steps, full-queue intake policy, and readiness checks. Sharing a subsystem does not mean copying diagnostic controls into competition.
5. Maintain a short inconsistency/tuning backlog: observed symptom, reproduction inputs, expected behavior, affected mechanism, available measurements, and the smallest useful check. Prioritize real reported issues over blanket coverage expansion. Unspecified failures are not presumed defects.

Acceptance: known baseline results and an agreed behavior matrix, with incomplete and physically unverified items labeled separately.

## Phase 1 — install a reproducible test-only foundation

1. Pin SimLoop `v0.2.0-alpha.2` and inspect the starter from that exact tag. Resolve its published artifact before committing to the integration; do not silently substitute `main` if publishing or compatibility fails.
2. Keep SimLoop, its logging dependencies, and desktop support out of the production APK. Prefer a repository-owned `simloop/simloop.gradle` containing the necessary test configuration; keep upstream licensing/attribution for copied material. Example robot sources are not needed in the final competition APK.
3. Preserve JUnit 4 discovery when enabling JUnit Platform: include Vintage, Jupiter for new tests, and the explicit Platform launcher required by Gradle 9. Verify that all baseline tests still execute.
4. Supply the real JVM `org.json` dependency for Limelight parsing and the documented Android stub configuration. Audit tests affected by `returnDefaultValues`; a returned Android default is not evidence of working hardware behavior.
5. Run OpMode simulations serially within a JVM. Use unique per-test log directories and prevent debug/release variants or separate workers from writing the same files.
6. Add a very small deterministic smoke test of an actual existing test OpMode. It must run INIT, START, ACTIVE, and STOP and assert an observable output change.
7. Keep simulation opt-in: configure a separate debug-JVM simulation task (proposed name `:TeamCode:simTeleOpTest`) and exclude simulation classes from ordinary unit-test execution. Reuse the debug test classpath and JUnit Platform without automatically attaching simulation to ordinary build/check tasks. Verify that exclusions remove only designated simulation classes, not existing controller tests.

Acceptance: dependencies resolve, smoke test passes twice, baseline JUnit 4 tests remain discovered, and debug packaging succeeds without simulation libraries in production runtime dependencies.

## Phase 2 — build the robot harness and close compatibility gaps

Proposed test sources under `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/`:

- `BioBuzzSimHardware`: named devices and mechanism-specific hardware-map builders.
- `BioBuzzSimModel`: fixed-step drivetrain, turret/encoder, hood, and flywheel behavior.
- `TeleOpScenario`: reusable lifecycle and gamepad scripts, sensor events, and stop injection.
- `TeleOpTrace`: sampled outputs, model truth, state transitions, and failure diagnostics.

Use these as a small shared test harness, not a second implementation of production controllers.

### Concrete compatibility work

| Area | Implementation requirement |
| --- | --- |
| Gate servo | `GateSubsystem` currently requires concrete `ServoImplEx`; SimLoop's `FakeServo` is `Servo` plus `PwmControl`. Retrieve through the SDK interfaces, require PWM capability with a clear initialization error, and preserve range, direction, endpoints, and timer behavior. |
| Turret and depollinator PWM | Replace `instanceof ServoImplEx` checks with capability-based `PwmControl` checks where appropriate so the fake executes the same PWM setup as real hardware. Verify pulse ranges and physical direction, not just logical positions. |
| Hardware names | Register actual configured names. Test permitted aliases separately, missing required devices, and missing optional Limelight/turret behavior. Standalone fixtures include only their intended devices; unrelated mechanism lookups must fail. |
| Turret feedback | Drive the external encoder from simulated turret motion, including gearing and sign. Do not set encoder feedback equal to the requested target immediately. Model coupled yaw servos as one mechanism. |
| Flywheel | Model velocity response, output-shaft conversion, load/recovery, and voltage changes. Keep production configuration distinct from measured physical model parameters. |
| Pinpoint | Use the SDK-driver fake and explicitly configure physical pod offsets/directions/resolution. Feed chassis motion into it once per tick; distinguish raw local odometry from the acquired field transform. |
| Limelight | Use a test field/tag layout and rotating camera mount consistent with the documented robot geometry. Inject delayed, missing, stale, conflicting, and invalid observations. Independently check coordinate conversion with known poses. |
| Time | Use `OpModeRunner.forTeamClass` for the actual OpMode, so team `System.nanoTime`, `System.currentTimeMillis`, and `ElapsedTime` use simulation time. Respect class-loader boundaries; exchange SDK types or primitive trace data instead of mixing team objects loaded by different loaders. |
| Lynx hubs | SimLoop has no Lynx-module emulation. Verify that an empty hub inventory works for laptop tests. Inspect cache ownership separately and measure real hub behavior on the robot; do not label it simulation-covered. |
| Raw servo programmer | Inspect fake controller metadata support before exercising this OpMode's inventory telemetry. If needed, add a narrow test-side controller adapter. Preserve its deliberately disconnected-servo workflow. |

Use 20-ms ticks initially; add selected 10-ms runs and timing-boundary scenarios after the baseline works. Supply measured model constants where available. Mark estimates with source and uncertainty. Never use guessed model values to overwrite robot calibration.

Acceptance: required devices initialize, PWM configuration is exercised, feedback evolves independently of commands, clocks agree, and repeat runs produce equivalent decoded traces.

## Phase 3 — verify standalone functions

Implement rows as major changes or reported inconsistencies make them useful. For each chosen row, include behavior assertions, lifecycle checks, STOP during activity, and hardware-isolation checks. Use existing pure-Java tests for exact mathematical/timer boundaries; use SimLoop to verify real hardware calls and OpMode coordination.

| Function / existing test OpMode | Scenarios when coverage is implemented |
| --- | --- |
| Drivetrain / `DrivetrainTestOpMode` | Forward Y sign, reverse, strafe, clockwise rotation, diagonal normalization, wheel selection, neutral, and STOP. Verify commanded motor signs and resulting model motion independently. |
| Odometry / `PinpointTestOpMode` | INIT reset/calibration sequence, readiness, configured pod signs, translation/rotation units, reset edge behavior, reference generation, and invalid/stale sample handling. |
| Intake / `IntakeTestOpMode` | Forward/reverse/neutral, conflicting buttons, direction, and shutdown. Competition automatic-full policy is checked separately. |
| Gate / `GateTestOpMode` | Closed/open physical PWM, trigger threshold, repeated open commands do not restart the delay, close/reopen does, and STOP closes as intended. |
| Transfer / `TransferTestOpMode` | Stage 1 loading with stage 2 off, sensor chatter, continuous 2-s full confirmation/latching, timed gate-open feed, release, inhibit priority, and recovery. |
| Depollinator / `DepollinatorTestOpMode` | Home/deploy endpoints, rising edges, both-button precedence, holding through START, and no unintended stop-time sweep. Confirm positional-servo STOP policy rather than inventing a return move. |
| Turret / `TurretTestOpMode` | Homing, early START, retry, relative zero, encoder sign, paired servo agreement, travel limits/excluded sector, hood bounds, flywheel control, and motor shutdown. |
| Camera / `LimelightTagTestOpMode`, `GoalStateTestOpMode` | ID filtering, parsing, capture age, duplicates, camera absence, geometrically valid/invalid observations, and diagnostic state transitions. Keep diagnostic goal-state gates separate from competition policy. |
| Calibration / `ShotCalibrationOpMode` | Controls, X-held feed (gate open + transfers until X release), timing, displayed averages, and session reset. Preserve current no-file-save behavior. Synthetic results do not create measured shot rows. |
| Raw calibration / `ServoZeroCalibrationOpMode` | No command during INIT, no unarmed command, selection changes disarm, only the selected servo changes, and no forced center move at STOP. |

Acceptance: every covered mechanism has passing actual-OpMode tests and an explicit remaining hardware-validation list. A useful test must fail when its expected behavior is deliberately broken; confirm this on selected critical assertions during harness development.

## Phase 4 — verify combined mechanisms and aim

- Exercise `IntakeShooterTestOpMode` using the same hardware model; verify its intentional bench policy.
- Add competition-policy scenarios for the shared mechanisms: empty queue -> loading -> full hold -> gate delay -> feeding -> recovery -> loading.
- Assert no stage-2 feed before the gate delay; assert feed-inhibit wins over all simultaneous feed requests. Test both gamepads, conflicting intake bumpers, held buttons, rapid mode changes, and STOP in every feeding phase.
- Test `AutoAimTestOpMode` as a diagnostic, then independently test competition aim/localization behavior. Passing its diagnostic readiness policy alone does not validate competition auto-aim.
- Preserve production uncalibrated behavior. Test calibrated success paths only through an explicit injectable configuration seam or test-only fixture that cannot ship as competition defaults. Do not turn production measurement flags on to get a green test.
- Test manual feeding while automatic readiness is unavailable, preserving the current deliberate override and inhibit hierarchy.

Acceptance: shared hardware sequencing works through real adapters, and bench/competition differences have explicit assertions.

## Phase 5 — run both real competition TeleOps

Run the same scenario suite against `RedMecanumTeleOp` and `BlueMecanumTeleOp`, with alliance-specific expected targets. Do not validate only a test-only substitute OpMode.

Scenarios for the full competition suite when implemented:

1. INIT, early START during homing, STOP before START, normal STOP, and failures after partial initialization. Every initialized motor is stopped; gate and positional-servo policies are checked explicitly. Cleanup must attempt other devices even when one stop action fails.
2. Driving while homing, adjusting hood/RPM, collecting, firing, and rehoming. Assert drivetrain output updates each simulation tick in supported active states.
3. Manual turret speed and measured-angle hold on release; auto-aim release restores manual control/RPM as specified; limits remain enforced.
4. Unknown starting pose: manual operation remains available, field-dependent automatic behavior stays unavailable until accepted observations establish a reference.
5. Acquisition from valid mapped tags of either alliance, while shooting still targets the selected alliance. Check audience/far selection, Y=0 tie behavior, crossing halves, capture latency, turret movement, and heading wrap.
6. Camera loss after acquisition preserves the approved no-drift policy; invalid/stale odometry still blocks automatic behavior. Resetting odometry invalidates field acquisition. Do not reintroduce camera-loss or motion-based firing gates.
7. Full-queue policy, manual feed, auto-feed when genuinely configured, inhibit, fallback without turret, and telemetry reasons when calibration or hardware is unavailable.
8. A scripted 180-second endurance regression with mechanism transitions and camera dropout. This is a stress-test duration, not a change to competition timing.

Compare red/blue mechanism traces under equivalent inputs. Only documented alliance/target/localization effects may differ. Repeat selected scenarios and compare decoded robot data, excluding known logger internals, rather than raw RLOG bytes.

Acceptance: both actual competition OpModes pass; trace milestones show that requested functions executed. No test may pass solely because initialization fell back, the OpMode exited early, or an automatic function stayed inhibited throughout a supposed success case.

## Phase 6 — optimize with evidence

This is the main robot-development priority: fix reproducible inconsistencies and tune the working mechanisms. Preserve baseline scenarios and compare one meaningful change at a time. Routine parameter adjustments should use focused tests and measured robot results; simulation remains optional for major behavioral changes and must not delay small fixes while the full model is unfinished.

| Objective | Desktop evidence | Robot evidence needed |
| --- | --- | --- |
| Responsive driving | New drive commands appear by the next simulated control tick during mechanism actions. | Actual input response and loop distribution under simultaneous load. |
| Turret control | Bounds, release behavior, correction settling, and modeled error traces. | Loaded speed, overshoot, vibration, stopping distance, and encoder accuracy. |
| Feeding reliability | Correct gate/transfer ordering and bounded readiness/recovery behavior. | Loaded gate clearance, release timing, jams, and flywheel recovery. |
| Accurate aiming | Independent geometry cases, target/alliance selection, calibration-boundary behavior. | Real camera transform/timing and measured distance/RPM/hood shot table. |
| Efficient loop | Call counts, one odometry update, one telemetry flush per owning loop, no new blocking paths. | Control Hub period p50/p95/p99/max, aiming for 50–100+ Hz under representative load. |

Fixed simulated time cannot benchmark real Java work time, I2C/USB latency, Lynx cache performance, battery sag, or thermal behavior. Instrument and measure these on the robot. Optimize telemetry rates, repeated reads/writes, or allocations only when measurement supports the change; preserve required device refresh behavior.

## Phase 7 — make verification routine for humans and agents

Add a small `tools/verify-teleop.ps1` entry point after the harness is usable. Default to ordinary local tests/build checks; provide an explicit `-Sim` option for selected scenarios and an explicit full-simulation option. Do not run simulation automatically for minor edits. Propagate nonzero exit codes and verify that expected tests actually ran. Ordinary checks retain this command, with simulation classes excluded once the separate task is implemented:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:lintDebug
```

The local verification script, automated simulation task and separate human review task now exist. Opt-in command:

```powershell
.\gradlew.bat :TeamCode:simTeleOpTest
```

Store generated logs/reports under build directories; keep fixtures and human-readable expectations in source control. Retain traces on failure with scenario, tick, input, expected/actual output, seed, model parameters, and reason. Report unsupported or unmeasured checks explicitly, never as PASS.

After coverage exists, add a concise policy to the repository's existing `AGENTS.md` and link this plan and the verification matrix. Preserve its FTC architecture rules; do not replace it with upstream agent instructions. Proposed policy:

> SimLoop is optional and intended for major feature additions or changes. Minor edits and routine tuning do not require it; use proportionate existing tests and physical measurements. For a major change, consider an observable scenario against the affected real OpModes and both competition TeleOps when shared behavior is involved. If simulation is used, identify the baseline, reproduce the issue where applicable, assert the intended behavior, and report results and model limits. Never relax assertions, substitute perfect feedback, or change calibration flags to hide a failure. Do not claim unsupported or unrun behavior was verified. Preserve existing build requirements and working robot behavior.

CI is deferred. A future separately requested CI integration can reuse the local entry point, JDK/Android SDK setup, and pinned dependencies; it should preserve the team's opt-in simulation policy rather than making every pull request run the full suite.

## Completion and feature promotion

Track each changed function through: expected behavior/reproduction -> proportionate desktop checks -> optional simulation for a major change -> physical validation where needed -> competition ready. When simulation is chosen, exercise the affected actual test OpMode and integrated red/blue behavior as appropriate. Record simulation as not run when omitted; omission alone does not block promotion. Keep software and physical readiness separate, with evidence for each.

For each change, the final report should state changed behavior, affected OpModes, test results, both-alliance coverage where relevant, remaining model limits, and the exact physical checks needed. Known successful unchanged mechanism checks do not have to be repeated. Any change to mechanics, wiring, direction, limits, timing, or tuning invalidates the corresponding prior evidence.

The October 28 milestone covered drivetrain/gate. The October 31 audit expands this to all installed
competition mechanisms and actual red/blue lifecycle/coordination, with human review. It does not claim
exhaustive coverage of every branch, raw servo controller metadata, webcam/USB capture, autonomous route
validation or physical model calibration. Consult the coverage inventory and audit before a new feature.

## References

- [SimLoop pinned starter configuration](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/simloop-starter/simloop.gradle)
- [Pinned FakeServo contract, including ServoImplEx incompatibility](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/SimLoop/src/main/java/org/horizon36596/simloop/fakehardware/FakeServo.java)
- [Pinned timer/class-loader workflow](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/SimLoop/docs/unchanged-team-code.md)
- [SimLoop validation status](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/SimLoop/docs/tested.md)
- Local baseline: `AGENTS.md`, `doc/CompetitionTeleOp.md`, `doc/TransferAndShooting.md`, `doc/HardwareReviewImplementationStatus.md`, current TeamCode sources and tests. Reconcile stale documentation with current team decisions during Phase 0.
