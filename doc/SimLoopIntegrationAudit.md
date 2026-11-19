# SimLoop integration audit

Audited October 30–31, 2026 against the implementation plan, the installed robot code, and official
SimLoop **v0.2.0-alpha.2** sources/docs. The initial implementation was **not complete**: it covered
actual drivetrain/gate OpModes only. This update provides a broader local verification and human-review
workflow. It does not claim exhaustive branch coverage, a calibrated robot twin, or physical acceptance.

Baseline revision for this audit: `d2c2f0d875c6139f9e777e3ff6891a17756c3692`, plus the existing dirty
working tree. Concurrent Zenith/autonomous changes were preserved. Reports describe the tested tree,
not a new commit or deployment. Original October 28 named unit-test baseline remains in the manifest.

## Changes and findings

| Finding | Resolution / evidence |
| --- | --- |
| First milestone lacked integrated robot fixtures | Named drive/intake/gate/transfer/depollinator/turret/flywheel/Pinpoint/Limelight fixture, shared intake/turret encoder wiring, independent finite-rate turret feedback |
| Turret/depollinator PWM setup required concrete ServoImplEx | Use SDK PwmControl interface, which works with the real controller and SimLoop; endpoint/PWM checks execute |
| Red/blue were never simulated | Run actual wrappers for drive/queue/feed/inhibit, early START, STOP, endurance and conditional camera localization |
| No calibrated automatic-success scenario | Test-only subclasses inherit the competition loop and inject artificial shot rows through a small coordinator factory; production profile/measurement flags unchanged |
| Camera/application timestamp epochs differed | Read SDK staleness plus capture/target latency, with epoch-equivalence/invalid-age tests; preserve freshness limits |
| Raw camera tag normal differs from robot convention | Resolved 2026-11-05 by a physical frame: robot Limelight matches raw SimLoop (+Z into print). Single Rx(pi) conversion in `LimelightObservation`; inverted-camera rejection regression retained |
| Pedro retained laptop time | Include com.pedropathing in the SimLoop class-loader instrumentation, along with team code and SolversLib |
| No human mechanism review | Separate keyboard-review task/script with both gamepads, queue signal, camera outage, Pinpoint fault and turret jam controls; live status and retained telemetry |
| No reproducible human bug report | Upstream gamepad track plus environment/setup/STOP sidecars; replay asserts every decoded output row against the original |
| Future additions could disappear from coverage records | AGENTS requires a major-change simulation decision; FullSim checks all annotated TeleOps against explicit coverage inventory; named-test manifest guards discovery |

Production changes are limited to SDK-interface compatibility, camera receipt-age calculation and an
explicit coordinator-construction seam. The testing model does not alter competition controls, shot
calibration flags, mechanical limits, motor directions or production target-normal interpretation.

## Comparison with the plan

Phases 0–1 (baseline/harness) and the local workflow in Phase 7 are implemented. Phases 2–4 now have
installed-hardware fixtures and actual mechanism/competition tests, with finer coverage in the
[matrix](TeleOpVerificationMatrix.md). Some diagnostics have lifecycle smoke coverage only. Synthetic
calibrated-shot success is clearly separated from actual production-default behavior.

Phases 5–6 still require physical measurements: shot table/release timing, loaded mechanism dynamics,
camera configuration/axes and Control Hub timing distributions. These cannot be completed honestly
without the robot and measurements. CI remains deferred by team decision. Optional simulation for
major changes remains optional; minor tuning is not forced through a full simulation suite.

## Official capabilities: used, optional, unsupported

| Upstream capability | Repository usage |
| --- | --- |
| SDK12 fake hardware / actual OpMode lifecycle | Used by actual test and competition OpModes; isolated and full fixtures |
| SimClockLoader / deterministic runs | Team, SolversLib, Pedro instrumented; serial runs and decoded replay comparison |
| FakeMotor, Servo/PwmControl, DigitalChannel, VoltageSensor | Used for installed devices; physical current/battery/servo dynamics not calibrated |
| FakeGoBildaPinpoint | Chassis coupling, physical pod mount, calibration and explicit faults |
| FakeLimelight3A / tilted tag layout / camera mount | Used geometrically with rotating turret, latency and raw-versus-normalized convention tests |
| KeyboardWindow, KeyboardGamepadSource, GamepadInputTrack, RealTimePacer | Used in separate local reviewer; keyboard only, not USB gamepad |
| RLOG / AdvantageScope | Not integrated: this adapter exports TSV and upstream gamepad tracks. Optional upstream feature, not represented as implemented |
| Parameter fitting, score/optimization loops, synthetic field/game pieces | Not integrated; no measured data or game-piece model supplied. They are not required to run actual TeleOp lifecycle tests |
| VisionPortal/webcam/native pipeline capture | Unsupported by this harness; Pollen vision controller tests do not prove USB/OpenCV behavior |
| Raw servo controller metadata | Upstream FakeServo.getController refuses it; raw disconnected-servo programmer explicitly classified unsupported |
| Lynx scheduling, Android services, physical I2C/USB, battery/thermal load | Not simulated; requires hardware validation |
| Autonomous authoring/deployment | Separate Zenith implementation/audit; this TeleOp workflow does not claim route or autonomous physical validation |

Using the library does not mean every optional upstream package must be added to the robot project.
Relevant installed hardware is represented, while unsupported and unmeasured behavior is visible rather
than replaced with optimistic fake feedback. [Model provenance](SimLoopRobotModel.md) lists every fixture
setting and the physical data needed to replace synthetic parameters.

## Verification record

- Full simulation suite: **24 Jupiter tests** with multiple scenario runs, including both competition
  alliances and 180 simulated seconds each. Original drivetrain/gate tests retained.
- Final `verify-teleop.ps1 -FullSim`: **185 ordinary tests and 24 simulations**, zero failures/errors/skips;
  all 17 TeleOp inventory entries classified; debug APK and lint passed. Ordinary tests/build/lint run
  separately from simulation, preserving opt-in behavior.
- Human keyboard window opened and inspected; live values rendered; window-close STOP completed.
- Headless review smoke and exact decoded replay passed. Actual keyboard-window session also replayed
  exactly, including early STOP. This is reviewer infrastructure validation, not a human sign-off on
  every mechanism or on the robot itself.
- Android debug runtime dependency inspection confirms SimLoop, its desktop artifact, PsiKit, JUnit
  and JVM-only JSON are absent from production runtime dependencies. Documentation link checks and
  `git diff --check` passed.
- Lint currently reports 14 warnings (12 upstream native-library 16KB-alignment and 2 unused-resource
  warnings); no lint errors. Java25/source8 and Gradle deprecation warnings remain.
- One intermediate full-check run failed a concurrently added autonomous test because its wrapper did
  not forward atSpeed to its recording fixture. The fixture was subsequently updated without relaxing
  the expected speed assertion; the final complete wrapper rerun passed.

Fresh evidence is in `build/simloop-verification-final.log`, `build/simloop-keyboard-replay.log`, JUnit
XML under `TeamCode/build/test-results/`, HTML under `TeamCode/build/reports/tests/`, and per-scenario
records under `TeamCode/build/sim/teleop/`. Generated artifacts stay out of source control. Preserve the
whole session directory plus revision/diff and reviewer observations when sharing a reproducible issue.

## Remaining robot evidence

Camera convention update (2026-11-05): a physical full-3D frame of raised Red far tags 30-33 showed raw
facing scores of -0.96 to -0.99, so the installed firmware reports +Z into the print like SimLoop.
Production converts once in `LimelightObservation`; the former normalizing adapter is now the
inverted-camera rejection fixture. Physical height/plane accuracy remains unvalidated.
Inverted-mount update (2026-11-13): the upside-down Limelight reports 3D poses in its inverted sensor frame;
`LimelightObservation` applies Rz(pi) about the optical axis once, and the SimLoop adapter reports the same
sensor frame. Tag heights now agree with a field tape measurement within 3-4 cm (see the verification matrix).

Keep existing hardware sign-offs. Obtain actual shot rows/release timing and model measurements where
needed. Do not claim that a green synthetic automatic-shot test authorizes production automatic feed.

## Official resources checked

- [Pinned SimLoop starter](https://github.com/Horizon-36596/simloop/tree/v0.2.0-alpha.2/simloop-starter)
- [Actual team code / class-loader clock boundaries](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/SimLoop/docs/unchanged-team-code.md)
- [Live keyboard and record/replay](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/SimLoop/docs/packages/live.md)
- [Fake hardware contracts](https://github.com/Horizon-36596/simloop/tree/v0.2.0-alpha.2/SimLoop/src/main/java/org/horizon36596/simloop/fakehardware)
- [Camera projection and printed-face -Z](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/SimLoop/src/main/java/org/horizon36596/simloop/vision/SimTagCamera.java)
- [Fixed SimLoop wall-clock epoch](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/SimLoop/src/main/java/org/horizon36596/simloop/simclock/SimClockHooks.java)
- [Upstream tested status](https://github.com/Horizon-36596/simloop/blob/v0.2.0-alpha.2/SimLoop/docs/tested.md)
- [Limelight legacy coordinate guide](https://github.com/LimelightVision/LimelightDocs/blob/master/docs/coordinate_systems_fiducials.rst)
- [AprilRobotics coordinate conventions](https://github.com/AprilRobotics/apriltag#coordinate-system)

The published documentation site was unavailable in some reads; pinned GitHub sources were used as the
primary version-specific reference. Do not substitute newer main-branch API assumptions for alpha.2.
