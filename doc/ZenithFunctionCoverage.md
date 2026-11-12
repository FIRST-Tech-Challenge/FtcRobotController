# Zenith integration coverage and evidence

Implementation/audit dates: 2026-10-30–31. The earlier implementation baseline was 154 ordinary tests; the fresh pre-audit working tree passed 178. The final working tree passes 185 ordinary tests and 24 simulations. This ledger records software coverage independently from editor and robot evidence. See [authoring guide](ZenithAuthoringGuide.md) for usage and [the plan/official-source comparison](ZenithIntegrationAudit.md) for findings and configuration limits.

**UI evidence:** [core local authoring/save/reopen verified in v0.1.2](ZenithUiAcceptance.md). All 43 actions and 23 conditions were listed; representative typed arguments, geometry, marker/wait and native group/branch edits were exercised. Actual UI-created JSON was generated, compiled, packaged and tested through native factories. Every field of every action was not separately exercised; source rename/delete used file maintenance plus team deployment, not a claimed native UI button.

**Hardware evidence:** pending for every integration row. Existing configuration provenance is retained; no tuning/calibration flag was changed. No robot, camera, motion, firing, servo travel or deployed footprint was commissioned. Homing cannot distinguish a stationary stall from estimated home; hood/depollinator conditions are estimates.

## Operational coverage

All named factories live in `AutonomousCommands`; `AutonomousOutputCoordinator` resolves latched output modes. Their actual runtime registry/resources are checked against `autos/robot.json` by `AutonomousIntegrationTest`; team deployment checks source/profile/parameter agreement independently. F = FULL_ROBOT, P = POLLEN_TEST. P deliberately omits turret, Limelight and depollinator.

| Robot function and Zenith exposure | Implementation owner | Profile | Software evidence / limits |
| --- | --- | --- | --- |
| Forward/reverse/strafe/diagonal, lines, curves, constant/linear/tangent/reversed/face-point/piecewise heading and short in-place hold turns | Native AutoBuilder/Ivy, PedroDrive/Foresight, adapters | F/P | CompetitionAutonomousTest native geometry/mirror/markers tests; PedroAdapterTest real numerical hold/path-speed checks. No physical trajectory verification. |
| Bounded `driveHold`, `driveStop` | PedroDrive + command factories | F/P | Real follower hold/stop output regression, including following update remaining stopped. |
| Native start pose and waypoints; `setFieldPose` known-reference reseed | PedroDrive/PinpointOdometry + ShotCoordinator invalidation | F/P | READY-deferred start; local pollen seed remains non-field-referenced; reseed mirrors once, invalidates readiness and waits for another bulk sample. Unit tests count exactly one Pinpoint read and no condition/follower re-read. |
| Intake forward/off/reverse/variable/timed: `intakeOn`, `intakeOff`, `intakeReverse`, `intakeSet`, `intakeRun` | IntakeSubsystem + output coordinator | F/P | Full-forward block/reverse allowance and stale-cleanup/replacement tests. |
| Stage-1 loading pause/stop/full collection: `loadOn`, `loadOff`, `transferStop`, `loadUntilFull` | TransferController/Subsystem | F/P | 0.2-second confirmation/0.5-second run-on boundaries and LOW recovery tests; cleanup faults attempt remaining outputs. Cached confirmation occurs before command evaluation. |
| Readiness-gated two-stage shooting: `shoot` | ShotCoordinator/FiringCoordinator | F | Existing readiness/recovery policy retained, no added motion-speed gate or tag prerequisite. Existing shot/controller tests plus ownership regressions; current unmeasured calibration still inhibits feed. |
| Explicit override `feedManual`; `feedStop`, `feedInhibit`, `feedResume`; bounded `gateOpen`, `gateClose` | Shared FiringCoordinator + GateSubsystem + transfer | F/P | Gate delay, both stages, inhibit dominance, resume without feed, cancellation and full retention tested. Override is explicit and never a readiness fallback. |
| Depollinator `depollinatorDeploy`, `depollinatorHome` | Existing DepollinatorSubsystem | F | Original angles/direction retained; commanded-posture and one-second unmeasured settle estimate. Stop retains posture. Expanded geometry remains unmeasured and blocks clearance assurance. |
| `turretHome`, `turretSetYaw`, `turretAdjustYaw`, `turretHold`, `waitTurretReady` | Existing TurretHomingController/TurretControl + factories | F | Estimated home, timeout, bounds/signs and one-time adjustment tests; valid home survives normal stop; unfinished home invalidated. No absolute home sensor/stall proof. |
| `hoodSet`, `hoodAdjust`, `hoodHome`, `waitHoodReady` | Existing TurretControl/Subsystem | F | Hood bounds and estimated tracking preserved. Manual hood survives later automatic updates. |
| `flywheelSetRpm`, `flywheelSpinUp`, `flywheelStop` | Existing flywheel controller/Subsystem + output modes | F | Independent fake motor dynamics for wait; natural success retains RPM; timeout/cancel stops; yaw/hood may continue while RPM is off. Existing conversion/cap/PIDF tests retained. |
| Alliance/hive `selectTarget`, odometry `aim`, `aimOff`, optional automatic spin | ShotCoordinator/TeleOpFieldReference + coordinator | F | FIELD compatibility default; AUTHORED side swaps exactly once under point symmetry; manual channel priority and invalidation tested. Autonomous never applies TeleOp camera pose correction. |
| `prepareShotPreset`, `shootPreset` | AutonomousShotPresets + shared ShotCoordinator/FiringCoordinator | F | Unknown names fail INIT; only diagnostic-example is present and explicitly UNMEASURED. Never feeds automatically; verified competition preset measurements are pending. |
| C270 collection `collectPollen` | PollenNavigator/PathFollower/Vision + Pedro | F/P | Canonical test source parses through native builder; existing visibility/loss/distinct-frame/blind-travel tests retained. Duration/trajectory unknown; camera calibration eligibility retained. Camera runtime itself needs Android/hardware testing. |
| `pollenVisionEnable`, `pollenVisionDisable` | INIT VisionPortal processor control + immutable loop snapshot | F/P | Camera construction remains INIT-only; missing camera rejects factories at INIT; enable discards pre-enable frames, disable cancels dependent collection/resources. Actual portal behavior not exercised on a camera. |
| `limelightSetPipeline` and cached observations | Existing TurretSubsystem | F | Only documented/configured pipeline 0 accepted; clears cached results/history. Missing Limelight rejects factory. Pipeline hardware transition unverified. |
| `stopAll`, terminal STOP, completion/deadline/pose fault | BioBuzzRobot/BioBuzzAutoFromFile + RobotShutdown | F/P | Normal stop retains home/posture and permits restart; cleanup attempts all outputs and preserves original exception; existing localization freshness guard and 29-second deadline retained. Exhaustive real-robot fault/STOP commissioning pending. |

Audit regressions additionally verify that reseeding conflicts with parallel feeding, native condition-wait SUCCESS/TIMEOUT survives following instant actions, terminal deadline outcomes survive interrupted cleanup, and twelve planner drive values agree with Java. Limelight pipeline changes now declare gate/transfer ownership and stop feeding. Group timeout fields ignored upstream are rejected by the team validator.

## Read-only and maintenance coverage

Conditions are cached and side-effect free: queue raw/confirmed status, loading/feeding, gate delay/inhibit, localization/drive state, turret home/target, estimated hood, flywheel speed, aim/shot readiness, pollen freshness/camera, Limelight freshness/selected tag, depollinator commanded/estimated posture, command failure and five-second remaining-time threshold. All 23 names have metadata help. Active telemetry includes routine/step/profile, outcome/reason/time, requested modes, target units, sample age/reference, calibration/readiness reasons and feed attempts.

| Existing function | Intended location / owner | Evidence boundary |
| --- | --- | --- |
| Robot-centric manual power and individual wheels | DrivetrainTestOpMode and PedroDriveTestOpMode | Maintenance retained; not an ordinary path action. |
| Local zero, IMU stationary calibration, encoder/pod directions/offsets, motor specs | INIT plus PinpointTestOpMode/drive configuration | No arbitrary mid-path IMU or pod reconfiguration; motor types stay cloned. |
| Encoder zero at current turret angle, bounded correction enable, raw paired yaw/hood, home-all helper | TurretTestOpMode/diagnostic configuration | Existing APIs and invalidation policy retained. `setAllServosToZeroPosition` means bounded home, not PWM zero. |
| Disconnected-servo programming and arming | ServoZeroCalibrationOpMode | Isolated maintenance only. |
| Flywheel raw/reverse power, controller selection, PIDF/bang-bang-fusion gain tuning/reset | Existing Turret/shooter diagnostics | No routine can silently rewrite tuning/controller selection. FLOAT behavior retained. |
| Camera mount/intrinsics/HSV/pickup, shot profile/release timing | Pollen camera/ShotCalibration OpModes and configuration | Existing measured flags authoritative; no invented measurements. |
| Goal geometry/tag/state/pose/velocity/current and encoder diagnostic getters | Existing calibration/test telemetry; relevant cached runtime conditions | Read-only information, not one command per getter. |
| Limelight field relocalization | Existing TeleOp policy/diagnostics | Future autonomous option; not enabled by this integration. |
| Hardware lookup, camera startup/close, bulk cache boundaries | BioBuzzRobot INIT/lifecycle, RobotLoop and subsystems | No command constructs hardware during execution. One bulk Pinpoint read/follower update/telemetry flush per active loop. |

## Verification record

The automated evidence consists of ordinary JUnit controller/runtime/factory tests, Node deployment fixture tests and the packaged APK asset check. Fake mechanism feedback evolves independently for flywheel wait assertions. Native route fixtures test parsing/scheduling/transforms rather than chassis physics. Android unit-test defaults do not verify a camera, servo position or real dynamics. Existing selected TeleOp simulation remains a separate check, not a full-autonomous simulation.

The final command results and counts are recorded in [ZenithVerification.md](ZenithVerification.md). Core UI round-trip evidence is retained; additional UI checks and all Phase 8 commissioning remain explicit. Native scanner helper-discovery advisories and the cleared-test-field season notice are retained and explained in the authoring guide. Native path timeouts do not set the team's named-action outcome, parallel outcomes are not aggregated, and the planner's physical geometry checks have the limits recorded in the audit.
