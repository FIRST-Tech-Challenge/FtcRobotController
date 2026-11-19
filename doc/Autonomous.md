# Autonomous: Pedro Pathing v3, Ivy, and Zenith

Every registered autonomous now extends Zenith's Ivy `AutoFromFile`. `BioBuzzRobot` connects the runtime to the existing hardware subsystems and `ShotCoordinator`. `WaypointFollower`, `CompetitionAutonomousController`, and `AutonomousStrategy` have been removed. The pollen navigator is an experiment in the **Test** group, separate from the real match routines.

Use the [Zenith authoring guide](ZenithAuthoringGuide.md) for the complete command catalog and supported deployment workflow. The [integration audit](ZenithIntegrationAudit.md) compares the code with the plan, official resources and current robot configuration. The [coverage ledger](ZenithFunctionCoverage.md) and [verification record](ZenithVerification.md) distinguish implemented software from pending editor and robot evidence. The original [implementation plan](ZenithAutonomousImplementationPlan.md) remains the acceptance specification.

## Installed versions

| Component | Version / repository |
| --- | --- |
| Pedro core | `3.0.0-20260828.185437-17`, Sonatype snapshots |
| Ivy Pedro adapter and core | `1.1.1`, Maven Central |
| Zenith robot runtime | `v0.1.2`, JitPack |
| Zenith CLI | `0.1.2`, npm, locked in package-lock.json |
| Pedro web tuner | `com.pedropathing:tuning:1.0.1` (Sinister/Sloth from repo.dairy.foundation), revhub excluded |

These are the versions documented and tested together by [Zenith](https://github.com/Horizon-36596/zenith/blob/v0.1.2/robot/README.md). This Pedro v3 API uses `Paths`, `Pose`, and `Follower.follow`; v1/v2 examples using `PathChain` do not apply. Keep the exact Pedro version in TeamCode/build.gradle and autos/robot.json identical. The npm override pins patched Zod 4.6.6 for the CLI; it does not enter the APK.

## Before robot motion

**Pedro tuning has not been measured on this robot.** `PedroConfiguration.TUNING_VERIFIED` is false. All Pedro/Zenith autonomous routines refuse to start until dynamics, braking and gains are verified (the [fallback autonomous](#fallback-autonomous-no-pedro) does not use Pedro). Since 2026-11-18 the Foresight dynamics, braking and gains in Java and robot.json are copied from the master chassis's tuner run (`pedro/Constants.java` on `master`) so both robots share one tuning; they are labeled `PORTED` and still need checking on V1's drivetrain. Pinpoint pod offsets and directions stay V1's own measurements. Camera and shooting calibration gates also remain active.

Use **Test: Pedro Drive**, which initializes only the drive subsystem (four motors and Pinpoint). Keep the chassis stationary through calibration. Sticks command at most 0.2 power, A requests 12 inches forward from the current pose, B requests a 90-degree CCW hold, and X cancels. Stick movement cancels a running test. A/B use Ivy commands, have a five-second timeout, and stop on completion. This diagnostic deliberately allows testing before TUNING_VERIFIED is set.

Verify forward, left and CCW motion and the matching odometry signs, then measure Pedro's dynamics, braking and gains with the web tuner below. Update `PedroConfiguration` and the matching kinematics/follower values in autos/robot.json together. Measure robot footprint and height too; the editor currently has explicit 18-inch example dimensions. Set TUNING_VERIFIED only after low-speed path, hold, stop, and turn checks pass.

### Pedro web tuner

V1 includes [Pedro's AutoTune procedures](https://pedropathing.com/docs/pathing/tuning), adapted from master's `pedro/` package: `pedro/Tuning.java` registers them and `pedro/procedures/` holds them. Connect to the robot's Wi-Fi, open **http://192.168.43.1:10158**, and run them in this order. Each step starts a temporary **Pedro Tuning** OpMode; Driver Station STOP ends it, stops the wheels and reports nothing for that step.

| Order | Procedure | What it needs | Where results go |
| --- | --- | --- | --- |
| 1 | Mecanum Tuner | Wheels off the floor; each motor spins at 0.5 power | `MecanumDrivetrain.LEFT/RIGHT_MOTOR_DIRECTION` (aborts if front and back disagree on a side) |
| 2 | Pinpoint Tuner | Push forward, push left, then spin 180 degrees CCW by hand | `PinpointOdometry` pod directions and offsets in mm; compare with the measured pod geometry before replacing it |
| 3 | Foresight Tuner | Lots of clear floor; drives at full power (default 48 in runs, 36 in braking runs) | `PedroConfiguration` constants, `create()` gains, and the `robot.json` values it prints |
| 4 | Tests | Hold, line, curve, odometry and driving tests of the installed follower | Check before setting TUNING_VERIFIED |

All procedures use V1's own hardware through `PedroTuningHardware`, not Pedro's revhub classes. The tuning library itself has no Pedro core dependency, so the strictly pinned core above is unchanged. What this means for V1:

- The Foresight tuner drives V1's open-loop `MecanumDrivetrain` at full power and reads V1's `PinpointOdometry` (measured pod offsets/directions) through the same adapters as `PedroDrive`. Pedro's model wants full-power velocities; `PedroDrive` applies the 0.35 wheel cap separately.
- V1 keeps BRAKE at zero power, while Pedro's stock mecanum floats while following. The tuner's coasting stop is therefore V1's braking stop, so natural deceleration is measured the way V1 autonomous actually slows.
- Tests run the installed `PedroConfiguration` follower at the 0.35 autonomous cap, matching match routines.
- The Foresight tuner prints Java for `PedroConfiguration` and a JSON fragment for robot.json. Its two-step translational gain uses the lower gain below 2.5 inches of error and the higher one above. Zenith's sim models one gain, so robot.json takes the lower one. `PedroAdapterTest` checks that Java and robot.json agree.

The pinned Pedro snapshot normalizes a zero displacement inside ForesightV3's hold calculation. `PedroForesight` guards only that numerical case using a 0.0001-inch offset, below Pedro's translation deadband, and delegates the control calculations to Pedro. Regression tests exercise exact-position holds and in-place turns.

## Author and deploy match routines

Open the repository's zenith.json in Zenith. It selects Ivy and `org.firstinspires.ftc.teamcode.BioBuzzRobot`. The project includes Zenith's BIOBUZZ field model, attributed below. Verify field geometry and robot dimensions against the actual setup before planning paths.

Draft match routes (`red-audience`, `red-far`, unmeasured, not deployed; blue runs them point-mirrored) and their plan are in [MatchAutonomousPlan.md](MatchAutonomousPlan.md). The legacy red/blue Route Required wrappers are disabled. New authored match files each generate their own full-robot entry; the integration example is clearly labeled and supplies no competition strategy.

1. Create routes in Zenith under autos/. The file's name must match its stem, such as `red-match`. Use explicit, measured start poses and waypoints in inches, field center origin, +x audience right, +y away from the audience, CCW radians.
2. Compose paths with the named commands below. Zenith's runtime preserves supported curves, heading modes, distance markers, sequences, parallel all/race/deadline groups, branches, waits, and current-pose starts. It validates names and file versions during INIT.
3. Install CLI dependencies once with `npm ci --ignore-scripts` (Node 22+). Run `npm run zenith:validate` and `npm run zenith:deploy -- --dry-run`, then `npm run zenith:deploy`. Deploy writes local APK assets and generated Ivy OpMode stubs; it does not install anything on the Control Hub.
4. Run `.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` and `.\scripts\check-zenith-apk.ps1`. Install the resulting APK through the team's normal robot workflow.

New routines get exactly one generated OpMode in the Generated group; authors need no Java edits. The generated robot uses the file's alliance. A hand-written wrapper may run a routine on the *other* alliance (Ivy mirrors it once), at most one per routine; team validation refuses same-alliance or second mirror wrappers. The blue DRAFT wrappers (`BlueFarAutonomous`, `BlueAudienceAutonomous`, `BlueAudienceCycleAutonomous`) do this. Team deployment retains pollen's sole hand-written Test entry. Use UI edit/save followed by `npm run zenith:deploy`, rather than treating the editor's native Deploy button as completing team deployment. `npm run zenith:check` rejects stale or duplicate outputs.

Each path's speed fraction is applied through `ForesightConfig.setPathSpeed`, in addition to the independent wheel-power cap. Estimates remain provisional until Java and robot.json contain measured, matching values. Put timeouts/end conditions on path and command steps. Parking and fallback decisions belong in the authored routine; the old controller's automatic park fallback was removed with it. The runtime does not provide live obstacle avoidance. A global 29-second deadline, localization loss, routine completion, Driver Station STOP, and exceptions stop outputs and close the gate. Explicitly canceling the Ivy routine ensures running commands receive their end hooks before scheduler reset. Camera shutdown follows motor shutdown.

## Fallback autonomous (no Pedro)

**Auto: Red Fallback (Shoot + Leave)** and **Auto: Blue Fallback (Shoot + Leave)** (Competition group) score with none of the gates above: no Pedro tuning, no Zenith route, no field pose and no shot calibration. Use them until a tuned match route exists. Each one preselects the same-alliance competition TeleOp.

Sequence (`FallbackAutoController`, hardware-free and unit-tested; `FallbackAutonomous` wires the hardware):

1. **Start delay**: 0–15 s, chosen in INIT with GP1 D-pad Up/Down. If the preload shot is on, the flywheel spins up during the delay.
2. **Preload shot** (optional; toggle it in INIT with GP1 X, and it defaults to off until `PRELOAD_SHOT_MEASURED`): the turret goes to a fixed heading, the hood and flywheel to fixed values, then the auto waits for READY. Feeding uses the same continuous gate + two-stage path as TeleOp's manual feed and is held for `FEED_SECONDS`. After `SPIN_UP_TIMEOUT_SECONDS` it feeds anyway if the turret is homed, and skips the shot if it is not. An unavailable turret only disables the shot.
3. **Leave**: drives to a point given relative to the start pose (forward/left inches) while holding the start heading, capped at 0.35 power. It is proportional on Pinpoint distance, with a 0.08 minimum power and 1 in / 3 deg tolerance.
4. **Stop** when done. It also stops on a 4 s leave timeout, at 29 s, or on a fault: a Pinpoint pose that stays stale or not READY for 0.25 s, or the robot getting 6 in *farther* from the target, which means a drive or odometry sign is wrong.

Values to measure at each alliance's real start spot (all are marked STARTING VALUES):

| Constant | Where | Default |
| --- | --- | --- |
| `PRELOAD_TURRET_HEADING_DEG` | `Red/BlueFallbackAutonomous` | 90 (turret home). Read it from the Driver Hub "Turret \| HOMED \| heading" line: front=0, right=90, back=180, left=270 |
| `LEAVE_FORWARD_INCHES`, `LEAVE_LEFT_INCHES` | `Red/BlueFallbackAutonomous` | 24 forward, 0 left |
| `PRELOAD_HOOD_DEG`, `PRELOAD_FLYWHEEL_RPM` | `FallbackAutonomous` | TeleOp's starting manual hood (home) and 3500 RPM |
| `PRELOAD_SHOT_MEASURED` | `FallbackAutonomous` | false (the shot starts OFF in INIT) |
| `FEED_SECONDS`, `SPIN_UP_TIMEOUT_SECONDS` | `FallbackAutoController` | 2.5 s, 3.0 s |

Practice checklist: test leave-only first on blocks or with a hand ready on STOP, and confirm it drives forward and stops. Then aim the turret by hand in TeleOp at the start spot to read the heading, and find a hood/RPM that scores. Enter those values and try the full sequence with preloads.

The fallback has no field pose, so it clears `AutoPoseHandoff` and saves nothing. TeleOp then localizes from the Limelight or the GP2 D-pad Left corner rehome as usual. SimLoop covers TeleOp only, and its OpMode inventory lists only `@TeleOp` classes, so this autonomous is covered by `FallbackAutoControllerTest` and needs on-robot checks.

## Pose handoff to TeleOp

While a field-referenced routine runs, `BioBuzzRobot` saves the latest READY Pinpoint pose in `AutoPoseHandoff` each loop; autonomous INIT and a Pinpoint localization fault clear it. The next competition TeleOp of the same alliance applies it during INIT if it is at most 60 s old, so auto aim is localized from the first TeleOp loop. Cleared-area local starts (the pollen test) are never saved. See [competition TeleOp](CompetitionTeleOp.md) for the TeleOp side and the practice caveat.

## Named command contract

The [authoring guide](ZenithAuthoringGuide.md) documents all 43 supported actions and 23 cached conditions. `autos/robot.json` provides editor controls, units/defaults/bounds and resource keys. Old command names/defaults remain compatible. Independent yaw/hood/RPM modes cannot be overwritten by later background aim updates; requested forward intake reverses on confirmed HIGH and resumes forward on LOW; explicit reverse is allowed. Transfer pause/end cleanup preserves queue knowledge.

Automatic shooting still requires verified calibration, homing, field pose, hood/yaw/RPM readiness (release timing no longer gates firing; current values are used); no visible tag or new motion-speed cutoff is added. Explicit `feedManual` is a separately named readiness bypass with the gate delay and inhibit priority. Preset measurements remain unverified and cannot automatically feed. `stopAll` is resumable and retains home/positional posture; terminal cleanup closes cameras and invalidates terminal turret state.

## Architecture and verification

`PedroDrive` owns the existing mecanum and Pinpoint subsystems. The shared [drivetrain](Drivetrain.md) uses goBILDA 5000 bare motors, 20:80 gears and coaxial 30:91 belt drive (12.133333 motor rev/wheel rev). It selects open-loop motor power for Pedro during INIT, preserving existing aliases, cloned motor-shaft specs (28 ticks/rev, 5800 RPM catalog-tested no-load at 12 V, internal gearing 1:1), directions and BRAKE. TeleOp keeps its existing mode. Pinpoint pod resolution is independent of the drive transmission; loaded velocity/braking in Java and the editor still require measurement on this hardware. Its Pinpoint adapter converts cached meters to inches without changing field axes or rotating global velocity twice. Start-pose writes wait until Pinpoint is READY so IMU calibration cannot overwrite them. Each loop clears REV caches, reads Pinpoint once, updates the turret, updates Pedro once, runs Ivy, applies mechanism outputs, and flushes telemetry once.

November 5 drive correction: the shared hardware directions are left FORWARD and
right REVERSE, reversing all four motor outputs for Pedro diagnostics, Red/Blue
competition autonomous, Pollen navigation and generated Zenith routines. Path
coordinates, Pinpoint pod directions and Pedro's left/CCW convention stay consistent.

Tests cover unit conversion, mecanum signs and saturation, actual Pedro hold/path-speed behavior, the web tuner's hardware wiring (full-power tuner drive, capped verification drive, one Pinpoint update per localizer update) and registration rules, the pollen planner and replanning, the packaged pollen asset, and Zenith curves, markers (including endpoint and interrupted markers), parallel timeouts, branches, deferred starts, cancellation, and invalid files. These are JVM checks, not evidence of physical robot tuning.

The canonical pollen source is `autos/pollen-navigation-test.auto.json`. Team deployment copies it and all field metadata into APK assets while preserving its isolated POLLEN_TEST profile and sole Test wrapper. Its start pose is explicitly a cleared-area local reference, not an absolute field reference. The cleared-area field intentionally has no season plugin, so scoring/ledger checks are inert and the CLI reports that notice. The camera-guided collection estimate is unknown; its 28-second bound is not a predicted trajectory.

Field model provenance: autos/field/biobuzz.field.json was copied unchanged from Horizon-36596/zenith v0.1.2, commit f7c3681a158c6f504ab211464559d52da8a46955, examples/starter/autos/field/biobuzz.field.json. Its per-feature provenance distinguishes specifications from approximations. [Zenith's MIT license](ZenithFieldLicense.txt) accompanies the copy.
