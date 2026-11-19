# Zenith routine authoring

Start here to compose autonomous routines. The software catalog contains 43 commands and 23 cached conditions. [The audit](ZenithIntegrationAudit.md) compares it with the plan and official runtime; [the coverage ledger](ZenithFunctionCoverage.md) maps every robot function. Software checks and the [core local UI round trip](ZenithUiAcceptance.md) pass. Physical commissioning and the explicitly listed additional UI checks remain separate acceptance work.

## What coders can change

| In Zenith, without editing Java | Requires robot/configuration work |
| --- | --- |
| Create/name routines, choose alliance/start pose, edit waypoints/lines/curves and heading modes, set path speed/timeouts. | Measure drive/braking/gains and footprint, change hardware/control laws, add new commands or measured shot presets. |
| Add any supported action below, set its parameters, place markers, sequence/parallel actions, branch or wait on named conditions. | Calibrate camera/pickup/color, shooter/release timing and servo/encoder references. Raw hardware programming remains in isolated test OpModes. |
| Save canonical source, then run team deployment/build to get one selectable OpMode per routine. | Install the built APK through the team's normal Control Hub workflow; verify the actual match route and STOP behavior. |

Use the official [Zenith v0.1.2 editor](https://github.com/Horizon-36596/zenith/releases/tag/v0.1.2) and open the local repository folder containing `zenith.json`. The browser editor also exists, but repository folder access must work before it can be used for this workflow. Its Code panel is a generated reference/export view; editing an arbitrary handwritten Java OpMode through that panel is not part of this integration.

## Source and deployment

Open the repository's `zenith.json` in a compatible Zenith editor. `autos/` is authoritative: edit routines there, never in APK assets or generated Java. Start with `autos/integration-example.auto.json`; it is explicitly an unmeasured cleared-area example, with no match strategy. `autos/pollen-navigation-test.auto.json` is the separate local-coordinate collection experiment. A new match file defaults to the full robot hardware profile. Name files in lowercase kebab-case and keep their `name` equal to the file stem.

The supported workflow is **UI edit/save → team deployment → build/install**:

```powershell
npm ci --ignore-scripts
npm run zenith:validate
npm run zenith:test
npm run zenith:deploy -- --dry-run
npm run zenith:deploy
npm run zenith:check
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
.\scripts\check-zenith-apk.ps1
```

Node 22+ is required. CLI/schema remain pinned through the lockfile; runtime is Zenith v0.1.2, Ivy 1.1.1 and Pedro 3.0.0-20260828.185437-17. Deployment invokes the pinned CLI's public planner without modifying its package. It retains the hand-written pollen Test wrapper, generates exactly one full-robot wrapper per other routine, deploys all field/robot/waypoint/profile assets, and writes content digests to `deployment-manifest.json`. Generated examples appear in the **Generated** group. Legacy red/blue placeholder wrappers are disabled; authored red/blue match files generate their own entries.

The editor's native Deploy button runs upstream generation and cannot be treated as completing the team workflow. It can produce a redundant full-robot pollen stub. `zenith:check` rejects that output; team deployment removes the recognized duplicate. Never select a raw generated pollen wrapper. Rename/delete routines in canonical source and run team deployment again: only obsolete manifest-managed outputs are deleted. Unexpected Java/assets or edited obsolete outputs are preserved with an actionable error. Source-control rollback followed by team deployment restores the corresponding generated assets; no deployment installs an APK on the robot.

In v0.1.2, **New auto** starts a generic `new-auto.auto.json` file; Ctrl+S writes it into `autos/`. The verified workflow uses source-file maintenance for rename/delete: rename the file and its JSON `name` together, update `title` for its Driver Station label, then deploy. No Java edit is needed. The UI catalog is global; the pollen profile does not filter it. Team validation rejects actions outside that test's hardware capabilities. Reopen the project after changing robot/catalog configuration so the editor reloads metadata.

Validation checks registry names, arguments, resources, profiles, stems, IDs, pins and native schema/planning. `zenith:check` additionally checks deployed freshness. The pinned scanner cannot infer these indirect factories' numeric helpers and produces repeated registry advisories; team contract checks and JUnit runtime-factory tests cover that gap. Repeated advisories are condensed, not discarded. The cleared-area field intentionally has no scoring season plugin. Planner transfer/ledger annotations never command hardware.

Keep the supplied `autos/robot.json` catalog: importing commands from Java alone cannot reconstruct its typed parameters, limits and units from these indirect factories. After code/configuration changes, `tools/verify-teleop.ps1` runs ordinary tests, packaging and lint; optional simulations have the scope documented in the verification record.

## Coordinates, parameters and ownership

| Quantity | Meaning |
| --- | --- |
| Path, start/waypoint, hold/reseed coordinates | Inches, center origin, +X audience right, +Y away from audience; headings CCW radians from +X. |
| Team pose/velocity telemetry | Meters, m/s and CCW radians/rad/s. |
| Turret yaw | Internal CCW degrees from right-facing home, [-175, 175]. Not chassis heading. |
| Hood | Downward travel [0, 30] degrees; launch elevation = +15 minus travel. Home offset and 2:1 linkage remain implemented in Java. |
| Flywheel | Output-shaft RPM [0, 6000], with 1:1 motor-to-flywheel gearing (goBILDA 5203 Yellow Jacket 6000 RPM). |
| Durations | Positive seconds up to 28; readiness waits default to 5 seconds. The global autonomous deadline is 29 seconds. |

Parameter type/default/range and unit help are in `autos/robot.json` using supported schema fields. Invalid arguments or unsupported hardware actions fail during INIT, including in unselected branches. Relative yaw/hood deltas resolve once at command START; the resulting target is checked then because preceding steps can change it.

Latched setters finish instantly but persist until replaced. Timed actions own their requirements until completion and clean up their own outputs. Requirements are `drive`, `intake`, `transfer`, `gate`, `depollinator`, `turretYaw`, `hood`, `flywheel`, `pollenCamera`, `limelight`. Put disjoint mechanisms alongside a path; do not put two writers of one channel in a parallel group or place a drive-owning command on its own path marker. Manual yaw, hood and RPM setters replace automatic ownership only for that channel. An explicit flywheel stop persists while automatic yaw/hood tracking can continue.

Absolute named hold/reseed poses are authored coordinates and receive the context's alliance transform exactly once. Live/current starts are already running-frame coordinates. `aim(side)` retains its FIELD/AUDIENCE default; `frame=AUTHORED` also swaps AUDIENCE/FAR under point symmetry. Local yaw/hood/RPM values never mirror. The pollen start seeds cleared-area coordinates without marking the sample absolute-field-referenced; this profile has no shooting hardware.

## Action semantics

| Actions | Persistence, completion and cleanup |
| --- | --- |
| `intakeOn`, `intakeOff`, `intakeReverse`, `intakeSet(power)` | Latched power. Confirmed HIGH stops positive loading; LOW restores forward; deliberate reverse remains allowed. |
| `intakeRun(power, seconds)` | Bounded run; stop on end/cancel. An old cleanup cannot override a replacement request. |
| `loadOn`, `loadOff`, `transferStop` | Stage 1 loads only with room; stage 2 stays off. Pause/stop retains confirmed-full knowledge. |
| `loadUntilFull(seconds)` | Own intake/transfer until confirmed full or timeout, then stop both. HIGH must remain present for 0.5 seconds to confirm. The sensor cannot count pieces. |
| `feedManual(seconds)` | Explicit readiness BYPASS, never an automatic fallback. Open gate, wait 0.10 seconds, feed both stages together; close/stop on end. Feed inhibit wins, even over this override. |
| `gateOpen(seconds)`, `gateClose` | Bounded gate-only opening keeps transfer stopped; close/end clears feed requests. No feed authorization is implied. |
| `feedStop`, `feedInhibit`, `feedResume` | Stop clears requests and preserves aim/pre-spin. Inhibit latches; resume clears it without starting feed. |
| `depollinatorDeploy`, `depollinatorHome` | Command posture; 30-degree home and 135-degree CW deployment. One-second settle is an UNMEASURED estimate, not position feedback. STOP retains posture. |
| `turretHome(timeoutSeconds)` | Existing nonblocking travel/stability estimate; success, timeout or fault. Cancel holds the last bounded command and invalidates unfinished home. A stationary stall is not detectable without an absolute sensor. |
| `turretSetYaw(yawDeg)`, `turretAdjustYaw(deltaDeg)`, `turretHold` | Set, adjust once, or capture measured yaw once. Hold retains a valid home. |
| `waitTurretReady(timeoutSeconds)` | Cached homing/error wait, 3-degree tolerance. No new chassis/turret speed feed cutoff. |
| `hoodSet(travelDeg)`, `hoodAdjust(deltaDeg)`, `hoodHome`, `waitHoodReady(timeoutSeconds)` | Bounded setpoints and estimated tracking. There is no hood position sensor. |
| `flywheelSetRpm(rpm)`, `flywheelSpinUp(rpm, timeoutSeconds)`, `flywheelStop` | Set or wait for independently sampled speed; natural spin-up success retains RPM. Timeout/cancel stops it. Spin-up never feeds. |
| `selectTarget(side, frame)`, `aim(side, frame, spin)`, `aimOff` | Target change requalifies readiness. Aim tracks field odometry; a visible Limelight tag is not required. `spin=false` leaves yaw/hood automatic and flywheel off. Aim-off holds yaw/current hood and stops feed/spin. |
| `shoot(seconds)` | Requests existing calibrated automatic aiming/feed; shared firing/recovery controller and readiness policy. Retains aim/pre-spin afterward; closes gate and stops transfer. Counts timed attempts, not balls. |
| `prepareShotPreset(preset)`, `shootPreset(preset, seconds)` | Fixed robot-local setpoints and shared firing controller. Only `diagnostic-example` exists; it is UNMEASURED and cannot automatically feed. Unknown names fail INIT. Add measured presets and verified release timing in Java configuration before automatic preset use. |
| `driveHold(xIn,yIn,headingRad,seconds)`, `driveStop` | Pedro bounded hold or immediate stop/cancel. End/cancel stops drive. |
| `setFieldPose(xIn,yIn,headingRad)` | Owns drive/gate/transfer; stop/cancel following and feeding, invalidate shot history/readiness, seed known pose, require a new sensor sample before readiness. Never resumes an old path. Not an IMU recalibration action. |
| `collectPollen(seconds)` | Cached calibrated camera observations; bounded Pedro search/pickup. Camera/pose loss stops motion. Outcomes distinguish navigation fault, timeout and completion. Trajectory/duration estimates remain unknown. |
| `pollenVisionEnable`, `pollenVisionDisable` | Control the existing INIT processor; enabling requires a new frame. Disable cancels/stops collection. No mid-loop camera construction. |
| `limelightSetPipeline(pipeline)` | Owns Limelight/gate/transfer. Stop feeding, accept only configured pipeline 0 and invalidate observations/history. Autonomous remains odometry-only. |
| `stopAll` | Resumable stop of motors/drive/feed and bounded positional hold; keeps valid home/reference. Terminal STOP additionally shuts down turret/camera resources. |

Every branch reads cached conditions. Use `commandFailed` after a bounded named action or native condition wait; its last SUCCESS/TIMEOUT/INHIBITED/FAULT/CANCELLED outcome, reason, step and timestamp survive following instant setters and duration-only waits. An outer native timeout interrupts its inner named command and therefore reports CANCELLED; put a native timeout slightly beyond the named bound if you want its own TIMEOUT outcome. Native condition waits must declare `timeoutS` and report TIMEOUT if their condition remains false. Native paths retain upstream arrival/timeout behavior and do not set the team's `commandFailed` outcome. This condition is the last completed result, not a summary of every member of a parallel group. Use specific readiness predicates after parallel preparation.

Group `timeoutS` is unsupported by Zenith v0.1.2 and is rejected by team validation. Bound the group's leaf paths, actions and waits. Exceptions, localization loss and the global deadline stop the robot; the initiating fault/deadline outcome survives cancellation cleanup. A normal failed preparation can branch to an authored safe stop/park. `stopAll` does not terminate the file; later actions can resume outputs, so place the remaining scoring work inside the success branch.

Conditions are: `queueFull`, `queueSensorHigh`, `loadingActive`, `feedingActive`, `gateFeedDelayElapsed`, `feedInhibited`, `localizationReady`, `driveIdle`, `turretHomed`, `turretAtTarget`, `hoodTrackingEstimated`, `flywheelAtSpeed`, `aimReady`, `shotReady`, `pollenVisible`, `pollenCameraReady`, `limelightFresh`, `selectedGoalTagVisible`, `depollinatorDeployedCommanded`, `depollinatorHomeCommanded`, `depollinatorSettledEstimated`, `commandFailed`, `matchTimeLow`. The last uses the fixed team threshold of five seconds remaining. These are named booleans; parameterized predicates are not a supported extension.

| Condition group | How to use it |
| --- | --- |
| Queue/loading | `queueSensorHigh` is raw digital status; `queueFull` requires continuous HIGH for 0.5 seconds and clears immediately on LOW. `loadingActive` and `feedingActive` describe actual commanded stage activity. Neither counts balls. |
| Gate/inhibit | `gateFeedDelayElapsed` says open-command delay has elapsed; it does not authorize shooting. `feedInhibited` reports the latched override. |
| Drive/localization | `localizationReady` includes READY/fresh/reference checks. `driveIdle` means Pedro IDLE; holding a reached endpoint is HOLD, so this is not a path-arrival predicate. |
| Shooting | `turretHomed`, `turretAtTarget`, `hoodTrackingEstimated`, `flywheelAtSpeed` expose individual readiness. `aimReady` only means the current solution is aimable. `shotReady` also needs valid feeding calibration and yaw/hood/RPM readiness; the firing controller still applies its stability/recovery policy. |
| Cameras | `pollenCameraReady` is camera status; `pollenVisible` also needs autonomous eligibility and a usable target. `limelightFresh` and `selectedGoalTagVisible` use cached validated observations. A visible tag alone does not authorize feeding. |
| Depollinator | Home/deployed predicates report commanded posture; settled is elapsed estimated travel time, not sensor feedback. |
| Flow | `commandFailed` inspects the preceding relevant bounded result; `matchTimeLow` becomes true at five seconds remaining while running. |

## Common recipes

For collection along a planned path, put `intakeOn` and `loadOn` before the path, then use `queueFull` as an end condition if desired. Stop intake/loading afterward. A marker may set intake or depollinator posture; a drive-owning action on that path conflicts with the path itself. Resource locks on instant setters end immediately, while their requested state persists.

For fixed manual preparation, sequence `turretHome`, a failure branch, `turretSetYaw`, `hoodSet` and `flywheelSpinUp`; wait on required readiness before an explicitly chosen feed action. For calibrated automatic shooting, `aim` pre-spins/tracks and `shoot` requests automatic feed; `feedStop` ends feeding while keeping aim/pre-spin. The current unmeasured shot configuration prevents automatic feeding.

This sequence fragment illustrates a failed preparation branch. Insert it in the steps list of an existing valid routine; it supplies no route or competition shot calibration:

```json
{
  "id": "boundedPreparation", "kind": "sequence", "steps": [
    { "id": "home", "kind": "command", "name": "turretHome",
      "args": { "timeoutSeconds": 3 }, "timeoutS": 3.1 },
    { "id": "homeResult", "kind": "branch", "condition": "commandFailed",
      "then": [ { "id": "failedHomeStop", "kind": "command", "name": "stopAll" } ],
      "else": [ { "id": "beginAim", "kind": "command", "name": "aim",
        "args": { "side": "AUDIENCE", "frame": "FIELD", "spin": true } } ] }
  ]
}
```

For a bounded native readiness wait, select a named condition and set `timeoutS`, then branch on `commandFailed` or check the specific condition. For estimated depollinator settling, wait on `depollinatorSettledEstimated`; verify clearance using the maximum deployed envelope, which is still unmeasured. Inspect `autos/integration-example.auto.json` for curves, parallel pre-spin, endpoint markers, estimated waits and an inhibited manual-feed demonstration.

## Verification boundaries and handoff

Pedro tuning remains unverified and blocks autonomous START. Pollen intrinsics/color/pickup eligibility, shot profiles retain their existing gates; release timing no longer gates firing (2026-11-15). Footprint and height are unmeasured placeholders; the expanded 18-inch box is not a verified deployed envelope. Measure the maximum mechanism envelope and use that conservatively before approving clearance. No metadata or example sets a measured flag to bypass these requirements.

Standalone Drivetrain, Pinpoint, Pedro, Intake, Transfer, Gate, Depollinator, Turret, Limelight, camera/shot calibration and disconnected-servo programmer OpModes remain the maintenance path. Raw motor/PIDF/pod/encoder/servo programming is intentionally outside routine actions; adding hardware/control laws/calibration still requires Java/configuration changes.

The [UI acceptance record](ZenithUiAcceptance.md) contains the completed v0.1.2 procedure, exact UI-created regression source and generated entry, rejected/corrected wait, and rename/delete cleanup. Additional checks when extending the project include named-waypoint editing and each new parameter/control type. UI estimates, including a zero total for unknown collection timing, are not runtime or physics guarantees. Mandatory team validation/build remains part of authoring even when the UI reports zero errors.

Robot acceptance then measures drive dynamics/signs/turns/braking/clearance, material handling delay/full behavior, turret home/bounds, estimated servo travel and flywheel recovery, camera calibration and shot timing. Test STOP and faults in a cleared area. Only then author actual alliance match strategy with measured coordinates, per-step bounds and an explicit stop/park policy.
