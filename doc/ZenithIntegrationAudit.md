# Zenith implementation audit

Audit dates: 2026-10-30–31. The audited working tree already contained the expanded integration; its fresh baseline passed 178 ordinary tests. This audit checked the production entry points, command factories, output coordinator, subsystem changes, profile/argument contracts, deployment tooling, native-runtime tests and autonomous documentation against [the implementation plan](ZenithAutonomousImplementationPlan.md). Existing and concurrently developed SimLoop work was preserved.

**Result:** the supported robot action catalog, managed source/deployment and core local editor workflow are implemented and verified in software/UI. Physical match readiness remains open, along with the additional checks explicitly listed in [the UI record](ZenithUiAcceptance.md). There are 43 named actions and 23 cached conditions. [The authoring guide](ZenithAuthoringGuide.md) explains every action; [the coverage ledger](ZenithFunctionCoverage.md) maps operational, sensor and maintenance functions.

## Comparison with the plan

| Plan phase | Current evidence | Remaining acceptance |
| --- | --- | --- |
| 0: baseline/inventory | Fresh 178-test pre-audit baseline, APK/lint; registry, profiles, hardware and upstream pins inspected. | No clean-tree claim: implementation work was already uncommitted. |
| 1: canonical project/deployment | Both routines opened in the actual editor; manifest-managed assets/stubs; isolated pollen Test wrapper; real UI-created routine generated, built, renamed and removed. Legacy placeholders disabled. | Real match files still require strategy/coordinates. Rename/delete use source-file maintenance plus team deploy. |
| 2: lifecycle/ownership | INIT hardware, cached sensing, Pedro then Ivy then resolved outputs; explicit yaw/hood/RPM modes; replacement cleanup leases; resumable stop versus terminal shutdown. | Real STOP, partial hardware failure and startup commissioning. No claim of exhaustive hardware-fault coverage. |
| 3: material handling | Independent intake, loading, gate, feed/inhibit and depollinator actions; confirmed-HIGH reversal, LOW recovery and gate delay tested. | Measure loaded gate/transfer/posture behavior and deployed envelope. |
| 4: aiming/shooting | Independent yaw/hood/RPM, home/readiness waits, targets, automatic aim/feed and fixed presets implemented. Manual setpoints survive background updates. | Measured shot table, release timing and usable competition presets; real homing/servo/RPM verification. |
| 5: vision/localization/feedback | INIT camera controls, cached conditions, known-pose reseed, drive hold/stop, outcomes and telemetry implemented. | Camera transitions/loss on Android; physical metadata measurements. Optional persistent trace export/full autonomous simulation are not configured. |
| 6: automated verification | 185 ordinary tests, 12 deployment tests, all 24 current simulations, APK/lint pass. Includes actual UI-authored JSON through native factories and packaged-asset checks. | Simulator camera convention remains conditional; no full autonomous dynamics/webcam/physical proof. |
| 7: actual UI round trip | Core local workflow verified in checksum-verified v0.1.2: 43-action/23-condition catalogs, numeric/enum/Boolean arguments, start/curve/marker/wait, group/branch save/reopen, generation/build and cleanup. | Named-waypoint library editing and every field of every action were not individually exercised. Native Deploy/cloud/PR features are outside the supported team workflow; see UI record. |
| 8: robot/match commissioning | Existing safety gates and team configuration retained. | Physical measurements, UI-authored robot integration run, STOP/fault checks and actual alliance match routes. |

## Audit fixes

1. `setFieldPose` now requires drive, gate and transfer. It closes/stops feeding while invalidating the reference; native parallel construction rejects simultaneous feeding instead of allowing undeclared output writes.
2. `limelightSetPipeline` now requires Limelight, gate and transfer and explicitly stops feeding before invalidating observations. Only configured pipeline 0 is accepted.
3. Native condition waits publish SUCCESS/TIMEOUT/CANCELLED through the existing step hook, including nested groups. `commandFailed` can observe a timed-out condition wait after a following instant setter. Native scheduling/timeout construction remains upstream.
4. Localization faults and the global deadline retain their terminal outcome when cancellation runs command end hooks. An interrupted action cannot replace the initiating fault/deadline with CANCELLED.
5. Team validation rejects `timeoutS` on sequence/parallel/branch groups: v0.1.2 ignores it. Bound leaf actions/waits/paths instead.
6. A regression compares twelve editor drive/braking/tolerance values against `PedroConfiguration`, preventing planner/runtime drift.
7. Updated stale documentation that placed pollen source in assets, described disabled match wrappers as selectable, omitted the manual-feed action, or described only the original command set. Deployed metadata was regenerated.
8. Retained the actual UI-saved JSON as a non-deployable test resource and verified it through native runtime factories, including its timeout outcome and final shutdown. Added the five audit regressions to the named test baseline.

See [the verification record](ZenithVerification.md) for exact commands and limits. No dependency or measured/verified flag was changed.

## Current robot configuration

These are code/configuration facts retained from the team's current subsystems. They are not new measurements or a remote inspection of Control Hub wiring.

| Mechanism | Current configuration used by autonomous |
| --- | --- |
| Mecanum | `leftfront`, `rightfront`, `leftback`, `rightback` plus existing aliases. goBILDA 5000 bare motors, 20:80 gears then coaxial 30:91 belt, 12.133333 motor rev/wheel rev. Cloned SDK specs remain 28 motor ticks/rev, 5800 RPM catalog-tested no-load at 12 V, internal gearing 1:1. Left motors FORWARD, right REVERSE after the November 5 all-directions correction; BRAKE. Pedro selects open-loop power during INIT; wheel cap 0.35; loaded dynamics unmeasured. See [drive units](Drivetrain.md). |
| Pinpoint | `pinpoint`, goBILDA 4-Bar pods, +32 mm X-pod and -74 mm Y-pod offsets; existing pod directions. Stationary INIT calibration, READY-deferred start, one bulk update per loop, cached global velocities converted from meters to inches. |
| Intake | `intake_motor`, existing forward direction and 10:8 gearing; positive autonomous intake stops at confirmed full, deliberate reverse remains allowed. |
| Transfer/full sensor | `transfer_motor_1` forward, `transfer_motor_2` reversed (both flipped 2026-11-18: motors wired backwards); `distance_sensor` is a digital HIGH/LOW input, not a distance measurement. Two-second continuous HIGH confirmation and held-full state; stage 2 only feeds. |
| Gate | `gate_servo`, reversed, 500–2500 microseconds/300 degrees; 150-degree closed and 60-degree open command, 0.20-second feed delay. Gate-only opening does not feed. |
| Depollinator | `depollinator`, existing Axon limits/direction, 30-degree home plus 135-degree CW deployment. One-second settle is an unmeasured estimate. |
| Yaw/hood | `turret_servo_1`, `turret_servo_2`, `turret_pitch`; configured-name aliases. Local yaw [-175,175] degrees, 1:1 servo gearing, right-facing home; external encoder 108:40 gearing. Current wiring: turret feedback reads the Through-Bore encoder on `intake_motor`'s encoder input (Expansion Hub quad port 1; the intake has no encoder), with the standalone `turret_encoder` alias and no shared-channel reconfiguration/reset; `transfer_motor_1` has its own motor encoder on port 2. Hood downward travel [0,30] degrees, 2:1 linkage and 8-degree home offset from servo 0.5. |
| Flywheel | `turret_motor` and existing shooter aliases; 1:1 flywheel/motor ratio, goBILDA 5203 Yellow Jacket 6000 motor RPM and 6000 output-RPM cap; FLOAT. Commands use output RPM. |
| Limelight | Optional `limelight`/`limelight3a`, pipeline 0, 55-degree upward mount, 337 mm lens height; missing hardware rejects its action at INIT. Autonomous aiming uses field odometry and does not apply TeleOp vision pose correction. |
| Pollen C270 | Optional `webcam_1`; team-confirmed mount 195 mm high, 248 mm forward, centered. The October audit used 10 degrees down; the team updated the shared configuration to **15 degrees down on November 5, 2026**. Missing hardware rejects camera actions at INIT. Color/pickup/intrinsics eligibility remains authoritative. |
| Profiles | FULL_ROBOT includes all mechanisms; POLLEN_TEST omits turret, Limelight and depollinator. Planned paths can omit optional cameras. Pollen uses a local cleared-area start, not an asserted absolute field reference. |

## Editor/model limits that authors must know

The path/command tree uses native Zenith semantics: curves and heading modes, current-pose starts, distance/end markers, sequences, all/race/deadline groups, named condition waits and branches. Typed/default/bounded command metadata matches runtime resources and arguments. No arbitrary Java OpMode source editor is supplied by this integration: routine authors edit canonical auto files; generated wrappers load them.

Preview timing uses unmeasured drive examples, an 18-inch placeholder footprint/height and unknown collection trajectories. No measured intake-mouth geometry or deployed depollinator envelope has been supplied. Season ledger records cannot be treated as physical ball counts or hardware actions. The cleared-area examples intentionally use a field without a scoring season plugin.

Planner turret-range metadata is omitted: Zenith's check compares a normalized bearing off the chassis nose with one min/max interval. This robot's nearly full-turn yaw arc is referenced to right-facing home and wraps across that boundary. Copying [-175,175] into that field would describe the wrong reachable sector. Runtime limits remain enforced in the existing turret/shot planner; editor range/clearance/scoring preview is not a substitute. A planner extension or explicitly documented conservative range is needed before claiming complete editor shooting-geometry checks.

`commandFailed` reports the last completed bounded named action or condition wait. It is not an aggregate of a parallel group's history, and native path arrival/timeout does not produce a team outcome. Branch immediately after the preparation/wait you intend to inspect, or test its specific readiness condition after parallel work. Duration-only waits and instant setters preserve the last result. A native timeout interrupting a named action reports CANCELLED; give the action enough time to publish its own TIMEOUT. `stopAll` is resumable and does not skip later steps: place subsequent work in the success branch.

No whole-robot physics bridge is configured in `zenith.json`. Zenith ideal/instant preview is available upstream. Simulation was chosen because the integration changes shared output, gate/feed, turret and shutdown behavior. The final 24-test suite covers actual isolated mechanisms and both competition wrappers; native autonomous trees are tested separately. Raw camera target-normal rejection is retained, and normalized-camera acquisition is a conditional test fixture pending a physical frame sample. No production calibration was changed to match the simulator. This is not full-autonomous physics or hardware evidence. The step hook carries live telemetry/outcomes; persistent trace-file recording is not enabled.

## Official-resource comparison

The pins match [Zenith v0.1.2's robot-runtime contract](https://github.com/Horizon-36596/zenith/blob/v0.1.2/robot/README.md): Pedro core `3.0.0-20260828.185437-17`, Ivy `1.1.1`, Zenith `v0.1.2`. The official latest release API also returned v0.1.2 during this audit; it was published October 27. [Ivy's official source](https://github.com/Pedro-Pathing/Ivy/tree/v1.1.1) is the command library. Current [Pedro documentation](https://pedropathing.com/) must be read with the pinned API in mind; older `PathChain` examples and newer APIs are not interchangeable with this snapshot.

- [Pinned native AutoBuilder](https://github.com/Horizon-36596/zenith/blob/v0.1.2/robot/ivy-runtime/src/main/java/org/horizon36596/zenith/ivy/AutoBuilder.java): native tree semantics, step hooks, leaf timeouts and group limitations.
- [Pinned AutoFromFile](https://github.com/Horizon-36596/zenith/blob/v0.1.2/robot/ivy-runtime/src/main/java/org/horizon36596/zenith/ivy/AutoFromFile.java): INIT/build/start/periodic/scheduler/afterLoop/reset lifecycle. The team subclass retains it while preserving original errors during cleanup.
- [Pinned deployment planner](https://github.com/Horizon-36596/zenith/blob/v0.1.2/packages/cli/src/commands/deploy.ts) and [project schema](https://github.com/Horizon-36596/zenith/blob/v0.1.2/packages/schema/src/link.ts): generated wrappers, fixed Generated group and opModeBase; team deployment adds reserved-profile/manifest handling.
- [Robot schema](https://github.com/Horizon-36596/zenith/blob/v0.1.2/packages/schema/src/robot.ts) and [season checks](https://github.com/Horizon-36596/zenith/blob/v0.1.2/packages/core/src/checks/season.ts): supported parameter/provenance/requirements fields and preview limits.
- [Official editor v0.1.2](https://github.com/Horizon-36596/zenith/releases/tag/v0.1.2): portable executable SHA256 `248e67f366f348b7ac210550ae22a8ce37fd33879eae35291d5aa159bf3e0b70`, matched against official SHA256SUMS. [Actual UI evidence](ZenithUiAcceptance.md) records the subsequent repository round trip.
