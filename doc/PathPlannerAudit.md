# Path planner audit and TeleOp robot functions (2026-11-19)

Branch `V1`. The task named the working branch only as `<WORKING_BRANCH>`; this audit compares against `master`, the
only branch with the old Foresight tuning (natural forward deceleration 44.3 in/s²) and a working Pedro 3 stack
(`pedro/Constants.java`, `pedro/AutoDriver.java`). `master` has no committed planner export, so the comparison is
master's Pedro stack against V1's `PedroDrive` + planner export.

## Part 1: why planner autos misbehave on V1

### Behavioral differences, master vs V1

| Area | master (worked) | V1 |
|---|---|---|
| Pedro library | released `revhub 3.0.1` / `core 3.0.1`, `Foresight` | pinned snapshot `core 3.0.0-20260828…`, `ForesightV3` + `PedroForesight` hold guard (Zenith) |
| Drivetrain | stock `Mecanum`: FLOAT while following, 0.01 write threshold | `MecanumDrivetrain` BRAKE at zero power, open loop, via `PedroMecanumAdapter` (same mixing) |
| Localizer | `PinpointLocalizer`: I2C read inside `follower.update()` | one read at loop top (`drive.readSensors()`); Pedro used it after all mechanism work |
| Tuning | master chassis, tuner with FLOAT | V1 chassis, tuner with BRAKE (`ForesightTuner` doc) |
| Path start | `line(follower.pose(), target)` | the drawn start point |
| Step done | `!follower.isBusy()`: end hold converged or 0.1 s | Pedro 3 "SETTLED" exported as `!follower.following()` = the parametric end |
| Heading | `.constant(target)` | `.linear(from, to)` since planVer 5 |
| Speed / cap | `maxPathSpeed` only when < 1 | speed 1.0, wheel cap 1.0 (was 0.35 / 0.35) |
| Mechanisms | none | `PlannerRobot` every loop; shots braked the drive |

### Suspects

| Suspect | Verdict | Evidence |
|---|---|---|
| Step completion ("SETTLED") | **Confirmed, fixed** | `stepCaseV3` wrote `!follower.following()`. In `ForesightV3`, `following()` goes false at `closestT ≥ 0.975` while the robot can still be moving. The next step started at once. Master waited for `isBusy()` (hold converged, or 100 ms). Fix: `PedroDrive.settled()`; SETTLED exports `drive.settled()`. |
| `BRAKE_WHILE_SHOOTING` / `drive.stop()` | **Confirmed, fixed** | Combined with the above, a volley after a path set zero power on BRAKE while the robot was still moving, so it skidded off its end point. Fix: the export brakes only once `drive.settled()` (or `T.SETTLE_TIMEOUT_S`, 0.5 s). |
| Paths start at the drawn point | **Confirmed, fixed** | After a skid, pollen pickup or relocalization the robot is off the drawn start and swerves back onto the line. Master started at `follower.pose()`. Fix: `T.PATHS_FROM_ROBOT_POSE` (default on) starts the first segment and the linear heading at the robot's pose. |
| Pose latency from mechanism code | **Plausible, reduced; needs a robot test** | `drive.update()` ran after `robot.apply()`, so Pedro used a pose aged by every mechanism write. Now `drive.update()` runs right after the step. New telemetry: "Pose age at Pedro update (ms)" and the loop period/work. Loop rate cannot be measured off the robot. |
| Mechanism code writing drive / holding the follower | Ruled out | Only `PollenPathFollower` touches the follower, and only during Collect pollen. |
| `poseSeen` gating, pose-stale stop | Ruled out | The sample is refreshed every loop before the check. A stale-pose stop prints a FAULT and ends the auto. |
| `.constant(H)` heading snap | Was a cause, already fixed | planVer 5 made `linear` the default; `testdrive` already used it. Explicit "constant" is still available. |
| Primary gains "derived from a ratio" | Ruled out | `ForesightTuner` computes primary = secondary × (ALPHA_LARGE / ALPHA_SMALL)² = (10.2/6.2)² = 2.7065556711758574, the same as every tuner run. |
| Braking deceleration 44 → 76 in/s² | Needs a robot test | Not a regression by itself. Master's 44 was measured on another chassis with FLOAT; V1's 76 was measured with BRAKE at zero power, which is how V1 autos drive. `autos/robot.json` copies match. |
| Speed / wheel power 0.35 → 1.0 | Needs a robot test | A cap below 1 also capped braking (documented overshoot). Full power matches the tuner run. Try speed 0.5 with the cap still at 1. |
| Pedro snapshot vs 3.0.1 | Not a confirmed cause | Same algorithm family; differences are path skipping, drivetrain-aware interpolation and `maxPathSpeed`. |
| Start pose written before Pinpoint settles | Needs a robot test (not changed) | TeleOp waits `READY_SETTLE_SECONDS`; `PedroDrive` applies the start pose on the first READY read. Changing it would alter the Zenith autos and their tests. Check the INIT pose (below). |

`PedroConfiguration.TUNING_VERIFIED` stays `false`.

### Robot test checklist

Battery ~14 V, robot on a cleared field, the same battery for every comparison run.

1. **INIT pose.** Run `testdrive` to INIT. "Pedro pose (in, deg)" must read the start pose (-12.4, -65.4, 179.8)
   and "Pose ready" true before START. If it reads near (0, 0, 0), the start pose was lost to the IMU reset.
2. **Test: Pedro Drive.** Hold, 24 in line, curve and 90° turn checks (doc/Autonomous.md). Watch "Pedro velocity",
   overshoot at the end and the time to settle. This decides whether the 76 / 112 in/s² braking fits.
3. **A/B with mechanism code off.** Set `T.MECHANISMS_ENABLED = false` in `testdrive`. Paths run exactly as planned
   and every robot step is skipped. Run it, then run again with `true`. Compare path overshoot and the telemetry
   "Java loop (ms, Hz)" period and "Pose age at Pedro update (ms)". A large difference points at loop time.
4. **Speed A/B.** Set the drive steps' speed to 0.5 (keep `T.MAX_WHEEL_POWER = 1`). If the 1.0 runs overshoot and the
   0.5 runs do not, re-run the Foresight tuner's braking identifiers.
5. **Volley.** Watch "Shot" (AIMING → FEEDING → DONE, or "fed before READY (timeout)") and "Aim range / hood / RPM".
   The robot should settle on its end point before the drive stops for the shot.
6. **Old behaviour for comparison.** `T.PATHS_FROM_ROBOT_POSE = false` restores drawn start points; a drive step's
   completion PARAMETRIC_END restores the old move-on-at-path-end timing.

## Part 2: TeleOp robot functions as planner steps

TeleOp's shot, feed, transfer and intake arbitration moved from `MecanumTeleOp` into `CompetitionShotPipeline`.
TeleOp and `PlannerRobot` both call it. All 125 SimLoop TeleOp traces were byte-identical before and after.

| TeleOp function | Shared class / method | Planner step (`PlannerRobot`) | Before |
|---|---|---|---|
| RT auto aim (yaw, hood, RPM), Limelight pose correction | `CompetitionShotPipeline.aim` → `ShotCoordinator.update` | Volley; Aim (yaw only); Flywheel spin-up (RPM + hood) | own aim math |
| RT + LT automatic fire, readiness + `READY_STABLE_SECONDS` | `FiringCoordinator` via `ShotCoordinator`, `CompetitionFeedController` | Volley; Shoot (wait ready) | fallback readiness, manual feed |
| GP1 A / GP2 RT manual feed | `Request.manualFeed` | Shoot (manual); volley timeout FIRE | yes |
| Auto-aim trims (GP2) | `AutoAimAdjustments.set` (new) | Volley trims | no |
| Nearest-cell target | `ShotCoordinator.setFollowNearestCell` (new opt-in) | Volley / Aim "nearest own cell" | no |
| Manual hood (GP2 D-pad) | `TurretSubsystem.setPitchAngleDegrees` | Hood | yes |
| Manual / idle flywheel RPM | `Request.manualRpm`, `FLYWHEEL_IDLE_RPM` | Flywheel RPM / IDLE / OFF | partly |
| Bang-bang flywheel control | `CompetitionShotPipeline.FLYWHEEL_CONTROLLER` | always | PIDF |
| Manual turret yaw (GP2 RS) | `TurretSubsystem.setTargetYawDegrees` | Aim fixed heading | yes |
| Turret rehome (GP2 X) | `TurretSubsystem.homeTurretAndZeroEncoder` | Re-home turret | yes |
| Collection, RB force, LB reverse, full-queue stop | `IntakeQueueController` via `CompetitionShotPipeline.feed` | Intake IN / UNTIL FULL / FORCE / REVERSE / OFF | copy of the rule |
| Stage-1 loading | `TransferSubsystem` via `CompetitionShotPipeline.feed` | Transfer LOAD / HOLD | partly |
| Gate | `GateSubsystem` via `CompetitionShotPipeline.feed` | Gate | yes |
| GP2 LT feed inhibit | `Request.inhibit` | not a step (Stop all / Gate cover it) | no |
| Depollinator (GP1 stick clicks) | `DepollinatorSubsystem` | Depollinator | yes |
| Linear slide (GP2 stick clicks) | `LinearSlideSubsystem` | Slide | yes |
| Corner rehome + Limelight refinement | `AutoStartRelocalization`, `ShotCoordinator.requestFieldRehomeRefinement` | Relocalize HIVE TAGS / SET POSE | INIT only |
| Auto pose handoff | `AutoPoseHandoff` | saved every loop | yes |
| Status lights | `RobotStatusLights` | automatic TeleOp colors; Lights step | colors only |
| Pollen collection (vision test) | `PollenNavigator`, `PollenPathFollower` | Collect pollen | yes |
| Rumble, diagnostics toggle | `DriverRumble`, `MatchClockRumble` | not applicable in autonomous | — |

A queue-empty stop is not possible: the transfer sensor only confirms a full queue, so a feed ends on time.

Plans: `tools/path-planner/plans/testdrive.plan.json` (reconstructed from `testdrive.java`) and
`all-steps.plan.json` (every step type; its export compiled, then it was removed from TeamCode).
