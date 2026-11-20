# Transfer loading and shooting sequence

November 4 software integration: [final TeleOp audit](TeleOpConsistencyChecklist.md) records collection while aiming, GP2 auto offsets, independent status lights and the retained current controls. Bench tuning/interlocks remain explicitly documented diagnostic policies.

**Single-stage transfer (2026-11-18).** `transfer_motor_2` no longer drives the transfer
back rollers (they were removed); it is mechanically linked to `transfer_motor_1` on
stage 1. Both motors keep their hardware configuration names and ports (Expansion Hub
motor ports 2 and 3), both are set FORWARD (motor 2 was REVERSE while it drove the back
rollers; a wrong direction makes the linked motors fight each other), and both always
receive the same stage-1 command. There is no separate
stage 2 any more: everywhere below, "stage 1" or "the transfer" means both linked motors.

Ordinary loading runs stage 1 (both motors). Stage 1 always runs at full open-loop
power; motor 1's own encoder on Expansion Hub port 2 is read only for speed
telemetry. Staggered shooting (every OpMode) also keeps stage 1 running while the gate is closed between held timed shots, as described below.

The table below describes the shared timed-shot defaults used by competition, autonomous,
calibration, transfer-only and the bench test (tuned on the robot 2026-11-15): stage 1
starts feeding 0.10 s after the gate opens, and the gate closes after a 0.20 s feed (0.30 s
total gate-open window). Stage 1 no longer waits for the gate-open midpoint (changed
2026-11-18). Competition selects
continuous/staggered as described below. **Test: Intake + Index + Shooter** now uses the competition
continuous policy (2026-11-19): RT + LT feeds after readiness, GP2 RT feeds bypassing readiness, and GP2 LT
inhibits. Unlike competition, the bench starts the transfer 0.10 s after the gate-open command for every feed
and has no flywheel idle. See [bench controls](IntakeShooterTest.md).

| Phase | Gate | Stage 1 (both linked motors) | Duration |
| --- | --- | --- | --- |
| Ordinary loading | Closed | Full loading power until full or paused | Until the full sensor is HIGH continuously for 0.5 s |
| Full confirmation / intake stop | Closed | Continue forward | 0.5 s run-on to seat the fourth ball; pause, feed or shutdown cancels it |
| Confirmed full | Closed | Off | While HIGH remains confirmed; LOW resumes loading |
| Shot readiness | Closed | Paused by the existing shooting controls | Readiness stable for 0.15 s for timed shots |
| Gate command delay | Held open | Off | 0.10 s after the open command |
| Timed shot feed | Held open | Full forward-feed power | From 0.10 s until the window closes at 0.30 s |
| Recovery / readiness wait | Closed | Full power (past full) after a completed timed feed | 0.00 s recovery, then until readiness requalifies |

Each fresh confirmation starts the 0.5 s stage-1 run-on once while stage 1 is loading. LOW immediately
clears full, resets the confirmation/run-on timers and resumes loading. Another HIGH needs a
fresh continuous 0.5 seconds. Bench, competition and autonomous intake stop (never
reverse) at confirmation and resume forward on LOW, with RB/LB overriding the sensor.
This replaces the previous full-queue intake reversal; the stage-1 run-on is unchanged.

An uninterrupted timed attempt commands the gate open for about **0.30 seconds**:
0.10 seconds after the open command plus 0.20 seconds of stage-1 feeding.
With the gate closed after a completed timed attempt, stage 1 keeps
running through the readiness wait; it stops at the next opening and restarts when that feed begins.
This closed-gate run does not apply after tracking-loss recovery, inhibit, release or STOP.
Continued requests repeat readiness, gate opening, feed and recovery. Releasing the shot request, feed inhibit, loss of
required readiness, or STOP may end a cycle early. After shooting, ordinary
stage-1 loading resumes according to the OpMode's existing controls.

Both final Red/Blue TeleOps start CONTINUOUS: once a feed is authorized, the gate-open command and the full-power transfer command (both motors) occur together, without a gate delay. Automatic readiness and GP2 LT inhibit still apply; manual feed bypasses readiness. STAGGERED opens the gate, starts stage 1 at 0.10 s, and closes the gate after the shared 0.20 s feed (0.30 s total open window). Stage 1 keeps running while the gate is closed through the recovery and any subsequent readiness wait. At the next opening the transfer stops, then repeats the shared 0.10 s delay. Release, inhibit, invalid automatic solution, tracking loss or STOP cancels transfer feeding; tracking-loss recovery does not continue closed-gate feed. Mode changes reset the sequence. Autonomous and diagnostic defaults are unchanged. The gate-only test stays open while its trigger is held and does not initialize transfer motors.

In both final competition TeleOps, RB on either gamepad also forces intake and
stage-1 loading forward past the full sensor. LB reverses only the intake and
pauses stage-1 loading; opposing bumpers stop collection. Release restores
automatic loading. These collection overrides leave gate commands
unchanged; shooting requests, gate/recovery pauses and GP2 LT retain their
existing transfer priority. Autonomous and bench control mappings are unchanged.

## Shared settings and affected OpModes

- `GateSubsystem.FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS` owns the diagnostic/autonomous command-to-feed delay;
  `ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS` uses that same value.
- `ShootingConfiguration.FEED_WINDOW_SECONDS` owns the diagnostic/autonomous feed duration.
  Recovery and readiness remain 0.35 s and 0.15 s.
- Both alliance competition TeleOps use `CompetitionFeedController` for mode
  selection and stage-1 feed timing, with the shared 0.10 s gate and 0.20 s feed settings.
  Both match-autonomous adapters and guided shot calibration retain their existing
  shared settings and policies. The combined intake/index/shooter
  test uses the bench-only overrides described above.
  Calibration requests one attempt; the other timed shooting controls repeat
  while requested. Autonomous scoring remains bounded by the route's scoring
  window and match deadline; check those windows when tuning feed duration.
- `Test: Transfer Only` automatically loads with stage 1 (both motors). Hold RB
  to request repeated timed gate-open feeds; LB pauses loading and
  inhibits feeding with priority. Releasing RB closes the gate and
  returns to ordinary stage-1 loading. No shooter/flywheel is initialized or
  powered, and shooter readiness is bypassed for this deliberate mechanism test.
  Start empty when checking directions and gate timing.
- Pollen collection keeps its gate closed for the whole run (stage 1 loads normally).
  Intake-only, turret-only and auto-aim tests do not initialize the transfer.

The team reports immediate gate settling. Software applies the requested fixed
0.20-second command-to-feed delay with no additional settling allowance.
The 0.50-second feed duration remains a starting value for loaded robot testing;
gate arrival has no position feedback.
`FEED_TO_EXIT_SECONDS` remains the separate, unmeasured 0.12-second exit-delay
estimate; it does not stop the motors. Release timing does not gate firing
(gate removed 2026-11-15). Confirm ball clearance, exit timing and flywheel recovery
on the robot when tuning these values. Timed attempts do not count launched balls.

## No stage 1 stall protection (removed 2026-11-19)

`TransferStallGuard` and every transfer stall limit or safety cut-off were removed by
request on 2026-11-19. `TransferSubsystem` now always applies `TransferController`'s
stage-1 command to both linked motors, including while they are stalled against a jam,
in every OpMode (TeleOp, autonomous, planner exports and test OpModes).
`TransferSubsystem.setIntakeRunning(...)`, `isStage1StallProtected()`,
`getStallState()`, the stall telemetry lines and the `DriverRumble` transfer-jam cue
are gone. Stage 1 (goBILDA 5203 1:1, about 9 A at stall per motor) overheated on
2026-11-16 after long continuous running into backed-up balls, so drivers and pit crew
must watch the transfer current telemetry (motor 1 / motor 2, 5 Hz) and clear jams by
hand.

The gate servo is wired directly to Expansion Hub servo port 3, bypassing the 6 V power injector. It receives immediate full-position targets without software ramping for maximum available speed. The 0.043 s/60-degree rating at 6 V is catalog reference only; actual direct-hub loaded speed is unmeasured.

## Desktop verification

Historical record from before the 2026-11-18 single-stage change: all 153 unit tests
passed with zero failures, errors or skips; debug packaging and lint passed (0 errors,
the existing 14 warnings: 12 native-library alignment warnings and 2 unused resources).
The 2026-11-18 single-stage change: all 309 unit tests pass; debug packaging passes; lint
reports 0 errors (18 warnings, none in changed files). 45 of the 49 simulation scenarios
pass, including the transfer-only test and both competition wrappers, with every scenario
asserting the two transfer motor powers never diverge. The 4 bench `IntakeShooterTestOpMode`
scenarios fail in INIT at `FtcDashboard.getInstance()` (no RobotCore native library on the
desktop JVM); the same 4 fail on the unmodified previous commit, so the bench OpMode's
transfer output is not simulation-covered for this change. SimLoop has no ball or linkage
model. No deployment or loaded timing measurement was performed.

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:lintDebug
```
