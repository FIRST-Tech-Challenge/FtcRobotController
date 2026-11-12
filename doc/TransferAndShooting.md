# Transfer loading and shooting sequence

November 4 software integration: [final TeleOp audit](TeleOpConsistencyChecklist.md) records shared intake jam recovery, collection while aiming, GP2 auto offsets, independent status lights and the retained current controls. Bench tuning/interlocks remain explicitly documented diagnostic policies.

Ordinary loading uses stage 1 only; stage 2 stays off. Competition shooting adds the explicit closed-gate recovery feed described below.

The table below describes the shared autonomous, calibration and transfer-only
defaults. Competition selects continuous/staggered as described below.
**Test: Intake + Index + Shooter** has an explicitly
requested bench-only sequence: stage 2 starts 0.10 s after opening, stage 1 starts
at the midpoint (0.25 s by default), and the gate closes after a 0.40 s stage-2
feed (0.50 s total gate-open command). Its continuous mode immediately opens the
gate and runs both transfers at full power until either trigger is released,
bypassing readiness and timing. See [bench controls](IntakeShooterTest.md).

| Phase | Gate | Stage 1 | Stage 2 | Duration |
| --- | --- | --- | --- | --- |
| Ordinary loading | Closed | Full loading power until full or paused | Off | Until the full sensor is HIGH continuously for 0.2 s |
| Full confirmation / intake stop | Closed | Continue forward | Off | 0.5 s run-on to seat the fourth ball; pause, feed or shutdown cancels it |
| Confirmed full | Closed | Off | Off | While HIGH remains confirmed; LOW resumes loading |
| Shot readiness | Closed | Paused by the existing shooting controls | Off | Readiness stable for 0.15 s for timed shots |
| Gate command delay | Held open | Off | Off | 0.20 s after the open command |
| Timed shot feed | Held open | Full forward-feed power | Full forward-feed power | 0.50 s |
| Recovery | Closed | Off | Off | 0.35 s before readiness requalifies |

Each fresh confirmation starts the 0.5 s stage-1 run-on once while stage 1 is loading. LOW immediately
clears full, resets the confirmation/run-on timers and resumes loading. Another HIGH needs a
fresh continuous 0.2 seconds. Bench, competition and autonomous intake stop (never
reverse) at confirmation and resume forward on LOW, with RB/LB overriding the sensor.
This replaces the previous full-queue intake reversal; the stage-1 run-on is unchanged.

An uninterrupted timed attempt commands the gate open for about **0.70 seconds**:
0.20 seconds after the open command plus 0.50 seconds of feeding. Both transfer stages
run together for the entire feed window. Continued requests repeat readiness,
gate opening, feed and recovery. Releasing the shot request, feed inhibit, loss of
required readiness, or STOP may end a cycle early. After shooting, ordinary
stage-1 loading resumes according to the OpMode's existing controls; stage 2
remains off.

Both final Red/Blue TeleOps start CONTINUOUS: once a feed is authorized, the gate-open command and both full-power transfer commands occur together, without a gate delay. Automatic readiness and GP2 LT inhibit still apply; manual feed bypasses readiness. STAGGERED opens the gate, starts both stages together at 0.10 s, and closes the gate after a 0.30 s stage-2 feed (0.40 s total open window). Both transfers keep running while the gate is closed through the 0.10 s recovery and any subsequent readiness wait. At the next opening both transfers stop, then repeat the shared 0.10 s delay. Release, inhibit, invalid automatic solution, tracking loss or STOP cancels transfer feeding; tracking-loss recovery does not continue closed-gate feed. Mode changes reset the sequence. Autonomous and diagnostic defaults are unchanged. The gate-only test stays open while its trigger is held and does not initialize transfer motors.

In both final competition TeleOps, RB on either gamepad also forces intake and
stage-1 loading forward past the full sensor. LB reverses only the intake and
pauses stage-1 loading; opposing bumpers stop collection. Release restores
automatic loading. These collection overrides leave gate commands and stage-2
behavior unchanged; shooting requests, gate/recovery pauses and GP2 LT retain their
existing transfer priority. Autonomous and bench control mappings are unchanged.

## Shared settings and affected OpModes

- `GateSubsystem.FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS` owns the diagnostic/autonomous command-to-feed delay;
  `ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS` uses that same value.
- `ShootingConfiguration.FEED_WINDOW_SECONDS` owns the diagnostic/autonomous feed duration.
  Recovery and readiness remain 0.35 s and 0.15 s.
- Both alliance competition TeleOps use `CompetitionFeedController` for mode
  selection and simultaneous stage starts, with competition-only 0.10 s gate and 0.30 s feed settings.
  Both match-autonomous adapters and guided shot calibration retain their existing
  shared settings and policies. The combined intake/index/shooter
  test uses the bench-only overrides described above.
  Calibration requests one attempt; the other timed shooting controls repeat
  while requested. Autonomous scoring remains bounded by the route's scoring
  window and match deadline; check those windows when tuning feed duration.
- `Test: Transfer Only` automatically loads with stage 1 while stage 2 stays off. Hold RB
  to request repeated timed gate-open feeds of both stages; LB pauses loading and
  inhibits feeding with priority. Releasing RB closes the gate, stops stage 2 and
  returns to ordinary stage-1 loading. No shooter/flywheel is initialized or
  powered, and shooter readiness is bypassed for this deliberate mechanism test.
  Start empty when checking directions and gate timing.
- Pollen collection keeps its gate closed and stage 2 off for the whole run.
  Intake-only, turret-only and auto-aim tests do not initialize the transfer.

The team reports immediate gate settling. Software applies the requested fixed
0.20-second command-to-feed delay with no additional settling allowance.
The 0.50-second feed duration remains a starting value for loaded robot testing;
gate arrival has no position feedback.
`FEED_TO_EXIT_SECONDS` remains the separate, unmeasured 0.12-second exit-delay
estimate; it does not stop the motors. `RELEASE_TIMING_MEASURED` remains false.
Confirm ball clearance, exit timing and flywheel recovery on the robot before
declaring release timing measured. Timed attempts do not count launched balls.

The gate servo is wired directly to Expansion Hub servo port 3, bypassing the 6 V power injector. It receives immediate full-position targets without software ramping for maximum available speed. The 0.043 s/60-degree rating at 6 V is catalog reference only; actual direct-hub loaded speed is unmeasured.

## Desktop verification

All 153 unit tests pass with zero failures, errors or skips. Debug packaging and
lint pass; lint reports 0 errors and the existing 14 warnings (12 native-library
alignment warnings and 2 unused resources). No deployment or loaded timing
measurement was performed.

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:lintDebug
```
