# Local robot simulation and review

SimLoop is optional for major behavior changes. Minor edits and routine tuning do not require it.
Local checks come first; CI remains deferred. The pinned version is `v0.2.0-alpha.2` (FTC SDK 12.0.0).
Read the [audit](../doc/SimLoopIntegrationAudit.md), [coverage matrix](../doc/TeleOpVerificationMatrix.md)
and [model assumptions](../doc/SimLoopRobotModel.md) before interpreting a pass as evidence.

The drivetrain fixture includes the reported goBILDA 5000 bare motors, 20:80 gears
and coaxial 30:91 belt reduction (12.133333 motor rev/wheel rev). It derives motor
ticks/in from those stages and the reported 104-mm goBILDA 30A mecanum wheels
(52-mm nominal radius, 26.411232 motor ticks/in). Drivetrain scenarios
check the actual isolated test, Red/Blue wrappers and Pedro diagnostic's manual
drive/STOP conversion; loaded compression/slip and autonomous path accuracy still
need physical validation. See [hardware and units](../doc/Drivetrain.md).

The November 5 direction regression checks all six stick directions in those four
actual OpModes. Physical mounting signs +,-,+,- replace the previous assumption
based on the team's all-directions-reversed report; they remain independent of
production motor directions. This is software regression evidence under that
reported orientation, not a fresh physical measurement.

## Automated checks (humans and agents)

Run from the repository root with the usual FTC JDK/Android SDK setup:

```powershell
.\tools\verify-teleop.ps1                                  # unit tests, APK, lint; no simulation
.\tools\verify-teleop.ps1 -Sim -Scenario Mechanisms         # selected simulations plus ordinary checks
.\tools\verify-teleop.ps1 -Sim -Scenario Sensors,Competition
.\tools\verify-teleop.ps1 -FullSim                         # all available automated scenarios
.\gradlew.bat :TeamCode:simTeleOpTest --tests '*GateSimTest' # quick targeted iteration
```

Selectors: `Drivetrain`, `Gate`, `Mechanisms`, `Sensors`, `Competition`. The script checks fresh JUnit
XML, failures/errors/skips, package separation and named tests in `tools/teleop-test-baseline.json`.
Update the simulation manifest when adding/renaming tests; preserve the original unit-test baseline.
Ordinary debug/release tests exclude the simulation package. The automated sim task excludes the live
reviewer. All simulation runs are serial. **Close the reviewer before running another Gradle build**:
Windows locks jars while the reviewer JVM is using them. Do not run two verification scripts concurrently.

Actual production OpModes run through `OpModeRunner.forTeamClass`, including INIT/START/ACTIVE/STOP.
Team code, SolversLib and Pedro use the simulated clock. SDK/fakes and primitive snapshots cross the
class-loader boundary. Test-only `SyntheticRedTeleOp`/`SyntheticBlueTeleOp` inherit competition coordination
and inject artificial shot rows explicitly; their results do not certify production automatic firing.

## Human keyboard review

```powershell
.\tools\review-teleop.ps1 -OpMode RedMecanumTeleOp -Seconds 120
.\tools\review-teleop.ps1 -OpMode BlueMecanumTeleOp -Profile raised-audience
.\tools\review-teleop.ps1 -OpMode TransferTestOpMode
.\tools\review-teleop.ps1 -OpMode TurretTestOpMode
```

Focus the **BioBuzz** window. INIT runs for 3.2 simulated seconds, then START; duration is 1–300 active
seconds. Its status line shows phase/tick, position, yaw, flywheel feedback, gate, intake and transfer.
Close the window or hold Escape to stop. Very brief taps between 20-ms samples may be missed, as with
the upstream held-key interface. This is keyboard input, not physical USB gamepad support.

| Keyboard | Simulated input |
| --- | --- |
| W/S, A/D | Left stick forward/back, left/right |
| I/K, J/L | Right stick Y, X |
| Z / C | Left / right trigger |
| Q / E | Left / right bumper |
| 1 / 2 / 3 / 4 | A / B / X / Y |
| Arrows | D-pad |
| F / H | Left / right stick click |
| Tab | Switch between gamepads 1 and 2 |
| Shift | Half stick/trigger travel |
| U held | Queue digital sensor HIGH; release = LOW |
| O held | Stop camera polling; release restores polling |
| P held | Pinpoint NOT_READY fault; release clears fault |
| V held | Freeze physical turret yaw; encoder stays at the physical angle |

Robot controls remain those of the selected OpMode. For competition: GP1 1 (A) or GP2 C requests manual
feed; GP2 Z inhibits it; GP1 C prepares automatic aim/hood/RPM and C+Z requests
shooting. GP2 2 (B) toggles continuous/staggered. Production unmeasured calibration blocks
firing. Hold U for over 0.5 seconds to review full-queue behavior. Sensor keys are environment controls,
recorded separately from gamepad input, and have no effect on mechanisms absent from the fixture.

Competition GP2 Left Arrow requests alliance-corner rehome and a three-second
Limelight refinement; the turret points toward its hive. Keep the chassis still
until telemetry reports VISION REFINED or CORNER ONLY (with the 55-degree camera the corner cannot see the raised tags, so the Sensors corner scenarios assert CORNER ONLY). Five fresh compatible
frames after measured turret settling refine position with the corner prior.
GP2 yaw input or turret home cancels refinement; automatic feed waits while
pending. The built-in raised-audience profile has no red-far raised tags; use
the dedicated Sensors corner-rehome scenario for both intended corner landmarks.

Profiles:
- `no-tags` (default): no camera detections; manual drive/feed works without absolute field reference.
- `raised-audience`: raw upstream camera convention (tag +Z into the print). This matches the physical
  Limelight frame recorded 2026-11-05, which `LimelightObservation` converts to the robot's outward +Z.
  It exercises geometric localization; it is **not proof of physical pose accuracy**.
- `raised-audience-inverted`: TEST-ONLY inverted target normal; the robot must reject it with a facing diagnostic.
Both profiles report poses in the inverted sensor frame of the upside-down Limelight (Rz(pi) about the optical
axis), as the physical camera does; SimLoop's upright stickers already match how the robot sees the field's.

Raised profiles start at alliance X = +/-12.75 in, Y = -40 in (`RAISED_TAG_VIEW_Y_INCHES`; the 55-degree camera loses the tags beyond about 52 in), heading = pi, with audience cells raised.
They require a camera-equipped fixture. The robot is not handed this field pose: it must acquire a
reference from camera observations. Camera settings, visibility, latency and finite turret response
remain simulated. Neither profile invents measured shot rows or scores real balls.

The script's `-OpMode` choices list supported review entries, including diagnostics and shot calibration.
Webcam/Pollen and raw disconnected-servo programming are unsupported (see audit). Synthetic calibrated
competition fixtures are deliberately excluded from the reviewer OpMode list.

## Record, replay and review evidence

Each run prints a unique directory under `TeamCode/build/sim/teleop/` containing:
- `scenario.txt`, `camera-convention.txt`: configuration and explicit convention limitations.
- `gamepads.track`: upstream `GamepadInputTrack` format, both gamepads, one active frame per tick.
- `trace.tsv`: every lifecycle sample, motor commands/feedback, pose, mechanism states and full telemetry.
- `runner-outcome.txt`: runner status; **behavior assertions live in the JUnit report**, not this file.
- Human sessions additionally have `review-setup.txt` (profile, INIT, early STOP) and `environment.tsv`.

Preserve the **whole session directory** when reporting a bug. Replay checks the 20-ms tick, setup,
environment length, early stop and every decoded output row against the original:

```powershell
.\tools\review-teleop.ps1 -Replay 'C:\path\to\session\gamepads.track'
.\tools\review-teleop.ps1 -Smoke  # headless recording-path smoke test; not interactive review
```

A replay difference fails the task and retains both traces; differences are expected when deliberately
changing robot behavior. Review them before accepting a new baseline. Only review-created tracks have
the required sidecars; automated scenarios are rerun by their named test, not by this review command.
JUnit reports: `TeamCode/build/reports/tests/simTeleOpTest/index.html` and `simTeleOpReview/index.html`.
Open TSV in a spreadsheet/text editor to inspect commands and telemetry. There is no RLOG/AdvantageScope
export in this adapter; no viewer installation is required.

For human sign-off, record OpMode/profile, revision and local changes, controls exercised, expected versus
observed behavior, trace path, PASS/FAIL/NOT TESTED, reviewer/date, and remaining physical checks. A green
runner alone is not reviewer approval. Known robot sign-offs are preserved in the verification matrix.

## Extending coverage

1. Classify the behavior change; choose simulation for a major change when useful, or state why omitted.
2. Start from the actual affected test OpMode and its minimal hardware fixture. Add independent physical
   feedback, a reproducible input/environment script, and an observable failure assertion.
3. Exercise both competition alliances for shared changes. Keep bench and competition control policies distinct.
4. Preserve model provenance and production calibration. Add fault/stop behavior where relevant.
5. Run the selected test and ordinary checks, update the manifest/matrix/docs, and report evidence and limits.

## Dependencies and license

`simloop.gradle` adapts the [pinned starter](https://github.com/Horizon-36596/simloop/tree/v0.2.0-alpha.2/simloop-starter)
under [BSD-3-Clause](LICENSE), copyright 2026 Horizon (FTC 36596). No upstream robot example or agent policy
is copied. SimLoop, its desktop classifier and PsiKit dependencies are test-only, never APK dependencies.
Jupiter/Vintage 5.11.3, Platform launcher 1.11.3 and real JSON 20250517 preserve JUnit 4 discovery and JVM
parsing. `returnDefaultValues=true` supports SDK matrix construction; it does not simulate Android services,
USB, VisionPortal, storage or Lynx scheduling. Fixed simulated ticks do not benchmark Control Hub timing.

November 4 integration: selected competition scenarios cover aim-only collection, firing pauses, auto offsets and three independent PWM light commands; Mechanisms covers intake priorities/STOP and LightsTestOpMode channel/pulse/STOP behavior. See doc/TeleOpVerificationMatrix.md for the bounded verification results and model limits.
