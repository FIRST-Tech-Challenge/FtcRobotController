# 0008: Autonomous uses Pedro Pathing 3, with odometry pods on the hub

Status: Accepted. Nothing has driven on the robot yet: Pedro must be tuned first.
Date: 2026-09-27

## Context

The drive is ported (0002). Autonomous is next. Decode used Road Runner 1.0; PORT-PLAN
left the choice open as D1 ("Road Runner 1.0 or PedroPathing 3"). The coach chose Pedro
and asked for practice autos the students can learn from: a C that turns, a C that doesn't
(mecanum), and a Z. No obstacles, no game pieces.

The robot has no Pinpoint. Its two odometry pods are still mounted, so they plug straight
into Control Hub encoder jacks, and heading comes from the hub IMU.

## Decision

- **Pedro Pathing 3** (`com.pedropathing:revhub:3.0.1`, `tuning:1.0.1`, from
  `repo.dairy.foundation`). Pedro's own Quickstart uses our exact SDK 12.0, AGP 8.13.2 and
  Gradle 9.1, so no SDK files change.
- **Localizer: Pedro's Two Wheel** (two pods + `RevHubIMU`). Each pod is named after the
  motor port whose encoder jack it's plugged into: `Constants.cfgForwardPodPort` and
  `cfgStrafePodPort`. Those two names are guesses until someone checks the robot.
- **Tuning with AutoTune** (the web page at http://192.168.43.1:10158). Order: Mecanum
  Tuner, Two Wheel Tuner, Foresight Tuner. Paste each result into
  `pedro/PedroConstants.java`, then set `foresightTuned = true`. The Foresight numbers
  (11 of them) can only come from the real robot; none are made up.
- **Autos refuse to move until tuned.** Without Foresight numbers, Pedro doesn't know
  how hard to push or when to brake. The autos show a message instead.
- **Safe Mode applies to Pedro too.** `SafeModeMecanum` wraps Pedro's drivetrain and
  shrinks every command so no wheel goes past 0.25. It shrinks all three parts (forward,
  strafe, turn) by the same amount, so the robot keeps its direction, just slower.
  - **The tuners ignore Safe Mode on purpose.** They measure the robot's real top speed.
    The Foresight Tuner drives at full power for about 4 feet. Floor only, clear space.
- **Every drive motor runs `RUN_WITHOUT_ENCODER`.** Two of the motor ports carry pod
  encoders, not their own motor's encoder. A motor holding its speed with the wrong wheel's
  encoder would fight Pedro.
- **Shapes live in `pedro/ShapePaths.java`, not in the OpModes.** No hardware there, so
  tests check each shape's corners and headings on a laptop. Each auto is 15 lines: a name
  and which shape.
- **The team's code builds Pedro's settings one line at a time** (`c.frontLeftName.set(...)`)
  instead of Pedro's `c -> { ... }` style, per 0006. The lines still match what AutoTune
  prints, so pasting works.
- **Pedro's tuner procedures are copied unchanged** from Pedro's Quickstart
  (`pedro/procedures/`, BSD 3-Clause Clear). They are Pedro's code, not the team's: the
  readability rule (0006) doesn't apply there.

## The shapes

Pedro measures in inches from where the robot starts: x forward, y left, heading in
radians with left turns positive. Each shape is 48 inches (two tiles).

| Auto | Path | Heading |
|---|---|---|
| Shape: C (turning) | One Bezier curve: forward, around to the left, back toward the driver. Ends 48 in left. | Follows the curve (0° to 180°), like a car |
| Shape: C (no turning) | The same curve | Always 0°: forward, slide left, back up |
| Shape: Z | Three lines: slide left 48, diagonal forward-right, slide left 48 | Always 0° |

## Consequences

- Before any auto runs: 1) check the pod ports, 2) run the three tuners, 3) paste results,
  4) set `foresightTuned = true`. The Driver Station says so if you forget.
- The Foresight Tuner and Pedro's Tests run at full power. Only the autos respect Safe Mode.
- With Safe Mode on, Pedro follows paths at a quarter speed. Its Foresight settings were
  measured at full speed, so it may brake early and creep to the end. That's expected.
- `SafeModeMecanum` overrides Pedro's `applyDrive`. If a Pedro update renames it, the
  build breaks (good: loud, not silent) and `SafeModeMecanumTest` checks the cap still works.
- When a Pinpoint shows up: switch `PedroConstants.CreateFollower` to Pedro's
  `PinpointLocalizer` and run the Pinpoint Tuner instead of Two Wheel.
- Tests: `ShapePathsTest` (shape corners, headings, lines connect); `SafeModeMecanumTest`
  (no wheel past 0.25 across 7 commands, direction kept, full speed untouched, motors
  without encoder). Removing the Safe Mode cap turns 7 of them red.
