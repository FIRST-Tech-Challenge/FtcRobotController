# Decisions

Why the code is shaped the way it is. One file per decision, numbered in order.

Read these before changing anything they cover. If a decision changes, write a new file
that replaces the old one and mark the old one **Replaced by NNNN**. Do not rewrite history.

| # | Decision | Status |
|---|---|---|
| [0001](0001-one-opmode-alliance-from-hardware.md) | One OpMode; alliance comes from a badge switch | Accepted, switch not built yet |
| [0002](0002-drive-port.md) | How the mecanum drive came over from Decode | Accepted, bench test pending |
| [0003](0003-quality-gates.md) | Tests and checks from day one, aimed at the coach | Accepted, partly built |
| [0004](0004-safe-mode.md) | Safe Mode: drive limited to 25% | Accepted; switch replaced by 0007 |
| [0005](0005-hub-imu-fallback.md) | Use the Control Hub IMU when there is no Pinpoint | Accepted, mounting not checked yet |
| [0006](0006-code-for-students.md) | Write the code for students to read | Accepted |
| [0007](0007-safe-mode-key.md) | Safe Mode input: magnetic key, checked once at INIT | Accepted; touch sensor stands in |
| [0008](0008-pedro-pathing-autos.md) | Autonomous uses Pedro Pathing 3, odometry pods on the hub | Accepted, not tuned yet |
| [0009](0009-launcher-velocity-control.md) | The launcher holds a speed with our own control loop | Accepted, not tuned yet |
| [0010](0010-bot-and-bot-properties.md) | OpModes talk to a Bot; settings live in BotProperties | Accepted, not on the robot yet |
| [0011](0011-vision-aim-assist.md) | Aim assist: hold the left bumper, face the nearest AprilTag | Accepted, turn direction unchecked |
| [0012](0012-ftc-dashboard.md) | FTC Dashboard mirrors the telemetry as live graphs | Accepted, not on the robot yet |
| [0013](0013-pinpoint-for-pedro.md) | Pedro uses the Pinpoint when the robot has one | Accepted, not tuned yet |
| [0014](0014-safe-mode-off-for-foresight.md) | Safe Mode switched off while Foresight is tested | Accepted, temporary |
| [0015](0015-indicator-light.md) | Indicator light: red, blinking blue, blue, green for 0, 1, 2, 3+ tags | Accepted, not on the robot yet |
| [0016](0016-paths-live-in-the-opmode.md) | An auto's paths live in the auto's own file | Accepted |
| [0017](0017-led-strip-prism.md) | The LED strip is a goBILDA Prism driver over I2C | Accepted, two CHECKs on the strip |

## Template

```markdown
# NNNN: Title

Status: Proposed | Accepted | Replaced by NNNN
Date: YYYY-MM-DD

## Context
What problem, and what was true when we decided.

## Decision
What we chose.

## Consequences
What this makes easier, what it makes harder, what to watch for.
```
