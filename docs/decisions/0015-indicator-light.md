# 0015: An indicator light that shows how many tags the camera sees

Status: Accepted. Not yet tried on the robot.
Date: 2026-10-17

## Context

The team has a goBILDA RGB indicator light. The coach asked for green when the camera
sees an AprilTag and red when it does not, so the drivers know when aim assist will work
without reading the Driver Station.

## Decision

- **`subsystems/Indicator`**, a Subsystem that asks the `Vision` each loop and sets the
  light: `indicatorRed` (0.28) with no tag; **blinking** `indicatorBlue` (0.611) and off every
  `indicatorBlinkMs` (300) with one tag; solid blue with two; `indicatorGreen` (0.5) with
  three or more. Off before START. The color is sent only when it changes.
- **The light is a Servo.** It plugs into any servo port on the Control Hub and is
  configured as a "Servo" named `cfgIndicator` (`led`, Decode's name). Each color is a
  servo position from goBILDA's table; the table is in `BotProperties` next to the values.
- **No light, no crash.** `tryGet`, like the Safe Mode key: a robot without the light says
  so once at INIT and carries on.
- **`indicatorDisabled`** in BotProperties leaves it out of an OpMode.

## Consequences

- With `visionDisabled`, or no camera, the light is always red. That is the truth: no
  tag is seen.
- Decode's blink patterns (launcher at speed, turret locked, lift) are not ported. Any of
  them is a few lines in `Process()` once those parts exist.
- Tests: `IndicatorTest` (8) and two in `BotTest`.
