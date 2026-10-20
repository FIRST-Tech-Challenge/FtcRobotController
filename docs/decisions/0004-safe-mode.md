# 0004: Safe Mode limits the drive to 25% unless a switch on the robot says otherwise

Status: Accepted. The switch (how Safe Mode is turned on) is **Replaced by 0007**, a
magnetic key checked once at INIT. The speed limit below still stands.
Date: 2026-09-27

## Context

Most driving this season happens on a bench, with the robot on blocks or on a table, and
kids standing close. A robot that lurches at full power off a table is how people and
robots get hurt. The coach wants the drive limited to a quarter of full power whenever
the robot is being worked on, in TeleOp and in Auto, without anyone having to remember a
setting at the Driver Station.

## Decision

- A **toggle switch on the robot**, wired to a hub digital port, picks the speed. It is
  read during INIT and locked at START. Flipping it mid-match does nothing.
- **Open switch = SAFE (25%). Closed switch = FULL.** The hub pulls digital inputs high
  inside the hub, so an open switch, an unplugged cable and a broken wire all read the
  same as "open". Every way the switch can fail lands on SAFE.
- **Anything missing means SAFE**, with a warning on the Driver Station: no `safeMode`
  device in the robot configuration, or an OpMode that never reads the switch.
  `HydraOpMode.mSafeMode` starts as true.
- The limit lives in one place, `Drive.LimitPower`. `Drive.SetMotorPower` and
  `Drive.SetAllMotorPower` run every drive power through it. Future Auto code must drive
  the motors through those two methods, not by calling `setPower` directly.
- In Safe Mode the power is **scaled** (times 0.25) and then **clamped** to ±0.25. Scaling
  keeps the stick smooth from zero to full; clamping catches field-centric's 1.41 and
  anything else out of range. The cap is `Constants.safeModeMaxPower`.
- `OpMode_Base.CreateHydraOpMode()` reads the switch, so every OpMode built the team way
  gets it for free. `Test: Drive Motors` always runs at the Safe Mode speed.

**Wiring:** a single-pole toggle between a digital port's signal pin and ground. In the
robot configuration on the Driver Station, add a **Digital Device** named `safeMode` on
that port. Label the switch positions SAFE and FULL on the robot.

## Consequences

- If the switch isn't flipped to FULL before a match, the robot plays at quarter speed.
  The Driver Station shows `SAFE MODE` during INIT and all through the match; check it
  before pressing START. This is the trade the coach chose: a slow match is recoverable,
  an injury is not.
- Until the switch is wired, every drive runs at 25%. That includes the first floor test
  from 0002. To go full speed before the switch exists, add a jumper from signal to
  ground on the port and configure it as `safeMode`.
- Only the drive is limited. Future mechanisms (intake, launcher, lift) are not. If they
  need a bench limit, extend this decision rather than inventing a second switch.
- Tests: `DriveLimitPowerTest` and `DriveLimitPowerProperties` cover the math;
  `SafeModeSwitchTest` covers every switch state; `DriveSafeModeTest` drives the real
  `Drive_Manual` with a slammed stick, boost, and a 45° heading and checks no motor passes
  0.25. Removing the cap from `Drive` makes `DriveSafeModeTest` fail.
