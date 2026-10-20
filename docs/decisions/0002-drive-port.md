# 0002: How the mecanum drive came over from Decode

Status: Accepted. Bench and floor test on last year's robot still to do.
Date: 2026-09-26
Source: Decode commit 84561ae

## Context

This season's game plays much like Decode's and the robot is mecanum again. Last year's
robot is available for testing. The goal: a drive that feels exactly like Decode's, built
on the framework pieces the rest of the robot will use.

## Decision

**Ported unchanged**

- The wheel math. Moved out of `Drive_Manual.Process()` into `objects/MecanumMath` so it can
  be tested without a robot. Checked against a verbatim copy of Decode's math on 400,000
  random inputs: identical to the last bit.
- `Debouncer`, `MotorSlewRate`, motor directions (left FORWARD, right REVERSE), BRAKE,
  RUN_WITHOUT_ENCODER, the slow/normal/boost triggers, circle-to-reset-heading with a rumble,
  the 1.1 strafe gain, slew rate 0.2.
- The loop from `HyDrive`: clear hub caches, all `HandleUserInput()`, all `Process()`, one
  `telemetry.update()`.
- Hardware names: `leftFront`, `rightFront`, `leftBack`, `rightBack`, `pinpoint`. The robot
  configuration from last year should load unchanged.
- Pinpoint calibration wait, version check, and bulk-read setup from `Imu_Pinpoint.InitPinpoint`.

**Changed**

- **Pinpoint driver:** uses the one built into SDK 12 instead of Decode's 1,088-line copy
  (PORT-PLAN P4). Same config tag (`goBILDAPinpoint`), so the saved configuration still works.
- **Gamepad is read in `HandleUserInput()`**, not `Process()`. Same loop, so no added delay.
  Autos call only `Process()`, so the drive sits still in auto instead of reading sticks.
- **Forward is where the robot points at INIT.** Decode set it from the alliance and the
  pose auto left behind. See 0001.
- `OpMode_Base` no longer takes a `VisionMode`. `HydraOpMode` has no `mVision`, which breaks
  the P1/P9 cycle.
- Magic numbers moved into `Constants`: strafe gain, hardware names, Pinpoint values.
- Wheel powers show on telemetry (they were commented out in Decode).

**Left out, for now**

| What | Why | Comes back with |
|---|---|---|
| Left bumper: aim at the AprilTag | Needs vision. Also dead in Decode: the square-button `else` at Drive_Manual:66-73 overwrote it every loop | P6 AimAssist, after P9 vision |
| Square: snap heading | Snap angle came from the alliance and Decode field geometry | P5 season FieldGeometry |
| `OpmodeHeading` pose hand-off from auto | No auto yet | P8 |
| Road Runner, x/y position | Driving needs heading only | P4, P5, decision D1 |
| FTC Dashboard telemetry | Not a dependency in BioBuzz yet | When tuning needs it |
| `if (false)` PID block, encoder ramp fields | Dead code; nothing read them | Never |

## Consequences

**Known issue kept on purpose:** `MecanumMath.Mix` scales by the stick values *before* the
field-centric rotation. Field-centric at a diagonal heading can ask a wheel for up to about
1.41, and the hub clips it to 1. At full stick, the robot's travel direction bends slightly
at some headings. Kept so the driver feels exactly what they had at Worlds. **Coach decision:**
fix it (scale by the rotated values) once drivers have tried the robot. Tests will pin the
current behavior first, so the fix is a deliberate, visible change.

**Bench test checklist, in order:**

0. Safe Mode: with the touch stand-in, INIT without pressing `liftHomeSwitch` = 25%
   (key in); hold it pressed while pressing INIT = full speed (key removed).
   See 0007-safe-mode-key.md.
1. Wheels off the ground, hand on STOP. Run **Test: Drive Motors**. Each D-pad direction
   spins one wheel forward. Fix any backward wheel in the wiring or config, not in code.
2. Still on blocks, run **HyDrive**. It says which heading source it picked (Pinpoint or
   Control Hub IMU). `Heading ready` should be true within a second. Turn the robot left
   by hand: `Yaw` should go up (see 0005 if not).
3. On the floor: drive forward, strafe, turn. Then turn the robot 90 degrees and push
   forward. It should still go away from the driver.
4. Press circle. The controller buzzes and the current facing becomes forward.
