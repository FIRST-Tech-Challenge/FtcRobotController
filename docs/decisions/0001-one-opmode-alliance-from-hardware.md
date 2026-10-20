# 0001: One OpMode; alliance comes from a badge switch

Status: Accepted. The switch is not built yet.
Date: 2026-09-26

## Context

Decode had a red and a blue copy of almost every OpMode: `TeleRed`, `TeleBlue`, and red
and blue versions of every auto (17 leaf autos). The only difference was a
`VisionMode_RedGoal` or `VisionMode_BlueGoal` passed to the constructor. That value then
spread into the IMU, turret, vision, indicator, and driver heading.

Problems this caused:

- **The driver has to pick the right one.** Pick TeleBlue on the red alliance and aiming
  and field-centric heading are wrong for the whole match.
- **Half the OpMode list is copies.** Every fix has to land twice.
- **Alliance leaks everywhere.** `Imu_Pinpoint`, `Turret`, and `HyDrive` each had their own
  red/blue `switch`.

## Decision

1. **One TeleOp**, `HyDrive`. Autos become Near or Far times strategy, never times color.
2. **The robot reads its alliance** from two limit switches pressed by a nub on the alliance
   badge: left pressed means blue, right pressed means red (PORT-PLAN P18).
3. **Read once during INIT**, stored in `HydraOpMode.mAlliance` (type `types/Alliance`).
   Every subsystem reads it from there. No subsystem takes a color in its constructor.
4. **Neither or both pressed** means `Alliance.UNKNOWN`: warn on the Driver Station and
   refuse to START an auto. TeleOp may still drive.

## Consequences

- `HydraOpMode.mAlliance` exists now and is always `UNKNOWN`. Nothing reads it yet. The
  drive does not need it: field-centric forward is set by where the robot points at INIT.
- **Anything alliance-specific must read `mAlliance`**, never a constructor argument or the
  OpMode name. An automated check for this is planned (0003).
- Bench-test the switch wiring before any auto depends on it.
- Open question: when auto hands its final pose to TeleOp, the alliance decides the
  driver's forward. Decode did this with `OpmodeHeading` plus a per-color offset. That
  comes back with the auto port (P8), reading `mAlliance`.
