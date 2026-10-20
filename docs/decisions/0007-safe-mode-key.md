# 0007: The Safe Mode input is a magnetic key, checked once at INIT

Status: Accepted. Replaces the switch part of 0004. The magnetic key isn't built yet; a
touch sensor stands in for bench testing.
Date: 2026-09-27

## Context

0004 used a toggle switch read during INIT. The coach settled on something harder to get
wrong: a REV Magnetic Limit Switch on the robot and a 3D printed key with a magnet. It
works like a "Remove Before Flight" pin:

- **Key in = Safe Mode.** The robot is safe to work on.
- **Key removed = full speed.** The robot is armed for a match.

Two more things the coach decided:

- The key only goes in or comes out **before INIT**. Nobody touches it during a match.
- A sensor blip mid-match must never change the robot's speed. The **power switch** is
  the real kill switch, not Safe Mode.

For testing now, the robot has a touch sensor named `liftHomeSwitch`. Pressing it
pretends the key is removed (full speed). Not pressed = key in (Safe Mode).

## Decision

- `SafeModeKey` reads the sensor **once**, when the OpMode is set up at INIT, and
  never again. It keeps the sensor as a local variable, so later code can't read it by
  accident. A test checks the sensor is read exactly once, even over 100 loops.
- The sensor is configured as a **REV Touch Sensor**, the way REV recommends for the
  magnetic switch. Both sensors then read the same way in code: `isPressed()`.
- `Constants.cfgSafeModeKey` names the sensor. `Constants.keyPressedMeansKeyIn` says what
  "pressed" means:

| Hardware | `cfgSafeModeKey` | `keyPressedMeansKeyIn` | Pressed means |
|---|---|---|---|
| Touch sensor (now, for testing) | `liftHomeSwitch` | `false` | key REMOVED (full speed) |
| Magnetic switch + key (later) | the magnetic switch's name | `true` | key IN (Safe Mode) |

- Missing from the configuration means SAFE, with a warning. Everything else in 0004
  stands: 25% limit, scale then clamp, applied in `Drive.LimitPower`.

## Consequences

- **The touch stand-in fails safe.** An unplugged touch sensor reads "not pressed",
  which is key in, which is Safe Mode.
- **The magnetic key does NOT fail safe when unplugged.** The hub can't tell "no magnet"
  from "no cable": both read "not pressed", which means key removed, which is full speed.
  That is the price of "Remove Before Flight" with one sensor. The Driver Station shows a
  reminder line during INIT. Check the cable, and check that it says `SAFE MODE` with the
  key in, before bench work.
- A lost key also means full speed. That's intended: no key = armed.
- To change the speed, you must stop the OpMode and press INIT again. Changing the key
  after INIT shows no change on the Driver Station, on purpose.
- Before a match: remove the key, then INIT, then check the Driver Station says
  `FULL SPEED`. After the match: key back in.
- `liftHomeSwitch` belongs to the lift. When the lift is ported, the key needs its own
  sensor and name.
