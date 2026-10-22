# 0017: The LED strip is a goBILDA Prism driver, talked to over I2C

Status: Accepted. Not yet tried on the strip; two protocol details are marked CHECK.
Date: 2026-10-20

## Context

The team has goBILDA's Prism RGB LED Driver (SKU 3118-2855-0001) and Prism LED strips.
The Prism takes commands over I2C or PWM; goBILDA says to use I2C on a REV hub. The coach
wants a mode to start the strip programming with, while the drive is disconnected. The
device is called `strip`, to keep it apart from `led` (the indicator) and the Limelight.

## Decision

- **`drivers/PrismLedDriver`** makes the Prism a device type the robot configuration
  lists, "goBILDA Prism LED Driver", the way the SDK's Pinpoint driver is written (two
  annotations and a constructor). It sets the address 0x38 and moves bytes, nothing more.
  Configure the port with that type, name `strip`. The Pinpoint keeps its own I2C port;
  the Prism goes on another one.
- **`subsystems/LightStrip`** owns the meaning of the bytes and the controls.
- **The Prism animates, we only send settings.** A message is a layer register (layer 0 is
  0x08) followed by a sub-register and its bytes: which animation, brightness, start and
  stop pixel, colors. `Solid(r, g, b)`, `Blink(...)` and `Off()` are the first three.
- **Demo controls on gamepad 1's D-pad** (up red, right green, down blue, left off), the
  one set of buttons nothing else uses. **Test: Light Strip** runs the strip with the
  drive and launcher off.
- **INIT reads back the device id and status register** and shows the raw bytes, so a
  dead strip can be told from a wrong address or a miswired cable in one look.
- **`stripDisabled`**, `cfgStrip`, `stripLedCount` (12 per 200 mm strip) and
  `stripBrightness` (50 of 100) live in `BotProperties`.

## CHECK on the strip

- **Animation numbers.** The guide's example proves Solid Color is 1. Blink (2), Pulse (3)
  and Rainbow (6) are numbered here in the order the guide lists animations; goBILDA's
  example code (github.com/goBILDA-Official/FtcRobotController-Add-Prism, branch
  Add-Prism) has the real list. It could not be fetched from the practice network.
- **Byte order of 4-byte values**, such as a blink period. `Bytes32` sends the lowest byte
  first. The status bytes shown at INIT settle it: `0C 3C 00 00` means 12 LEDs at 60 FPS
  lowest byte first; `00 00 3C 0C` means the opposite, and `Bytes32` flips.
- **Pinout.** The Prism's I2C connector is 4-pin JST PH like the hub's, but check the
  guide's pinout against REV's before plugging in.

## Consequences

- Only layer 0 is used. The Prism has ten layers and saved Artboards; those are the next
  steps once Solid and Off work.
- `Blink` is untested and may need the two CHECKs above before it does what it says.
- Tests: `PrismLedDriverTest` (3) and `LightStripTest` (6) pin the exact bytes sent; two in `BotTest`.
