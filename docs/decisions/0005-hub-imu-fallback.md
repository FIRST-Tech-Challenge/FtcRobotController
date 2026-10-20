# 0005: Use the Control Hub IMU when there is no Pinpoint

Status: Accepted. The hub mounting is copied from Decode and not yet checked on this robot.
Date: 2026-09-27

## Context

The drive port (0002) took its heading from a goBILDA Pinpoint. The test robot has no
Pinpoint right now, but the Control Hub has an IMU built in, which every Control Hub
configuration lists as `imu`. PORT-PLAN had marked Decode's `Imu_Hub` to drop because
Decode never used it. It comes back here because it is the only heading source this
robot has.

## Decision

- `ImuSelector.Pick` checks the robot configuration when the OpMode starts. If a device
  named `pinpoint` is there, use the Pinpoint. If not, use the hub IMU named `imu`. The
  Driver Station shows which one it picked ("Heading from: ...").
- `Imu_Hub` implements the same `Imu` interface as `Imu_Pinpoint`, so the drive code
  doesn't know or care which one it has.
- The hub's mounting is in `Constants.hubLogoFacing` / `hubUsbFacing`, copied from
  Decode (logo LEFT, USB UP). Decode had it buried in the class.
- If the hub IMU fails to start, INIT still finishes and the drive works robot-centric
  (heading stays 0), with a fault message. A missing heading shouldn't stop the robot
  from driving on the bench.

## Consequences

- **Check the mounting on this robot first.** Run HyDrive, stay in INIT, and turn the
  robot left by hand: `Yaw` should go up. If it goes down or barely moves, the logo/USB
  directions in `Constants` are wrong for how this hub is mounted.
- Adding a Pinpoint later needs no code change: add it to the configuration as
  `pinpoint` and the selector will use it. The Pinpoint's odometry numbers in `Constants`
  came from Decode's robot and will need re-tuning for a new robot when position
  tracking arrives (P5).
- The hub IMU drifts more than the Pinpoint over a match. For field-centric TeleOp that
  means pressing circle to re-set "forward" now and then.
- The hub IMU isn't part of the hub's bulk read, so every read costs time. It's read
  once per loop in `Process()`.
- PORT-PLAN still lists `Imu_Hub` as dropped. This decision replaces that line.
