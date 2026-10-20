# 0013: Pedro uses the Pinpoint when the robot has one

Status: Accepted. Not yet tuned. Replaces the localizer part of 0008.
Date: 2026-10-16

## Context

Decision 0008 set Pedro up with its Two Wheel localizer: two odometry pods plugged into
the hub's encoder jacks, heading from the hub IMU, because the test robot had no
Pinpoint. The robot now has a goBILDA Pinpoint, with both pods plugged into it. TeleOp
already handles that: `ImuSelector` picks `Imu_Pinpoint` when the configuration has a
`pinpoint`. Pedro did not.

## Decision

- **`PedroConstants.CreateLocalizer`** picks the localizer the way `ImuSelector` picks the
  heading: a `pinpoint` in the robot configuration means Pedro's `PinpointLocalizer`,
  otherwise the Two Wheel localizer from 0008. The autos and the tuners both go through it.
- **One set of Pinpoint numbers.** `PinpointLocalizerConfig` builds Pedro's settings from
  the same `BotProperties` values `Imu_Pinpoint` uses (`pinpointInPerTick`,
  `pinpointParYTicks`, `pinpointPerpXTicks`): offsets in inches, ticks per inch, both pods
  `FORWARD`. `Imu_Pinpoint.Configure` is now a static method so `Test: Pinpoint` sets the
  device up exactly as the drive does.
- **Pedro's Pinpoint Tuner** is copied unchanged from the Quickstart into
  `pedro/procedures/` and listed in `Tuning.java` next to the Two Wheel Tuner. Run one or
  the other, not both.
- **`Test: Pinpoint`** shows status, version, raw X and Y ticks, inches, and heading, with
  circle to zero. It replaces `Test: Odometry Pods` on a Pinpoint robot; that test stays
  for a robot without one.

## Consequences

- The tuning guide's step 3 and step 5b each have a Pinpoint path and a pods-on-the-hub
  path. The robot's configuration decides which one applies; nothing in code is edited.
- After the Pinpoint Tuner, its pasted offsets live in `PinpointLocalizerConfig` while
  `Imu_Pinpoint` keeps using the `BotProperties` numbers. Keep them in agreement: when the
  tuner's offsets differ from Decode's by more than half an inch, update
  `pinpointParYTicks` and `pinpointPerpXTicks` too (offset in inches divided by
  `pinpointInPerTick`).
- Tests: `PedroConstantsTest` (3): offsets and ticks per inch match the drive's numbers;
  a configured `pinpoint` selects the Pinpoint localizer.
