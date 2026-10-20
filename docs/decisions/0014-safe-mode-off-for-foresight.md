# 0014: Safe Mode switched off while Foresight is tested

Status: Accepted, temporary. Turn Safe Mode back on when the autos are proven.
Date: 2026-10-17

## Context

Pedro's Foresight numbers were measured at full speed (the tuners ignore Safe Mode on
purpose, 0008). Running the autos at a quarter of that speed makes Pedro brake early and
creep, so the shape autos cannot show whether the tuning is right. The coach asked for
Safe Mode off for now.

## Decision

- **`safeModeEnabled` in `BotProperties`**, a plain boolean, is the master switch. It is
  `false` today. `SafeModeKey` then sets full speed without reading the key and the Driver
  Station shows `SAFE MODE IS OFF ... FULL SPEED, key or no key` in every OpMode.
- The limit (`safeModeMaxPower`) and the key settings stay locked (`static final`). The
  switch only decides whether they are used.
- **Back on means one edit:** `safeModeEnabled = true`. Nothing else changed.

## Consequences

- Every drive is at full speed, in TeleOp and Auto, whatever the key says. Floor only,
  clear space, hand on STOP. Nobody near the robot.
- `Test: Drive Motors` still runs at the Safe Mode speed; it never read the key.
- Tests that check the key set `safeModeEnabled = true` themselves, so they keep
  guarding the key logic while the default is off. One test checks the off state.
