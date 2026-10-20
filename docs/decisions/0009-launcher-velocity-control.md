# 0009: The launcher holds a speed with our own control loop

Status: Accepted. Not yet tried on the robot. The gains are Decode's starting numbers.
Date: 2026-10-03

## Context

The team added a launcher motor (`nectarLauncher`) and drove it with fixed powers from the
D-pad. A fixed power gives a different speed as the battery drains and after each ball
slows the wheel, so shots land in different places. The launcher needs to hold a **speed**.

Decode did this in `subsystems/Launcher.java` and `objects/LaunchMotor.java` with FTCLib's
`PIDFController`. Checked in FTCLib 2.1.1's source: its F term is `kF x target`, a
feed forward, so most of the power comes from the target alone. PORT-PLAN P11 says to swap
in `HydraPIDFController`, but that class's F is `kF x sign(error)` (a small friction push),
so Decode's numbers would give about 0.0002 power from F instead of about 0.6. Don't.

Two ways to hold a speed were considered:
- **A. Our own loop** (chosen): our code measures RPM and picks the power. Decode's numbers
  carry over, and students can see and test every step of the math.
- **B. The hub's built-in velocity control** (`RUN_USING_ENCODER` + `setVelocity`). Fewer
  lines, but the math is hidden in the hub and Decode's numbers don't carry over. A fine
  fallback.

## Decision

- **`objects/VelocityController`**: power = `kF x target + kP x error + kI x (error over
  time)`, kept between 0 and 1. No D: Decode's was 0. Time is passed in, so tests control it.
  - **Never negative.** A launcher only spins one way, like Decode.
  - **The integral starts over** when the target changes (like Decode) and when the
    trigger is pressed again after a stop.
- **`subsystems/Launcher`**: owns the motor. Every 50 ms (like Decode) it measures the RPM
  and sets the power. `RUN_WITHOUT_ENCODER` with `FLOAT`, like Decode, so the wheel coasts
  when off.
- **Controls** (`Test: Launcher`, gamepad 1), as the coach asked:
  - **Hold right trigger** to power the launcher. Let go: no power.
  - **Right / left bumper**: target speed up / down 50 RPM, once per press.
  - The target starts at **1000 RPM**. Telemetry shows target RPM, actual RPM, power, and
    at speed (within 50 RPM).
- **Matches Decode:** with `kI = 0`, our power equals Decode's (FTCLib plus Decode's
  0-floor, capped at 1) on 200,000 random target/speed pairs (0 mismatches).
- **One difference, on purpose:** FTCLib caps the *error sum* at +/-1, so Decode's I term
  could never add more than `kI` = 0.0001 power: it did almost nothing. Ours caps the
  *I term* at +/-1, so kI can actually help. That's why `launcherKI` starts at **0**.
- **Safe Mode does not limit the launcher.** Safe Mode limits the drive (0004). The
  launcher only runs while the trigger is held.

## Tuning on the robot

Wheel guard on, hands clear. In `types/Constants.java`:

0. **Check `launcherTicksPerRev`.** 28 is a bare motor shaft (Decode). A gearbox changes it.
   If the actual RPM looks wildly wrong, this is why.
1. **kF.** Set `launcherKP` to 0. For a few targets, hold the trigger and write down the
   power when the speed settles. kF = power / RPM. Average them.
2. **kP.** Start at 0.001. Raise it until the speed recovers quickly after a ball, without
   bouncing above and below the target.
3. **kI.** Leave at 0 unless the speed settles a little below target and stays there.
4. **At-speed band** (`launcherAtSpeedRpm`): set it from how much the actual RPM wobbles
   when steady.

## Consequences

- `Test: Launcher` is its own OpMode. `Test: Drive Motors` is back to checking drive wiring.
- Not yet in HyDrive. Add `Launcher` to HyDrive's `mSystems` once the test works.
- Later pieces (feed servo, distance-to-RPM table, turret, Auto) build on `Launcher`.
- Tests: `VelocityControllerTest` (10) checks each part and the limits. `LauncherTest`
  (9) uses a pretend motor and gamepad: starts at 1000, bumpers step 50, no trigger = no
  power, RPM math. Removing the trigger check fails 2 tests; allowing negative power fails 1.
