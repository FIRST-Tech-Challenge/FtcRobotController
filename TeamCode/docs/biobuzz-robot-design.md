# BIOBUZZ Robot Design (2026-2027)

*The current robot concept and the decisions behind it. Update this file when a decision changes. Anything still open is a GitHub Issue labeled `needs-decision`, and this file points to it with a `[#nn]` marker.*

## Concept

- **Drivetrain:** reusing both mecanum chassis from last season. Pinpoint odometry, Pedro Pathing for auto.
- **Shooter:** **weighted flywheel** plus an **independently controlled hood wheel** (smaller, on the back/top side in a fixed hood).
- **Turret:** the shooter sits on a **rotating turret** aimed with **Limelight 3A + AprilTags**. Vision sets both turret angle and flywheel speed.
- **Game pieces:** the shooter is designed for **POLLEN only** for now (2.8 in balls).

## Hardware facts

- **Turret gearing:** the turret ring has **47 teeth** and the gear that rotates it has **16 teeth**. That is a **47/16 = 2.9375 : 1** reduction, so one turn of the 16T gear moves the turret about **122.6°** *(derived; 360 ÷ 2.9375)*. Any gearbox on the turret motor adds to this ratio. Use this ratio when converting encoder ticks to turret degrees.
- **Intake and index/transfer:** **one motor drives all of it**: the intake, the index, and the transfer. It is a goBILDA 5203, **probably the 1620 RPM, 3.71:1 version** `[#65]` (see the open decisions). Because it is one motor, the intake, index, and transfer always run together and can't be controlled separately.
- **Encoder counts on the intake motor:** the 28 counts per revolution in `CLAUDE.md` is at the *motor* shaft. At the output shaft of a 3.71:1 gearbox that becomes about 28 × 3.71 ≈ **104 counts per output revolution** *(derived)*. The intake doesn't need velocity control, so this only matters if we decide to use encoders on it.

## Why

- Hive tips are worth 20 points and drive two of the ranking points. Everything else is worth 1–5. See `biobuzz-game-essentials.md`.
- The raised cell's opening is **53.5–65.6 in high**, so the ball has to climb about 40 in from a shooter exit around 15–18 in.
- The shooting side flips after every tip. A turret lets us shoot from anywhere on the correct side without precise chassis aiming.

## Subsystem plan (proposal; follows `ARCHITECTURE.md`)

These are plain subsystem classes (in the `subsystems/` folder) used by iterative OpModes. This season has no FTCLib: no base class, no commands, and no scheduler.

| Class | Job |
|---|---|
| `ShooterSubsystem` | Flywheel + hood wheel velocity control; answers `isAtSpeed()` |
| `TurretSubsystem` | Turret angle control, soft limits, homing |
| `VisionSubsystem` | Limelight wrapper: `tx`, `ty`, visible tag IDs, alliance filtering |
| `IntakeSubsystem` | Collect balls and move them through the transfer / feeder chute to the shooter; max 4 held |
| `DriveSubsystem` | Mecanum drive for TeleOp (robot-centric first); Pedro Pathing drives in Auto |

The specific classes, states, and methods are decided one session at a time, proposal first.

## Control approach

- **Aim:** turn the turret until the target cluster's `tx` ≈ 0. Filter tags to our alliance, and target the **higher** cluster. See the geometry doc.
- **Distance:** `d = (50 − lensHeight) / tan(cameraTilt + ty)`.
- **Flywheel speed:** a **lookup table** of `ty` → flywheel RPM, measured at 4–6 spots and interpolated between them (the `ShotTable` class, measured with `ShotTableAlpha`). The hood wheel follows at `HOOD_WHEEL_RATIO` times the flywheel. We key the table on `ty` instead of distance: the camera is fixed on the turret, so `ty` already tracks distance and we skip the lens height and tilt. Re-measure if the camera moves. Don't use physics equations. Students can re-tune a table at an event.
- **Firing gate:** only feed a ball when the flywheel (and hood wheel) are within a velocity tolerance **and** the turret is on target.
- **Velocity control:** `DcMotorEx.setVelocity()` with tuned PIDF. Confirm the encoder counts per revolution first (28 or 112, see `CLAUDE.md`).
- **Hood wheel:** start at a **fixed ratio** (hood wheel RPM ÷ flywheel RPM, `HOOD_WHEEL_RATIO`) to the flywheel speed. **2.0 is the best ratio found in robot testing so far** (about no ball spin, because the hood wheel is half the flywheel diameter), and it is now `HOOD_WHEEL_RATIO` in `ShooterSubsystem`. Keep checking it with `ShooterRatioAlpha`, and tune it separately only if testing shows spin matters.
- **Target entry angle:** a ball coming down at ~30° arrives square-on to the tilted window and has the largest usable opening.

## Fallback mode (required)

Build this first, and keep it working all season:

- The turret locks straight ahead at a known angle.
- There are **2–3 preset shooting spots**, each with fixed flywheel and hood wheel speeds, selected by the driver.
- There is no vision dependency. If the Limelight fails or the vision code isn't ready, the robot can still tip the hive.

## Limelight 3A mounting

- It goes **on the turret**, facing the same way as the shooter, **as low as practical**.
- **Tilt it up about 40°** as a starting point. Recalculate once the lens height is known (see the geometry doc).
- It connects to the Control Hub's **USB 3.0 port** with one USB-C to USB-A cable (power and data). The Control Hub sees it as an Ethernet device. Plan the cable routing and turret rotation limits so that cable can't wrap or snag.

## Open decisions

- [x] **Vision source:** **Limelight 3A is final.** (A webcam with `VisionPortal` would have given SDK 12.0 clusters with the origin at the cell opening, but we are not using it, so we apply the opening offset ourselves.)
- [ ] **Intake motor ratio:** goBILDA 5203 at 1620 RPM (3.71:1) is the likely choice, not final. Confirm it before writing `Intake` constants. `[#65]`
- [x] **One motor for intake and transfer:** decided in #63. While the fire trigger is held, the gate opens and the intake feeds **only while the flywheel is at speed and the turret is aimed** (`FiringSequence`).
- [ ] Turret homing method (limit switch versus absolute encoder) `[#61]`. The range is decided: about 270° (soft limits at ±120°). A 360° turret is not feasible for this design (coach, 2026-11-13, #12).
- [x] Turret drive: **a motor with its built-in encoder** (goBILDA 5203, 435 RPM). The separate encoder issue, #32, was closed on 2026-11-13.
- [ ] Lens height and final camera tilt `[#83]`
- [ ] Number of balls needed to tip the hive `[#60]`
- [ ] Whether nectar (3.6 in) is ever added to the shooter `[#68]`
