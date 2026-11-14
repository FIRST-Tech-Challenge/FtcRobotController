# BIOBUZZ: Turret, Shooter, and Limelight Context

*Cyber Coyotes FTC (Reed City). Handoff doc for Claude Code. Suggested repo location: `docs/biobuzz-turret-shooter-context.md`. If a Team Update changes anything, the Competition Manual wins. Items marked **[#nn]** (the number of a GitHub Issue) are unresolved: stop and ask the coach before proceeding past them. Values marked **derived** have not been measured on a real field.*

## 1. How to work with the coach

- Read `ARCHITECTURE.md` and this file first.
- **Proposal first:** show the design (classes, fields, method signatures), wait for approval, then write code.
- **One subsystem or class per session.**
- Push back on problems proactively. The coach prefers that over rubber-stamping.
- Teaching clarity matters: this code is read by middle school students. Prefer verbose, readable code over clever code.

## 2. Goal

Build a shooter on a rotating turret for BIOBUZZ:

- A **turret** driven by one motor mounted underneath, turning left and right to line up with the target.
- A **weighted flywheel** with **variable output**, plus a **companion hood wheel motor** (independently controlled).
- A **Limelight 3A** reads AprilTags to aim the turret and to estimate distance, which sets flywheel and hood wheel speed.
- Shooter is designed for **pollen** (2.8 in ball) only for now.
- **All motors are goBILDA 5203.** Turret: 435 RPM (13.7:1). Flywheel: 1620 RPM (3.7:1). Hood wheel: 6000 RPM (1:1). The encoder counts for each are in `CLAUDE.md` (Known gotchas). The intake motor is believed to be 1620 RPM (3.7:1) **[#65]**.

## 3. Stack and conventions

- Java, FTC SDK (12.0), Android Studio, Android build flavors. Pedro Pathing 2.1.2 for Auto. FTCLib 2.1.1 is in the project but **2026-27 is not command-based**.
- **2026-27 approach:** standard FTC SDK style. One class per mechanism for hardware code. Test OpModes are iterative OpModes (existing examples: `AlphaIntakeTesting`, `AlphaShooterTesting`).
- Shared subsystem classes live in `src/main/`. Robot-specific OpModes live in flavor directories: `src/practiceBot/`, `src/team11940/`, `src/team22091/`.
- Naming: packages lowercase, classes PascalCase, variables and methods camelCase, constants ALL_CAPS, hardware variables location-first (e.g., `leftIntakeServo`), hardware map names snake_case (e.g., `left_intake_servo`), doc filenames kebab-case.
- Hardware map names (decided): `turret_motor`, `flywheel`, `hood_wheel`. Limelight config entry is `limelight`.
- This is the final season of this Java setup. The program moves to SystemCore in 2027-28, so keep things simple and standard.

## 4. Hardware and API gotchas (learned the hard way)

- **Limelight 3A** needs an Ethernet device entry named `limelight` in the robot config. It lives at `172.29.0.1` on the Control Hub's internal subnet (not 192.168.1.11). Firmware 2025.x. RPIBoot USB drivers are needed on Windows for flashing. The REV Control Hub supports only one Limelight 3A.
- Limelight AprilTag settings that worked: exposure 1000, gain 8, up to 90 FPS. Retune under real lighting: the tags sit in the cell's shadow with arena lights behind them.
- **goBILDA 5203 encoder:** **28 counts per motor-shaft revolution** (the 4x quadrature is already included in that 28). It is 28 at the encoder shaft, and at the gearbox output shaft it is 28 x the gear ratio, per the goBILDA spec sheets in `TeamCode/docs/` (`5203-2402-0001`, `-0003`, `-0014`): **28** for the 6000 RPM 1:1 motor, **103.8** for the 1620 RPM 3.7:1 motor, **384.5** for the 435 RPM 13.7:1 motor. Max about 2,800 ticks/sec at full power for every one of them. (An older version of this note said 112 and 11,200 ticks/sec. That was wrong, and a 4x error like it previously made the flywheel underperform.)
- Troubleshoot cables and port assignments before code.
- Android Studio on Windows must be run as administrator to deploy (permanent fix: set `GRADLE_USER_HOME`). `compileSdk` override to 36 is needed in TeamCode's `build.gradle`.
- Hardware: REV Control Hub and Expansion Hub, goBILDA Pinpoint odometry. Controllers: PS5 DualSense (primary), Logitech F310 (backup).

## 4a. Vision source decision

The Limelight 3A is the FTC-supported, legal camera (`LL_3A`) and is already in the team's code. The alternative is a webcam with the SDK's `VisionPortal`, whose AprilTag library groups each cell's four tags into a **cluster** with its origin at the center of the cell opening. **Decided: Limelight 3A (final).** The Limelight does not know about clusters, so our code must add the offset from the tags to the opening.

## 5. Game facts that drive the design

- Hive tips when enough balls land in the raised cell. A tip is worth 20 points. Everything else is worth 1 to 5. Hive tipping is the game.
- After every tip the raised cell flips to the other side, so the robot must drive around to keep scoring.
- **AprilTag family:** 36h11, 3.25 in square. Four tags per cell cluster, on the cell's bottom face, facing down and 30° outward toward the shooting robots.

| Cell | Tag IDs |
|---|---|
| Red, far (scoring) side | 30, 31, 32, 33 |
| Red, audience side | 34, 35, 36, 37 |
| Blue, audience side | 38, 39, 40, 41 |
| Blue, far (scoring) side | 42, 43, 44, 45 |

- Tag centers sit at +/-2.75 in (inner) and +/-6.5 in (outer) from the cluster center.
- **Always filter by alliance tag IDs.** The other alliance's hive is 25.5 in to the side on the same frame.
- **Both of our alliance's clusters can be in view.** The target is the higher cluster (larger `ty`, about 50 in versus about 34 in, derived).
- The support arm runs down the middle of the cluster and may block the inner tags or one pair from some angles.

### Derived geometry (raised cell, verify on the practice field)

| Feature | Height above tiles | Horizontal from pivot |
|---|---|---|
| Tag cluster center | ~50 in | ~15 in |
| Opening center (aim point) | ~59.5 in | ~17.5 in |
| Opening bottom edge | 53.5 in (published) | ~21 in |

- Opening is 20 in wide x 14 in tall, tilted back 30° from vertical.
- The aim point is about 9.5 in above the tags and about 2.7 in closer to the robot. Side to side it lines up with the cluster center, so **turning the turret until the cluster `tx` = 0 aims at the cell.**
- Best ball entry: coming down at about 30° at the window. The ball must climb about 40 in (shooter exit around 15-18 in to window at 55-64 in). Robot height limit is 29 in once expanded (R105).

## 6. Vision rules

- **Do not use the tags for field position.** The SDK release notes say the BIOBUZZ AprilTags move (the hive tips), so they are unsuitable for absolute localization. **Use Pinpoint odometry for position and the tags only for aiming.**
- SDK 12.0 AprilTag detection returns either a single-tag or a cluster type and must be checked and cast. This applies to the SDK `VisionPortal` path, not the Limelight API.
- Mount the Limelight **on the turret**, facing the same way as the shooter, as low as practical, tilted up about 40° (recommendation; recalculate with the real lens height).
- Distance estimate: `d = (50 - lensHeight) / tan(tilt + ty)`. Distance to the opening is about `d - 2.7 in`.
- Early aim can average the visible tags (worst case about 4.6 in off center; the window tolerates about +/-8.6 in for a 2.8 in ball). A later upgrade is adding each tag's known offset.
- Tag size, not tilt, limits range. Expect detection trouble around 8-10 ft. Test max range on a mock cell.

## 7. Planned design (pending approval)

Three classes, one per mechanism, plus test OpModes. No code written yet.

- `Turret`: motor, zeroing at init, ticks to degrees, soft limits, manual and auto control.
- `Shooter`: flywheel and hood wheel motors, velocity control, at-speed checks, set speeds from a shot table.
- `LimelightVision` helper: alliance filtering, valid/stale checks, `tx`, `ty`, distance estimate. (Sensors are plain helper classes, not subsystems.)

## 8. Build order (test each step before the next)

1. Hardware config and direction check (positive power = turret right, flywheel forward, hood wheel correct direction).
2. Turret manual control, zeroing, soft limits (protect cable wrap and frame).
3. Flywheel and hood wheel open-loop tests.
4. Velocity control with PIDF for both; "at speed" tolerance check.
5. Limelight AprilTag pipeline: read `tx`, `ty`, valid flag; alliance filter; pick the higher cluster.
6. Turret auto-aim: proportional on `tx` with deadband and max-power clamp, add D if it oscillates. Define behavior when the tag is lost.
7. Distance estimate and lookup table of distance to flywheel and hood wheel speed (measure at 4-6 distances, interpolate).
8. Shoot gate: feed only when aligned and both motors at speed.
9. Pinpoint heading fallback to keep the turret pointed at the hive when tags drop out.
10. Integrate in TeleOp, then Auto.

Log `tx`, flywheel and hood wheel speed, and turret angle to telemetry in every test OpMode.

## 9. Open items

- [x] Motor models and gear ratios: turret 435 RPM (13.7:1), flywheel 1620 RPM (3.7:1), hood wheel 6000 RPM (1:1), all goBILDA 5203. The intake motor's label still needs reading **[#65]**.
- [x] Hardware map names: `turret_motor`, `flywheel`, `hood_wheel`.
- [x] Vision source: Limelight 3A (final).
- [ ] Measure actual tag and opening heights on the practice field or field CAD. **[#88]**
- [ ] Find the Limelight 3A's max reliable detection range on a 3.25 in tag under gym lighting. **[#64]**
- [ ] Turret travel range and cable management approach. (360° was ruled out on 2026-11-13, #12.) **[#29]**, **[#6]**
- [ ] How many balls does it take to tip the hive? (Not in the manual; check field CAD, a Team Update, or test.) **[#60]**
- [ ] Confirm the robot cannot drive over or inside the frame base bars. **[#68]**
