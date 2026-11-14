# BIOBUZZ Autonomous Strategy

*DRAFT for the coach to review. Status as of 2026-11-14: **A-0 (`AutoEmergencyLeave`) and A-1 (`AutoLeaveAndPark`) are written, and so is a shoot-and-park test (`ShootAndParkAuto`). A-0 has run on a robot (#56). The Pedro Autos cannot drive until Pedro is tuned on a robot (`FORESIGHT_TUNED` is still false, #57).** Setup, tuning and Auto-writing rules: `pedro-setup-and-auto-guide.md`. This doc says what Auto is allowed to do, what it is worth, and the order we build it in. Facts come from the Competition Manual (TU02) sections 9-11 and `biobuzz-game-essentials.md`. If the manual and this file disagree, the manual wins. A `[#nn]` marker (the number of a GitHub Issue) marks a decision or measurement nobody has made yet: ask the coach before building anything that depends on it.*

## 1. What the rules say about AUTO

| Fact | Detail | Rule |
|---|---|---|
| Length | 30 seconds. No driver input at all. | G401 |
| Staying on our side | During AUTO we may only play on **our alliance's half** of the field (red = columns A, B, C; blue = columns D, E, F). Driving into the other half is "risky" and can be called STRATEGIC. | G402 |
| Start position | Touching the perimeter wall, on our half, **not** in the LOADING ZONE, not touching a FLOWER, inside the 18 in cube, holding exactly **4 POLLEN**, and not moving after INIT. | G304, R102 |
| LEAVE (3 pts) | The robot is no longer touching the perimeter wall at the end of AUTO. | 10.5.4 |
| PARK (5 pts) | The robot is at least partly in our LOADING ZONE at the end of AUTO. (The zone is about 23 in wide by 11 in deep.) | 10.5.4 |
| HIVE TIP (20 pts) | A tip that finishes **before TELEOP starts** counts as an AUTO tip. Launching into the raised cell is the only legal way to cause one. | 10.5, G417 |
| Capacity | No more than 4 balls at once. | G407 |
| Nectar | A tip lets our human player add one NECTAR. We do not touch the other alliance's nectar. | G426, G408 |
| Picking up balls | The manual says robots may use sensors to collect POLLEN from the FLOWERS in AUTO (the bottom retrieval opening). | 8, G418 |
| Spilled balls | Do not catch balls falling out of a tipped hive until they have touched something else. | G409 |
| Transition | The robot may not move under power during the 8 s between AUTO and TELEOP. Do not press INIT on a TeleOp that twitches a motor until TELEOP starts. | G403 |
| OpMode choice | We must pick the Auto OpMode on the Driver Station with the **30 second timer enabled**, and press INIT before the match. | G305 |

## 2. What AUTO is worth

| Action | Points | Notes |
|---|---|---|
| LEAVE | 3 | Easy. Counts toward SWARM. |
| PARK at the end of AUTO | 5 | Counts toward SWARM. We can leave the zone again in TELEOP. |
| HIVE TIP in AUTO | 20 | The big one. Also counts toward POLLINATOR 1 and 2, and unlocks a nectar. |

LEAVE + PARK is **8 points** and needs no shooter at all. One tip is worth more than twice that.

## 3. The ladder: build one rung, finish it, then the next

Each rung is its own OpMode. The earlier rungs stay in the Driver Station list all season, so if a later one breaks at an event we can pick a safer one in 10 seconds.

**Reliability gate (proposal):** don't start the next rung until the current one has run **5 times in a row** on the robot without a miss. Write each run in `biobuzz-test-and-match-log.md`.

| Rung | Goal | Points | Needs | Status |
|---|---|---|---|---|
| **A-0 Emergency** | Drive off the wall and stop. No Pedro, no sensors, just `DriveSubsystem` for a fixed time. Used when everything else is broken. | 3 | `DriveSubsystem` only | **`AutoEmergencyLeave` written** (a timed straight drive, starting guesses 0.30 power for 0.80 s, to be tuned by measuring). Open loop, so it does not park and the distance changes with the battery. Not run on a robot yet. |
| **A-1 LEAVE + PARK** | Leave the wall and end in our LOADING ZONE. | 8 | Pedro Pathing set up and tuned, `PinpointSubsystem` (Pedro reads the Pinpoint itself) | **`AutoLeaveAndPark` written** (blue D6 to the LOADING ZONE, red built in by a 180 degree rotation), not run on a robot yet. See `ARCHITECTURE.md`, "Autos". |
| **A-2 Shoot the preloads** | From a fixed spot, shoot the 4 preloaded POLLEN, then do A-1. Uses the **fallback mode** from `biobuzz-robot-design.md`: turret at a preset angle, fixed flywheel RPM, no camera needed. | 8 + a tip if 4 balls are enough | A-1, plus a **firing sequence** (gate and intake in order, only when the wheels are at speed) | Firing sequence not written |
| **A-3 Shoot with vision** | Same as A-2, but the turret aims with the Limelight and the RPM comes from the shot table. | Same as A-2, with better accuracy | A-2, `VisionSubsystem`, `ShotTable` (already in TeleOp) | **`ShootAndParkAuto` written** (camera lock, shot table RPM, then the flywheel-only at-speed check, gate OPENED with a paced intake feed, gate LIFTED for the last ball, then park; skips the shot and parks if there is no lock or the flywheel never reaches speed). First robot test: the flywheel, hood wheel and turret lock worked, but the intake was off so no balls moved, and the both-wheels check took about 4 s; the changes for those are not yet re-tested. It went straight to the vision version, so A-2's preset-angle fallback is **not** built. Not run on a robot yet. |
| **A-4 Reload and shoot again** | After the first load, collect more POLLEN (FLOWER or GARDEN on our half), then shoot again. | Second chance at a tip | A-3, intake running while driving, a Pedro path to the pollen | Not started |
| **A-5 Stretch** | Two tips in AUTO. | 40 | Everything above, plus a lot of speed | Only if A-4 is boringly reliable |

## 4. Facts we still need before choosing a rung `[#60]`

These decide how high up the ladder Auto is worth climbing. Please do not guess; measure or look them up.

1. **How many balls does it take to tip the hive?** The manual does not say. Check the field CAD, a Team Update, or test on a real hive. If 4 is enough, A-2 can score a tip. If it takes 8, both robots' preloads are needed and the partner plan matters. If it takes 12 or more, Auto tips need reloading (A-4).
2. **Which cell is raised at the start, and which side does it face?** Look at Figure 10-2 in the manual. The raised cell is the one we shoot at, and where we can stand to do it **while staying on our half** (G402) decides our shooting spot.
3. **Where can the robot legally stand?** Pick the 2 or 3 start spots on the wall for each alliance and write them down (tile and side). The robot must touch the wall and stay out of the LOADING ZONE.
4. **How long do the moves take?** Measure on the practice bot: start to the shooting spot, spin-up time, time to shoot 4 balls, shooting spot to the LOADING ZONE. Fill in the table in section 6. A route that does not fit in 30 seconds with some spare time is the wrong route.
5. **Is the shot good from the Auto spot?** The shot table was measured at 5 spots in TeleOp. The Auto spot should be one of them (or close), so the camera and RPM are already proven there.
6. **Can the robot's Pinpoint heading be trusted for 30 seconds?** Test by repeating one path 10 times and checking where it ends.

## 5. Rules for writing the Auto code

These follow `ARCHITECTURE.md`. They are here so the Auto plan and the code plan agree.

- **Pedro Pathing for movement.** An iterative OpMode (`extends OpMode`) with `follower.update()` in `loop()` and a `pathState` switch that moves from step to step. Follow Pedro's own example.
- **Same subsystems as TeleOp, except the drivetrain.** Pedro's `Follower` drives the four drive motors in Auto, so Auto never uses `DriveSubsystem` (except the A-0 Emergency OpMode, which does not use Pedro). Never use both in one OpMode.
- **No `sleep()` or blocking waits.** Use `ElapsedTime` timers and the `pathState` switch. A shoot step is a state that waits until the shooter is at speed and the turret is aimed.
- **Choose alliance and start spot during INIT** with gamepad buttons in `init_loop()`, and show both big on the telemetry screen. TeleOp starts as BLUE and the wrong alliance aims at the other hive (see `biobuzz-troubleshooting.md`), so Auto should not silently default. Whether alliance is set per robot or per match is `[#60]`.
- **Write the paths once.** Make the blue path, and build the red one from it with one small helper that flips X across the field, so we never keep two copies that drift apart.
- **Fallbacks.** If the Limelight sees nothing, shoot from the preset turret angle. If the Pinpoint is not READY at INIT, the drive team picks A-0 Emergency instead.
- **End of Auto:** every motor off (`stop()`), the turret floats or holds where it is. The hub keeps the turret's encoder count into TeleOp, so TeleOp does not need to re-home. If TeleOp later uses the Pinpoint pose, whether it starts from Auto's final pose or from zero is `[#60]`.
- **Queue TeleOp automatically.** The `@Autonomous` annotation has a `preselectTeleOp` option so the Driver Station loads `BiobuzzTeleOp` after Auto. The manual (G305 note) suggests it.
- **Every tunable number is a named constant** at the top with a comment (start spot coordinates, shooting spot, RPM, timers).

## 6. Timing table (fill in from the practice bot)

| Step | Seconds | Measured on | Date |
|---|---|---|---|
| INIT done to first move | | | |
| Start spot to shooting spot | | | |
| Turret aim (from preset or camera) | | | |
| Flywheel spin-up to READY | | | |
| Shoot 4 balls | | | |
| Shooting spot to LOADING ZONE | | | |
| **Total (must be under 30, with spare)** | | | |

## 7. Partner card (hand this to our alliance partner before the match)

Auto only works if our two robots do not drive into each other. Fill this in per robot and bring it printed.

| | Robot 1 | Robot 2 (partner) |
|---|---|---|
| Team | | |
| Start spot (tile and side of wall) | | |
| Auto rung we run (A-1, A-2, ...) | | |
| Where we shoot from | | |
| Path crosses the middle of our half? | | |
| Ends in the LOADING ZONE? | | |
| Time we finish | | |

Agree on one rule before every match: **who goes first if our paths cross**, and who stays out of the way.

## 8. Decisions for the coach

- [ ] Rung to target for the first event (suggestion: A-1 done and solid, A-2 as the stretch).
- [ ] Start spot(s) for each robot (11940, 22091, practice bot). *First guess in code: blue tile D6, back to the top wall, facing down at the hive (red: C1). Not yet tested on the field.*
- [ ] Alliance selection: per robot or per match, and chosen in INIT. *Now: chosen in INIT with gamepad1 X (BLUE, default) or B (RED).*
- [ ] Do we allow A-4 (collecting in Auto) or is it TeleOp-only this season?
