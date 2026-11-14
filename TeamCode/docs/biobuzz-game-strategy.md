# BIOBUZZ Game Strategy

*DRAFT for the coach to review. Status as of 2026-11-03 (robot is at `TeleOp - Alpha 8`). Facts come from the Competition Manual (TU02) and `biobuzz-game-essentials.md`; if they disagree with this file, the manual wins. Numbers marked **(derived)** are arithmetic from the manual's point table, not measurements. A `[#nn]` marker (the number of a GitHub Issue) marks something nobody has decided or measured yet. The Auto plan is in `biobuzz-auto-strategy.md`.*

## 1. The one-sentence strategy

**Tip our hive as many times as we can, park at the end, and make sure our partner also earns LEAVE.** A tip is worth 20; nearly everything else is worth 1 to 5. Each tip also counts toward two ranking points and unlocks one more nectar.

## 2. Where points come from

| Action | AUTO | TELEOP | Our plan |
|---|---|---|---|
| HIVE TIP | 20 | 20 | **Main job.** Turret shooter, pollen only. |
| LEAVE | 3 | n/a | Always. Auto rung A-0 or better. |
| PARK (at least partly in our LOADING ZONE at the end) | 5 | 5 | Always. Plan the drive back (section 6). |
| Ball still in the raised cell at the end | n/a | 2 each | Free bonus from the last shots. |
| Ball in a FLOWER we own | n/a | 2 each | Tier 3. Only if it comes cheap. |
| Bottom NECTAR bonus | n/a | 5 | Needs nectar. Our shooter is pollen-only, so skip. |
| Ball in our GARDEN | n/a | 1 each | **Skip.** Not worth a mechanism. |

## 3. Ranking points (they decide event rank)

| RP | How | What it takes |
|---|---|---|
| Win | 3 | More match points than the other alliance. |
| Tie | 1 | |
| **SWARM** | 1 | Alliance LEAVE + PARK points add up to **16 or more**. |
| **POLLINATOR 1** | 1 | Alliance tips our hive **4 or more** times. |
| **POLLINATOR 2** | 1 | Alliance tips our hive **7 or more** times. |

**SWARM arithmetic (derived).** One robot can earn at most 13 LEAVE + PARK points (LEAVE 3 + AUTO PARK 5 + TELEOP PARK 5). That means:

- Both robots do LEAVE (3) and TELEOP PARK (5): 8 + 8 = **16**. SWARM.
- One robot does all three (13), and the partner **only** does LEAVE (3): 13 + 3 = **16**. SWARM.
- One robot alone can never reach 16, so SWARM always needs the partner to do something.

So the most useful thing we can say to a partner before a match is: "Please just LEAVE in Auto. We will do the rest." It costs them almost nothing.

**POLLINATOR needs many tips, and we do not yet know what a tip costs.** Planning for 4 or 7 tips in 2 minutes 30 seconds only makes sense once we have the number from section 4 `[#60]`.

## 4. The scoring cycle (what the robot does over and over)

1. **Reload.** Get up to 4 balls (the limit, G407).
2. **Drive to the correct side of the hive.** The raised cell flips after every tip, and it faces the other way each time (see `biobuzz-hive-apriltag-geometry.md`). The turret lets us shoot from anywhere on the right side without lining up the chassis precisely.
3. **Aim.** The turret tracks the hive with the Limelight (ALWAYS-TRACK). The camera must see our alliance's tags, so the alliance setting has to be right.
4. **Spin up.** Pre-rev with the driver's **D-pad Up** while driving to the spot, then the operator **holds the Right Trigger**. The firing sequence waits until the flywheel is at speed and the camera says the turret is aimed.
5. **Shoot.** The sequence opens the gate and **keeps feeding while the operator holds the trigger** (3 or 4 balls is normal). It pauses by itself if the flywheel drops off speed or the turret leaves the hive. If the last ball needs help, the operator **holds Y** to lift it in. Let go of the trigger to stop.
6. **Check for a tip**, then go to step 1 again, from the other side.

**Planning formula.** If it takes `N` balls to tip (`[#60]`, the manual does not say), we need `ceil(N / 4)` trips per tip.
`time per tip = (reload + drive + aim + spin-up + shoot) x ceil(N / 4)`
Fill in the pieces from timed practice runs (`biobuzz-test-and-match-log.md`). The tip count we can honestly aim for is `120 seconds / time per tip`.

## 5. Where balls come from

There are only **40 POLLEN** on the field in total, so we are competing for them.

| Source | How many | Notes |
|---|---|---|
| Our preload | 4 | Every robot starts holding 4. Free. |
| FLOWERS | 4 in each of the 4 flowers (16) | Take them out of the **bottom** retrieval opening only (G418). |
| GARDENS | 4 in each of the 2 gardens (8) | Not a protected zone. Either alliance can take from either garden. |
| Balls spilled when a hive tips | varies | Do not catch them in the air (G409). Pick them up after they land. Do not park under the hive waiting for a tip (also G409). |
| NECTAR | 8 per alliance | Our shooter is for pollen only. Never pick up the other alliance's nectar (G408). Whether nectar is ever added to the shooter is `[#68]`. |

Do not corral balls into a corner to keep the other alliance away from them (G411).

## 6. Match timeline

| Time (field clock) | Phase | What we do |
|---|---|---|
| 2:30 to 2:00 | AUTO (30 s) | Run the Auto rung. See `biobuzz-auto-strategy.md`. |
| 2:00 to 1:52 | Transition (8 s) | **Nothing moves** (G403). Switch the Driver Station to TeleOp (or let the queued TeleOp load). |
| 1:52 to 1:00 | TELEOP | Scoring cycles (section 4). |
| **1:00** | Nectar unlocked, FLOWER scoring opens | Our human player can add all remaining nectar. Nothing changes for our pollen shooter, so keep cycling. |
| 0:20 | "Train Whistle" | **Start thinking about PARK.** Decide now whether one more cycle fits. |
| 0:00 | End | Robot must be **motionless** (G404) and, for PARK, partly in the LOADING ZONE. |

**PARK needs a plan.** The LOADING ZONE is small (about 23 in by 11 in) and our human player also drops nectar there (G427), so an alliance partner may be heading to the same spot. Decide in advance: how many seconds the drive back takes from our usual shooting spots (measure it, `[#68]`), and the field-clock time when the driver must stop shooting and go. Write it here once measured: **start the PARK drive at ____ seconds left.**

## 7. Rules that shape the strategy

| Rule | What it means for us |
|---|---|
| G402 | In AUTO, stay on our half. |
| G407 | Never hold more than 4 balls. Mark 5+ as a bug, not a feature. |
| G408 | Never touch the other alliance's nectar. |
| G409 | Do not catch balls from a tipping hive before they touch something else. |
| G410 | No nectar in a FLOWER before 1:00 left (major foul per nectar). |
| G411 | No hoarding or blocking balls from the other alliance. |
| G417 | **Only way to tip is to launch into the raised cell.** No pushing or ramming the hive, no shooting at the outside of a cell, and no shooting at the other alliance's hive to stop its tip. |
| G417 note | Launching at the **lowered** cell while the hive is tipping can stop the tip from counting. Pause shooting if a tip is in progress. |
| G422, G425 | Drive team stays in the alliance area and never touches the robot or balls on the field. |

## 8. Roles on the drive team

A drive team has up to 4 students/mentors: a drive coach, up to 3 drivers, and a human player.

| Role | Job |
|---|---|
| **Driver** (gamepad 1) | Drive, intake and reverse (bumpers), pre-rev (D-pad Up), alliance (Start), field / robot centric (Back). Fires only in a one-person match (LOGO, then Right Trigger). |
| **Operator** (gamepad 2) | Fires (hold Right Trigger), alliance (Start), field / robot centric (Back), speed source DIAL / TABLE (LOGO), gate between volleys (X / Y), FORCED SHOT when the checks are wrong (hold B), floating the turret by hand (A). Checks the alliance on the screen. |
| **Human player** | Adds nectar through the LOADING ZONE only (G427): one per hive tip, or all of it once 1:00 is left (G426). Counts tips. Does not touch the robot. |
| **Drive coach** | Watches the clock and the field, calls the PARK time, picks the OpMode and presses INIT/start for the drivers (allowed by G423). |

Controls are in `biobuzz-teleop-controls.md`. Who is doing which job at each event is `[#68]`.

## 9. Working with the partner

Talk to the partner before **every** qualifying match, because the partner changes every time.

- What does your robot do in Auto, and where does it start? (Use the partner card in `biobuzz-auto-strategy.md`.)
- Can you LEAVE? That alone, plus our 13, makes SWARM (section 3).
- Which robot goes to which side of the hive after a flip? Two robots going to the same side of the same small spot is the most likely traffic jam.
- Who parks in the LOADING ZONE, and when? If only one robot fits comfortably, agree who.
- Is the partner's robot allowed to shoot into our hive? Two shooters into one cell works, but keep out of each other's shot lines.

## 10. Decisions for the coach

- [ ] How many balls does it take to tip? (Biggest unknown.)
- [ ] Do we ever go for FLOWERS (Tier 3), or stay pollen-and-tip only?
- [ ] Priority when they conflict: win the match, or earn POLLINATOR/SWARM ranking points?
- [ ] Do we play any defense? (G411 and G417 limit it a lot. Suggestion: no.)
- [ ] Which robots and drive team members go to which events.
