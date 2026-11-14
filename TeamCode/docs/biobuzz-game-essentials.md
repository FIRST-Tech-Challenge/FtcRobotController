# BIOBUZZ Game Essentials

*The parts of the 2026-2027 game that affect code and robot design. Source: Competition Manual (V1 / TU02), Sections 9–10. If a Team Update changes something, the manual wins. Update this file when it does.*

## Match structure

| Period | Length | Notes |
|---|---|---|
| AUTO | 30 s | No drivers |
| Transition | 8 s | |
| TELEOP | 2 min | The last **60 s** unlocks FLOWER scoring and releases all remaining NECTAR |

Two robots per alliance (red or blue). Your partner changes every match.

## Scoring elements

| Element | Size | Count | Notes |
|---|---|---|---|
| POLLEN (yellow) | ~2.8 in | 40 | Each robot **starts holding 4** |
| NECTAR (red / blue) | ~3.6 in | 8 per alliance | Most start locked and are released as the alliance tips its hive |

The balls are not perfectly round and vary in size. Any mechanism that touches balls must tolerate that.

## Match points

| Action | AUTO | TELEOP |
|---|---|---|
| LEAVE (move off the wall) | 3 | – |
| PARK (in loading zone at end of period) | 5 | 5 |
| **HIVE TIP** | **20** | **20** |
| Each ball still in the raised cell at the end | – | 2 |
| Bottom nectar bonus (your nectar is lowest in a flower) | – | 5 |
| Each ball in a flower you own | – | 2 |
| Each ball in your garden | – | 1 |

**A hive tip is worth 20. Everything else is worth 1–5. This game is about tipping the hive.** Each tip also releases one more of your alliance's locked nectar.

## Ranking points (these decide event rank)

| How | RP |
|---|---|
| Win | 3 |
| Tie | 1 |
| SWARM: alliance LEAVE + PARK points ≥ 16 | 1 |
| POLLINATOR 1: alliance tips ≥ 4 times | 1 |
| POLLINATOR 2: alliance tips ≥ 7 times | 1 |

One robot can earn at most 13 LEAVE + PARK points, so **SWARM needs your partner too**.

## Rules that constrain the robot and code

- **Size:** the robot starts inside an 18 in cube and may expand to at most **18 × 24 × 29 in tall** (R102, R105).
- **Capacity:** the robot may **hold no more than 4 balls** (G407). Code that counts balls should enforce this.
- **Tipping:** the only legal way to tip is **launching balls into the raised cell**. No pushing or ramming the hive (G417).
- **Flowers:** no nectar in a flower before the 60-second mark. Each early ball is a major foul (G410). Any auto-score-into-flower logic must check match time.
- **Opponent nectar:** never pick it up (G408). If we ever intake nectar, color detection has to reject the other alliance's color.

## How the hive works, briefly

See `biobuzz-hive-apriltag-geometry.md` for the numbers.

- Each alliance has a HIVE: a see-saw with a CELL on each end. One cell is raised and the other is lowered.
- The raised cell's opening is a **window at 53.5–65.6 in** that faces outward and leans back 30°. Launch balls through it.
- Enough balls tip the hive. The other cell rises, **facing the opposite side of the field**, so **after every tip the robot shoots from the other side of the hive.**

## Strategy priority (drives build order)

1. **Tier 1, drivetrain only:** LEAVE + PARK in auto and PARK in teleop. That's 13 points and our half of SWARM, and it needs only code and practice.
2. **Tier 2, launch, then intake:** tips are the game. Since robots start with 4 pollen, a launcher alone can score in auto. The intake comes second so we can reload in teleop.
3. **Tier 3, last minute:** nectar into flowers, for the bottom bonus and ownership.
4. **Skip:** we don't build a mechanism just for the garden.

## Still unknown `[#60]`

- How many balls does it take to tip the hive? The manual doesn't say. Find out from field CAD, a Team Update, or testing.
- Can our drivetrain reach the hive, shoot, and get back to the loading zone in 30 s of auto?
