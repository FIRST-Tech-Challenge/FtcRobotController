# Red Far Cycle and autonomous tag relocalization (2026-11-18)

This work is kept separate from the team's own autos: `FallbackAutonomous`, the Red/Blue fallback wrappers, `integration-example` and `pollen-navigation-test` are untouched. It adds the [`red-far-cycle`](../autos/red-far-cycle.auto.json) route, tag relocalization in autonomous (Java), three blue mirror wrappers and this document. The earlier drafts `red-audience` and `red-far` ([MatchAutonomousPlan.md](MatchAutonomousPlan.md)) still exist. Their redundant `turretHome` step was removed, because the turret now homes during INIT. Item 1 of that plan's implementation work ("tag-localized aiming") is now [implemented below](#autonomous-tag-relocalization).

## All autonomous plans

| Plan | Driver Station entry | Who made it | Status |
| --- | --- | --- | --- |
| Red / Blue Fallback (Shoot + Leave) | Competition: `Auto: Red/Blue Fallback (Shoot + Leave)` | team | runnable now; no Pedro, no field pose |
| Integration example | Generated: `EXAMPLE: Zenith integration (tuning required)` | team | cleared-area authoring example, not a match route |
| Pollen navigation test | Test: `Test: Pollen Navigation (Pedro + Ivy)` | team | Test-group camera collection experiment |
| Red Audience (4+4+4+4) | Generated: `DRAFT Auto: Red Audience …` from [`red-audience`](../autos/red-audience.auto.json) | draft | 21.6 s nominal ([plan](MatchAutonomousPlan.md)) |
| Blue Far (Red Audience mirrored) | Competition: `DRAFT Auto: Blue Far (mirrors red-audience)` (`BlueFarAutonomous`) | draft | same route |
| Red Far (5 s, 4+4, cross field) | Generated: `DRAFT Auto: Red Far …` from [`red-far`](../autos/red-far.auto.json) | draft | 19.2 s nominal ([plan](MatchAutonomousPlan.md)) |
| Blue Audience (Red Far mirrored) | Competition: `DRAFT Auto: Blue Audience (mirrors red-far)` (`BlueAudienceAutonomous`) | draft | same route |
| **Red Far Cycle (5 s, far 4+4, audience 4+4)** | Generated: `DRAFT Auto: Red Far Cycle …` from [`red-far-cycle`](../autos/red-far-cycle.auto.json) | draft | 29.2 s nominal, **does not fit yet** ([timing](#timing-does-not-fit-yet)) |
| Blue Audience Cycle (Red Far Cycle mirrored) | Competition: `DRAFT Auto: Blue Audience Cycle (mirrors red-far-cycle)` (`BlueAudienceCycleAutonomous`) | draft | same route |
| Autonomous tag relocalization | all Pedro/Zenith full-robot autos | implemented | JVM-tested; not tested on the robot |

None of the Pedro/Zenith routes can START until `PedroConfiguration.TUNING_VERIFIED` is set, and none can feed automatically until the shot table is calibrated.

Blue never has its own route file. Each blue wrapper loads the red file with `new BioBuzzRobot(TurretSubsystem.Alliance.BLUE)`, so Ivy point-mirrors every pose once, (x, y, h) → (−x, −y, h + π), and `aim(frame=AUTHORED)` swaps AUDIENCE/FAR. Each wrapper preselects `Mecanum TeleOp - Blue`. Team validation (`scripts/zenith-project.cjs`) now allows exactly one other-alliance wrapper per routine. It still refuses same-alliance or second mirror wrappers.

## Red Far Cycle (Blue Audience Cycle mirrored)

Written exactly as requested. The earlier `red-far` (cross the field after one flower) still exists as the alternative.

Start: left side on the far wall, facing +X toward blue, flower1 just behind. Preload 4. Intake on the front. The depollinator goes out before each flower and home right after.

| # | Step | Pose (in, heading) | Hive: with partner tip first / solo |
| --- | --- | --- | --- |
| 0 | Turret already homed and relocalized in INIT; aim FAR with spin, wait 4.5 s so the first feed is about 5 s after START | (−10, 63, 0°) | |
| 1 | **Volley 1**: 4 preload at the far cell, about 48 in, turret yaw ≈ −3° | (−10, 63, 0°) | far 4/8 / far is down, no score |
| 2 | Depollinator out; pull straight off the wall, turn, head-on into flower1, `loadUntilFull` 2 s | → (−24, 57, 90°) | |
| 3 | Depollinator home; back off 6 in | → (−24, 51, 90°) | |
| 4 | **Volley 2**: 4 into the far cell, about 37.6 in, yaw ≈ −73° | (−24, 51, 90°) | far 8/8 → **tips**, audience up / down, no score |
| 5 | Pre-aim AUDIENCE; drive down the red side past the hive; depollinator deploys at 80% of the drive | → (−50, −24, 180°) | |
| 6 | Head-on into the red-wall flower (flower0), `loadUntilFull` 2 s | → (−57, −24, 180°) | |
| 7 | Depollinator home; back off to a legal audience-side shot spot | → (−50, −34, 180°) | |
| 8 | **Volley 3**: 4 into the audience cell, about 41.8 in, yaw ≈ −63° | (−50, −34, 180°) | audience 4/8 / 4 → **tips**, far up |
| 9 | Line up behind the red garden, sweep it, `loadUntilFull` 2 s | → (−48, −63) → (−62, −63, 180°) | |
| 10 | **Volley 4**: 4 garden pollen at the audience cell from the sweep end, about 68.7 in, yaw ≈ −46° | (−62, −63, 180°) | audience 8/8 → **tips** / down, no score |
| 11 | `stopAll` | | |

**Solo result: one volley (4 pollen) scores and the hive tips once.** The full sequence (16 scored, 2 tips) needs the far cell up at 5 s, which only happens if a partner has already tipped red's hive. Kept that way on purpose for pairing later.

In the route file, only volley 3 carries a Zenith ledger `expect`. Volleys 1, 2 and 4 shoot at a cell that Zenith's solo hive model has lowered, so an `expect` there would raise a `LEGAL_APPROACH` error and block team deploy. When the partner-aware version exists, add a planner-only `transfer` step for the partner's tip and give all four volleys `"expect": {"launchesInto": "hiveRedUpCell", "count": 4}`.

Blue Audience Cycle mirror: start (10, −63, 180°), left side on the audience wall facing red, flower3 behind. Volleys 1–2 go at the blue audience cell (down at the start solo) from (10, −63) and (24, −51). The blue-wall flower (flower2) is approached at (57, 24, 0°), volley 3 goes into the raised blue far cell from (50, 34) and tips it, then the blue garden (far wall) feeds volley 4 from (62, 63).

### Timing does not fit yet

`zenith estimate` gives **29.2 s nominal** (23.3–35.0 s), against the 29 s global deadline (`PedroConfiguration.MAX_AUTONOMOUS_SECONDS`). On top of that come the four readiness waits, which Zenith can't estimate.

- Already applied, as you chose: every collect is bounded at 2 s (it was 3–4.5 s), and readiness waits are 0.5–1 s.
- Homing now happens in INIT, but that doesn't buy time here: the start delay became 4.5 s to keep the first feed at about 5 s.
- The biggest remaining levers are measurements. `shoot` is timed at 2.5 s per volley, 10 s in total; if four balls clear in 1.5 s, that saves 4 s. Raising the 0.35 path-speed cap after Pedro tuning saves about 2–3 s over the drives (~8.7 s).
- If it still runs long, the 29 s deadline stops the robot during volley 4, the volley that doesn't score solo. The deadline stops outputs safely; it is not a fault.

## Autonomous tag relocalization

Implemented 2026-11-18 for every full-robot Pedro/Zenith autonomous (generated red entries and the blue wrappers). It does not apply to the fallback autos or the pollen test. To turn it off, set `BioBuzzRobot.TAG_RELOCALIZATION_ENABLED = false`; autonomous then aims from odometry only, as before.

### INIT (robot stationary)

1. Ivy writes the authored start pose (mirrored for blue) into Pinpoint once it is READY. It does this once, during INIT, and START does not write it again.
2. The turret homes during INIT. Its constructor starts homing, as in TeleOp.
3. `AutoStartRelocalization` requests the existing corner refinement (`FieldRehomeRefinement`) toward a landmark chosen from the known start state. Only raised cells carry usable tags. The landmark is the cell on the robot's half that is raised at the start of every match:

   | Routine start | Landmark | Why |
   | --- | --- | --- |
   | Red Audience, Blue Audience (all variants) | red audience cell, tags 34–37 | blue's audience cell starts lowered |
   | Red Far (all variants), Blue Far | blue far cell, tags 42–45 | red's far cell starts lowered, so tags 30–33 are rejected |

4. `ShotCoordinator.AimMode.AUTONOMOUS_RELOCALIZING` points the turret at that landmark and fuses 5 consistent, stationary frames. The correction is translation only, at most 0.15 m, with a 3 s timeout.
5. If refinement succeeds, `BioBuzzRobot` writes the refined pose back into Pinpoint once (`PedroDrive.setFieldPose`), so Pedro's paths and the aim share it. Telemetry shows `Start relocalization: REFINED from hive tags (moved x.x in)`.
6. If it fails or times out, the start pose stays, telemetry shows `START POSE ONLY (attempt n): <reason>; retrying`, and the refinement is requested again until START. If the last attempt saw tags more than 6 in from the start pose, telemetry adds **`CHECK PLACEMENT`**. The robot never jumps to such a camera pose; re-place the robot instead.

### START and the route

- START cancels a refinement that is still collecting, because a pending refinement blocks feeding. The current pose stays.
- During the route, the same mode applies TeleOp's periodic corrections to the **aiming** pose: at most one per 0.5 s, mapped raised tags only, with the existing capture-age, height and jump checks. Pedro keeps following Pinpoint, so the path never jumps mid-drive. Mid-route corrections therefore improve the shots, not the path.
- Pose handoff to TeleOp still saves the Pinpoint pose, which includes the INIT refinement.

### Tests and open items

- `AutoStartRelocalizationTest` (JVM, 5 tests) covers:
  - the landmark for all four starts and their tag IDs
  - the wait → request → retry → `CHECK PLACEMENT` → apply-once sequence
  - the meters-to-inches write-back
  - refinement at the Red Far start from the blue far tags, with a 0.30 m misplacement rejected and the start pose kept
  - the relocalizing mode aiming the turret at the chosen landmark (not red's nearest cell) while odometry-only mode ignores requests
  - START cancelling refinement
- Not covered: a full `BioBuzzRobot` INIT loop with a simulated Limelight. The repository has no fake Limelight, so the write-back path is only unit-tested in pieces.
- On the robot: INIT at each of the four start spots and read `Start relocalization`. Move the robot about 3 in and check the reported correction. Move it about 10 in and check for `CHECK PLACEMENT`. Cover the camera and check that `START POSE ONLY` keeps the start pose.
- **Rule check still open:** the turret moves during INIT (homing and the landmark look), as it already does in TeleOp. Confirm in the game manual that this is allowed in autonomous.
- Red Far's blue-far tags are near the edge of the camera's view. The look points the turret at them, but check that frames are actually accepted.
- If START comes in the same loop as the write-back, Pinpoint is briefly not READY and the existing start check refuses to run. That window is one loop (~20 ms).
- The original Zenith implementation plan ([ZenithAutonomousImplementationPlan.md](ZenithAutonomousImplementationPlan.md)) said not to enable TeleOp's vision correction during autonomous routes. This change does so deliberately, at the team's request, and keeps the off switch.
- Simulation decision: omitted. SimLoop covers TeleOp only and has no autonomous scenarios. TeleOp's own path through `ShotCoordinator` is unchanged (the TeleOp corner rehome still uses the nearest cell), and the full unit suite covers it.

## Measure before trusting the cycle route

Everything in the earlier plan's measurement list, plus:
- the red-wall flower approach (flower0): standoff, and whether 2 s collects 4
- whether (−50, −34) is a clean legal audience-side spot
- the 68.7 in garden shot. The shot table must now cover about **35–69 in**.
- actual feed time for 4 balls, which is the main timing lever

## Verification

- `./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`: 319 tests, 0 failures, 0 errors. The debug APK builds and contains the three route assets.
- `npm run zenith:validate`: 0 errors. `npm run zenith:test`: 13/13, including a new mirror-wrapper test. `npm run zenith:deploy` generated the three red entries and assets, and `npm run zenith:check` is clean.
- `scripts/check-zenith-apk.ps1` was not run (no PowerShell on this Mac); the APK's asset list was checked by hand instead.
- Nothing has been run on the robot.
