# Pedro Pathing 3: Robot Setup and Auto Guide

*How to get a robot ready for Pedro, and the rules for writing every Pedro Auto after this one. Status as of 2026-11-04: the code is in the repo and compiles against the real libraries, but **nothing has run on a robot yet**. A `[#nn]` marker (the number of a GitHub Issue) marks things only a robot test can settle. If this doc and Pedro's own docs disagree, Pedro's docs win: https://pedropathing.com/ and https://github.com/Pedro-Pathing*

## 1. What is installed

| Piece | Where | Notes |
|---|---|---|
| Pedro libraries | `build.dependencies.gradle` | `com.pedropathing:revhub:3.0.2` (path follower and REV hub support) and `com.pedropathing:tuning:1.0.3` (AutoTune). Copied from the official Quickstart. |
| `Constants.java` | `teamcode/pedro/` | The Foresight numbers, plus the drivetrain and Pinpoint settings **read from** `DriveSubsystem` and `PinpointSubsystem`, and `Constants.create(hardwareMap)` that builds the Follower. |
| `Tuning.java` | `teamcode/pedro/` | The menu for the AutoTune web page. |
| `procedures/` | `teamcode/pedro/procedures/` | Pedro's own tuning code, copied unchanged from the Quickstart. **Do not edit it.** It uses lambdas and `sleep()`; that is fine, it is not our code. |
| `PedroForwardBackAlpha` | `teamcode/` | The first Auto: forward 24 in, pause 1 s, back to the start. |

**Pedro 3 is a full rewrite.** Most tutorials, videos, and old team code online are for Pedro 2 (`FollowerConstants`, `PathChain`, `followPath()`, `Constants.createFollower()`). **None of that exists in Pedro 3.** In Pedro 3 you use `Paths.line()`, `follower.follow(path)`, `PoseFactory`, and `Foresight`. When you search, check the Pedro 3 docs.

## 2. One-time robot setup (do this for each robot: practice bot, 11940, 22091)

1. **Cables.** Pinpoint on an I2C port that is **not port 0** (the Control Hub's own IMU uses port 0). Pinpoint sticker side (the side with the ports) **up**. Forward pod in the **X** port, sideways pod in the **Y** port.
2. **Robot configuration.** Names must match exactly: `pinpoint`, `left_front`, `left_rear`, `right_front`, `right_rear`. Same names as `DriveSubsystem` and `PinpointSubsystem`, so no new config entries are needed.
3. **Gradle sync.** The first sync downloads Pedro. If it fails, read the red error:
   - "Could not resolve com.pedropathing..." means the computer is offline or `repo.dairy.foundation` is blocked. Try another network.
   - A `compileSdk` error means Android Studio wants a newer one. `CLAUDE.md` (Dev environment) says what to do. We did not need it when this was written, because Pedro's libraries accept any `compileSdk`.
4. **Deploy** to the Control Hub, then connect the laptop to the robot's wifi.
5. **Open AutoTune:** http://192.168.43.1:10158 in a browser. The procedures come from `Tuning.java`.
6. **Fresh battery, robot on the floor** (not on blocks), a clear space of at least 8 feet in front and to the left, and someone ready to stop the OpMode.

## 3. Tuning order

Do these in order. Stop and fix the first one that fails. Only the Foresight procedure's "Java" tab gets pasted into `Constants.java`. The drivetrain and Pinpoint settings live in `DriveSubsystem` and `PinpointSubsystem`.

| # | AutoTune procedure | What it checks | What to paste |
|---|---|---|---|
| 1 | **Mecanum Procedure** | Each wheel spins the right way. | Nothing, if it agrees with `DriveSubsystem`. If it finds different directions, change them **in `DriveSubsystem`** (the one place). |
| 2 | **Pinpoint Procedure** | Pod type, directions, and pod offsets. When asked, choose **Swing Arm**. | Nothing, if it agrees with `PinpointSubsystem` (offsets -3.75 and 0.0, forward pod FORWARD, sideways pod REVERSED). If it finds different numbers, change them **in `PinpointSubsystem`** (the one place); TeleOp and Auto both read from there. |
| 3 | **Tests > Localization** | Push the robot by hand: forward must make X go up, left must make Y go up, turning left must make heading go up. Push it 48 inches and check X. | Nothing. This also settles the open `DISTANCE_CALIBRATION_MULTIPLIER` question in `PinpointSubsystem`. |
| 4 | **Foresight Procedure** | The long one. **The robot drives itself.** Measures top speed, braking, and the correction gains. | `foresightConfig`: replace the whole example block. |
| 5 | Set `FORESIGHT_TUNED = true` in `Constants.java` | | Only after step 4 is pasted. |
| 6 | **Tests > Line Test** (and Hold Test) | Rebuild and deploy first (step 5 switches these tests on). Line Test drives forward and back 48 in until you stop it. Hold Test fights you if you push the robot. | Nothing. |

Write each tune in `biobuzz-test-and-match-log.md` (date, robot, battery voltage). **Every robot needs its own Foresight numbers.** The practice bot's numbers are not 11940's. Re-tune after any change to the wheels, the robot's weight, or the drivetrain.

## 4. Running `PedroForwardBackAlpha`

1. Tuning done and `FORESIGHT_TUNED = true`. (Until then the OpMode refuses to drive and says why on the screen.)
2. Put tape on the floor at the front of each wheel. Place the robot on the floor, facing a clear 3 feet, and leave it still.
3. INIT "Pedro Forward Back - Alpha". INIT recalibrates the Pinpoint, so the robot must not move. Push the robot by hand now if you want: the pose on the screen should change.
4. START. It drives forward 24 in, holds 1 second, and drives back.
5. **Pass:** back within about 1 inch of the tape, facing the same way, **5 runs in a row**. Log each run.

If it fails, see section 7.

## 5. Rules for writing every Pedro Auto

These add to `ARCHITECTURE.md` and `biobuzz-auto-strategy.md`.

1. **Copy the shape of `PedroForwardBackAlpha`.** `init()` builds the Follower, poses, and paths. `start()` sends the first path. `loop()` calls `follower.update()` **first**, then a `switch` on an `enum` of steps, then telemetry.
2. **One path per step; the step ends when `!follower.isBusy()`.** `isBusy()` stays true until the robot has arrived *and* settled. If you want to start the next move earlier (arrived but still moving), use `follower.atParametricEnd()` instead, and test it.
3. **Never call `sleep()`.** Use an `ElapsedTime` for pauses and waits. A shoot step is a state that waits for "shooter at speed" and "turret aimed" (A-2 and up).
4. **Build paths once, in `init()`,** never inside `loop()`.
5. **Poses are made with a `PoseFactory` from named constants.** Every X, Y, and heading is a `private static final` at the top with a comment. No numbers buried in the code.
6. **Know the coordinates.** Pedro's X is "right" and Y is "up" on the field, the origin is the bottom-left corner, and heading 0 faces +X. Counterclockwise is positive. This is **not** the FTC field coordinate system. Read https://pedropathing.com/ (Reference > Coordinates) before the first real path. Our map of the BIOBUZZ field (audience at the bottom, red on the left, blue on the right, origin at the bottom-left corner of tile A1, 141.5 in square, see `TeamCode/docs/pedro-fieldcoordinates-dark.png`) and the start, waypoint and park poses are in `pedro/FieldConstants.java`. The real field width, the robot's size and its center of turning still need measuring `[#55]`.
7. **Write the blue path once, build red from it, and RED IS A 180 DEGREE ROTATION, NOT A MIRROR.** Red's LOADING ZONE is top-left and blue's is bottom-right, so the field looks the same after turning it 180 degrees around its middle, not after flipping it left to right. So `mirrorX()` is **wrong** for this field. `FieldConstants.makePose()` does it: `x' = FIELD_SIZE - x`, `y' = FIELD_SIZE - y`, `heading' = heading + 180`. (An earlier version of this rule said to use `mirrorX()`; that was a mistake.) Blue D6 (top wall, facing down) becomes red C1 (bottom wall, facing up). Red has not been driven yet `[#58]`.
8. **Start pose = where the robot really sits.** Call `follower.setPose(startPose)` in `init()` (and again in `start()`, as our scoring Autos do). If the robot is placed crooked or in the wrong spot, every path is wrong. Mark the start spot on the practice field with tape.
9. **Stop in `stop()`:** `follower.stop()` then `follower.drivetrain.stop()`, as in `PedroForwardBackAlpha`.
10. **Never use `DriveSubsystem` or `PinpointSubsystem` in an OpMode that has a Follower.** Pedro owns the drive motors and the Pinpoint. The other subsystems (shooter, turret, intake, vision) are fine, and we call their `update()` in `loop()` right after `follower.update()`.
11. **Do not copy Pedro's command library "Ivy".** Pedro's guides use it, and they recommend it over a state machine. Our rule is no command-based code (`CLAUDE.md`), so we use the `enum` and `switch`. This is a deliberate choice, not an oversight.
12. **Keep the 30 seconds in mind.** Add up the step times (`biobuzz-auto-strategy.md` section 6). A route that does not finish with spare time is the wrong route.
13. **Plan for a dead Pinpoint.** If the Pinpoint is not working at INIT, Pedro cannot drive. The drive team picks the A-0 Emergency Auto instead (`biobuzz-auto-strategy.md`).
14. **Annotation:** `@Autonomous(name = "...", group = "...")`. For match Autos add `preselectTeleOp = "TeleOp - Alpha 14"` (the name on the screen for `BiobuzzTeleOp`) so the Driver Station loads TeleOp after Auto.
15. **After a change to the robot, run `PedroForwardBackAlpha` again** before trusting any longer Auto.

## 6. Open items

- Everything above is untested on a robot `[#57]`. The first real run may show that a number or step here needs fixing; update this doc when it does.
- **One place for each number.** The motor names and directions are in `DriveSubsystem`, and the Pinpoint name, pod offsets, pod type and pod directions are in `PinpointSubsystem`. `pedro/Constants.java` reads them (they are `public static final` for that reason). Only the Foresight numbers are typed in `Constants.java`.
- `PinpointSubsystem` also has `DISTANCE_CALIBRATION_MULTIPLIER` and `INVERT_HEADING`, which fix readings in software. **Pedro does not use those two.** They are 1.0 and false now, which changes nothing. If the 48 inch push test says the distance is off, do not just change the multiplier: Pedro would not see it. Ask the coach (Pedro's Pinpoint config has a ticks-per-unit setting for that).
- The Foresight numbers in `Constants.java` are an **example from another robot** until each robot is tuned. `FORESIGHT_TUNED` stops the Auto from using them.
- Which start spots and which field coordinates each robot will use (see `biobuzz-auto-strategy.md` section 8). `[#60]`

## 7. If something goes wrong

| Symptom | Likely cause | What to check |
|---|---|---|
| OpMode says NOT TUNED and does not move | `FORESIGHT_TUNED` is still false. | Section 3, steps 4 and 5. |
| INIT crashes with "pinpoint" not found | Config name wrong, or Pinpoint on I2C port 0. | `biobuzz-troubleshooting.md` and section 2. |
| Robot drives the wrong way (backward, or sideways) | A motor direction or a pod direction is reversed. | Mecanum and Pinpoint AutoTune procedures; `DriveAlpha` for the wheels. |
| Robot drives the right way but ends short or long by a steady amount | Wrong pod type or distance multiplier. | `podType` is set to Swing Arm in `Constants.java`. Push 48 in in `Tests > Localization`. |
| It spins, wanders, or curves instead of going straight | Pod offsets wrong, Pinpoint moved during INIT, or a loose pod. | Pinpoint AutoTune, `PinpointAlpha` test 5 (turn in place). |
| Overshoots, or shakes at the end | Foresight not tuned for this robot or battery. | Re-run the Foresight procedure on a fresh battery. |
| Never finishes a step (state never changes) | `isBusy()` stays true because it cannot settle. | Watch "Distance to end of path". Check the end constraints in Pedro's docs (Reference > End Constraints). |
| Works on one robot, not another | Each robot needs its own tune. | Section 3. |

Also see `biobuzz-troubleshooting.md`.
