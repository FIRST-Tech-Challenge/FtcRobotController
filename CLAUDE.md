# CLAUDE.md — Cyber Coyotes FTC (2026-2027 BIOBUZZ)

Claude Code loads this file automatically at the start of every session. It holds the rules for working in this repo. Structure and conventions are in `ARCHITECTURE.md`; game and robot details are in `TeamCode/docs/`.

## Who this code is for

- Reed City Cyber Coyotes, a middle school FTC program with two teams, **ACME Fabrications (11940)** and **ACME Innovations (22091)**, plus a practice bot.
- The programmers are **8th graders**. Write code a 13-year-old can read, trace, and change:
  - Prefer clear, verbose code to clever code. Don't use lambdas, streams, or deep inheritance.
  - Explain *why* in comments, not just *what*. The existing `*Alpha.java` OpModes show the comment style we want.
  - Use one idea per method. Name things for what they do in the game.
- The coach reviews all work. Pushback is welcome: flag problems, risky assumptions, or a better approach instead of quietly going along.

## Session protocol (required)

1. **Read `ARCHITECTURE.md` first.** It is the source of truth for structure and conventions.
2. **Propose before coding.** Show the design (classes, states, methods, how it wires into TeleOp/Auto) and **wait for approval** before writing code unless prior authorization is already written.
3. **One subsystem class (plus its test OpMode) or one OpMode per session.**
4. **Open decisions live in GitHub Issues, not in the docs.** An open issue labeled `needs-decision` is an unresolved decision. Docs and code point to it with a `[#nn]` marker (the issue number). The issue holds the question, the options, who decides, a due date, and later the answer. The file holds only the number.
   - **Before starting work,** search the open `needs-decision` issues for any that touch the task, and look for `[#nn]` markers in the files you will change. If one blocks the work, **stop and ask**. If you can't reach GitHub, stop and ask too. Don't assume the answer.
   - **Small robot checks are `// TODO` comments in the code,** next to the constant they are about ("time this on the robot", "starting guess"). They are not decisions and they do not block work. The ones with no other issue are listed in #88. **The old `[CONFIRM]` markers are gone.** Don't add new ones: use a `// TODO` for a number to check, or an issue and a `[#nn]` marker for a decision.
   - **Draft, don't create.** When you find a decision with no issue, show the coach the draft (title, question, options, who decides, due date). Create the issue only after the coach approves.
   - **When the coach answers and closes the issue,** replace the marker with the answer (leave no marker behind) and put `Closes #nn` in the commit message.
   - GitHub may not be reachable at events (no internet in the pits), which is why the `[#nn]` pointer stays in the file.
5. When a game fact matters, check `TeamCode/docs/` first. If the docs and the Competition Manual disagree, the manual wins. Say so.

## The approach this season: standard FTC SDK style

- **No FTCLib and no command-based code** (no commands, no schedulers, no base classes like `SubsystemBase`). We follow the patterns in the FTC SDK samples.
- **Reference code:** `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/`. Start from the closest sample and keep its structure and comment style. **Don't modify anything under `FtcRobotController/`.**
- **Our code:** everything under `TeamCode/src/`.
- **Iterative OpModes** (`extends OpMode`: `init`, `init_loop`, `start`, `loop`, `stop`), like `BasicOpMode_Iterative` and our `IntakeAlpha`.
- **One subsystem class per mechanism** (`ShooterSubsystem`, `TurretSubsystem`, `IntakeSubsystem`, …) in the `subsystems/` folder. A subsystem is a plain Java class with no base class. It owns its own hardware and methods, and OpModes use the subsystems. Subsystem class names end in `Subsystem`; OpMode names don't. See `ARCHITECTURE.md`.

- **Constants live with the class they describe**, not in a central `Constants` file. Mechanism numbers go in the subsystem, test-only numbers go in the test OpMode, and a shared class is made only when a second class needs a value. See "Where constants live" in `ARCHITECTURE.md`.

## Stack

- Java, FTC SDK 12.x, **Pedro Pathing** for autonomous. The team does not use RoadRunner or FTCLib.
- Hardware: REV Control Hub + Expansion Hub, Limelight 3A, goBILDA Pinpoint odometry, goBILDA 5203 motors.
- Android build flavors deploy robot-specific OpModes to the correct Control Hub (`practiceBot`, `team11940`, `team22091`). Shared code lives in `src/main/`. The flavors are not set up in this repo yet; see `ARCHITECTURE.md`.

## Naming conventions

| Thing | Convention | Example |
|---|---|---|
| Packages | lowercase | `mechanisms` |
| Classes | PascalCase | `ShooterPowerAlpha` |
| Subsystem classes | PascalCase, ends in `Subsystem` | `TurretSubsystem` |
| Variables / methods | camelCase | `targetVelocity` |
| Hardware variables | location first | `leftIntakeServo` |
| Hardware config names | snake_case | `left_intake_servo` |
| Constants | ALL_CAPS | `MAX_TURRET_DEGREES` |
| Doc filenames | kebab-case | `biobuzz-game-essentials.md` |

## Known gotchas

These cost us time before, so don't repeat them:

- **goBILDA 5202/3/4 encoder ticks:** it is **28 counts per motor revolution** (7 pulses × 4 edges). The goBILDA spec sheets in `TeamCode/docs/` (`5203-2402-0001`, `-0003`, `-0014`) say so, and goBILDA's own sample code uses 28. Last season's notes said 112, and that was wrong. The number to use is **counts per OUTPUT-shaft revolution**, which is 28 × the gearbox ratio: **28** for the 6000 RPM 1:1 motor (hood wheel), **103.8** for the 1620 RPM 3.7:1 motor (flywheel), and **384.5** for the 435 RPM 13.7:1 motor (turret). The encoder is on the motor shaft, before the gearbox, so at full power with no load every one of these motors reads about **2,800 ticks/sec** from `getVelocity()`. That is a good quick sanity check on the robot, but the number is no longer in question.
- **Continuous rotation servos:** 0.0 = full reverse, 0.5 = stop, 1.0 = full forward. `setDirection(REVERSE)` has caused problems.
- **Limelight 3A:** needs an Ethernet device in the robot config named `limelight`. It lives at `172.29.0.1` on the Control Hub's internal subnet, not `192.168.1.11`. Exposure 1000 / gain 8 is a good starting point, and it can run at 90 FPS.
- **FTC SDK 12.0 breaking change:** `AprilTagDetection` may be a single tag or a **cluster**. Check with `instanceof AprilTagSingleDetection` / `AprilTagClusterDetection` and cast. Older AprilTag code will not compile unchanged.
- **Flywheels:** use `FLOAT` zero-power behavior, never `BRAKE`, on a heavy flywheel.
- **Check the cables before the code.** Motors or sensors plugged into the wrong port are the most common time-waster.

## Dev environment

- Android Studio Narwhal 3 Feature Drop or later (the SDK needs AGP 8.13.2 / Gradle 9.1). If Android Studio offers to downgrade AGP, say no.
- On Windows, deploying may need Android Studio run as administrator. The permanent fix is to set the `GRADLE_USER_HOME` environment variable.
- The TeamCode `build.gradle` may need `compileSdk 36` to override a `build.common.gradle` mismatch.

## Docs index

- `ARCHITECTURE.md`: structure, patterns, and rules for all team code
- `TeamCode/docs/biobuzz-game-essentials.md`: scoring, ranking points, and the rules that affect code and design
- `TeamCode/docs/biobuzz-hive-apriltag-geometry.md`: hive and cell dimensions, tag IDs and layout, camera math
- `TeamCode/docs/biobuzz-robot-design.md`: current robot concept (turret shooter + Limelight), build order, and open decisions
- `TeamCode/docs/biobuzz-auto-strategy.md`: Autonomous plan (the build ladder A-0 to A-5), what AUTO is worth, and facts still needed
- `TeamCode/docs/pedro-setup-and-auto-guide.md`: Pedro Pathing 3 robot setup, tuning order, and the rules for writing every Pedro Auto
- `TeamCode/docs/biobuzz-game-strategy.md`: match strategy, ranking points, scoring cycle, timeline, and drive team roles
- `TeamCode/docs/biobuzz-match-day-checklists.md`: pit, field, and match checklists
- `TeamCode/docs/biobuzz-troubleshooting.md`: symptom, cause, and what to check
- `TeamCode/docs/biobuzz-test-and-match-log.md`: where tuned numbers live, plus test, match, and scouting log templates

## Heads up

2026-2027 is the **last season** of FTC Java in its current form. In 2027-2028 FTC moves to SystemCore with more standard OpModes, which is one reason we're using the standard SDK style now.
