# Agent Instructions: FTC TeamCode (BIOBUZZ 2026-27)

Read this file fully before changing any code. If something here conflicts with what you see in the code, trust the code, then tell the user so this file can be fixed.

Items marked **[VERIFY]** are best guesses that the team should confirm or correct.

---

## 1. Project at a glance

- **What:** FTC (FIRST Tech Challenge) robot code, Java, FTC SDK.
- **Season:** BIOBUZZ presented by RTX (2026-27, part of FIRST CANOPY).
- **Code lives in:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- **Autonomous:** uses the Pedro Pathing library. **Currently NOT working.** This is the top priority.
- **TeleOP:** **Working. Do not break it.**

### Game context (BIOBUZZ presented by RTX, 2026-27)
- **Match Structure:** 2 Alliances of 2 teams. The match consists of a 30-second Autonomous (AUTO) period, an 8-second transition period, and a 2-minute Teleoperated (TELEOP) period.
- **Scoring Elements:** The robot interacts with **POLLEN** (plastic balls ~3 in.) and **NECTAR** (colored elements). The robot must collect these to score.
- **Field Setup:** There are 40 POLLEN on the field (4 in each of the 4 FLOWERS = 16, 4 in the red GARDEN, 4 in the blue GARDEN, and 4 pre-loaded in each robot). NECTAR is staged in the cells of the HIVE and in the Alliance Areas.
- **Primary Objectives:**
  - **HIVE TIP:** (20 pts AUTO/20 pts TELEOP). Achieved by launching enough scoring elements into the HIVE's CELLS to flip it to its other stable side (the damper must contact the frame). Tipping the HIVE unlocks more NECTAR for the Alliance.
  - **FLOWERS:** (2 pts per element + 5 pt Bottom Nectar Bonus). Placing NECTAR and POLLEN into FLOWERS. An alliance "owns" a FLOWER if their NECTAR is the top-most placed.
  - **GARDEN:** (1 pt per element). Placing elements in the GARDEN area.
  - **LEAVE / PARK:** LEAVE (3 pts AUTO only) = robot off the perimeter wall. PARK (5 pts AUTO & TELEOP) = robot partially in the LOADING ZONE.
- **Ranking Points (RP):** Win (3 RP), Tie (1 RP). SWARM RP (LEAVE + PARK points >= 16). POLLINATOR 1 RP (4+ TIPS). POLLINATOR 2 RP (7+ TIPS).
- **Robot Constraints (Key Rules to Remember):**
  - Pre-load: 4 POLLEN.
  - Start: Must touch the perimeter wall. Cannot start in the LOADING ZONE or touching a FLOWER.
  - Control Limit: Max 4 NECTAR + POLLEN at a time (G407). Do not control opponent's NECTAR (G408).
  - No catching scoring elements (G409). Expansion limits apply (R105, G415).
- **Coding Impact:** The robot has a drivetrain, an intake and an outtake. Most game-related code is about moving (`Drivetrain`), picking up pollen/nectar (`Intake`), and scoring it into the HIVE or FLOWERS (`Outtake`).
- **Do not guess scoring values or rules from memory.** If a task depends on specific scoring values, field dimensions or legal robot sizes, ask the user for the current Game Manual numbers.

---

## 2. Folder map (where things go)

All paths below are inside `teamcode/`.

| Path | What belongs here |
|---|---|
| `RobotConstants.java` | Tunable numbers and config values for **non-Pedro** code: thresholds, speeds, servo positions, hardware names. |
| `RobotVars.java` | Shared **mutable** state (e.g. current mode, alliance flags). Keep it small. Don't turn it into a dumping ground. |
| `mechanisms/` | One class per physical subsystem: `Drivetrain`, `Intake`, `Outtake`, `Limelight`, `Stopper`, plus `MecanumStateHandler` (mecanum drive math helper). **All hardware control logic lives here.** |
| `opmodes/` | OpMode infrastructure. `OpModeCommand.java` is a base class/interface. |
| `opmodes/teleop/` | TeleOP. `Drive_Code.java` is the main driver-controlled OpMode. `RobotDummy1` and `SensorGoBildaPinpointExample` are examples or tests. |
| `pedroPathing/Constants.java` | **Pedro-specific** configuration (follower, drivetrain, localizer, path constraints). Don't mix these with `RobotConstants`. |
| `pedroPathing/Tuning.java` | Pedro tuning OpMode(s). |
| `pedroPathing/procedures/` | Tuner classes (`PinpointTuner`, `ThreeWheelTuner`, `TwoWheelTuner`, `OTOSTuner`, `MecanumTuner`, `ForesightTuner`). Used for calibration, not for matches. |
| `pedroPathing/auto/` | Autonomous OpModes (see section 5). |
| `utils/commands/` | Small helpers: `AllianceColor`, `GamepadController`. |
| `utils/opmodes/` | Currently seems to be an empty or duplicate folder. **[VERIFY]** Don't add files here without asking. |

### Rules for adding code
1. **Hardware control goes in `mechanisms/`.** OpModes should call mechanism methods (`intake.start()`), not talk to motors or servos directly.
2. **A new physical subsystem gets a new class in `mechanisms/`**, following the style of an existing one (open `Intake.java` or `Outtake.java` first and copy its structure).
3. **Numbers go in constants, not inside methods.** Use `RobotConstants` for general code and `pedroPathing/Constants.java` for Pedro settings.
4. **New autonomous routines go in `pedroPathing/auto/`.** New TeleOP code goes in `opmodes/teleop/`.
5. **Reuse before you create.** Search for an existing helper before writing a new one. Don't add a new pattern, folder or library if an existing one does the job.
6. **If you're unsure where something belongs, ask.** Don't invent a new location.

### Conventions
- Java naming: `PascalCase` classes, `camelCase` methods and variables, `UPPER_SNAKE_CASE` constants.
- Existing files have inconsistent names (`Drive_Code`, `driveForwardAuto`, `fourPollenAutoRowen`). **Don't rename existing files unless asked**, because OpMode names and Driver Hub selections may depend on them. Use PascalCase for new files.
- Comments should explain **why** (e.g. "slower here so pollen doesn't bounce out"), not restate what the code does.

---

## 3. Protecting TeleOP

TeleOP works, and the team may need it at any time.

- **Do not edit files in `opmodes/teleop/` unless the task is explicitly about TeleOP.**
- **Mechanisms are shared** between TeleOP and Auto. Before changing any file in `mechanisms/`, search for everything that calls it (`Grep` the class and method names).
  - Prefer **adding** new methods over changing existing signatures or behavior.
  - If a change would alter what TeleOP does, **stop and tell the user first.**
- Same for `RobotConstants`, `RobotVars` and `utils/`: changing a value or signature affects every user of it.
- In your final summary, always list which shared files you touched and what could be affected.

---

## 4. Hardware config names

Motor, servo and sensor names in code must match the **Driver Hub robot configuration** exactly. Never rename or invent a hardware name. If a task needs a new device, tell the user which name to add to the configuration.

---

## 5. Autonomous (Pedro Pathing): the broken area

### Current autos in `pedroPathing/auto/`
- Current season (pollen): `ZeroPollenAuto`, `FourPollenAuto`, `fourPollenAutoRowen`
- Team autos: `Hawkbotics`, `MoreHawkbotics`
- Simple/test: `driveForwardAuto`, `ExampleAuto`
- **Likely leftovers from last season's DECODE game:** `ThreeArtifactAuto`, `TwelveArtifactAuto` **[VERIFY]**. Don't delete them. Don't build on them without asking.

### How to approach "Auto doesn't work"
Do **not** guess. Work from the symptom, and ask the user what it is if they haven't said (robot doesn't move / moves wrong direction / drifts / oscillates / crashes at init / doesn't follow the path).

**CRITICAL RULE: Always reference the current Pedro Pathing documentation (currently Pedro Pathing 3) for ANY Autonomous-related code.** The API changes between versions, and writing code from memory is a primary cause of broken autos. If the docs are not available, state that explicitly and ask the user for a link or a local copy.

Check these in order, since most Pedro failures are configuration, not path logic:
1. **Version and docs.** Check which Pedro Pathing version is in the Gradle files and read that version's documentation. Do not write Pedro code from memory. If the docs aren't available to you, say so.
2. **`pedroPathing/Constants.java`.** Check the follower setup, drivetrain/motor names and directions, localizer choice, and path constraints.
3. **Localizer.** Which one is used (Pinpoint, OTOS, three-wheel, two-wheel, drive encoders)? Does the configuration (device name, pod offsets, encoder directions, units) match the real robot? A wrong localizer is the most common cause of "robot goes the wrong way or drifts."
4. **Tuning.** Have the tuners in `procedures/` actually been run for the current robot? Tuned values go in the constants, not in the auto OpModes.
5. **The auto OpMode.** Check starting pose, alliance mirroring, the path sequence and the state transitions. Check that `follower.update()` is called every loop.
6. **Mechanism interaction.** Check whether intake/outtake calls block or interfere with path following.

### Rules for working on Auto
- **ALWAYS reference the current Pedro Pathing documentation** (currently Pedro Pathing 3) before touching any file in `pedroPathing/`. Do not rely on memory for API syntax or configuration.
- Make the **smallest change** that tests one hypothesis. Don't rewrite whole autos.
- Use a clear **state machine** (enum + switch) for sequences, with no long blocking loops or sleeps.
- Don't change tuning values or poses "to see what happens" without saying so in your summary. The team can only test on the physical robot.
- Fix the shared cause (Constants, localizer) before editing individual autos.

---

## 6. Workflow for every task

1. **Understand the request.** If it's ambiguous or high-risk, ask before coding.
2. **Inspect first.** Read the relevant files. Search for usages (`Grep`). Never change code you haven't read.
3. **State a short plan:** which files, what change, and why.
4. **Make focused edits.** One concern per change. Don't reformat or refactor unrelated code.
5. **Check your work:**
   - Does it compile? (Check types, imports, signatures; run the build if you can.)
   - Did you change anything in TeleOP or shared mechanisms? Re-check every caller.
   - Are hardware names and constants consistent?
6. **Report back** with:
   - What changed, and in which files
   - What you were unsure about
   - **What must be tested on the robot** (you can't test hardware)

### Don't
- Don't change Gradle files, SDK versions or dependencies without asking.
- Don't delete files or old autos without asking.
- Don't commit secrets, tokens or personal data.
- Don't claim something works if it hasn't been run on the robot. Say "untested."

---

## 7. Notes for the team (delete when done)

- Fill in: team name and number, Pedro Pathing version, which localizer the robot uses, and what exactly "Auto doesn't work" looks like.
- Check the **[VERIFY]** items above.
- Keep a short list of mechanism names and hardware config names here so agents don't have to guess.
