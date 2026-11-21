# Creating and testing autos with Zenith: a beginner's guide

This guide is for someone who has never written an FTC autonomous before. It walks through what the pieces are,
how to set up your laptop, how to build a routine in the Zenith editor, and how to test it safely, from your desk
to a full-speed run on the field. When you need the exact rules for one action, the
[Zenith authoring guide](ZenithAuthoringGuide.md) is the reference.

## 1. The big picture

During the first 30 seconds of a match the robot drives itself. We call that program an **auto**.

On this robot an auto is **not** hand-written Java. It is a JSON file in `autos/` that you draw and edit in
**Zenith**, a visual path editor. The chain looks like this:

```
Zenith editor  ──save──►  autos/my-auto.auto.json  ──npm run zenith:deploy──►  generated OpMode + assets
                                                                                   │
Driver Station  ◄──select & run──  Robot Controller app  ◄──build & install──────┘
```

| Piece | What it does |
| --- | --- |
| **Zenith** | The editor. You place a start pose, draw paths, and add *actions* (shoot, intake, wait...) in order. |
| **Pedro Pathing** | The library that actually follows your paths using odometry (where the robot thinks it is). |
| **Ivy** | The command scheduler. It runs your steps one after another or at the same time. |
| **`AutonomousCommands.java`** | The bridge. Every action you see in Zenith (`volley`, `intakeOn`, ...) is defined here and calls the same subsystem code TeleOp uses. |
| **`autos/robot.json`** | The action catalog Zenith shows you: names, parameters, units, limits. |

You can build a complete match auto without touching Java. You only need Java when the robot gets a new
mechanism or a new kind of action.

### Shooting works exactly like TeleOp

Since 2026-11-21, every shooting action in Zenith runs the same code as the final competition TeleOp: the same
shot table, flywheel control, turret correction and continuous feed. The one you will use most is **`volley`**,
the full volley procedure the drivers get from RT + LT:

1. aim the turret at the target cell and set hood and flywheel speed from the shot table,
2. wait until the turret is on target, the hood is settled and the flywheel is at speed (and stays that way briefly),
3. open the gate and run the transfer together for `feedSeconds`,
4. finish, then keep aiming, drop to idle speed, or spin down (your choice).

If the robot is not ready in time, `volley` either fires anyway (only if the turret knows where it is pointing) or
skips the shot.

## 2. One-time laptop setup

You need Windows (the team scripts are PowerShell), and about an hour for the downloads.

1. **Git** and a GitHub account with access to `PhoenixForceFTC/BioBuzz10100`.
2. **Android Studio** (Narwhal 3 Feature Drop or newer). Let it install the Android SDK it asks for.
3. **Node.js 22 or newer** (for the Zenith command-line checks).
4. **The Zenith v0.1.2 desktop editor** from the
   [official release page](https://github.com/Horizon-36596/zenith/releases/tag/v0.1.2). Use v0.1.2 exactly;
   the project is pinned to it.
5. Clone and switch to the team branch:

   ```powershell
   git clone https://github.com/PhoenixForceFTC/BioBuzz10100.git
   cd BioBuzz10100
   git checkout V1
   npm ci --ignore-scripts
   ```

6. Open the folder in Android Studio and let Gradle sync finish (the first sync downloads a lot).
7. Check everything works before you change anything:

   ```powershell
   npm run zenith:validate
   .\gradlew.bat :TeamCode:testDebugUnitTest
   ```

   Both should finish without errors. If they don't, fix that first; it is much harder to find later.

8. Connect once by hand to the robot's Wi-Fi (`10100-RC`) so Windows remembers the password. The deploy script
   switches networks for you after that.

## 3. Field coordinates (read this once, carefully)

Everything in Zenith is in **inches**, measured from the **center of the field**:

- **+X** points toward the audience's right, **+Y** points away from the audience.
- Headings are in **radians**, counterclockwise, with 0 meaning "facing +X". So 0 faces audience-right,
  π/2 (1.571) faces away from the audience, π (3.142) faces audience-left, -π/2 faces the audience.
- The field runs from about -72 to +72 inches in both directions.

We author routines **for Red**. A Blue auto runs the same file **point-mirrored**: every point (x, y) becomes
(-x, -y) and the heading turns 180°. That is why `aim` and `volley` have a `frame` setting: with
`frame = AUTHORED`, "AUDIENCE" and "FAR" swap automatically on Blue, so the robot still shoots at the cell you meant
when you drew it. Use `AUTHORED` for match routines.

## 4. Anatomy of an auto

Open `autos/integration-example.auto.json` in Zenith to see a working example. Every auto has:

- **name**: must equal the file name without `.auto.json`, lowercase with dashes (`red-far-cycle`).
- **title**: what appears on the Driver Station.
- **alliance**: `RED` for our routines.
- **start pose**: where the robot sits at START. Measure this on a real field.
- **steps**: the list of things to do. A step is one of:

| Step kind | Use it for |
| --- | --- |
| **path** | Drive a line or curve. Set its speed and a `timeoutS` so it can't drive forever. Paths can carry **markers** that fire an action partway along (for example `intakeOn` at 50%). |
| **command** | Run one action (`volley`, `intakeOn`, `depollinatorDeploy`...). |
| **wait** | Wait a number of seconds, or until a condition is true (always give it a `timeoutS`). |
| **parallel** | Run several steps at the same time, for example drive while the turret pre-aims. |
| **sequence** | Group steps in order, often inside a parallel or branch. |
| **branch** | Do one thing or another depending on a condition, for example "if the last action failed, park". |

## 5. Build your first auto, step by step

Goal: start on the wall, drive to a shooting spot while aiming, fire one volley, drive to park.

1. **Open the project.** Start Zenith, choose *Open folder*, pick the `BioBuzz10100` folder (the one with
   `zenith.json`). The existing routines appear.
2. **New auto.** Click *New auto*, then save with **Ctrl+S**. It writes `autos/new-auto.auto.json`.
   Rename the file to something meaningful, for example `red-practice-volley.auto.json`, and change `name`
   inside it to `red-practice-volley` and `title` to `PRACTICE: Red volley`. Reopen it in Zenith.
3. **Set the start pose.** Drag the robot or type X, Y and heading. Use your measured start position.
4. **First step: `stopAll`.** A safe, known starting state.
5. **Pre-aim while driving.** Add a **parallel** step containing:
   - a **path** from the start to your shooting spot. Keep the speed low at first (30 to 50%) and give it a
     timeout a bit longer than the editor's estimate.
   - a **command** `aimNearest` with `spin = true` (or `aim` with `side`, `frame = AUTHORED`). The turret starts
     tracking and the flywheel spins up while you drive, so the volley is ready sooner.
6. **Shoot.** Add a **command** `volley`:
   - `target`: `NEAREST` (TeleOp's rule) or `AUDIENCE` / `FAR` with `frame = AUTHORED`
   - `feedSeconds`: 2.5 to empty the robot (it counts time, not balls)
   - `readyTimeoutSeconds`: 1 to 3
   - `onTimeout`: `FIRE` in a match, `SKIP` while you are still testing aim
   - `after`: `IDLE` if you will shoot again soon, `OFF` at the end
   - set the step's `timeoutS` a little above `readyTimeoutSeconds + feedSeconds` (for example 5.6 for 3 + 2.5)
7. **Check it worked.** Add a **branch** on `commandFailed`: in the *then* arm, `stopAll`; in the *else* arm,
   carry on. (`commandFailed` is true when the volley timed out, skipped or was cancelled.)
8. **Park.** Add a final **path** to your parking spot, then a last `stopAll`.
9. **Save** (Ctrl+S). Read the editor's warnings: `TIMEOUT_TIGHT`, `WAIT_NO_TIMEOUT` and similar messages are
   worth fixing now.

A minimal shooting fragment looks like this in the file:

```json
{ "id": "driveAndAim", "kind": "parallel", "mode": "all", "steps": [
    { "id": "toShot", "kind": "path", "...": "drawn in the editor" },
    { "id": "preAim", "kind": "command", "name": "aimNearest", "args": { "spin": true } }
] },
{ "id": "volley1", "kind": "command", "name": "volley",
  "args": { "target": "NEAREST", "feedSeconds": 2.5, "readyTimeoutSeconds": 3,
            "onTimeout": "FIRE", "after": "IDLE" },
  "timeoutS": 5.6 }
```

### The actions you'll use most

| You want to... | Use |
| --- | --- |
| Shoot like TeleOp | `volley` |
| Pre-aim and spin up while driving | `aimNearest` or `aim` (in parallel with a path) |
| Nudge the aim if shots land consistently off | `aimTrim` (yaw ±10°, hood ±5°, RPM ±500), before the volley |
| Collect | `intakeOn` + `loadOn` before a path (end the path on `queueFull` if you like), then `intakeOff` |
| Collect until full, with a time limit | `loadUntilFull` |
| Flower/depollinator | `depollinatorDeploy`, then wait on `depollinatorSettledEstimated`; `depollinatorHome` after |
| Linear slide / shot blocker | `slideRaise`, `slideLower`, `waitSlide` |
| Keep the flywheel warm between volleys | `flywheelIdle` |
| Stop everything | `stopAll` |
| Wait | a **wait** step: seconds, or a condition with a `timeoutS` |

Rules that save you from confusing bugs:

- Don't put two actions that control the same mechanism in the same parallel group (for example `volley` and
  `flywheelSetRpm`). The editor lists each action's `requires`.
- Don't put a drive action (`driveHold`, `setFieldPose`) on a path marker; the path already owns the drive.
- `stopAll` does not end the routine. Anything after it still runs.
- The whole auto is cut off at 29 seconds no matter what.

## 6. Check, deploy, build, install

After saving in Zenith, from the repository folder:

```powershell
npm run zenith:validate        # catches bad names, arguments, missing timeouts
npm run zenith:deploy          # generates the OpMode and copies the files the robot reads
npm run zenith:check           # confirms everything generated is up to date
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

Do **not** use the Deploy button inside the Zenith editor. It runs a different generator; `npm run zenith:deploy`
is the team workflow.

Then install on the robot:

- First install, or after changing `autos/` files, libraries or any `build.gradle`: `.\tools\deploy.ps1 -Full`
  (or the **Robot - Full Install** run configuration in Android Studio). Auto JSON files are app assets, so they
  need a full install.
- Java-only changes afterward: `.\tools\deploy.ps1` (Sloth hot reload, much faster).

Your routine appears on the Driver Station under the **Generated** group with its `title`. Blue versions of
match routines are small hand-written wrappers in group **Competition** (see `BlueFarAutonomous.java`): they run
the red file with `BioBuzzRobot(TurretSubsystem.Alliance.BLUE)`.

Commit the `.auto.json` file **and** everything `zenith:deploy` changed. Never hand-edit files in
`TeamCode/src/main/assets/autos/` or `zenith/generated/`; they are overwritten.

## 7. Testing ladder: desk to match

Move up one rung at a time. If a rung fails, go back down.

**Rung 0: one-time robot prerequisites.** The robot refuses to START any auto until Pedro is tuned:
`PedroConfiguration.TUNING_VERIFIED` is `false` today on purpose. Tuning is done with Pedro's web tuner
(see [Autonomous.md](Autonomous.md)) and the `Test: Pedro Drive` OpMode; only flip the flag after those numbers
are measured and checked. Also confirm the start pose and shooting spots are measured on a real field; the draft
routines say "placeholder" for a reason.

**Rung 1: desk checks.** `zenith:validate` and the unit tests pass. Read the editor's timeline: does the total fit
in 29 seconds even if every wait runs to its timeout?

**Rung 2: INIT only.** Select the auto, press INIT, don't press START. Telemetry should show the hardware profile,
`Start relocalization` progress (the Limelight looks at hive tags and refines the start pose), and no
`CHECK PLACEMENT` warning. If it says CHECK PLACEMENT, the robot is not where the file says it starts.

**Rung 3: wheels off the ground.** Put the robot on a stand, run the auto, and watch the mechanisms: intake, turret
aiming, flywheel spinning up, gate and transfer during the volley. Use `onTimeout = SKIP` here if you don't want
balls flying.

**Rung 4: on the floor, slow, cleared area.** Path speeds around 30%, one driver's hand on STOP. Check the robot
drives the shape you drew and stops where you expect. Watch the **Volley** and **Last bounded action** telemetry
lines: they tell you whether each volley fed after READY (SUCCESS), fed early or skipped (TIMEOUT), or was
cancelled.

**Rung 5: real speed, real game pieces.** Raise path speeds step by step. Check shots land; if they're
consistently off, fix the start pose or odometry first, and only then consider a small `aimTrim`.

**Rung 6: match rehearsal.** Run it from the real start position, with a timer, several times in a row. Test STOP
in the middle of a run: everything should stop. Then run the matching TeleOp right after; it receives the
auto's final pose.

### Reading the auto's telemetry

| Line | Meaning |
| --- | --- |
| Autonomous step / remaining | Which step is running and how many seconds are left. |
| Last bounded action | The last action that can succeed or fail, with SUCCESS / TIMEOUT / INHIBITED / FAULT / CANCELLED and why. |
| Volley | The volley phase: AIMING, FEEDING, DONE or SKIPPED, with a note such as "fed before READY (timeout)". |
| Output modes yaw / hood / flywheel | AUTO means aiming is in control; MANUAL means a fixed value you set. |
| Targets yaw / hood / flywheel | What the turret, hood and flywheel are being told to do. |

## 8. Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| START throws "Measure and verify PedroConfiguration" | Pedro tuning isn't verified yet (rung 0). |
| START throws "Pinpoint has no READY, fresh autonomous start pose" | Pinpoint not ready: wait in INIT longer, check its wiring. |
| `zenith:validate` says "condition waits require timeoutS" | Give every condition wait a `timeoutS`. |
| My new auto isn't on the Driver Station | You didn't run `npm run zenith:deploy`, or you installed with hot reload instead of a full install. |
| Volley always says "SKIPPED: turret not homed" | The turret never homed; check the turret encoder and look at INIT telemetry. |
| Volley always says "fed before READY" | Readiness wasn't met in time: flywheel too slow to spin up, robot still moving, or the pose is off. Pre-aim in parallel with the drive and give it more `readyTimeoutSeconds`. |
| The robot shoots at the wrong cell on Blue | Use `frame = AUTHORED` for named sides. |
| Two steps fight over a mechanism or one gets cancelled | They require the same mechanism and are in the same parallel group. |
| Slide actions do nothing | The slide isn't configured on this robot; telemetry says "SKIPPED: linear slide not configured". |

## 9. Where to go next

- [Zenith authoring guide](ZenithAuthoringGuide.md): every action and condition, with exact behavior.
- [Autonomous.md](Autonomous.md): Pedro setup and tuning order.
- [MatchAutonomousPlan.md](MatchAutonomousPlan.md) and [MatchAutonomousPlanCycle.md](MatchAutonomousPlanCycle.md):
  the draft match strategies behind `red-far`, `red-far-cycle` and `red-audience`.
- [CompetitionTeleOp.md](CompetitionTeleOp.md): what the drivers' controls do, which is what Zenith's shooting
  actions copy.
