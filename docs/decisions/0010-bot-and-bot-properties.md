# 0010: OpModes talk to a Bot; settings live in BotProperties

Status: Accepted. Not yet tried on the robot.
Date: 2026-10-04

## Context

`types/Constants.java` was hard to teach. It was one long list of `static final` values
that every OpMode shared, far from the code that used them. Three problems came from that:

- **A change for one test changed everything.** Raising `launcherStartRpm` to 4000 for a
  bench test also changed it for every other OpMode, and broke 4 launcher tests.
- **An OpMode could not leave a part out.** Each OpMode built its own list of subsystems
  by hand. Testing the launcher alone meant writing a different OpMode body.
- **Two subteams, one file.** The team is splitting into drive + auton and launcher + aim
  assist. Both would be editing the same shared values.

Each OpMode also repeated the same plumbing: make the `HydraOpMode`, check the Safe Mode
key, build subsystems, set up the hubs, run the INIT loop.

## Decision

- **`bot/Bot`** is this season's robot as one object. It builds the `HydraOpMode`, checks
  the Safe Mode key, builds every part that is switched on, and sets up the hubs. An OpMode
  builds a Bot and tells it what to do:

  ```java
  BotProperties properties = new BotProperties();
  Bot bot = new Bot(this, properties);
  bot.Init();

  while (opModeInInit()) {       // waiting for START
      bot.StartLoop();
      bot.ShowStatus();
      bot.EndLoop();
  }

  while (opModeIsActive()) {     // START was pressed
      bot.StartLoop();
      bot.HandleUserInput();     // TeleOp only
      bot.Process();
      bot.EndLoop();
  }
  ```

- **`bot/BotProperties`** holds every setting with its normal value. It replaces
  `types/Constants.java`; the names are unchanged. The fields are plain, not `static
  final`, so an OpMode changes only what it needs, for itself only:

  ```java
  BotProperties properties = new BotProperties();
  properties.driveDisabled = true;       // this OpMode only tests the launcher
  properties.launcherStartRpm = 4000;
  ```

- **Switching a part off:** `driveDisabled` and `launcherDisabled`. A part that is off is
  never built, so its hardware does not have to be in the robot configuration. A launcher
  test stand with no drive motors works.
- **Safe Mode is locked.** `safeModeMaxPower`, `cfgSafeModeKey` and `keyPressedMeansKeyIn`
  stay `static final` in BotProperties. An OpMode can't change them:
  `properties.safeModeMaxPower = 1;` does not compile.
- **Subsystems get the settings in their constructor**, next to the `HydraOpMode`:
  `new Launcher(op, properties, gamepad)`. They read `mProperties.launcherKP` where they
  used to read `Constants.launcherKP`. The `Subsystem` contract did not change.
- **`opMode/OpMode_Base` is gone.** Everything it did is in the Bot. OpModes extend
  `LinearOpMode`.
- **Autos** build a Bot with `driveDisabled = true`, because Pedro Pathing drives the
  wheels there, and get their Follower from `bot.CreateFollower()`. `PedroConstants` keeps
  Pedro's own numbers and takes the robot's names from BotProperties.
- **No behavior change.** HyDrive is still drive only (`launcherDisabled = true`).
  `Test: Launcher` is still launcher only, on gamepad 1 (`driveDisabled = true`). The
  shape autos still use neither.

## Consequences

- **Older decisions say `Constants.something`.** Read that as the same name in
  BotProperties. They also say `OpMode_Base.CreateHydraOpMode()` checks the key; the Bot's
  constructor does that now, so it is still true that no OpMode can forget it.
- **Adding a part** (intake, feed servo) is three steps: write its `Subsystem`, give it a
  "disabled" setting in BotProperties, build it in the Bot's constructor the way the
  launcher is built.
- **Putting the launcher in HyDrive** is deleting one line in `HyDrive.java`. Decide the
  gamepad first: the launcher's right trigger and the drive's boost are both on gamepad 1.
  The Bot hands the launcher gamepad 1 today.
- **A setting changed in an OpMode is easy to miss.** Driving feels different in one
  OpMode and nobody remembers why. Keep changes at the top of `runOpMode()`, with a comment
  saying why.
- **Settings can be changed while the robot runs**, since the fields are not `final`. Don't:
  set them before building the Bot.
- **Extra telemetry:** `Test: Launcher` and the autos now show the Safe Mode line, which
  parts are off, and `LoopTime`.
- **The hubs switch to one-read-per-loop when the Bot is built**, which in an auto is now
  before the Follower is built instead of after. `bot.StartLoop()` must be the first line
  of every loop, or sensors keep returning old readings.
- Tests: `BotTest` (16) builds the Bot against a pretend robot where missing hardware
  throws, like the real one. It covers each part switched off, a changed setting reaching
  its part, Safe Mode through the whole TeleOp loop, the key being read once, and `Init()`
  waiting for a part that is still warming up. `LauncherTest` gained one test: a changed
  start speed is used and the normal one is left alone.
