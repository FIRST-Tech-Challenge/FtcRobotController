# 0012: FTC Dashboard mirrors the telemetry as live graphs

Status: Accepted. Not yet tried on the robot.
Date: 2026-10-15

## Context

Tuning the launcher's speed loop (0009) and Pedro (0008) needs graphs, not numbers
flashing past on the Driver Station. The coach asked for a web dashboard to monitor the
robot. Decode used FTC Dashboard (its `@Config` fields are what PORT-PLAN P3 refers to), and
decision 0002 put it off "until tuning needs it". It does now.

## Decision

- **FTC Dashboard 0.6.0** (`com.acmerobotics.dashboard`), the standard one. On a laptop
  joined to the robot's Wi-Fi, open `http://192.168.43.1:8080/dash`.
- **The Bot wraps the Driver Station's telemetry** in the dashboard's `MultipleTelemetry`
  (`Bot.WithDashboard`). Every `addData` line goes to both. Numbers become live graphs.
  Nothing in the subsystems changes.
- **When the dashboard is not running**, the Bot uses the Driver Station alone. That is
  the case in unit tests, and when the dashboard is switched off in the Robot Controller
  app's menu.
- **Not used: live-editable `@Config` settings.** They need `public static` fields, and
  `BotProperties` is per-OpMode on purpose (0010). A tuning OpMode may add a small
  `@Config` class later and copy the winning numbers into `BotProperties`.
- **The dashboard's own copy of the SDK is excluded** in `build.dependencies.gradle`, so
  the build uses ours (12.0.0). Version 0.6.0 is from May 2026, before SDK 12.0.

## Consequences

- First graph to open: `Launcher target RPM` against `Launcher actual RPM`, for tuning
  `launcherKF` and then `launcherKP`. Then `LoopTime` and `Yaw` while driving.
- Matches: no laptop joins the robot's Wi-Fi during a match. The library stays in.
- The dashboard sends about ten times a second; the loop runs about fifty. The dashboard's
  telemetry keeps the latest values between sends.
- If a build or an INIT breaks after an SDK update, this dependency is the first suspect.
- The vision branch also streams the webcam's picture to the dashboard (`Vision_Webcam`).
