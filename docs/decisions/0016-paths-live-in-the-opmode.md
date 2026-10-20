# 0016: An auto's paths live in the auto's own file

Status: Accepted.
Date: 2026-10-20

## Context

Decision 0008 put the practice shapes in `pedro/ShapePaths.java`, apart from the OpModes,
so they could be unit tested without a robot. The students who will build the real autos
found that a path lived two files away from the OpMode that drives it. The coach asked for
the paths to be in the OpMode itself: one file to open, one file to change.

## Decision

- **Each auto is one file.** `BuildPaths()` holds the points and the lines or curves,
  with the shape's picture in the comment above. `AutoShapeZ` is the one to copy.
- **`PedroAuto` keeps everything shared**: building the Bot, the tuned check, the follower,
  the loop, and `StartPose()` (0, 0, facing forward), which an auto may override.
- **The tests still run on a laptop.** `ShapeAutosTest` builds each OpMode and asks it for
  its paths; nothing in `BuildPaths()` touches hardware. The one rule to keep: no
  `hardwareMap` in `BuildPaths()`.
- `pedro/ShapePaths.java` is gone.

## Consequences

- Two autos that drive the same curve now each spell it out. That is decision 0006's
  trade: a short copy beats a shared helper.
- A new auto is: copy `AutoShapeZ.java`, rename the class and the `@Autonomous` name,
  change the points. The tuning guide says the same.
