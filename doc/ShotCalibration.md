# Guided hive shot calibration

Select **Calibrate: Hive Shot Table** in the Driver Station's **Test** group.
It initializes only the turret assembly (including its encoder), gate, both (linked) transfer
motors and transfer full sensor. It does not drive the intake, drivetrain, Pinpoint or
Limelight, but the turret encoder is read through the `intake_motor` port, so that
configuration entry must exist. All controls use gamepad 1.

Use the same hive opening, height, ball type and mechanical configuration throughout
a session. Measure **horizontal distance from the turret rotation axis to the vertical
projection of the upward-facing cell opening center**, not distance to a tag, the
hive pivot, or the hive base. The ball exit is centered on the turret axis in plan
view, so this is also the horizontal muzzle-to-opening distance. Enter it on the
PLACE screen after START using Dpad left/right in 1-inch steps, then A to confirm.
From TUNE, B returns to PLACE to edit the distance; there is no typed numeric field.
Hood degrees mean 0–30 degrees of downward travel from home. Home is +15 degrees
above horizontal; 15 degrees of travel is horizontal; 30 degrees is -15 degrees.
Launch elevation = 15 - hood travel. Dpad Up lowers elevation and Down raises it;
telemetry displays both values. RPM is flywheel output RPM inferred from the motor encoder and the 1:1 motor-to-flywheel speed ratio, not an independently measured
flywheel shaft speed. A changed drive ratio requires recalibration.

1. During INIT, allow turret homing to finish. After START, position the robot at
   a measured distance. Dpad left/right edits that distance in inches; A confirms.
   The first suggested distance is 24 inches. Suggested distances are editable.
2. Load **one ball per trial**, keeping clear of the mechanism. The intake and
   automatic transfer loading are disabled; no motion occurs to refill the queue.
   In TUNE, RS-X aims yaw. Dpad left/right changes RPM by 50; up/down changes hood
   command by 0.5 degrees. Hold RB to spin; hold X to feed (changed 2026-11-18).
3. Keep RB and X held through the attempt. Feeding waits for stable RPM, hood settling
   and yaw tracking, then opens the gate (shared `ShootingConfiguration.firing()` gate delay)
   before running the transfer (both linked motors). Once feeding starts, the gate stays open and
   the transfer keeps running until X is released; RPM dips do not interrupt it. Releasing X
   before feeding starts records no trial.
4. After X is released, flywheel and transfer stop and the gate closes. Observe the
   result, then press A for HIT or B for MISS. X discards an uncertain/no-ball trial.
   A confirmed HIT updates the running distance average shown on the Driver Hub.
   Misses and discarded trials do not change the averages.
5. Repeat as many times as desired; there is no shot-count limit or three-hit
   requirement. Every A-confirmed HIT immediately updates the arithmetic mean of
   target RPM and hood travel at that exact entered distance. MISS and discarded
   trials do not affect the means. Changing settings or revisiting a distance
   preserves its prior confirmed samples for the current session. Telemetry shows
   the sample count and averaged RPM/launch elevation. Y moves to the next distance
   after at least one confirmed HIT, initially suggesting 12 inches farther away.
   Move the robot manually and enter its actual measured distance.
6. Collect at least two distinct distances; cover the full intended shooting range,
   with additional points where the response changes rapidly. BACK from PLACE/TUNE
   opens REVIEW. Dpad left/right queries the interpolated RPM and hood
   at one-inch intervals. Test intermediate distances physically as well. BACK
   returns to distance entry. All distance averages remain visible throughout the
   active loop, including while tuning and rating shots. Record the displayed
   rows before pressing STOP; STOP exits without saving.

LB aborts, closes the gate and stops the transfer (both motors) and flywheel. Releasing
RB during a trial also aborts; no hit/miss is recorded. A ball already fed can still
exit. LS click in PLACE/TUNE rehomes. Hardware outputs stop in `finally` on exit.

The TeleOp uses memory and Driver Hub telemetry only. It does not read or write
calibration files, maintain trial logs, or export data. The live table displays
distance in inches, average target flywheel RPM, average launch elevation above
horizontal, and average downward hood travel from home. Values refresh every loop.
A new run starts empty. Keep the OpMode running in REVIEW while recording the
rows or taking a screenshot of the Driver Hub, then share those rows to enter
them into the source-code lookup table.

## Using the table

After collecting the displayed averages, enter them as ordered rows in
`ShootingConfiguration.PROFILE` in place of `ShotProfile.example()`, with
`calibrated = true`. The geometry ID must be exactly
`ShootingConfiguration.CALIBRATION_GEOMETRY_ID`: competition targets carry that ID,
and the planner rejects any other ("Profile does not support target geometry").
Each row uses distance in meters, downward hood travel in degrees and average
flywheel target RPM:

```java
new ShotProfile.Row(distanceInches * .0254,
        15 - averageLaunchElevationDeg, averageRpm)
```

If using the displayed hood travel directly, use it as the second argument instead
of converting launch elevation. At least two distinct distances are required.
Calibration does not modify the competition profile. `ShotProfile.sample()` in
the robot code already performs linear interpolation independently
to both hood and RPM between adjacent rows. Outside the measured range a notebook/preview
profile's `sample` returns `null`. The competition profile (2026-11-15) is built with
`extrapolates = true`: it continues the nearest end segment's line, clamped to 0-30 deg hood and the
flywheel target ceiling. Those extrapolated settings are unmeasured; validate them before relying on them.

Closest/middle/farthest samples define two linear segments and provide estimated
settings at every distance between those endpoints. They do not establish settings
outside that interval or guarantee successful shots at unsampled distances. Test
the interpolated settings, particularly halfway between samples, and add distances
where errors appear. Averaging successful but very different angle/RPM pairs can
produce an unsuccessful intermediate pair; keep one consistent shot trajectory
and validate the averages themselves.

Flight-time data and projectile lead prediction have been removed (the turret command's actuation-delay lead, `AIM_LEAD_SECONDS`, is separate and does not change range, hood or RPM). Current pose determines the range, yaw, hood and RPM each loop; automatic feeding has no chassis/turret speed gate. Mount geometry, the approved opening geometry and release timing still require configuration. Drivers decide whether to fire while moving.

## Task 9 acceptance procedure (2026-11-14)

Task 9 in the [verification matrix](TeleOpVerificationMatrix.md) needs two physical results: the shot table
and the release timing. The shot table is now supplied (2026-11-15: 12/24/36/48/60 in in `ShootingConfiguration.MEASURED_SHOT_TABLE_IN_RPM_HOOD_DEG`);
release timing is no longer a firing gate (2026-11-15, team decision): the current `ShootingConfiguration` /
`GateSubsystem` timing values are used as-is and the `RELEASE_TIMING_MEASURED` flag was removed. Measuring it
(below) is optional for tuning those values.

**2026-11-19 team decision: task 9 is COMPLETE / NOT REQUIRED.** The current shot table and timing values
work on the robot and competition feeding is continuous only, so the remaining range cross-check, midpoint
validation and release-timing measurement are not planned. The steps below stay for optional re-tuning.

### A. Shot table

1. Choose the closest and farthest distances the drivers will actually shoot from in a match. Plan one
   distance every 12 in across that range (for example 24, 36, 48 ... in), plus extra points where the
   required RPM or hood changes quickly.
2. Use fully charged batteries and match balls. Keep one cell (opening and height) for the whole session.
3. At each distance run the guided procedure above until the shot is consistent. Aim for at least three
   confirmed HITs per distance with one trajectory; do not average two very different RPM/hood pairs.
4. Range cross-check (recommended at two or three distances): without moving the robot, run
   `Test: Auto Aim`, let it localize, and compare **Odometry target range** (m) with the tape distance.
   That is the value competition looks up in the table. Agreement within about 1 in (0.025 m) means the
   table transfers directly. A larger, consistent difference is a field-map/localization issue to report;
   do not adjust the table to hide it.
5. Before STOP, take a Driver Hub screenshot of REVIEW (all rows), or write down each row:
   distance (in), average RPM, average hood travel (deg).
6. Validate halfway between two rows: use REVIEW to read the interpolated settings, set them in TUNE at that
   distance and confirm hits. Add a row where it misses.

Feed timing in this OpMode is the shared gate delay, then the transfer for as long as X is held;
competition TeleOps default to continuous feed (gate and transfer together). With one ball per trial this does not change the
trajectory. Multi-ball RPM sag and recovery belong to task 10.

### B. Release timing

`FEED_TO_EXIT_SECONDS` (0.12 s estimate) is the time from the feed start to the ball leaving the flywheel.
It keeps a started shot "committed" so a ball already fed is still expected to exit.

1. Run `Test: Intake + Index + Shooter` and tap GP2 RB for CONTINUOUS mode (default is STAGGERED), where
   the gate and transfer start together as in competition. Load one ball, hold RT to spin to a table
   RPM, then add LT to feed.
2. Film from the side with phone slow motion (240 fps or faster), framing the gate and the flywheel exit.
3. Count frames from the first visible gate motion to the ball fully leaving the flywheel; seconds =
   frames / fps. Repeat at least five times at the nearest and farthest table RPMs.
4. Also confirm that each ball fully clears before the gate closes, and note how long the flywheel takes
   to return to its target RPM after a shot (telemetry shows current RPM).
5. Report the largest exit time and the recovery times. Software then sets `FEED_TO_EXIT_SECONDS` from
   the measurement (there is no measured flag to set; timing does not gate firing).

### What to send back

The table rows (distance in, RPM, hood travel deg), the range cross-check readings, and the release timing
measurements. Software enters them in `ShootingConfiguration`, re-runs the shot-planner tests and the
competition simulation, and then automatic firing becomes available for task 10.
