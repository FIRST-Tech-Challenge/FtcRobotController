# Limelight goal-state rejection diagnostics

`Test: Limelight Goal State 3D` is camera-only. Select Red with GP1 B and Far with
D-pad Right for tags 30–33; D-pad Left selects Audience (Red tags 34–37).

The corrected-field photo supplied November 5, 2026 shows Red/Far tags 30–33 at
roughly 1.11 m forward with areas 0.006–0.007%, all `usable false`. These displayed
translations and areas pass the existing range/area checks. Raw rotation and facing
readings are still needed to identify the rejection; this is not physical proof of
a target-normal convention change.

`LimelightDiagnostics` now supplies the same cached-pose readings to both camera
tests and every vision-enabled turret, including `Test: Auto Aim`, the Red/Blue
competition TeleOps, and shared autonomous telemetry. No second hardware poll is
performed for diagnostics. `ShotCoordinator` and `FieldRehomeRefinement` report
the actual failed prerequisite or per-tag check rather than silently skipping it.

The diagnostics now report:

- `Decision`: the specific rejected tag/check, retained when state is UNKNOWN.
- `Camera capture age (s)`: incoming frame capture age, independent of acceptance.
- `Goal classification pose age (s)` in the goal-state test and
  `Accepted vision pose age (s)` in the coordinator: age of the last pose to reach
  those respective stages. Infinite age means none has reached that stage; it
  does not by itself indicate a stale camera stream or absent field reference.
- Per-tag `R/P/Y (deg), facing`: roll, pitch and yaw in the robot's outward-+Z tag
  basis (SDK roll + 180 deg, see below), plus the cosine between the outward normal and
  the ray toward the camera. Acceptance requires strictly greater than 0.12.

## Physical tag-normal convention (resolved 2026-11-05)

With Test: Limelight Goal State 3D on Red / Far and tags 30-33 seen head-on at about
1.12 m, every tag passed range/area and failed only the facing check, with raw scores of
-0.96 to -0.99 (SDK yaw about -179 deg, roll/pitch within about 12 deg). The installed
Limelight therefore reports tag +Z into the print, as raw SimLoop does.
`LimelightObservation.LIMELIGHT_TAG_Z_INTO_PRINT` applies Rx(pi) (roll + pi for the SDK's
intrinsic Z-Y-X angles) once at the SDK boundary, so all camera tests, Auto Aim and both
competition TeleOps share it. Thresholds and downstream axes are unchanged.
`PhysicalLimelightFrameTest` keeps the photographed values as a regression.

Follow-up measurements (team, 2026-11-05, robot and turret flat on the floor):
- Limelight front glass 55 deg from vertical (35 deg from horizontal), so the lens axis is
  **55 deg above horizontal**. `CAMERA_PITCH_UP_DEG = 55` (previously 30).
- Lens about 12 in +/- 2 in (code kept 335 mm, within that range); lens 33.5 in from the wall.
- 2026-11-06 re-measure: lens 337 mm above the tiles (code now uses 337); front glass/face 35 deg from horizontal (confirmed),
  so the lens axis stays 55 deg up; goal face (opening) 30 deg from vertical, so the tag underside is 30 deg
  from horizontal; goal center about 5 ft, so the field is treated as nominal. Nominal TU06 Section 9 values
  in use: tag 3.25 in, raised opening 53.5-65.6 in (aim height 59.55 in), cluster row 9.938 - 2.75 in from
  the cell front, tag center height 49.91 in (1.268 m). The 2026-11-05 photo tags (1.12 m along the lens
  axis) compute to 1.26 m with a 55 deg axis and 0.99 m with 35 deg, so 55 deg is the lens-axis value.
- Tag centers about 49-50 in (map: 1.268 m = 49.9 in); printed tag size 3.25 in (= 82.55 mm).
- Tag stickers 60 deg from vertical, i.e. **30 deg from horizontal** on the raised cell's
  underside, facing down toward the robot. This matches the field map and the goal-state
  classifier. Field acquisition and corner refinement previously compared against the
  60-deg opening plane; they now use `HiveTagGeometry.RAISED_TAG_PLANE_DEG` (30).

Geometry check: with tags near image center, the wall distance alone required at least about
48 deg of camera elevation, which ruled out 35 deg. At 55 deg the photographed frames compute
tag heights of 1.245-1.275 m and tags about 0.63 m ahead of the lens, matching the map.
Single-tag plane angles read about 36-46 deg against 30 +/- 10, so plane acceptance is
marginal per tag and still needs a fresh on-robot frame; tolerances were not relaxed.
(These figures predate the 2026-11-13 inverted-mount correction below; with it the same frame reads
1.243-1.245 m and planes 23.5-29.8 deg for tags 30, 31, 33, with tag 32 a 47-deg single-tag outlier.)

Consequence: from the corner-rehome pose the raised tags sit about 28 deg below the lens axis,
beyond the ~20.5-deg half image height, so corner refinement now falls back to CORNER ONLY.
Normal TeleOp acquisition needs the robot within roughly 52 in of the hive tag row.

Competition/Auto Aim `Vision decision` describes current processing, while
`Pose fusion decision` preserves the pose-history result separately.
`Field localization` distinguishes a fresh field reference, stale/unready odometry with
a retained reference, and the specific prerequisite preventing acquisition.
Pinpoint telemetry describes its own raw reference; a vision-derived field
transform belongs to the aiming coordinator, not the Pinpoint device. The isolated
Pinpoint and manual turret tests still require only their own hardware.

After deploying, hold the robot stationary with the raised tags visible and record
`Decision` or `Vision decision`, camera/accepted ages, and one tag's metric and R/P/Y/facing
lines. Keep the known hive state, camera mounting and turret orientation with the
record. Do not change normal axes, calibration flags or acceptance thresholds to
make observations pass without that evidence.

This update adds diagnostics and preserves rejection reasons. Normal axes,
range/area/facing thresholds, shot calibration flags, driver controls and the
autonomous no-camera-corrections policy remain intact. The helper for nominal
raised-tag rejection is shared by competition acquisition and corner refinement.

Simulation decision: run the Sensors and Competition scenarios against the actual
OpModes, including a new rejection-reporting scenario across both camera tests,
Auto Aim and both competition wrappers. Preserve the existing raw rejection and
explicitly normalized acquisition/camera-loss cases. These verify software
reporting and coordination; resolving the physical SDK pose convention still
requires a real camera frame. Diagnostic coverage is limited to initialization,
rejection reporting and STOP, not physical aim accuracy or complete behavior.

Verification: 219 ordinary tests, debug APK assembly, lint, and all 16 selected
Sensors/Competition/runtime simulations passed. Expected simulation case names
were checked against the manifest. The camera-only rejection scenario initially
used default fake yaw commands that slewed the mount out of view; its fixture now
starts at a known stationary home angle. Camera normal conventions and rejection
assertions were preserved. Changes have not been deployed or physically retested.

## TeleOp field fusion (2026-11-05)

Symptom on the robot: after RT, the turret jerked 3-4 times with the robot still, then tracked with drift.
Causes in the shared `PoseHistory` / `ShotCoordinator` path (Auto Aim and both competition TeleOps):
- Heading came from one tag's steep outward normal; on the photographed frame three tags disagreed by 12 deg.
  It was then kept for the whole match, so heading error became drift.
- Every 0.5 s a different "best" tag nudged position 35%, and frames captured while the turret slewed after RT
  were fused, so latency error moved the aim target and the turret chased it.

Now: every accepted tag in a frame is fused; heading uses each tag's horizontal +X edge (same frame: 5 deg spread)
and is averaged over tags and the three acquisition frames; later frames correct heading by 35% (rotating about the
capture position, at most 8 deg error accepted) as well as position; frames are skipped while the turret or chassis
turned faster than 20 deg/s around capture. `Vision decision` shows "Turret moving near camera capture" or
"Chassis turning near camera capture" while it waits.

## Limelight orientation setting and inverted mount (corrected 2026-11-13)

The camera is mounted upside down. The Limelight web UI "Stream Orientation: Upside-Down" rotates only the
video stream; the AprilTag ID labels in that stream are drawn upside down, and the per-tag 3D camera-space
poses (`t6t_cs`) stay in the inverted sensor frame. The earlier note here (axes upright with the setting on)
was wrong. `LimelightObservation.LIMELIGHT_MOUNTED_INVERTED` applies Rz(pi) about the optical axis once at the
SDK boundary: right and down are negated and the SDK yaw gets + pi, on top of the existing tag +Z conversion.

Evidence, 2026-11-13 at Red audience (1280x960, stationary, tags 34-36 and 38-39 at about 1.2 m):
- The upright stream showed the tags about 7 deg below center; the raw pose put them 7.3 deg above.
- Raw, the blue tags were camera-left of red, but the blue hive is at +X (camera-right looking +Y). After the
  flip the tags read left to right 34, 35, 36, 38, 39 with 3.75 / 5.5 / 16.25 in spacing, matching the map to 1 cm.
- Raw poses computed 1.41-1.44 m tag height and 23-46 deg plane. Field measurement: tag centers 47.5-48 in
  (1.21 m), underside 29.9 deg from horizontal; Limelight face 34.7 deg from horizontal (lens axis 55.3 deg up,
  confirming `CAMERA_PITCH_UP_DEG` 55). After the flip all five tags read 1.229-1.250 m; 34, 36, 38, 39 read
  24-33 deg and pass; tag 35 reads 58 deg (raw pitch -18 deg, a single-tag outlier) and is rejected per tag.
- The 2026-11-05 Red far frame was nearly centered vertically, which hid the error; it now reads 1.243-1.245 m.

The underside stickers are seen upright (tag +X camera-right), not upside down as recorded on 2026-11-05;
`PoseHistory.tagEdgeFieldYaw` and the tag map follow. If the orientation setting or the mount changes, re-run
Test: Limelight Goal State 3D and compare tag heights and left-to-right order before driving.

## Drift-only camera corrections (2026-11-06)

After the first field reference, Pinpoint odometry owns the pose. Each settled camera frame (all accepted tags,
turret and chassis turning under 20 deg/s) is compared with odometry. The last five such frames within one second
are averaged; if they agree with odometry within 4 cm and 1.5 deg, nothing changes (`Vision decision : Odometry agrees
with camera`). A consistent larger difference is real drift: it is blended in over 0.4 s (`Odometry drift corrected`).
Differences over 0.5 m or 8 deg are not drift: the odometry reference itself is probably wrong (pods slipped, a
corner rehome or autonomous handoff from the wrong place). Since 2026-11-10 they re-localize the pose only when
confirmed: at least 10 settled frames spanning 0.5 s within 1 s, each fusing 2+ agreeing tags, all implying the same
correction (4 cm / 1.5 deg) and a pose on the field (`PoseHistory.RELOCALIZE_*`). The correction blends in over 0.4 s
like drift (`Odometry re-localized from camera`). Until then frames are reported and ignored (`Camera disagrees with
odometry ... confirming n/10`, `... needs 2+ tags`, `... inconsistent`); any frame that agrees with odometry restarts
the count. The driver view shows **Camera pose check** (CAMERA DISAGREES / RE-LOCALIZED) while this happens.

## Camera delay breakdown

`Camera delay (ms)` splits `Camera capture age`: **hub** is SDK staleness (time since the Control Hub parsed
the result; the SDK polls every 10 ms by default, so this should stay under about 15 ms), **capture** and
**processing** are the Limelight's own reported `cl` and `tl`, and **new frames/s** counts distinct camera
frames actually reaching the robot (repeat polls of one frame are not counted).
Observed 2026-11-05 at 960x720 / 40 FPS with Full 3D: capture age 0.105-0.190 s, not yet broken down.
Observed 2026-11-13 at 1280x960: hub 4 + capture 38 + processing 126 ms, 8 new frames/s, capture age 0.167 s
(over the goal-state test's 0.15 s limit). The team is returning the pipeline to 960x720.

What to change, by the dominant term (Limelight web UI, pipeline 0; re-check tag heights afterward):
- **processing large** (3D AprilTag work): lower resolution to 640x480, raise the AprilTag detector
  downscale, restrict the ID filter to 30-45, crop to the region where hive tags appear.
- **capture large**: shorten exposure / raise gain so the sensor runs at its full frame rate.
- **new frames/s well under the pipeline FPS**: the camera is not delivering frames that fast; check the
  pipeline FPS readout in the web UI and the USB connection.
- **hub large**: polling is starved; check Control Hub CPU / USB, not camera settings.

