# BIOBUZZ TU06 field reference for robot software

Reviewed October 23 and October 26, 2026 against the supplied [Section 9 ARENA PDF](https://ftc-resources.firstinspires.org/ftc/game/manual-09), version TU06, printed pages 63-79 (17 PDF pages). The dimensioned hive, cell and tag diagrams were inspected visually; text extraction alone omits dimensions and mangles several tag IDs. This note records field data, not instructions to perform field setup or physical checks. Match paths remain deferred.

Section 9.1, printed p.63, identifies the CAD model as the authoritative construction reference and describes manual dimensions as nominal, generally with +/- 1 in. (+/- 2.5 cm) tolerance unless otherwise specified. Numerical conversions below do not imply tighter accuracy. No dimensions were measured from image pixels.

## Confirmed nominal dimensions

| Item | Reference value | Source |
| --- | --- | --- |
| Field interior | Approximately 144 x 144 in. (3.6576 x 3.6576 m by inch conversion) | Section 9.2, p.64 |
| Hive pivot height above tiles | 43.95 in. (1.11633 m by inch conversion) | Figure 9-8, p.70 |
| Hive lateral center spacing | 25.5 in. (647.7 mm) | Figure 9-10, p.71 |
| Frame base width/depth | 49.46 / 38.95 in. | Figure 9-8, p.70 |
| Hive cell separation / depth | 18.84 / 12.04 in. | Figure 9-9, p.71 |
| Raised opening bottom / top above tiles | 53.5 / 65.6 in. (1.3589 / 1.66624 m) | Figure 9-10, p.71 |
| Opening width / peak height | 20 / 14 in. (508 / 355.6 mm) | Figure 9-11, p.72 |
| Opening shoulder height | 7.61 in. (193.294 mm) | Figure 9-11, p.72 |
| Hive tilt shown in side view | 30 degrees for the tilted cell floor relative to horizontal; the perpendicular opening plane is 60 degrees from horizontal | Figure 9-10, p.71; plane interpretation agrees with the team's clarification |
| Pollen | Yellow, approximately 2.8 in. (71 mm in the manual), not perfectly spherical | Section 9.8, p.74 |
| AprilTag family / individual size | 36h11, 3.25 in. square (82.55 mm by inch conversion) | Section 9.9 and Figure 9-15, pp.74-75 |

The code's 144-in. field, 25.5-in. hive spacing, 43.95-in. pivot height, 18.84/12.04-in. cell geometry and 20/14/7.61-in. opening dimensions agree with this reference. The 30-degree hive-tilt constant and the team's 60-degree opening-plane angle describe different planes; they are not competing values.

Per the latest October 26 instruction to prefer drawing values, the nominal aim height is now the midpoint of the drawing's opening top/bottom heights: `(65.6 + 53.5)/2 * 0.0254 = 1.51257 m`. This replaces the earlier 1.50-m estimate. Mid-height is the chosen software aim point; it is not the pentagonal opening's area centroid or the SDK virtual cluster origin. The shared `FieldGeometry` class supplies all nominal structural dimensions to the other subsystems.

## AprilTag clusters and ordering

Figures 9-16 and 9-17, printed pp.76-77, confirm the existing alliance/side mapping:

| Cell | IDs | Order from left to right in the audience-oriented overhead drawing |
| --- | --- | --- |
| Red far / scoring side | 30-33 | 33, 32, 31, 30 |
| Red audience | 34-37 | 34, 35, 36, 37 |
| Blue audience | 38-41 | 38, 39, 40, 41 |
| Blue far / scoring side | 42-45 | 45, 44, 43, 42 |

Figure 9-15 gives tag-center offsets from the cluster center: **-6.5, -2.75, +2.75, +6.5 in.** across the sticker (**-165.1, -69.85, +69.85, +165.1 mm**). Each sticker's IDs increase from left to right when viewed facing its printed surface. **Corrected 2026-11-13 (supersedes the 2026-11-05 reversal):** with the team's field +X toward blue convention, far-side IDs **decrease** with X and audience-side IDs **increase** with X. A robot under the cell sees each sticker upright: IDs increase left to right in the upright camera frame. The 2026-11-05 reading (tag 30 camera-right) came from raw Limelight poses, which stay in the inverted sensor frame of the upside-down camera; in the upright camera tag 30 is camera-left of 31, 32, 33. The 2026-11-13 Red audience frame confirms the order across both audience cells: tags 34, 35, 36, 38, 39 read left to right with 3.75 / 5.5 / 16.25 in spacing after `LimelightObservation`'s inverted-mount conversion (raw, the blue cell appeared on the wrong side). `HiveTagGeometry.nominalRaisedTags` and `PhysicalLimelightFrameTest` encode this order. Do not give all four IDs the cluster center or the opening center.

Additional dimensions in Figure 9-15:

- The two reference-hole centers are 7.0 in. (177.8 mm) either side of the cluster centerline.
- Tag centers are 2.75 in. (69.85 mm) from the reference-hole line toward the front of the cell, measured in the sticker plane.
- The reference-hole line is 9.938 in. (252.4252 mm) behind the front-of-cell reference, along the underside.
- Consequently, the tag-center line is **7.188 in. (182.5752 mm)** behind that reference in the cell plane. This is not a field-horizontal distance.
- The sticker sits on the bottom of the cell, with its bottom edge toward field center (p.75).

Under the drawing's 30-degree raised-cell floor tilt, the 7.188-in. setback projects to about **158.1 mm horizontally** and **91.3 mm vertically**. The configured nominal tag height is now `53.5*0.0254 - 7.188*0.0254*sin(30 degrees) = 1.2676124 m`, replacing the earlier 1.25-m estimate. This is a drawing-derived surface model; the exact front-edge/underside construction references still require CAD or installed-field verification.

Configure the Limelight AprilTag pipeline for **36h11 and 3.25-in. individual tags** when configuring the camera. Java currently selects pipeline 0; it does not set tag family/size. Camera configuration was not accessed or changed here. The metric 3D observation path depends on correct camera tag family and size configuration.

## Moving hive state

Section 9.6, printed p.69, states that each hive pivots between two stable positions, with one cell facing upward. The AprilTag clusters are attached to those cells and move with them. A tag ID identifies a cell, not its current raised/lowered state. Low-goal shooting is outside the team's scope. Per the October 26 decision, competition TeleOp does not track hive state: drivers decide when to shoot. The state observer remains a standalone diagnostic/legacy adapter. A moving cell must still be rejected as an incompatible localization landmark; its ID alone cannot certify a fixed field position.

The PDF's linked [Event Field Setup Guide, section 11.1, p.23](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide) specifies the initial raised cells as **red audience** and **blue scoring/far**. This is an initial setup condition, not a state guarantee later in the match. This does not determine the competition target; odometry chooses the nearest cell of the selected alliance.

## October 26 nominal opening and individual-tag field map

Figure 9-9 labels the full hive length as 42.91 in. (109.0 cm). The inch value now supplies the common source: half-length is **0.544957 m**, replacing the rounded 0.545 m. Retaining the approved half-length projection, the opening Y magnitude is `0.544957*cos(30 degrees) - (1.51257 - 53.5*0.0254)/tan(60 degrees) = 0.38322519 m`. Competition uses X = +/-0.32385 m (blue/red), Y = +/-0.38322519 m (far/audience), Z = 1.51257 m.

The longitudinal projection still assumes the half-length locates the front relative to the pivot. The drawings do not fully specify the pivot-to-floor surface offset, so this is a nominal approximation, not a verified absolute transform. The separately dimensioned pivot height does not make that missing surface reference disappear; an exact field Y map requires the CAD transform or a field measurement. No image-pixel measurements were used.

Applying the same approved half-length projection to the underside tag row gives `Y magnitude = (0.544957 - (9.938 - 2.75)*0.0254)*cos(30 degrees)`, approximately **0.31383184 m**. `HiveTagGeometry` now supplies all 16 individual tag centers using this nominal Y and the manual's lateral offsets. This extends the approved drawing model; it is not a survey or a CAD-verified transform. Coordinates use the field-center origin, +X toward blue, +Y away from the audience and +Z upward. Each row describes that cell **when raised**, so the four clusters do not occupy these raised poses simultaneously.

| Tag ID | Cell | X (m) | Y (m, nominal) | Z (m, nominal) |
| --- | --- | ---: | ---: | ---: |
| 30 | Red far | -0.15875 | +0.31383 | 1.26761 |
| 31 | Red far | -0.25400 | +0.31383 | 1.26761 |
| 32 | Red far | -0.39370 | +0.31383 | 1.26761 |
| 33 | Red far | -0.48895 | +0.31383 | 1.26761 |
| 34 | Red audience | -0.48895 | -0.31383 | 1.26761 |
| 35 | Red audience | -0.39370 | -0.31383 | 1.26761 |
| 36 | Red audience | -0.25400 | -0.31383 | 1.26761 |
| 37 | Red audience | -0.15875 | -0.31383 | 1.26761 |
| 38 | Blue audience | +0.15875 | -0.31383 | 1.26761 |
| 39 | Blue audience | +0.25400 | -0.31383 | 1.26761 |
| 40 | Blue audience | +0.39370 | -0.31383 | 1.26761 |
| 41 | Blue audience | +0.48895 | -0.31383 | 1.26761 |
| 42 | Blue far | +0.48895 | +0.31383 | 1.26761 |
| 43 | Blue far | +0.39370 | +0.31383 | 1.26761 |
| 44 | Blue far | +0.25400 | +0.31383 | 1.26761 |
| 45 | Blue far | +0.15875 | +0.31383 | 1.26761 |

The table now consistently uses the drawing-derived tag height. Per the team's instruction, competition TeleOp enables periodic relocalization using this nominal map and the documented Limelight mount. Accepted frames are limited to one per 0.5 s and must pass capture-age, finite-pose, tag identity, height and position-jump checks. The SDK's capture/pipeline latency is used; additional transport delay remains at its existing zero estimate. No drift is assumed between corrections, and neither uncertainty nor loss of a camera target creates a feeding timeout. Camera and installed-field validation remain physical checks, not software enable switches.

## SDK cluster origin versus individual tag centers

The [FIRST AprilTag-cluster technical note](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html) explains that SDK 12 returns an `AprilTagClusterDetection` for a recognized cluster, with a virtual pose origin near the cell opening. That origin is different from the sticker center and from the center of an individual tag. Detecting different members of a cluster still refers to the same cluster origin; do not add a member's offset again to an already resolved cluster pose.

The installed FTC Vision **12.0.0** source, `AprilTagGameDatabase.getBioBuzzCluster`, specifies these member coordinates relative to the SDK cluster origin (inches): X = **-6.5, -2.75, +2.75, +6.5**, Y = **+7.1874**, Z = **-5.622**, with 3.25-in. tags. The code records these as SDK reference constants, not as field coordinates. Its four cluster entries use zero field-position placeholders and identity field orientation, so that database does not supply a verified absolute field map.

This robot reads Limelight `FiducialResult` poses for **individual tags**, not the SDK cluster-detection API. The legacy `OpeningGeometry` conversion therefore remains unverified until the SDK cluster axes/origin are explicitly reconciled with the Limelight tag-local frame. Copying the SDK Y/Z values directly into that conversion would mix coordinate systems. Competition current-pose aiming uses the field opening coordinates above and does not need this diagnostic conversion.

Competition TeleOps no longer assume a blue-corner start. They acquire initial field position and heading from three consistent raised-tag observations, then track Pinpoint motion with periodic translation corrections; see [current decisions](CompetitionTeleOp.md). The former corner reference remains (1.566319, -1.670450) m at 180 degrees for bench comparison only. Existing +32/-74 mm Pinpoint pod offsets describe sensor placement relative to the robot center and are not additional field-position offsets.

The [official field CAD](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad) is the remaining reference for verifying the nominal pivot-to-cluster transform. Pollen and autonomous route work remain deferred primitive examples. An external landmark can relocalize odometry; resetting Pinpoint alone cannot establish an unknown absolute field pose.
