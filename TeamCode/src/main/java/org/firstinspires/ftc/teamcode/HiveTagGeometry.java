package org.firstinspires.ftc.teamcode;

/** Nominal TU06 Fig. 9-15/17 geometry; full derivation and SDK-origin distinction in doc/FieldReference.md. */
public final class HiveTagGeometry {
    private HiveTagGeometry() { }
    public static final double TAG_SIZE_METERS = 3.25 * .0254;
    /** Stickers lie on the raised cell's underside: 30 deg from horizontal, facing down and out
     * of the hive toward the robot (team-measured 60 deg from vertical, 2026-11-05). Not the 60-deg opening plane. */
    public static final double RAISED_TAG_PLANE_DEG = FieldGeometry.HIVE_TILT_DEG;
    public static final double FRONT_TO_REFERENCE_HOLES_METERS = 9.938 * .0254;
    public static final double TAG_ROW_FROM_REFERENCE_HOLES_METERS = 2.75 * .0254;
    public static final double FRONT_TO_TAG_ROW_METERS = FRONT_TO_REFERENCE_HOLES_METERS - TAG_ROW_FROM_REFERENCE_HOLES_METERS;
    public static final double RAISED_TAG_ROW_Y_METERS = FieldGeometry.NOMINAL_FRONT_Y_METERS
            - FRONT_TO_TAG_ROW_METERS * Math.cos(Math.toRadians(FieldGeometry.HIVE_TILT_DEG));
    public static final double DRAWING_TAG_HEIGHT_METERS = FieldGeometry.RAISED_OPENING_BOTTOM_HEIGHT_METERS
            - FRONT_TO_TAG_ROW_METERS * Math.sin(Math.toRadians(FieldGeometry.HIVE_TILT_DEG));
    // FTC Vision 12.0 AprilTagGameDatabase.getBioBuzzCluster member coordinates.
    // These are relative to the SDK virtual opening origin, NOT field coordinates or Limelight tag poses.
    public static final double SDK_MEMBER_Y_METERS = 7.1874 * .0254;
    public static final double SDK_MEMBER_Z_METERS = -5.622 * .0254;
    public static double offsetFromStickerCenterMeters(int id) {
        if (id < 30 || id > 45) throw new IllegalArgumentException("Not a hive tag");
        double[] inches = {-6.5, -2.75, 2.75, 6.5};
        return inches[(id - 30) % 4] * .0254;
    }
    public static GoalTarget.Tag[] nominalRaisedTags(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side) {
        int first = alliance == TurretSubsystem.Alliance.RED
                ? (side == TurretSubsystem.HiveSide.FAR ? 30 : 34)
                : (side == TurretSubsystem.HiveSide.AUDIENCE ? 38 : 42);
        double centerX = TurretSubsystem.getHiveOpeningCenterXInches(alliance) * .0254;
        // A robot below the cell sees the underside stickers upright: IDs increase left to right in the
        // upright camera frame (tag +X camera-right). Far side (robot looks -Y, camera-right is -X): IDs
        // decrease with field X. Audience side (camera-right is +X): IDs increase with X.
        // Physical frame 2026-11-13 (Red audience, inverted-mount conversion): tags 34, 35, 36, 38, 39
        // left to right with 3.75 / 5.5 / 16.25 in spacing, matching this map to 1 cm.
        double direction = side == TurretSubsystem.HiveSide.AUDIENCE ? 1 : -1;
        double rowSign = side == TurretSubsystem.HiveSide.AUDIENCE ? 1 : -1;
        GoalTarget.Tag[] tags = new GoalTarget.Tag[4];
        for (int i = 0; i < tags.length; i++) tags[i] = new GoalTarget.Tag(first + i,
                centerX + direction * offsetFromStickerCenterMeters(first + i),
                -rowSign * RAISED_TAG_ROW_Y_METERS, ShootingConfiguration.HIGH_GOAL_TAG_CENTER_HEIGHT_METERS);
        return tags;
    }
}
