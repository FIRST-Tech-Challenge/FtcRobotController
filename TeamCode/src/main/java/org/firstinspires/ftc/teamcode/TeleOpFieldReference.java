package org.firstinspires.ftc.teamcode;

/** Team-approved nominal field targets and blue-corner placement geometry (red is its point mirror). */
public final class TeleOpFieldReference {
    private TeleOpFieldReference() { }
    public static final double FIELD_HALF_WIDTH_METERS = FieldGeometry.FIELD_HALF_WIDTH_METERS;
    public static final double CENTER_FROM_REAR_METERS = .262481;
    public static final double CENTER_FROM_SIDE_METERS = .15835;
    public static final double START_X_METERS = FIELD_HALF_WIDTH_METERS - CENTER_FROM_REAR_METERS;
    public static final double START_Y_METERS = -FIELD_HALF_WIDTH_METERS + CENTER_FROM_SIDE_METERS;
    public static final double START_HEADING_DEG = 180;
    public static final double OPENING_Y_OFFSET_METERS = FieldGeometry.OPENING_Y_OFFSET_METERS;
    /**
     * The selected cell changes only once the robot is this far past the field midline (Y = 0). Switching cells
     * swings the aim 25-65 deg and restarts feed readiness, so position noise or slow driving along the
     * midline must not flip it back and forth.
     */
    public static final double SIDE_SWITCH_HYSTERESIS_METERS = .05;

    /** Cell on the robot's half of the field, keeping {@code previous} within the midline hysteresis band. */
    public static TurretSubsystem.HiveSide nearestSide(double fieldY, TurretSubsystem.HiveSide previous) {
        if (previous == TurretSubsystem.HiveSide.FAR)
            return fieldY < -SIDE_SWITCH_HYSTERESIS_METERS ? TurretSubsystem.HiveSide.AUDIENCE : previous;
        if (previous == TurretSubsystem.HiveSide.AUDIENCE)
            return fieldY > SIDE_SWITCH_HYSTERESIS_METERS ? TurretSubsystem.HiveSide.FAR : previous;
        return fieldY < 0 ? TurretSubsystem.HiveSide.AUDIENCE
                : fieldY > 0 ? TurretSubsystem.HiveSide.FAR : null;
    }
    public static GoalTarget target(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side) {
        // RAISED describes the aim geometry only. Drivers decide when this cell is shootable.
        // Nominal manual-derived tag map; camera fusion still requires explicit validation.
        return new GoalTarget(alliance, side, GoalTarget.HiveState.RAISED,
                TurretSubsystem.getHiveOpeningCenterXInches(alliance) * .0254,
                (side == TurretSubsystem.HiveSide.AUDIENCE ? -1 : 1) * OPENING_Y_OFFSET_METERS,
                ShootingConfiguration.HIGH_GOAL_CENTER_HEIGHT_METERS, true,
                ShootingConfiguration.CALIBRATION_GEOMETRY_ID, HiveTagGeometry.nominalRaisedTags(alliance, side));
    }
    /** Localization may use either alliance and either cell, independently of the selected shot target. */
    public static GoalTarget targetForTag(int id) {
        for (TurretSubsystem.Alliance alliance : TurretSubsystem.Alliance.values()) {
            TurretSubsystem.HiveSide side = TurretSubsystem.getHiveSideForTag(alliance,id);
            if (side != null) return target(alliance,side);
        }
        return null;
    }
    /** Same nominal raised-tag checks for initial localization and corner refinement. */
    public static String raisedTagRejectionReason(TagObservation tag, ShooterMount mount) {
        if (tag == null) return "No metric tag pose";
        if (tag.id < 30 || tag.id > 45) return "Tag " + tag.id + " outside hive field map";
        String rejection = tag.rejectionReason();
        if (rejection != null) return "Tag " + tag.id + ": " + rejection;
        if (!mount.measured) return "Camera mount unmeasured";
        double height = mount.cameraHeight+tag.levelPosition(mount)[2];
        double plane = tag.planeDegrees(mount);
        if (!(Math.abs(plane-HiveTagGeometry.RAISED_TAG_PLANE_DEG) <= PoseHistory.TAG_PLANE_TOLERANCE_DEG))
            return String.format(java.util.Locale.US,"Tag %d: plane %.1f deg; expected %.1f +/- %.1f deg",
                    tag.id,plane,HiveTagGeometry.RAISED_TAG_PLANE_DEG,PoseHistory.TAG_PLANE_TOLERANCE_DEG);
        if (!(Math.abs(height-ShootingConfiguration.HIGH_GOAL_TAG_CENTER_HEIGHT_METERS) <= .1))
            return String.format(java.util.Locale.US,"Tag %d: height %.3f m; expected %.3f +/- 0.100 m",
                    tag.id,height,ShootingConfiguration.HIGH_GOAL_TAG_CENTER_HEIGHT_METERS);
        return null;
    }
    /** Bearing from the turret pivot; independent of shot-profile calibration and camera visibility. */
    public static double yawRadians(MotionSample pose, GoalTarget target, ShooterMount mount) {
        return yawRadians(pose, target.x, target.y, mount);
    }
    /** Same pivot bearing to a bare point (meters, in the pose's frame), e.g. a path-planner aim point. */
    public static double yawRadians(MotionSample pose, double targetX, double targetY, ShooterMount mount) {
        RobotMath.Vec pivot = new RobotMath.Vec(pose.x, pose.y).plus(mount.pivot.rotate(pose.heading));
        return RobotMath.wrap(Math.atan2(targetY - pivot.y, targetX - pivot.x) - pose.heading - mount.zeroYaw);
    }
    /** Horizontal distance from the turret pivot to a point (meters); the shot profile's range input. */
    public static double pivotRangeMeters(MotionSample pose, double targetX, double targetY, ShooterMount mount) {
        RobotMath.Vec pivot = new RobotMath.Vec(pose.x, pose.y).plus(mount.pivot.rotate(pose.heading));
        return Math.hypot(targetX - pivot.x, targetY - pivot.y);
    }
}
