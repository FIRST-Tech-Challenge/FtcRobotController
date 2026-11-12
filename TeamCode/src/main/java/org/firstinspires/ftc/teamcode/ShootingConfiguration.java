package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Enter measured values here. Numeric examples remain explicitly ineligible for automatic shots. */
public final class ShootingConfiguration {
    private ShootingConfiguration() { }
    // Drawing-derived nominal raised geometry replaces the earlier team height estimates.
    public static final double HIGH_GOAL_CENTER_HEIGHT_METERS = FieldGeometry.OPENING_AIM_HEIGHT_METERS;
    public static final double HIGH_GOAL_TAG_CENTER_HEIGHT_METERS = HiveTagGeometry.DRAWING_TAG_HEIGHT_METERS;
    // Opening plane relative to horizontal, not a launch/hood angle. Retained for shot calibration.
    public static final double HIGH_GOAL_OPENING_PLANE_DEG = FieldGeometry.OPENING_PLANE_DEG;
    public static final double TURRET_PIVOT_FORWARD_MM = 50.68184865;
    public static final double TURRET_PIVOT_LEFT_MM = 0;
    public static final double CAMERA_FORWARD_FROM_PIVOT_MM = -99.53073974;
    public static final double CAMERA_LEFT_FROM_PIVOT_MM = 0;
    public static final double CAMERA_YAW_FROM_MUZZLE_DEG = 0;
    // Limelight lens axis above horizontal. Team-measured 2026-11-05/06: front glass/face 35 deg from
    // horizontal (55 deg from vertical), so the lens axis is 55 deg up. Was 30. The 2026-11-05 photo
    // frame (tags 1.12 m along the axis) reads the field-guide tag height only with 55 deg.
    public static final double CAMERA_PITCH_UP_DEG = 55;
    // Lens height above the tiles, team-measured 2026-11-06 (was 335). Camera/pivot planar geometry confirmed by the team.
    public static final double CAMERA_HEIGHT_MM = 337;
    // Camera/muzzle offsets use turret-local +X along the muzzle and +Y left.
    // Ball exit is centered on the pivot in plan view and does not move significantly with hood.
    // Zero yaw (servo 0.5 home) faces robot-right; the offset is set once in TurretSubsystem.
    public static final ShooterMount MOUNT = new ShooterMount(
            new Vec(TURRET_PIVOT_FORWARD_MM / 1000, TURRET_PIVOT_LEFT_MM / 1000),
            new Vec(CAMERA_FORWARD_FROM_PIVOT_MM / 1000, CAMERA_LEFT_FROM_PIVOT_MM / 1000),
            new Vec(0, 0), Math.toRadians(TurretSubsystem.TURRET_ZERO_ROBOT_HEADING_OFFSET_DEG),
            Math.toRadians(CAMERA_YAW_FROM_MUZZLE_DEG),
            Math.toRadians(CAMERA_PITCH_UP_DEG), CAMERA_HEIGHT_MM / 1000, true);
    // Legacy diagnostic tag-to-opening transform. SDK cluster coordinates use a different
    // origin/axes from Limelight individual-tag poses; verify that conversion before enabling.
    public static final OpeningGeometry OPENING = new OpeningGeometry("TU06-high-transform-unverified", false, 0, 0);
    public static final String CALIBRATION_GEOMETRY_ID = "TU06-drawing-midheight-1.51257m-tag1.2676124m-plane60deg-hood-home-offset2deg";
    // No measured low-state pose supplied: unsupported low observations remain TRANSITIONING.
    public static final double LOWERED_TAG_HEIGHT_METERS = Double.NaN, LOWERED_TAG_PLANE_DEG = Double.NaN;
    public static final boolean GOAL_OBSERVATION_VALIDATED = false; // Camera settings / pose quality / tolerances await validation.
    public static final boolean FIELD_RELOCALIZATION_ENABLED = true; // Team-authorized TU06 nominal map + documented camera mount.
    public static final boolean LEVEL_CHASSIS_ASSUMED = true; // No chassis pitch/roll sensor is supplied.
    public static final ShotProfile PROFILE = ShotProfile.example();
    public static final boolean RELEASE_TIMING_MEASURED = false;
    public static final boolean VISION_TIMING_MEASURED = false; // Legacy diagnostic firing only; TeleOp uses SDK capture timing.
    public static final double ADDITIONAL_VISION_TRANSPORT_SECONDS = 0;
    public static final double GATE_TO_FEED_DELAY_SECONDS = GateSubsystem.FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS;
    public static final double FEED_TO_EXIT_SECONDS = .12;
    // Gate stays open throughout the command-to-feed delay and this two-stage feed window.
    // Starting values for loaded testing, not measured release calibration.
    public static final double FEED_WINDOW_SECONDS = .50, RECOVERY_SECONDS = .35, READY_STABLE_SECONDS = .15;
    public static final boolean STARTING_FIELD_POSE_CONFIGURED = false;
    public static final double START_X_METERS = 0, START_Y_METERS = 0, START_HEADING_RAD = 0;
    public static GoalTarget target(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side) {
        // Legacy diagnostic entry point: observations must establish the target and cell state.
        // Competition uses the nominal openings and tag map in TeleOpFieldReference instead.
        return GoalTarget.unknown(alliance, side);
    }
    public static FiringCoordinator firing() {
        // Shared by competition, calibration and transfer/shooter tests; timings remain estimates.
        return new FiringCoordinator(GATE_TO_FEED_DELAY_SECONDS, FEED_TO_EXIT_SECONDS, FEED_WINDOW_SECONDS, RECOVERY_SECONDS, READY_STABLE_SECONDS);
    }
}
