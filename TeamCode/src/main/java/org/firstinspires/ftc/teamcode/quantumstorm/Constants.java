package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Constants {

    // =========================================================
    // DEVICE NAMES
    // Must match the active Robot Configuration exactly (case-sensitive)
    // =========================================================

    // Drive motors
    public final static String FRONT_LEFT_DRIVE = "left_front_drive";
    public final static String FRONT_RIGHT_DRIVE = "right_front_drive";
    public final static String BACK_LEFT_DRIVE = "left_back_drive";
    public final static String BACK_RIGHT_DRIVE = "right_back_drive";

    // Mechanism motors
    public final static String INTAKE_MOTOR = "intake";

    // Shooter 1 keeps the original "shooter" port name; shooter 2 is new
    public final static String SHOOTER_MOTOR = "shooter";
    public final static String SHOOTER_2_MOTOR = "shooter2";

    // CR servos
    public final static String RIGHT_INTAKE_SERVO = "right_intake_servo";
    public final static String LEFT_INTAKE_SERVO = "left_intake_servo";
    public final static String SHOOTER_SERVO = "shooter_servo";

    // Limelight camera + the servo that tilts it
    public final static String LIMELIGHT = "limelight";
    public final static String LIMELIGHT_TILT_SERVO = "limelight_tilt_servo";

    // =========================================================
    // POWER SETTINGS
    // =========================================================

    public final static double INTAKE_POWER = 1.0;

    // Used by the autonomous Shooter mechanism
    public final static double SHOOTER_POWER = 0.60;

    // TeleOp starting powers. Shooter 1 runs negative because its motor
    // is mounted facing the opposite way from shooter 2.
    public final static double SHOOTER_1_POWER = -0.60;
    public final static double SHOOTER_2_POWER = 0.60;

    public final static double SHOOTER_POWER_INCREMENT = 0.05;

    // Clamp bounds for the adjustable TeleOp shooter powers
    public final static double SHOOTER_POWER_MIN = -1.0;
    public final static double SHOOTER_POWER_MAX = 1.0;

    public final static double SHOOTER_SERVO_POWER = 1.0;

    // A trigger is analog; past this it counts as pressed
    public final static double TRIGGER_THRESHOLD = 0.5;

    // Power used by MecanumDriveTest when spinning one wheel at a time
    public final static double DRIVE_TEST_POWER = 0.3;

    // =========================================================
    // LIMELIGHT
    // =========================================================

    // Tilt servo positions (0..1). TODO: find with "Servo Position Helper"
    public final static double LIMELIGHT_TILT_UP = 0.3;     // legacy position for existing camera code
    public final static double LIMELIGHT_TILT_DOWN = 0.7;   // position for color pipelines
    // Standard 180-degree servo mapping: 30 degrees / 180 degrees = 0.167 position.
    public final static double LIMELIGHT_APRILTAG_TILT_POSITION = 30.0 / 180.0;
    public final static double LIMELIGHT_SERVO_TRAVEL_DEGREES = 300.0;
    public final static double LIMELIGHT_SERVO_NEUTRAL_POSITION = 0.5;
    public final static double LIMELIGHT_SERVO_MAX_OFFSET_DEGREES = 120.0;
    public final static double LIMELIGHT_COLOR_PIPELINE_DWELL_SEC = 10.0;
    public final static double LIMELIGHT_COLOR_SCAN_CYCLE_SEC = 30.0;

    // Pipeline indices must match the pipelines configured in the Limelight web UI.
    public final static int LIMELIGHT_APRILTAG_PIPELINE = 0;
    public final static int LIMELIGHT_YELLOW_PIPELINE = 1;
    public final static int LIMELIGHT_BLUE_PIPELINE = 2;
    public final static int LIMELIGHT_RED_PIPELINE = 3;
    // Existing ball-search code uses the blue color pipeline until its target is specified.
    public final static int LIMELIGHT_COLOR_PIPELINE = LIMELIGHT_BLUE_PIPELINE;
    public final static int LIMELIGHT_BALL_PIPELINE = LIMELIGHT_COLOR_PIPELINE;

    // =========================================================
    // PINPOINT ODOMETRY
    // =========================================================

    public final static String PINPOINT = "odometry_1";   // I2C device name in Robot Configuration

    // Pod offsets from the robot's center (tracking point), in mm.
    // X pod (forward-counting): sideways distance, LEFT of center = positive.
    // Y pod (strafe-counting):  forward distance, FORWARD of center = positive.
    // TODO: measure on the robot
    public final static double PINPOINT_X_POD_OFFSET_MM = 0;
    public final static double PINPOINT_Y_POD_OFFSET_MM = 0;

    // Robot uses the goBILDA 4-Bar Odometry Pack (2 pods + 1 Pinpoint computer)
    public final static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // X pod must count UP driving forward, Y pod must count UP strafing LEFT.
    // Flip to REVERSED if the "Test: Pinpoint Pose" OpMode shows the wrong sign.
    public final static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public final static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // =========================================================
    // FIELD LOCATIONS
    // FTC field coordinates: origin at the field center, inches,
    // field spans -72..+72 on both axes; heading in degrees, CCW positive.
    // Check the season's game manual for which way +X and +Y point.
    // TODO: measure each one with "Test: Pinpoint Pose" (push the robot
    //    to the spot and copy the X / Y / heading telemetry here).
    //    How the field locations work
    //
    //    FTC uses a standard field coordinate system:
    //      - The origin (0, 0) is the center of the field, and distances are in inches. The field runs from −72 to +72 in each direction.
    //      - Heading is in degrees, counter-clockwise positive.
    //      - Which way +X and +Y point is set in each season's game manual, so check that before measuring.
    //
    //    The easiest way to get each location is to measure it with the robot rather than calculate it. I added a test OpMode for this:
    //      1. Place the robot at the starting spot (e.g. RED_1_START_POSE) and press INIT. Keep it still while the Pinpoint calibrates.
    //      2. Drive or push it to the Garden and read X / Y / Heading from the Driver Hub.
    //      3. Copy those numbers into Constants.RED_GARDEN_POSE etc. Repeat for each location and each alliance.
    // =========================================================

    // Starting spots: one per alliance + starting position (Team One / Team Two)
    public final static Pose2D RED_1_START_POSE  = fieldPose(0, 0, 0);
    public final static Pose2D RED_2_START_POSE  = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_1_START_POSE = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_2_START_POSE = fieldPose(0, 0, 0);

    // Hive 1 for each start = the same-color hive closest to that start,
    // at the spot/heading the robot shoots from. (4 hives on the field: 2 red, 2 blue.)
    public final static Pose2D RED_1_HIVE1_POSE  = fieldPose(0, 0, 0);
    public final static Pose2D RED_2_HIVE1_POSE  = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_1_HIVE1_POSE = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_2_HIVE1_POSE = fieldPose(0, 0, 0);

    // Where to shoot from once Hive 1 has tipped and its active Cell is on the far face
    public final static Pose2D RED_1_HIVE1_FLIPPED_POSE  = fieldPose(0, 0, 0);
    public final static Pose2D RED_2_HIVE1_FLIPPED_POSE  = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_1_HIVE1_FLIPPED_POSE = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_2_HIVE1_FLIPPED_POSE = fieldPose(0, 0, 0);

    // Red alliance field elements
    public final static Pose2D RED_GARDEN_POSE = fieldPose(0, 0, 0);
    public final static Pose2D RED_FLOWER_POSE = fieldPose(0, 0, 0);
    public final static Pose2D RED_PARK_POSE   = fieldPose(0, 0, 0);   // touching the perimeter

    // Blue alliance field elements
    public final static Pose2D BLUE_GARDEN_POSE = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_FLOWER_POSE = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_PARK_POSE   = fieldPose(0, 0, 0);  // touching the perimeter

    // =========================================================
    // AUTONOMOUS ROUTINE
    // =========================================================

    public final static int AUTO_PRELOAD_BALLS = 4;                // Pollen loaded before the match
    public final static int AUTO_BALLS_TO_SHOOT = 4;               // shoot only when holding this many
    public final static double AUTO_PARK_TIME_SEC = 26.0;          // after this, go park (30 s auto, leaves time to drive)
    public final static int AUTO_MAX_RETRIES = 3;                  // recoverable errors before giving up and parking
    public final static double AUTO_APRILTAG_TIMEOUT_SEC = 3.0;    // no tag / not aligned -> ERROR
    public final static double AUTO_SHOOTER_SPINUP_SEC = 1.0;      // flywheel spin-up before feeding
    public final static double AUTO_FEED_TIME_SEC = 2.0;           // feed time to empty the indexer
    public final static double AUTO_AIM_TOLERANCE_DEG = 2.0;       // AprilTag tx within this = aligned
    public final static double AUTO_AIM_TURN_KP = 0.02;            // turn power per degree of tag offset
    public final static double AUTO_AIM_MIN_TURN_POWER = 0.15;    // tune on robot to overcome static friction
    public final static double AUTO_PICKUP_TIMEOUT_SEC = 3.0;      // time spent intaking at garden/flower
    public final static double AUTO_GROUND_SEARCH_TIMEOUT_SEC = 6.0;
    public final static double AUTO_GROUND_DRIVE_POWER = 0.3;      // forward speed toward a seen ball
    public final static double AUTO_GROUND_SEARCH_TURN_POWER = 0.2; // spin speed when no ball seen
    public final static double AUTO_GROUND_TURN_KP = 0.02;         // turn power per degree of ball offset

    // =========================================================
    // AUTONOMOUS MOVE-TO-POSE TUNING (used by Navigator)
    // =========================================================

    public final static double MOVE_POSITION_TOLERANCE_IN = 1.0;   // "arrived" when this close
    public final static double MOVE_HEADING_TOLERANCE_DEG = 2.0;
    public final static double MOVE_KP_TRANSLATION = 0.05;         // power per inch of error
    public final static double MOVE_KP_HEADING = 0.02;             // power per degree of error
    public final static double MOVE_MAX_POWER = 0.6;
    public final static double MOVE_TIMEOUT_SEC = 5.0;             // give up -> ERROR state

    private static Pose2D fieldPose(double xInches, double yInches, double headingDegrees) {
        return new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees);
    }
}
