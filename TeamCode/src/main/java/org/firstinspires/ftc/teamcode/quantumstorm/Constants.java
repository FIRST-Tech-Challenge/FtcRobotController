package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Constants {

    // =========================================================
    // OPMODE NAMES
    // Autonomous preselectTeleOp must match the TeleOp name exactly
    // =========================================================

    public final static String TELEOP_NAME = "Biobuzz-Quantum Storm Tele-Op";

    // =========================================================
    // DEVICE NAMES
    // Must match the active Robot Configuration exactly (case-sensitive)
    // =========================================================

    // Drive motors
    public final static String FRONT_LEFT_DRIVE = "left_front_drive";
    public final static String FRONT_RIGHT_DRIVE = "right_front_drive";
    public final static String BACK_LEFT_DRIVE = "left_back_drive";
    public final static String BACK_RIGHT_DRIVE = "right_back_drive";

    // TeleOp applies its own joystick sign. Physical testing confirms autonomous
    // needs the opposite longitudinal sign so its initial wall-departure command
    // moves the chassis backward, away from the wall.
    public final static double AUTO_FORWARD_POWER_SIGN = -1.0;

    // Mechanism motors
    public final static String INTAKE_MOTOR = "intake";

    // Single shooter motor
    public final static String SHOOTER_MOTOR = "shooter";

    // CR servos
    public final static String SHOOTER_SERVO = "shooter_servo";

    // Position servo (goBILDA Dual Mode Servo in servo mode) that swings the flower intake
    public final static String FLOWER_SERVO = "flower_servo";

    // Limelight camera + the servo that tilts it
    public final static String LIMELIGHT = "limelight";
    public final static String LIMELIGHT_TILT_SERVO = "limelight_tilt_servo";

    // =========================================================
    // POWER SETTINGS
    // =========================================================

    public final static double INTAKE_POWER = 1.0;

    // Ordinary open-loop power, shared by autonomous and TeleOp.
    public final static double SHOOTER_POWER = 0.54;
    public final static double SHOOTER_POWER_INCREMENT = 0.01;
    public final static double SHOOTER_POWER_MIN = -1.0;
    public final static double SHOOTER_POWER_MAX = 1.0;

    public final static double SHOOTER_SERVO_POWER = 1;

    // A trigger is analog; past this it counts as pressed
    public final static double TRIGGER_THRESHOLD = 0.5;

    // Power used by MecanumDriveTest when spinning one wheel at a time
    public final static double DRIVE_TEST_POWER = 0.3;

    // =========================================================
    // FIELD-CENTRIC DRIVE (Starter Bot) - Control Hub built-in IMU
    // =========================================================

    // A new Control Hub configuration adds the built-in IMU as "imu" automatically
    public final static String IMU = "imu";

    // Which way the Control Hub's REV logo and USB ports face on the robot.
    // TODO: match the real mounting, or heading telemetry will be wrong
    public final static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public final static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // A positive drive() "forward" moves this chassis BACKWARD, so field-centric
    // flips it after rotating into robot directions. Flipping the joystick
    // instead only works when the robot is square to the driver.
    public final static double FIELD_CENTRIC_FORWARD_SIGN = -1.0;

    // Drive speed while RB is held, for precise lining up
    public final static double FIELD_CENTRIC_SLOW_SCALE = 0.4;

    // =========================================================
    // FLOWER INTAKE SERVO (goBILDA Dual Mode Servo 2000-0025-0002)
    // Angles are arm degrees: 0 = arm pointing straight UP (neutral), installed with
    // the servo centered by the goBILDA programmer; negative = toward forward and down.
    // =========================================================

    // In servo mode it turns 300 degrees over a 500..2500 us pulse
    public final static int FLOWER_SERVO_PWM_MIN_US = 500;
    public final static int FLOWER_SERVO_PWM_MAX_US = 2500;
    public final static double FLOWER_SERVO_RANGE_DEG = 300;

    // Servo position (0..1) at arm 0 degrees. 0.5 = the programmer's center (1500 us).
    // TODO: tune so the "Flower Angle" telemetry matches the real arm
    public final static double FLOWER_SERVO_ZERO_POSITION = 0.5;

    // Flip if a more negative angle moves the arm BACKWARD instead of forward/down
    public final static boolean FLOWER_SERVO_REVERSED = true;

    // Hard limits: the arm is never commanded outside these
    public final static double FLOWER_ANGLE_MIN_DEG = -135;
    public final static double FLOWER_ANGLE_MAX_DEG = 0;

    public final static double FLOWER_UP_DEG = 0;       // neutral, pointing up; also where it goes on INIT
    public final static double FLOWER_DOWN_DEG = -135;  // forward and down, for intaking
    public final static double FLOWER_NUDGE_DEG = 2;    // per LB + D-pad press on gamepad 2, for tuning

    // =========================================================
    // LIMELIGHT
    // =========================================================

    // Tilt servo positions (0..1). TODO: find with "Servo Position Helper"
    public final static double LIMELIGHT_TILT_UP = 0.3;     // look at the hive AprilTag
    public final static double LIMELIGHT_TILT_DOWN = 0.7;   // look at balls on the floor
    public final static double LIMELIGHT_SERVO_TRAVEL_DEGREES = 300.0;
    public final static double LIMELIGHT_SERVO_NEUTRAL_POSITION = 0.5;
    public final static double LIMELIGHT_SERVO_MAX_OFFSET_DEGREES = 120.0;
    public final static double LIMELIGHT_COLOR_PIPELINE_DWELL_SEC = 3.0;

    // Pipeline numbers as set up in the Limelight web UI
    // The configured Limelight profile uses slot 1 for Hive AprilTags and
    // slot 0 for the yellow-pollen color pipeline.
    public final static int LIMELIGHT_APRILTAG_PIPELINE = 1;
    public final static int LIMELIGHT_COLOR_PIPELINE = 0;
    public final static int LIMELIGHT_YELLOW_PIPELINE = LIMELIGHT_COLOR_PIPELINE;
    // Additional color slots must match the Limelight web UI configuration.
    public final static int LIMELIGHT_BLUE_PIPELINE = 2;
    public final static int LIMELIGHT_RED_PIPELINE = 3;
    // Keep the old name available for existing ball-search code.
    public final static int LIMELIGHT_BALL_PIPELINE = LIMELIGHT_COLOR_PIPELINE;

    /**
     * Practice-only switch for testing a printed AprilTag away from the field.
     * When true, Team 1 permits preload feeding from any fresh AprilTag frame.
     * Set false before a match to restore official own-Hive ID/height/pose checks.
     */
    // Practice field / printed-tag mode. Keep false for the normal robot
    // routine: shooting then requires the correct Hive-side AprilTag IDs.
    public final static boolean PRACTICE_ALLOW_ANY_FRESH_APRILTAG = false;

    // Team One preload shots require calibrated own raised-Cell tags.
    // Garden pickup uses a slow, odometry-bounded stroke after the approach.
    public final static double AUTO_GARDEN_ADVANCE_IN = 18;
    public final static double AUTO_GARDEN_ADVANCE_SEC = 5;
    // Real robot Garden stroke: enough forward command to pull pollen into the intake.
    public final static double AUTO_GARDEN_ADVANCE_POWER = .30;
    /** First straight Garden pass before the back-right-forward second pass. */
    public final static double AUTO_GARDEN_FIRST_PASS_SEC = 2.0;
    /** Pinpoint heading required for every Team 1 straight Garden intake pass. */
    public final static double TEAM_ONE_GARDEN_INTAKE_HEADING_DEG = 90.0;
    public final static boolean TEAM_ONE_HIVE_HEIGHT_CALIBRATED = true;
    public final static int TEAM_ONE_TAG_HEIGHT_AXIS = 2; // robot-space Z, verify on hardware
    public final static double TEAM_ONE_ROBOT_ORIGIN_HEIGHT_IN = 0;
    public final static double TEAM_ONE_UPPER_TAG_HEIGHT_IN = 70.0;
    public final static double TEAM_ONE_LOWER_TAG_HEIGHT_IN = 36.57;
    // Front-mounted camera/intake must face the Hive after rear-facing preload shots.
    public final static double TEAM_ONE_TAG_VIEW_TURN_DEG = 180;

    // Team Two must see raised far-Cell tags WHILE at the opposite-wall shot pose.
    // A rear-facing/aimable camera and measured robot-space heights are required.
    // False deliberately prevents shooting, but does not prevent timed parking.
    public final static boolean TEAM_TWO_HIVE_HEIGHT_CALIBRATED = true;
    public final static int TEAM_TWO_TAG_HEIGHT_AXIS = 2;
    public final static double TEAM_TWO_ROBOT_ORIGIN_HEIGHT_IN = 0;
    public final static double TEAM_TWO_UPPER_TAG_HEIGHT_IN = 70.0;
    public final static double TEAM_TWO_LOWER_TAG_HEIGHT_IN = 36.57;
    // These optional slots must provide BOTH normalized color and distance readings.
    // No device names are guessed, and no ball counts are inferred from intake time.
    public final static String[] TEAM_TWO_POLLEN_SLOT_SENSOR_NAMES = {};
    public final static boolean TEAM_TWO_INVENTORY_CALIBRATED = false;
    public final static double TEAM_TWO_SLOT_OCCUPIED_MAX_CM = 4.0;
    public final static double TEAM_TWO_SLOT_MIN_ALPHA = .02;
    public final static double TEAM_TWO_SLOT_YELLOW_BLUE_RATIO = 1.5;
    public final static double TEAM_TWO_SLOT_YELLOW_RG_MIN = .65;
    public final static double TEAM_TWO_SLOT_YELLOW_RG_MAX = 1.65;

    // =========================================================
    // PINPOINT ODOMETRY
    // =========================================================

    public final static String PINPOINT = "odometry_1";   // I2C device name in Robot Configuration

    // Pod offsets from the robot's center (tracking point), in mm.
    // X pod (forward-counting): sideways distance, LEFT of center = positive.
    // Y pod (strafe-counting):  forward distance, FORWARD of center = positive.
    // TODO: measure on the robot
    public final static double PINPOINT_X_POD_OFFSET_MM = -170;
    public final static double PINPOINT_Y_POD_OFFSET_MM = - 150;

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
    public final static Pose2D RED_2_START_POSE  = fieldPose(-12.75, 62.9, 90);
    public final static Pose2D BLUE_1_START_POSE = fieldPose(0, 0, 0);
    public final static Pose2D BLUE_2_START_POSE = fieldPose(12.75, -62.9, -90);

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

    // Feed pulse must be calibrated to advance ONE pocket, not two balls.
    public final static double SPINDEXER_FEED_PULSE_SEC = org.firstinspires.ftc.teamcode.biobuzz.FeedCadence.PULSE_SECONDS;
    public final static double SPINDEXER_SHOT_GAP_SEC = org.firstinspires.ftc.teamcode.biobuzz.FeedCadence.GAP_SECONDS;

    public final static int AUTO_PRELOAD_BALLS = 4;                // Pollen loaded before the match
    public final static int AUTO_BALLS_TO_SHOOT = 4;               // shoot only when holding this many
    public final static double AUTO_PARK_TIME_SEC = 26.0;          // after this, go park (30 s auto, leaves time to drive)
    public final static int AUTO_MAX_RETRIES = 3;                  // recoverable errors before giving up and parking
    public final static double AUTO_APRILTAG_TIMEOUT_SEC = 3.0;    // no tag / not aligned -> ERROR
    public final static double AUTO_SHOOTER_SPINUP_SEC = 1.0;      // flywheel spin-up before feeding
    /** Team 1 keeps feeding longer so all four preloads can clear before Garden travel. */
    public final static double AUTO_TEAM_ONE_FEED_TIME_SEC = 5.0;
    /** Match Team 1's longer feed window so all four preloads can clear. */
    public final static double AUTO_TEAM_TWO_FEED_TIME_SEC = AUTO_TEAM_ONE_FEED_TIME_SEC;
    public final static double AUTO_AIM_TOLERANCE_DEG = 2.0;       // AprilTag tx within this = aligned
    public final static double AUTO_AIM_TURN_KP = 0.02;            // turn power per degree of tag offset
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
