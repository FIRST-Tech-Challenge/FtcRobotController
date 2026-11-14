package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * One place for every hardware name and tunable number.
 * Change values here, not inside subsystems or OpModes.
 */
public final class RobotConstants {

    private RobotConstants() {} // constants only; never instantiated

    // ---- Hardware map names (must match the Driver Hub configuration) ----
    public static final String FRONT_LEFT_MOTOR  = "FL";
    public static final String BACK_LEFT_MOTOR   = "BL";
    public static final String FRONT_RIGHT_MOTOR = "FR";
    public static final String BACK_RIGHT_MOTOR  = "BR";
    public static final String PINPOINT          = "pinpoint";
    public static final String LIMELIGHT         = "limelight";
    public static final String LAUNCHER_MOTOR    = "launcher";
    public static final String FEEDER_SERVO      = "feeder";
    public static final String INTAKE_MOTOR      = "intake";
    public static final String LIME_LITE_SERVO = "limelightservo";

    // ---- Drive motor directions (flip one if that wheel spins the wrong way) ----
    public static final DcMotor.Direction FRONT_LEFT_DIRECTION  = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction BACK_LEFT_DIRECTION   = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction FRONT_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction BACK_RIGHT_DIRECTION  = DcMotor.Direction.FORWARD;

    // ---- Odometry ----
    public static final double PINPOINT_X_OFFSET_MM = -8.0; // 8 mm Right of center
    public static final double PINPOINT_Y_OFFSET_MM = 1.0;  // 1 mm Up (Forward) of center

    // Which goBILDA pod is on the robot, and which way each pod counts.
    // Flip a direction to REVERSED if X doesn't go up going forward, or Y doesn't go up going left.
    public static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // ---- Autonomous ----
    public static final double AUTO_POWER = 0.5;             // drive and turn power, start low
    public static final double AUTO_PERIOD_SECONDS = 30;     // length of autonomous
    public static final double AUTO_OVERDRIVE_SECONDS = 40;  // after this, stop the routine and run onOverdrive()
    public static final double AUTO_START_DELAY_SECONDS = 0; // alliance delay: raise it so we don't run into our partner
    public static final int AUTO_PAUSE_MS = 250;             // short pause after each step

    // ---- Launcher + intake: all speeds, PIDF, feeder positions and timing are in LauncherConfig ----
    public static final DcMotor.Direction LAUNCHER_MOTOR_DIRECTION = DcMotor.Direction.REVERSE; // confirmed on the robot

    // ---- Drivetrain: 435 RPM 12V motors ----
    public static final double DRIVE_MAX_RPM = 435;
    public static final double DRIVE_TICKS_PER_REV = 383.6;  // 435 RPM goBILDA motor (check the spec sheet)

    // ---- Aiming: gains and deadbands are in AimConfig ----

    // ---- AprilTag fiducial ID ranges (Limelight pipeline), used by Vision.isHiveTipped and aiming ----
    // From the game manual (Fig 9-17). "Scoring" = far side from audience.
    public static final int TAG_RED_SCORING_MIN = 30;
    public static final int TAG_RED_SCORING_MAX = 33;
    public static final int TAG_RED_AUDIENCE_MIN = 34;
    public static final int TAG_RED_AUDIENCE_MAX = 37;
    public static final int TAG_BLUE_AUDIENCE_MIN = 38;
    public static final int TAG_BLUE_AUDIENCE_MAX = 41;
    public static final int TAG_BLUE_SCORING_MIN = 42;
    public static final int TAG_BLUE_SCORING_MAX = 45;
}
