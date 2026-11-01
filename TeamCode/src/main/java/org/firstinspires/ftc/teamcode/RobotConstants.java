package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    public static final double AUTO_POWER = 0.3;             // drive and turn power, start low
    public static final double AUTO_PERIOD_SECONDS = 30;     // length of autonomous
    public static final double AUTO_OVERDRIVE_SECONDS = 20;  // after this, stop the routine and run onOverdrive()
    public static final double AUTO_START_DELAY_SECONDS = 0; // alliance delay: raise it so we don't run into our partner
    public static final int AUTO_PAUSE_MS = 500;             // short pause after each step

    // ---- Launcher (live-tunable values are in LauncherConfig) ----
    public static final double LAUNCHER_MAX_RPM = 6000;      // free speed of the flywheel motor
    public static final double LAUNCHER_TICKS_PER_REV = 28;  // 6000 RPM goBILDA motor, no gearbox (check the spec sheet)

    // ---- Drivetrain: 435 RPM 12V motors ----
    public static final double DRIVE_MAX_RPM = 435;
    public static final double DRIVE_TICKS_PER_REV = 383.6;  // 435 RPM goBILDA motor (check the spec sheet)

    // ---- Aiming (TODO: tune) ----
    public static final double AIM_KP = 0;
    public static final double AIM_DEADBAND_DEG = 1.0;
}
