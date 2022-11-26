package com.acmerobotics.roadrunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * Motor count.  This value should either be 2 or 4, to account for either 2 wheel or 4 wheel
     * drive robots.  Mechanum drivetrains should always have 4 motors, tank drivetrains may be
     * 2 or 4 motor drivetrains.  Other drivetrains are not supported.
     */
    public static int MOTOR_COUNT = 4;

    /*
     * Names for your motors from the robot configuration, if you only have a 2 wheel drive
     * drivetrain then the rear variants are ignored in the tank drive implementation.
     */
    public static String LEFT_FRONT = "leftFront";
    public static String RIGHT_FRONT = "rightFront";
    public static String LEFT_REAR = "leftRear";
    public static String RIGHT_REAR = "rightRear";

    public static String IMU = "imu";

    /*
     * Count of encoders.  Either 3 or 2.
     */
    public static int ENCODER_COUNT = 3;

    public enum DrivetrainKind {
        MECANUM,
        TANK,
    }
    public static DrivetrainKind DRIVETRAIN_KIND = DrivetrainKind.MECANUM;

    /*
     * These are motor constants that should be listed online for your motors.
     *
     * HD Hex Motor
     *   Bare Motor RPM: 6000
     *   Bare Motor Ticks per Rev: 28
     *
     *   5:1 + 4:1 reduction = 18.9 reduction
     *   6000 / 18.9 = 317.4 RPM
     *   28 * 18.9 = 529.2 Ticks per shaft revolution
     */
    public static double TICKS_PER_REV = 529.2;
    public static double MAX_RPM = 317.4;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     *
     * PIDFCoefficients(double p, double i, double d, double f)
     */
    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 8, 13.5);
    // public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(10, 0, 8, 13.2);
    // MAX VEL = 40
    // public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(20, 0, 0, 13.7);
            // getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_DIAMETER_MM = 75;
    public static double IN_PER_MM = 0.03937;
    public static double WHEEL_RADIUS = (WHEEL_DIAMETER_MM * IN_PER_MM) / 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    /*
     * The real track width is 15, but the tuner indicates that 31.15 is the right value
     * 15.275
     */
    public static double TRACK_WIDTH = 14.9; // in

    public static double WHEEL_BASE = 11.5;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    // public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    // public static double kV = 0.018633;
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    // public static double kA = 0.002;
    public static double kA = 0.0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     *
     * Measured MAX_VEL = 45.5
     * Measured kF = 13.5
     *
     * Measured MAX_ANG_VEL = 214.3
     * Measured MAX_ANG_ACCEL = 3.8 radians
     */
    public static double MAX_VEL = 35;
    public static double MAX_ACCEL = 35;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
