package org.firstinspires.ftc.teamcode.Roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {
// todo not used currently













    /*
     * Motor and Encoder Constants
     */
    public static final double TICKS_PER_REV = 537.6; // Example for specific motor model
    public static final double MAX_RPM = 312.5;

    /*
     * Encoder Configuration
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * Physical Constants
     */
    public static double WHEEL_RADIUS = 0.98; // inches
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 12.46; // inches // todo measure this

    /*
     * Feedforward Parameters
     */
    public static double kV = 0; // Based on empirical testing
    public static double kA = 0;  // Based on empirical testing
    public static double kStatic = 0; // Based on empirical testing

    /*
     * Motion Constraints
     * Set to ~80% of the robot's actual capabilities for safe and reliable operation.
     */
    public static double MAX_VEL = 52.57; // inches per second
    public static double MAX_ACCEL = 52.57; // inches per second squared
    public static double MAX_ANG_VEL = Math.toRadians(245.87); // radians per second
    public static double MAX_ANG_ACCEL = Math.toRadians(245.87); // radians per second squared

    /*
     * REV Hub Orientation (for correct robot control)
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    /*
     * Utility Methods
     */
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}
