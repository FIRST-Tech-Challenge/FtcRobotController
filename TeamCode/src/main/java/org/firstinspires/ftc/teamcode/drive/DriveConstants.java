package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 */
@Config
public class DriveConstants {

    // ======   Main Settings  ===========================================
    public static final boolean RUN_USING_ENCODER = true;
    public static final boolean USE_TWO_WHEEL_LOCALIZER = false;

    /*
     * MOTOR constants
     */
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 300;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(10, 2, 0,
            getMotorVelocityF((MAX_RPM / 60) * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.89; // in
    public static double GEAR_RATIO   = 1.0;  // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH  = 14.4; // For 3 Wheel Kiwi drive, enter drive wheel circle diameter
    public static double kV           = 0.019;
    public static double kA           = 0.003;
    public static double kStatic      = 0;

    public static double MOTOR_INCHES_PER_TICK = (WHEEL_RADIUS * 2 * Math.PI * TICKS_PER_REV);
    public static double MOTOR_IPS_PER_RPM = (GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0);

    /*
     * ODOMETRY TRACKING: Three and Two Wheel constants
     */
    public static double TRACKER_TICKS_PER_REV = 8192;
    public static double TRACKER_WHEEL_RADIUS  = 1.181; // in
    public static double TRACKER_INCHES_PER_TICK = (TRACKER_WHEEL_RADIUS * 2 * Math.PI / TRACKER_TICKS_PER_REV);

    // Three Wheel
    public static double TRACKER_LATERAL_DISTANCE = 12.7;    // in; distance between the left and right wheels
    public static double TRACKER_FORWARD_OFFSET   = 3.00;     // in; X  offset of the lateral wheel

    // Two Wheel
    public static double TRACKER_PARALLEL_X = 0; // X is the up and down direction
    public static double TRACKER_PARALLEL_Y = 6.35; // Y is the strafe direction

    public static double TRACKER_PERPENDICULAR_X = 3.0;
    public static double TRACKER_PERPENDICULAR_Y = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities.
     */
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(5, 2, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 2, 0);

    public static double MAX_VEL   = 46;  // in/s  100% = 58 IPS
    public static double MAX_ACCEL = 80; // in/s/s
    public static double MAX_ANG_VEL = Math.toRadians(240);   // Measured at 540 deg/sec
    public static double MAX_ANG_ACCEL = Math.toRadians(360);
/*
     * Conversion methods.
     */
    public static double motorEncoderTicksToInches(double ticks) {
        return ticks * MOTOR_INCHES_PER_TICK;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * MOTOR_IPS_PER_RPM;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
