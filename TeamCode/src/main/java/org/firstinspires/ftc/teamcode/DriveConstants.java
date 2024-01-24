package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class DriveConstants {
    public static double kA = 0;
    public static double kStatic = 0;

    public static boolean COMMON_FEED_FORWARD = false;

    public static double MAX_VEL = 50.98709507678554;
    public static double MAX_ACCEL = 35;
    public static double MAX_ANG_VEL = 2.3416948318481445;
    public static double MAX_ANG_ACCEL = Math.toRadians(279.98601117318435);

    public static double KP = 1.1;

    public static double TICKS_PER_REV = 537.6;
    public static double MAX_RPM = 312;

    public static boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(19, 0, 9,13.5);

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 0.99639; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 11.54; // in

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double KI = 2.7;
    public static double KD = 0;
    public static double minIntegralBound = -400;
    public static double maxIntegralBound = 400;

    public static double LATERAL_MULTIPLIER = 1.17602326;

    public static boolean frontLeftInverted = true, frontRightInverted = true, rearRightInverted = true, rearLeftInverted = true;
    public static boolean frontLeftAutonomousInverted = true, frontRightAutonomousInverted = false, rearRightAutonomousInverted = false, rearLeftAutonomousInverted = true;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double DEFAULT_SPEED_PERC = 0.6;
    public static double FAST_SPEED_PERC = 1;
    public static double SLOW_SPEED_PERC = 0.3;

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}
