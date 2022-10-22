package org.firstinspires.ftc.teamcode.robots.reachRefactor.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config(value = "FFConstants")
public class Constants {

    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    // distance measurements
    public static final double MIN_CHASSIS_LENGTH = 14.8;
    public static final double MAX_CHASSIS_LENGTH = 25;
    public static final double TRACK_WIDTH = 13.5;
    public static final double DISTANCE_SENSOR_TO_FRONT_AXLE = 2.755906;
    public static final double DISTANCE_TARGET_TO_BACK_WHEEL = 8.75;
    public static final double SHOULDER_TO_ELBOW = 14.031496;
    public static final double ELBOW_TO_WRIST = 11.0236;

    public static final double HIGH_TIER_SHIPPING_HUB_HEIGHT = 20.25;
    public static final double MIDDLE_TIER_SHIPPING_HUB_HEIGHT = 16.625;
    public static final double LOW_TIER_SHIPPING_HUB_HEIGHT = 5.75;
    public static final double HIGH_TIER_RADIUS = 6;
    public static final double MIDDLE_TIER_RADIUS = 7.5;
    public static final double LOW_TIER_RADIUS = 9;

    public static final double SHOULDER_AXLE_TO_GROUND_HEIGHT = 13.75;

    public static double DIFF_WHEEL_RADIUS = 6.5 / 16.0 + 2.991;
    public static double SWERVE_WHEEL_RADIUS = 6.5 / 16.0 + 3.175;

    // ratios
    public static double SWIVEL_TICKS_PER_REVOLUTION = 1696.5;
    public static double DIFF_TICKS_PER_REV = 768;
    public static double SWERVE_TICKS_PER_REV = 768;
    public static double DIFF_TICKS_PER_INCH = 30.7708333333;
    public static double SWERVE_TICKS_PER_INCH = 30.7708333333;

    //----------------------------------------------------------------------------------------------
    // Control Constants
    //----------------------------------------------------------------------------------------------

    public static double EPSILON = 1e-6; // small value used for the approximately equal calculation in MathUtils
    public static double TRIGGER_DEADZONE = 0.1; // gamepad trigger values below this threshold will be ignored
    public static double JOYSTICK_DEADZONE = 0.05;
    public static double LOW_BATTERY_VOLTAGE = 12.5;

    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 10;
    public static double MAX_ANG_VEL = Math.toRadians(120);
    public static double MAX_ANG_ACCEL = Math.toRadians(60);

    public static final double MAX_RPM = 150;
    public static boolean USE_CUSTOM_VELOCITY_PID = true;
    public static PIDFCoefficients DIFF_MOTOR_VELOCITY_PID = new PIDFCoefficients(10, 3, 0,
            getMotorVelocityF(MAX_RPM / 60 * 1120));
    public static PIDFCoefficients SWERVE_VELOCITY_PID = new PIDFCoefficients(10, 3, 0,
            getMotorVelocityF(MAX_RPM / 60 * 1120));

    //----------------------------------------------------------------------------------------------
    // Simulation
    //----------------------------------------------------------------------------------------------
    public static boolean USE_MOTOR_SMOOTHING = false;

    //----------------------------------------------------------------------------------------------
    // Enums
    //----------------------------------------------------------------------------------------------
    public enum Alliance {
        RED(1), BLUE(-1);

        private final int mod;

        Alliance(int mod) {
            this.mod = mod;
        }

        public int getMod() {
            return mod;
        }
    }

    public enum Position {
        START_RED_UP(new Pose2d(12, -72, Math.toRadians(270))),
        START_RED_DOWN(new Pose2d(-36, -72, Math.toRadians(270))),
        START_BLUE_UP(new Pose2d(12, 72, Math.toRadians(90))),
        START_BLUE_DOWN(new Pose2d(-36, 72, Math.toRadians(90))),

        RED_SHIPPING_HUB(new Pose2d(-12, -24)),
        BLUE_SHIPPING_HUB(new Pose2d(-12, 24));

        private final Pose2d pose;

        Position(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    public static double diffEncoderTicksToInches(double ticks) {
        return ticks / DIFF_TICKS_PER_INCH;
    }

    public static double swerveEncoderTicksToInches(double ticks) {
        return ticks / SWERVE_TICKS_PER_INCH;
    }

    public static double diffInchesToEncoderTicks(double inches) {
        return inches * DIFF_TICKS_PER_INCH;
    }

    public static double swerveInchesToEncoderTicks(double inches) {
        return inches * SWERVE_TICKS_PER_INCH;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}
