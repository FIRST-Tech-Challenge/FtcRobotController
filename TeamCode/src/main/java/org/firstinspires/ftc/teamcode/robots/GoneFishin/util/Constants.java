package org.firstinspires.ftc.teamcode.robots.GoneFishin.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config(value = "FFConstants")
public class Constants {


    //Subsystems
    //----------------------------------------------------------------------------------------------

    //Crane Subsystems
    public static final int BULB_SERVO_OPEN= 0;
    public static final int BULB_SERVO_CLOSED = 0;
    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    // driveTrain

    public static final double WHEELS_WIDTH = 12.132362205;
    public static final double DIFF_TICKS_PER_INCH = 3600;




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



    public static double diffInchesToEncoderTicks(double inches) {
        return inches * DIFF_TICKS_PER_INCH;
    }


    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}
