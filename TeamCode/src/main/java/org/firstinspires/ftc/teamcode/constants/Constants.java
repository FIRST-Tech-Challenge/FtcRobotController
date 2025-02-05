package org.firstinspires.ftc.teamcode.constants;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public final class Constants {

    public static class FileConstants {
        @SuppressLint("sdCardPath")
        public static final String SD_CARD_PATH
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/team26396/";

        public static final String CONSTANTS_FILE_PATH = SD_CARD_PATH + "Constants/";
        public static final String APRIL_TAG_LOG_FILE_PATH = SD_CARD_PATH + "AprilTagLogs/";
    }

    public static class HardwareConstants {

        public static String FRONT_LEFT_DRIVE_MOTOR_NAME  = "frontLeftMotor";
        public static String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontRightMotor";
        public static String BACK_LEFT_DRIVE_MOTOR_NAME   = "backLeftMotor";
        public static String BACK_RIGHT_DRIVE_MOTOR_NAME  = "backRightMotor";

        public static String IMU_NAME  = "imu";
        public static String HANG_MOTOR_LEFT  = "HM1";
        public static String HANG_MOTOR_RIGHT  = "HM2";
        public static String ARM_MOTOR_NAME  = "liftMotor";
        public static String LINEAR_SLIDE_MOTOR_NAME  = "armMotor";

        public static String ROLL_SERVO = "roll";

        public static String X_YAW_SERVO = "yaw";

        public static String Y_PITCH_SERVO = "pitch";

        public static String CLAW_SERVO = "claw";

    }

    public static class ConfigurationConstants {
        public static boolean USE_VISION  = true;
    }

    public static class DrivebaseConstants {
        public static String FRONT_LEFT_DRIVE_MOTOR_NAME  = "frontLeftDriveMotor";
        public static String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontRightDriveMotor";
        public static String BACK_LEFT_DRIVE_MOTOR_NAME   = "backLeftDriveMotor";
        public static String BACK_RIGHT_DRIVE_MOTOR_NAME  = "backRightDriveMotor";

        public static String IMU_NAME = "imu";
        public static IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        public static double DRIVE_DEAD_ZONE  = 0.05;
        public static double STRAFE_DEAD_ZONE = 0.05;
        public static double TURN_DEAD_ZONE   = 0.05;
    }

    public static class LocalizerConstants {
        // Dead Wheels
        public static String FRONT_DEAD_WHEEL_NAME = "intakeMotor";
        public static String LEFT_DEAD_WHEEL_NAME  = DrivebaseConstants.BACK_LEFT_DRIVE_MOTOR_NAME;
        public static String RIGHT_DEAD_WHEEL_NAME = DrivebaseConstants.BACK_RIGHT_DRIVE_MOTOR_NAME;

        public static double TRACK_WIDTH_INCHES              = 11.21;
        public static double DEAD_WHEEL_DIAMETER_INCHES      = 1.88976;
        public static double DEAD_WHEEL_CIRCUMFERENCE_INCHES = Math.PI * DEAD_WHEEL_DIAMETER_INCHES;
        public static double ENCODER_TICKS_PER_REVOLUTION    = 2000.0;
        public static double Q = 1.0;
        public static double R = 1.0;
        public static int N = 1;

        // Spark Fun
        public static String OPTICAL_ODOMETRY_NAME = "sparkFunOTOS";
        public static double LINEAR_SCALAR = 1.0;
        public static double ANGULAR_SCALAR = 1.0;
        public static int  IMU_CALIBRATION_SAMPLES = 255;
    }

    public static class AprilTagConstants {
        public static int GAIN          = 0;
        public static int EXPOSURE_MS   = 2;
        public static int WHITE_BALANCE = 4000;
    }

    public static class VisionConstants {
        public static int ERODE_PASSES = 5;

        public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(255, 0, 0);

        public static Scalar LOW_HSV_RANGE_BLUE  = new Scalar(97, 50, 0);
        public static Scalar HIGH_HSV_RANGE_BLUE = new Scalar(125, 255, 255);

        public static Scalar LOW_HSV_RANGE_RED_ONE  = new Scalar(160, 100, 0);
        public static Scalar HIGH_HSV_RANGE_RED_ONE = new Scalar(180, 255, 255);

        public static Scalar LOW_HSV_RANGE_RED_TWO  = new Scalar(0, 100, 0);
        public static Scalar HIGH_HSV_RANGE_RED_TWO = new Scalar(10, 255, 255);

        public static Scalar LOW_HSV_RANGE_YELLOW = new Scalar(20, 80, 0);
        public static Scalar HIGH_HSV_RANGE_YELLOW = new Scalar(30, 255, 255);

        public static final Point CV_ANCHOR        = new Point(-1, -1);
        public static final Scalar CV_BORDER_VALUE = new Scalar(-1);
        public static final int CV_BORDER_TYPE     = Core.BORDER_CONSTANT;
    }
}
