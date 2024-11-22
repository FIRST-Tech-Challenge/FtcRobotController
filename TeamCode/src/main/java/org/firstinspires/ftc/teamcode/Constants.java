package org.firstinspires.ftc.teamcode; 

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public abstract class Constants {
    //miscellaneous
    public static final int INCH_TO_TILE = 24;

    public static final double MOTOR_TICKS_PER_REVOLUTION = 751.8;

    public enum PIDSubsystemState {
        MANUAL,
        MOVING_TO_TARGET,
        AT_TARGET
    }

    public static abstract class DriveConstants {
        public static final String FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";
        public static final String FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";
        public static final String BACK_LEFT_MOTOR_NAME = "rearLeftMotor";
        public static final String BACK_RIGHT_MOTOR_NAME = "rearRightMotor";
        public static final String IMU_NAME = "imu";

        public static final String LEFT_ENCODER_NAME = "leftEncoder";
        public static final String RIGHT_ENCODER_NAME = "rightEncoder";
        public static final String CENTER_ENCODER_NAME = "centerEncoder";

        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final DcMotorSimple.Direction LEFT_ENCODER_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction RIGHT_ENCODER_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction CENTER_ENCODER_DIRECTION = DcMotorSimple.Direction. REVERSE;

        public static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        ); //fixed orientation

        public static final RevHubOrientationOnRobot IMU_PARAMETERS_ROADRUNNER = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        public static final double DEADZONE = 0.1;
    }

    public static abstract class IntakeConstants {
        public static final String HORIZONTAL_SLIDE_LEFT_MOTOR_NAME = "horizontalSlideLeftMotor";
        public static final String HORIZONTAL_SLIDE_RIGHT_MOTOR_NAME = "horizontalSlideRightMotor";

        public static final DcMotorSimple.Direction HORIZONTAL_SLIDE_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction HORIZONTAL_SLIDE_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static final double ARM_JOINT_MAXIMUM_ANGLE_ROTATED = 120; //degrees
        public static final double ARM_JOINT_MINIMUM_ANGLE_ROTATED = -30; //Defined as the horizontal line form the floor that meets the arm joist axis

        public static final double ARM_JOINT_P = 1;
        public static final double ARM_JOINT_I = 1;
        public static final double ARM_JOINT_D = 1;
        public static final double ARM_JOINT_PID_POWER_TOLERANCE = 0.01; //inches

        public static final String LINEAR_SLIDE_MOTOR_NAME = "linearSlideMotor";
        public static final DcMotorSimple.Direction LINEAR_SLIDE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final double LINEAR_SLIDE_PULLEY_CIRCUMFERENCE = 1 * Math.PI; //inches

        public static final double LINEAR_SLIDE_P = 1;
        public static final double LINEAR_SLIDE_I = 1;
        public static final double LINEAR_SLIDE_D = 1;
        public static final double LINEAR_SLIDE_PID_POWER_TOLERANCE = 0.01; //inches

        public static final double MAXIMUM_FORWARD_EXTENSION = 42-5; //INCHES
        public static final double MINIMUM_BACKWARD_EXTENSION = 0;

        public static final String ACTIVE_INTAKE_SERVO_NAME = "activeIntakeServo";
        public static final String INTAKE_WRIST_SERVO_NAME = "intakeWristServo";

        public static final double INTAKE_WRIST_SERVO_MIN_ANGLE = 0; //degree,defined as the angle between the intake and the arm
        public static final double INTAKE_WRIST_SERVO_MAX_ANGLE = 180;
        public static final double INTAKE_WRIST_SERVO_UP_POSITION = 0;
        public static final double INTAKE_WRIST_SERVO_DOWN_POSITION = 180;

        public static class FloorScanningMode {
            public static final double MINIMUM_FLOOR_SCANNING_REGRESSION_ANGLE = -1; //DEGREES
            public static final double MAXIMUM_FLOOR_SCANNING_REGRESSION_ANGLE = -1;

            public static final double ACTIVE_INTAKE_FORWARD_EXTENSION = 1; //INCHES, ALL IN INCHES
            public static final double DRIVE_BASE_REAR_END_FROM_ARM_JOIST_AXIS = 1;
            public static final double ARM_JOINT_AXIS_HEIGHT_FROM_FLOOR = 1;

            public final double LINEAR_SLIDE_FLOOR_SCANNING_MAXIMUM_EXTENSION = (42 - ACTIVE_INTAKE_FORWARD_EXTENSION - DRIVE_BASE_REAR_END_FROM_ARM_JOIST_AXIS) / Math.cos(Math.toRadians( MAXIMUM_FLOOR_SCANNING_REGRESSION_ANGLE ));

            public static final double FLOOR_SCANNING_CURSOR_MAXIMUM_LIMIT =  42 - ACTIVE_INTAKE_FORWARD_EXTENSION - DRIVE_BASE_REAR_END_FROM_ARM_JOIST_AXIS;
            public static final double FLOOR_SCANNING_CURSOR_MINIMUM_LIMIT = ARM_JOINT_AXIS_HEIGHT_FROM_FLOOR / Math.tan(Math.toRadians(MINIMUM_FLOOR_SCANNING_REGRESSION_ANGLE));
        }

        public enum IntakeSubsystemState {
            FLOOR_SCANNING_MODE,
            HIGH_BASKET_POSITION,
            HANG_SPECIMEN_POSITION,
            GET_SPECIMEN_POSITION,
            AT_BAY
        }

        public static class intakeControl {
            public static double HIGH_BASKET_POSITION_WRIST_ANGLE = -1;
            public static double HIGH_BASKET_POSITION_LINEAR_SLIDE_LENGTH = -1; //INCH
            public static double HIGH_BASKET_POSITION_ARM_JOINT_ANGLE = -1;

            public static double HANG_SPECIMEN_POSITION_WRIST_ANGLE = -1;
            public static double HANG_SPECIMEN_POSITION_LINEAR_SLIDE_LENGTH = -1;
            public static double HANG_SPECIMEN_POSITION_ARM_JOINT_ANGLE = -1;

            public static double GET_SPECIMEN_POSITION_WRIST_ANGLE = -1;
            public static double GET_SPECIMEN_POSITION_LINEAR_SLIDE_LENGTH = -1;
            public static double GET_SPECIMEN_POSITION_ARM_JOINT_ANGLE = -1;

            public static double AT_BAY_WRIST_ANGLE = -1;
            public static double AT_BAY_LINEAR_SLIDE_LENGTH = -1;
            public static double AT_BAY_ARM_JOINT_ANGLE = -1;

            public static double ARM_JOINT_PID_TIMEOUT = 3;
            public static double LINEAR_SLIDE_PID_TIMEOUT = 5;
        }
    }

    public static abstract class OuttakeConstants {
        public static final String PINCH_SERVO_NAME = "pinchServo";
        public static final String OUTTAKE_WRIST_SERVO_NAME = "outtakeWristServo";
    }
}