package org.firstinspires.ftc.teamcode; 

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public abstract class Constants {

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
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );//fixed orientation

        public static final RevHubOrientationOnRobot IMU_PARAMETERS_ROADRUNNER = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        public static final double DEADZONE = 0.1;
    }

    public static final int INCH_TO_TILE = 24;

    public static final double SLIDE_PULLEY_CIRCUMFERENCE = 5.0*0.393701*Math.PI;

    public static final double MOTOR_TICKS_PER_REVOLUTION = 751.8;

    public static abstract class IntakeConstants{
        public static final String HORIZONTAL_SLIDE_LEFT_MOTOR_NAME = "horizontalSlideLeftMotor";
        public static final String HORIZONTAL_SLIDE_RIGHT_MOTOR_NAME = "horizontalSlideRightMotor";

        public static final DcMotorSimple.Direction HORIZONTAL_SLIDE_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction HORIZONTAL_SLIDE_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        /*why both motor do not need changing direction is bcz gear problem, imagine O is motor and 0 is the gear:
        someone in build team can explain,but it works
         */

        public static final double MAXIMUM_FORWARD_EXTENSION = 42-5;//INCHES
        public static final double MINIMUM_BACKWARD_EXTENSION = 0;
        public static final int EXTENSION_TOLERANCE_INCHES = 2;

        public static final String ACTIVE_INTAKE_SERVO_NAME = "activeIntakeServo";
        public static final String INTAKE_WRIST_SERVO_NAME = "intakeWristServo";

        public static final double INTAKE_WRIST_SERVO_MIN_ANGLE = 0;
        public static final double INTAKE_WRIST_SERVO_MAX_ANGLE = 270;
        public static final double INTAKE_WRIST_SERVO_UP_POSITION = 0;
        public static final double INTAKE_WRIST_SERVO_DOWN_POSITION = 180;



    }

    public static abstract class OuttakeConstants{
        public static final String VERTICAL_SLIDE_LEFT_MOTOR_NAME = "verticalSlideLeftMotor";
        public static final String VERTICAL_SLIDE_RIGHT_MOTOR_NAME = "verticalSlideRightMotor";

        public static final String PINCH_SERVO_NAME = "pinchServo";
        public static final String OUTTAKE_WRIST_SERVO_NAME = "outtakeWristServo";
    }



}
