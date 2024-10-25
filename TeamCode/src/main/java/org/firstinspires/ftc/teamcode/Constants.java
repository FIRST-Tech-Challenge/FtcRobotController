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
        
        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        @Deprecated
        public static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ); 
        
        public static final double DEADZONE = 0.1;
    }

    public static abstract class OdometryConstants {
        public static final double WHEEL_DIAMETER = 35; //Millimeters
        public static final int TICKS_PER_REVOLUTION = 2048; //confirm but I think it's right
        public static final double WIDTH = 217.5; //Millimeters
        public static final double WHEEL_CIRCUMFERENCE = 0.0;
    }
}
