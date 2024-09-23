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

        public static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ); 
        
        public static final double DEADZONE = 0.1;
    }

}
