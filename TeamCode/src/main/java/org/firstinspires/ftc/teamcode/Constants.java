package org.firstinspires.ftc.teamcode; 

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class Constants {

    public static abstract class DriveConstants {
        public static final String FRONT_LEFT_MOTOR_NAME = "";
        public static final String FRONT_RIGHT_MOTOR_NAME = "";
        public static final String BACK_LEFT_MOTOR_NAME = "";
        public static final String BACK_RIGHT_MOTOR_NAME = "";
        
        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD; //Arbitrary
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD; //Arbitrary
        public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE; //Arbitrary
        public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE; //Arbitrary
        
        public static final double DEADZONE = 0.1;
    }

}
