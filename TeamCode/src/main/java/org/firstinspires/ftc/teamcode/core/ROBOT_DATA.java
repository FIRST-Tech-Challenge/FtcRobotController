package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class ROBOT_DATA {

    public static final String LEFT_FRONT_DRIVE_ID = "leftFrontDrive";
    public static final String RIGHT_FRONT_DRIVE_ID = "rightFrontDrive";
    public static final String LEFT_REAR_DRIVE_ID = "leftRearDrive";
    public static final String RIGHT_REAR_DRIVE_ID = "rightRearDrive";
    public static final String IMU_ID = "imu";
    public static final String SHOULDER_POTENTIOMETER_ID = "shoulderSensor";
    public static final String ELBOW_LIMIT_SWITCH_ID = "elbowSensor";

    public static final DcMotor.Direction LEFT_FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_FRONT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction LEFT_REAR_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_REAR_DIRECTION = DcMotor.Direction.REVERSE;

    // Config variables
    public static double GAMEPAD_DRIVE_SMOOTHER = 2;
    public static double GAMEPAD_TURN_SMOOTHER = 2;

    //Config variables
    public static double HALF_TRACK_WIDTH = 6;
    public static double HALF_WHEEL_BASE = 6;

    public static final double TICKS_PER_REV = 328;

    public static final double WHEEL_CIRCUMFERENCE = Math.PI * 3.85826772;

    //Config variables
    public static double MAX_DRIVE_SPEED = 80; // Put this in inches per sec
    public static double MAX_TURN_SPEED = 6.28318531; // Put this in radians per sec

    //Config variables (the two P-gains only)
    public static double SHOULDER_P_GAIN = 1;
    public static final double SHOULDER_MAX_ANGLE_DEGREES = 265;
    public static final double SHOULDER_MIN_ANGLE_DEGREES = 10;
    public static final double SHOULDER_STALL_RANGE = 3;

    public static final double ACCEL_TIME = 2;
    public static final double MAX_AUTO_SPEED = 0.75;

    // Config variables
    public static double RAMSETE_B = 2d;
    public static double RAMSETE_ZETA = 0.6;
    public static double RAMSETE_W = 1;
}
