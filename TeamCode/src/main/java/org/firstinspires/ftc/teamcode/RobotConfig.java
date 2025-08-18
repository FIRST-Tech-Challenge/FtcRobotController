package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotConfig {
    // Wheels config
    public final static String frontLeftWheelName ="backRightMotor";// "frontLeftMotor";
    public final static DcMotor.Direction frontLeftWheelDirection = DcMotor.Direction.REVERSE;
    public final static String frontRightWheelName = "backLeftMotor"; // "frontRightMotor";
    public final static DcMotor.Direction frontRightWheelDirection = DcMotor.Direction.FORWARD;
    public final static String backLeftWheelName = "frontRightMotor";// "backLeftMotor";
    public final static DcMotor.Direction backLeftWheelDirection = DcMotor.Direction.REVERSE;
    public final static String backRightWheelName = "frontLeftMotor"; // "backRightMotor";
    public final static DcMotor.Direction backRightWheelDirection = DcMotor.Direction.FORWARD;
    public final static double countPerMotorRev = 537.7;
    public final static double driveGearReduction = 1.0;
    public final static double wheelDiameterInches = 4.0;

    // IMU config
    public final static String imuName = "imu";
    public final static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public final static RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
}
