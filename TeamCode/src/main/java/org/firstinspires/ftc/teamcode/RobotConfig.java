package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConfig {
    // Wheels config
    public static String frontLeftWheelName = "frontLeftMotor";
    public static DcMotor.Direction frontLeftWheelDirection = DcMotor.Direction.REVERSE;
    public static String frontRightWheelName = "frontRightMotor";
    public static DcMotor.Direction frontRightWheelDirection = DcMotor.Direction.FORWARD;
    public static String backLeftWheelName = "backLeftMotor";
    public static DcMotor.Direction backLeftWheelDirection = DcMotor.Direction.REVERSE;
    public static String backRightWheelName = "backRightMotor";
    public static DcMotor.Direction backRightWheelDirection = DcMotor.Direction.FORWARD;
    public static double countPerMotorRev = 537.7;
    public static double driveGearReduction = 1.0;
    public static double wheelDiameterInches = 4.0;

    // IMU config
    public static String imuName = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // camera
    public static boolean cameraEnabled = false;
    public static String cameraName = "limelight";
    public static double cameraHeight = 420;
    public static double cameraAngel = -60;
    public static double targetHeight = 35;
}
