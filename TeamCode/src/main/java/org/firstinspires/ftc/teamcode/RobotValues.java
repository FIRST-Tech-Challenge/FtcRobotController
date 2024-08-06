package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;


public class RobotValues {
    public static DcMotor.Direction LEFTDIR = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction RIGHTDIR = DcMotor.Direction.FORWARD;
    public static LogoFacingDirection LOGO_DIR = LogoFacingDirection.UP;
    public static UsbFacingDirection USB_DIR = UsbFacingDirection.BACKWARD;

    // define robot-specific motor/wheel constants
    public static final double TICKS_PER_REVOLUTION = 384.5; //Encoder Resolution
    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_DIAMETER = 96 / 25.4; //converting mm to in

    // define calculated constants
    public static final double WHEEL_CIRCUMFERENCE =
            WHEEL_DIAMETER * Math.PI;
    public static final double DISTANCE_PER_REVOLUTION =
            WHEEL_CIRCUMFERENCE * GEAR_RATIO;
}