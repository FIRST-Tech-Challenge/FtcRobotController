package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Config {
    // Encoder/Inch = 360/Ticks per motor shaft revolution
    public static final double TICKS_PER_REVOLUTION = 537.6; //Encoder pulses per rev by supplier information
    public static final double GEARBOX_REDUCTION = 7.0 / 9.0; //Gearbox on motor * gearbox on robot
    public static final double WHEEL_DIAMETER_INCHES = 4; //Wheel diameter in inches
    public static final double ENCODER_PER_INCH = (TICKS_PER_REVOLUTION * GEARBOX_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final RevHubOrientationOnRobot HUB_FACING = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    );

    public static final double TURNING_P_GAIN = 0.03;

//    public static class APRILTAGS
//    {
//        //Change these at competition to the correct tags. Home field tags are wrong.
//        public static final int leftBlue = 1;
//        public static final int middleBlue = 2;
//        public static final int rightBlue = 3;
//
//        public static final int X_P_Gain =
//
//        public static final int Y_P_Gain =
//
//        public static final int YAW_P_Gain =
//    }
}
