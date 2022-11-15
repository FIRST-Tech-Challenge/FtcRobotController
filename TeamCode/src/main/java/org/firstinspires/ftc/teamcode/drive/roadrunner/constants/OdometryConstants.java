package org.firstinspires.ftc.teamcode.drive.roadrunner.constants;

import com.acmerobotics.dashboard.config.Config;


//only touch left(forwawrd back) and rear (side)
@Config
public class OdometryConstants {

    public static double leftXcm = 1.6;
    public static double leftYcm = -12.5;
    public static double leftAngleDeg = 180.0;

    public static double rightXcm = -1.6;
    public static double rightYcm = -19.6;
    public static double rightAngleDeg = 0.0;

    public static double rearXcm = 0;
    public static double rearYcm = 0;
    public static double rearAngleDeg = -90;

    public static boolean reverseOutput = false;

//    public static Pose2d rightOdometryPose = new Pose2d(INCH.fromCm(-2.2), INCH.fromCm(-19.5), 0.0);
//    public static Pose2d rearOdometryPose = new Pose2d(INCH.fromCm(-17.6), INCH.fromCm(0.0), Math.PI / 2);
//    public static Pose2d leftOdometryPose = new Pose2d(INCH.fromCm(-2.2), INCH.fromCm(18.25), 0.0);

}