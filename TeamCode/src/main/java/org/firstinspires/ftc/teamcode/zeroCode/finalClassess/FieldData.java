package org.firstinspires.ftc.teamcode.zeroCode.finalClassess;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.zeroCode.utils.LiveMap;

public final class FieldData {
    /*A hive would be considered tipped, if the part that is closest to the flower is lower.
    * These starting values, I got from the CAD model. */
    public static boolean blueAllianceFacingAudiance = false;
    public static boolean redAllianceFacingAudiance = true;

    //These varibles will probobly not be updated. If there is extra they may be incorporated though.
    public static int pollenInBlueHive = 0;
    public static int nectarInBlueHive = 0;
    public static int pollenInRedHive = 0;
    public static int nectarInRedHive = 0;

    //All in CM
    public static double pollenDiameter = 7.112;
    public static double pollenRadius = pollenDiameter/2;
    public static double nectarDiameter = 9.144;
    public static double nectarRadius = nectarDiameter/2;

    private static Pose2D robotPose;
    private static double robotHeading = 0;

    public static LiveMap fieldMap;

    //The Audiance Red Hive starts facing up
    //The Scoring Blue Hive starts facing up
    public enum hiveTags{
        audianceBlueHive,
        scoringBlueHive,
        audianceRedHive,
        scoringRedHive
    }

    public static void updateRobotPose(Pose2D pose2D) {robotPose = pose2D;}
    public static Pose2D getPose2D() {return robotPose;}
    public static double getHeading() {return robotHeading;}
}
