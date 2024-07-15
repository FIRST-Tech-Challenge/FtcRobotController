package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

// BLUE AUTO POSITIONS
@Config
public final class AutoConstants {
    // Field, features, and bot dimensions
    public static double fieldX = 144;
    public static double fieldY = 144;
    public static double maxX = fieldX / 2.0;
    public static double maxY = fieldY / 2.0;
    public static double botFrameWidth = 14.125;
    public static double botFrameLength = 17;
    public static double botCenterOffsetFromFront = 7.95 + .75;
    public static double botCenterOffsetFromRear = botFrameLength - botCenterOffsetFromFront;
    public static double botCenterOffsetFromLeft = botFrameWidth / 2.0;
    public static double tileSideLength = 24;
    public static double boardXOffset = 10;
    public static double boardYOffset = 25.5 + 5.25;
    public static double boardAprilTagSpacing = 6;

    // Maneuver constants
    public static double boardApproachOffset = 10.25;

    // Start headings in degrees
    public static double blueStartHeading = Math.toRadians(-90.0);
    public static double redStartHeading = Math.toRadians(90.0);
    // Near start poses
    public static Pose2d blueNearStartPose = new Pose2d(tileSideLength / 2.0, maxY - botCenterOffsetFromRear, Math.toRadians(-90));
    public static Pose2d redNearStartPose = reflectY(blueNearStartPose);

    // Near spike mark poses
    public static Pose2d blueNearCenterSpikePose = new Pose2d(24.75, 36, Math.toRadians(223));
    public static Pose2d redNearCenterSpikePose = reflectY(blueNearCenterSpikePose);
    public static Pose2d blueNearLeftSpikePose = new Pose2d(37, 36, Math.toRadians(180));
    public static Pose2d redNearRightSpikePose = reflectY(blueNearLeftSpikePose);
    public static Pose2d blueNearRightSpikePose = new Pose2d(15.25, 36, Math.toRadians(180));
    public static Pose2d redNearLeftSpikePose = reflectY(blueNearRightSpikePose);

    // Near navigation poses
    public static Pose2d blueNearEscapeCornerPose = new Pose2d(tileSideLength + 10, tileSideLength*3-botFrameWidth/2 - 4.5, Math.toRadians(180));
    public static Pose2d redNearEscapeCornerPose = reflectY(blueNearEscapeCornerPose);
    public static Pose2d blueNearEscapeCenterPose = new Pose2d(tileSideLength + 10, tileSideLength*3-botFrameWidth/2 - 4.5 - 45, Math.toRadians(180));
    public static Pose2d redNearEscapeCenterPose = reflectY(blueNearEscapeCenterPose);

    // Far start poses
    public static Pose2d blueFarStartPose = new Pose2d(-tileSideLength * 1.5, maxY - botCenterOffsetFromRear, Math.toRadians(-90));
    public static Pose2d redFarStartPose = reflectY(blueFarStartPose);

    // Far spike mark poses
    public static Pose2d blueFarLeftSpikePose = new Pose2d(-35.5, 46, Math.toRadians(-45));
    public static Pose2d redFarLeftSpikePose = reflectY(blueFarLeftSpikePose);
    public static Pose2d blueFarCenterSpikePose = new Pose2d(-48, 38.5, Math.toRadians(-60));
    public static Pose2d redFarCenterSpikePose = reflectY(blueFarCenterSpikePose);
    public static Pose2d blueFarRightSpikePose = new Pose2d(-42, 49, Math.toRadians(-110));
    public static Pose2d redFarRightSpikePose = reflectY(blueFarRightSpikePose);

    // Far navigation poses
    public static Pose2d blueFarCenterEscapePose = new Pose2d(-54, 41, Math.toRadians(-90));
    public static Pose2d redFarCenterEscapePose = reflectY(blueFarCenterEscapePose);
    public static Pose2d blueFarRightEscapePose = new Pose2d(-35, 48, Math.toRadians(-90));
    public static Pose2d redFarRightEscapePose = reflectY(blueFarRightEscapePose);
    public static Pose2d blueFarMainstreetPrepPose = new Pose2d(-54, 12, Math.toRadians(-90));
    public static Pose2d redFarMainstreetPrepPose = reflectY(blueFarMainstreetPrepPose);
    public static Pose2d blueFarMainstreetStartPose = new Pose2d(-35, 12, Math.toRadians(0));
    public static Pose2d redFarMainstreetStartPose = reflectY(blueFarMainstreetStartPose);
    public static Pose2d blueFarMainstreetEndPose = new Pose2d(34, 12, Math.toRadians(0));
    public static Pose2d redFarMainstreetEndPose = reflectY(blueFarMainstreetEndPose);

    // Board poses (same for near and far)
    public static Pose2d blueCenterBackdrop = new Pose2d(72 - boardXOffset - boardApproachOffset, 72 - boardYOffset - boardAprilTagSpacing + 0.5, Math.toRadians(0));
    public static Pose2d redCenterBackdrop = reflectY(blueCenterBackdrop);
    public static Pose2d blueLeftBackdrop = new Pose2d(72 - boardXOffset - boardApproachOffset, 72 - boardYOffset + 1.5, Math.toRadians(0));
    public static Pose2d redRightBackdrop = reflectY(blueLeftBackdrop);
    public static Pose2d blueRightBackdrop = new Pose2d(72 - boardXOffset - boardApproachOffset, 72 - boardYOffset - boardAprilTagSpacing*2 + 3, Math.toRadians(0));
    public static Pose2d redLeftBackdrop = reflectY(blueRightBackdrop);
    public static Pose2d blueCornerParkPose= new Pose2d(72 - botFrameLength * 1.25, 72-botFrameWidth/2 - 4.5, Math.toRadians(180));
    public static Pose2d redCornerParkPose = reflectY(blueCornerParkPose);
    public static Pose2d blueCenterParkPose= new Pose2d(72 - botFrameLength * 1.25, 72-botFrameWidth/2 - 4.5 - 47.5, Math.toRadians(180));
    public static Pose2d redCenterParkPose = reflectY(blueCenterParkPose);

    public static Pose2d reflectY(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), Math.toRadians(360) - pose.getHeading());
    }

}