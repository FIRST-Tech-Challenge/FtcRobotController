package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
public class UnworkingTrajectories {
    public static TrajectorySequence testing;
    public static TrajectorySequence propCloseLeftRed;
    public static TrajectorySequence propCloseMidRed;
    public static TrajectorySequence scoreCloseRed;

    public static TrajectorySequence redClosePark;
    public static TrajectorySequence propCloseRightRed;
    public static TrajectorySequence propFarLeftRed;
    public static TrajectorySequence propFarMidRed;
    public static TrajectorySequence propFarRightRed;
    public static TrajectorySequence propCloseLeftBlue;
    public static TrajectorySequence propCloseMidBlue;
    public static TrajectorySequence propCloseRightBlue;
    public static TrajectorySequence propFarLeftBlue;
    public static TrajectorySequence propFarMidBlue;
    public static TrajectorySequence propFarRightBlue;
    public static TrajectorySequence parkCloseLeftRed;
    public static TrajectorySequence parkCloseMidRed;
    public static TrajectorySequence parkCloseRightRed;
    public static TrajectorySequence parkFarLeftRed;
    public static TrajectorySequence parkFarMidRed;
    public static TrajectorySequence parkFarRightRed;
    public static TrajectorySequence parkCloseLeftBlue;
    public static TrajectorySequence parkCloseMidBlue;
    public static TrajectorySequence parkCloseRightBlue;
    public static TrajectorySequence parkFarLeftBlue;
    public static TrajectorySequence parkFarMidBlue;
    public static TrajectorySequence parkFarRightBlue;
    public static TrajectorySequence redParkClose;
    public static TrajectorySequence redParkFar;
    public static TrajectorySequence blueParkFar;
    public static TrajectorySequence blueParkClose;
    public static TrajectorySequence toMoveBack;
    public static TrajectorySequence redScore;
    public static TrajectorySequence scoreCloseLeftRed;
    public static TrajectorySequence scoreCloseMidRed;
    public static TrajectorySequence scoreCloseRightRed;
    public static TrajectorySequence scoreFarLeftRed;
    public static TrajectorySequence scoreFarMidRed;
    public static TrajectorySequence scoreFarRightRed;
    public static TrajectorySequence scoreCloseLeftBlue;
    public static TrajectorySequence scoreCloseMidBlue;
    public static TrajectorySequence scoreCloseRightBlue;
    public static TrajectorySequence scoreFarLeftBlue;
    public static TrajectorySequence scoreFarMidBlue;
    public static TrajectorySequence scoreFarRightBlue;
    public static TrajectorySequence redParkFarForward;
    public static TrajectorySequence blueParkCloseForward;
    public static TrajectorySequence blueParkFarForward;
    public static TrajectorySequence redParkCloseForward;
    public static TrajectorySequence CloseRedSplines;
    public static TrajectorySequence park;
    public static TrajectorySequence back;
    public static TrajectorySequence toParkCloseRed;
    public static TrajectorySequence toParkCloseBlue;

    public static double toLineShort = 6;
    public static double toLineLong = 11;
    public static double toMiddle = 25;
    public static double parkRight = 38;
    public static double parkLeft = 97;
    public static double prepPark = 20;//this is for far left and far right
    public static double prepParkMid = 4;//this is for mid
    public static double toTurn = 18;
    public static double backBoard = 19;

    public static double farBoxToPerp = 22;

    //for left side, right and left should be modified to not go backwards, but rather go forward then strafe right
    //for the front face, it should just be regular
    //need the variables from the middle of the "box" to the perpendicular of the parking placement and then distance from that point to placement

    //Add more as needed
    public static Pose2d startPoseCloseRed = new Pose2d(12,-62, Math.toRadians(90)); //This should be close to correct
    public static Pose2d startPoseFarRed = new Pose2d(-35, -62, Math.toRadians(90)); //This too
    public static Pose2d startPoseCloseBlue = new Pose2d(12,62, Math.toRadians(-90));//And this
    public static Pose2d startPoseFarBlue = new Pose2d(-35,62, Math.toRadians(-90)); //Also this
    public static Pose2d buildedCloseRed = new Pose2d(12,-62, Math.toRadians(0));
    public static Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));
//    protected static void setDistance(int newDistance) {
//        distance = newDistance;
//    }

    public static void generateTrajectories(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        //Think back to when the guest speaker for programming was talking to use about the potato example
        //When we were building the potato, we created the methods to add the toppings and then create the potato
        //This works in the same way where you are adding the commands you want the bot to run and then building it
        //Now add the commands to have the robot move to place the pixel
        //After you add the code to place the pixel on the tape, create new sequences to drive to the parking area

        //Tests with splines. Use after robot is very tuned
//        propCloseLeftRedSplines = drive.trajectorySequenceBuilder(startPoseCloseRed)
//                .splineToSplineHeading(new Pose2d(9.5 , -30, Math.toRadians(180)), Math.toRadians(180))
//                .back(10)
//                .splineTo(new Vector2d(58, -58), Math.toRadians(0))
//                .build();

        testing = drive.trajectorySequenceBuilder(startPose)
                .forward(12)
                .build();


        //Red close
        propCloseLeftRed = drive.trajectorySequenceBuilder(startPoseCloseRed)
                .forward(toMiddle)
                .turn(Math.toRadians(90))
                .forward(toLineLong)
                .back(toLineLong)
                .lineToLinearHeading(new Pose2d(12, -62, 0))
                .build();

        propCloseMidRed = drive.trajectorySequenceBuilder(startPoseCloseRed)
                .forward(toMiddle)
                .lineToLinearHeading(new Pose2d(12, -62, 0))
                .build();

        propCloseRightRed = drive.trajectorySequenceBuilder(startPoseCloseRed)
                .forward(toMiddle)
                .turn(Math.toRadians(-90))
                .forward(toLineLong)
                .back(toLineLong)
                .strafeRight(toMiddle)
                .build();

        scoreCloseLeftRed = drive.trajectorySequenceBuilder(propCloseLeftRed.end())
                .forward(34)
                .strafeLeft(backBoard+6)
                .build();

        scoreCloseMidRed = drive.trajectorySequenceBuilder(propCloseMidRed.end())
                .forward(34)
                .strafeLeft(backBoard)
                .build();

        scoreCloseRightRed = drive.trajectorySequenceBuilder(propCloseRightRed.end())
                .forward(34)
                .strafeLeft(backBoard-6)
                .build();

        parkCloseLeftRed = drive.trajectorySequenceBuilder(scoreCloseLeftRed.end())
                .strafeRight(backBoard+6)
                .forward(12)
                .build();

        parkCloseMidRed = drive.trajectorySequenceBuilder(scoreCloseMidRed.end())
                .strafeRight(backBoard+6)
                .forward(12)
                .build();

        parkCloseRightRed = drive.trajectorySequenceBuilder(scoreCloseRightRed.end())
                .strafeRight(backBoard-6)
                .forward(12)
                .build();
        //Red far
        propFarLeftRed = drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(toMiddle)
                .turn(Math.toRadians(90))
                .forward(toLineLong)
                .back(toLineLong)
                .strafeRight(prepPark)
                .turn(Math.toRadians(180))
                .build();

        propFarMidRed = drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(toMiddle+toTurn)
                .turn(Math.toRadians(-180))
                .back(prepParkMid)
                .turn(Math.toRadians(90))
                .build();

        propFarRightRed = drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(toMiddle)
                .turn(Math.toRadians(-90))
                .forward(toLineLong)
                .back(toLineLong)
                .strafeLeft(prepPark)
                .build();

        scoreFarLeftRed = drive.trajectorySequenceBuilder(propFarLeftRed.end())
                .forward(81)
                .strafeRight(backBoard-9)
                .build();

        scoreFarMidRed = drive.trajectorySequenceBuilder(propFarMidRed.end())
                .forward(81)
                .strafeRight(backBoard-3)
                .build();

        scoreFarRightRed = drive.trajectorySequenceBuilder(propFarRightRed.end())
                .forward(81)
                .strafeRight(backBoard+3)
                .build();

        parkFarLeftRed = drive.trajectorySequenceBuilder(scoreFarLeftRed.end())
                .strafeLeft(backBoard-9)
                .forward(12)
                .build();

        parkFarMidRed = drive.trajectorySequenceBuilder(scoreFarMidRed.end())
                .strafeLeft(backBoard-3)
                .forward(12)
                .build();

        parkFarRightRed = drive.trajectorySequenceBuilder(scoreFarRightRed.end())
                .strafeLeft(backBoard+3)
                .forward(12)
                .build();

        //Blue close
        propCloseLeftBlue = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .forward(toMiddle)
                .turn(Math.toRadians(90))
                .forward(toLineLong)
                .back(toLineLong)
                .strafeLeft(toMiddle)
                .build();

        propCloseMidBlue = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .forward(toMiddle)
                .lineToLinearHeading(new Pose2d(12, 62, 0))
                .build();

        propCloseRightBlue = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .forward(toMiddle)
                .turn(Math.toRadians(-90))
                .forward(toLineLong)
                .back(toLineLong)
                .lineToLinearHeading(new Pose2d(12, 62, 0))
                .build();

        scoreCloseLeftBlue = drive.trajectorySequenceBuilder(propCloseLeftBlue.end())
                .forward(34)
                .strafeRight(backBoard-6)
                .build();

        scoreCloseMidBlue = drive.trajectorySequenceBuilder(propCloseMidBlue.end())
                .forward(34)
                .strafeRight(backBoard)
                .build();

        scoreCloseRightBlue = drive.trajectorySequenceBuilder(propCloseRightBlue.end())
                .forward(34)
                .strafeRight(backBoard+6)
                .build();

        parkCloseLeftBlue = drive.trajectorySequenceBuilder(scoreCloseLeftBlue.end())
                .strafeLeft(backBoard-6)
                .forward(12)
                .build();

        parkCloseMidBlue = drive.trajectorySequenceBuilder(scoreCloseMidBlue.end())
                .strafeLeft(backBoard)
                .forward(12)
                .build();

        parkCloseRightBlue = drive.trajectorySequenceBuilder(scoreCloseRightBlue.end())
                .strafeLeft(backBoard+6)
                .forward(12)
                .build();

        //Blue far
        propFarLeftBlue = drive.trajectorySequenceBuilder(startPoseFarBlue)
                .forward(toMiddle)
                .turn(Math.toRadians(90))
                .forward(toLineLong)
                .back(toLineLong)
                .strafeRight(prepPark)
                .build();

        propFarMidBlue = drive.trajectorySequenceBuilder(startPoseFarBlue)
                .forward(toMiddle+toTurn)
                .turn(Math.toRadians(-180))
                .back(prepParkMid)
                .turn(Math.toRadians(-90))
                .build();


        propFarRightBlue = drive.trajectorySequenceBuilder(startPoseFarBlue)
                .forward(toMiddle)
                .turn(Math.toRadians(-90))
                .forward(toLineLong)
                .back(toLineLong)
                .strafeLeft(prepPark)
                .turn(Math.toRadians(180))
                .build();

        scoreFarLeftBlue = drive.trajectorySequenceBuilder(propFarLeftBlue.end())
                .forward(81)
                .strafeLeft(backBoard+3)
                .build();

        scoreFarMidBlue = drive.trajectorySequenceBuilder(propFarMidBlue.end())
                .forward(81)
                .strafeLeft(backBoard-2)
                .build();

        scoreFarRightBlue = drive.trajectorySequenceBuilder(propFarRightBlue.end())
                .forward(81)
                .strafeLeft(backBoard-10)
                .build();

        parkFarLeftBlue = drive.trajectorySequenceBuilder(scoreFarLeftBlue.end())
                .strafeLeft(backBoard-3)
                .forward(12)
                .build();

        parkFarMidBlue = drive.trajectorySequenceBuilder(scoreFarMidBlue.end())
                .strafeLeft(backBoard+2)
                .forward(12)
                .build();

        parkFarRightBlue = drive.trajectorySequenceBuilder(scoreFarRightBlue.end())
                .strafeLeft(backBoard+10)
                .forward(12)
                .build();

        redParkClose = drive.trajectorySequenceBuilder(startPoseCloseRed)
                .forward(2)
                .strafeRight(parkRight)
                .build();
        redParkFar = drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(toMiddle+toLineShort+toTurn+prepParkMid)
                .strafeRight(parkLeft)
                .build();
        blueParkClose = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .strafeLeft(parkRight)
                .build();
        blueParkFar = drive.trajectorySequenceBuilder(startPoseFarBlue)
                .forward(toMiddle+toLineShort+toTurn+prepParkMid)
                .strafeLeft(parkLeft)
                .build();
        toMoveBack = drive.trajectorySequenceBuilder(propCloseLeftRed.end())
                .back(2)
                .build();
        redParkFarForward = drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(toMiddle+toLineShort+toTurn+prepParkMid)
                .turn(Math.toRadians(-90))
                .forward(parkLeft)
                .build();
        blueParkCloseForward = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .forward(parkRight)
                .build();
        blueParkFarForward = drive.trajectorySequenceBuilder(startPoseFarBlue)
                .forward(toMiddle+toLineShort+toTurn+prepParkMid)
                .turn(Math.toRadians(90))
                .forward(parkLeft)
                .build();
        redParkCloseForward = drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(parkRight)
                .build();
    }
}

