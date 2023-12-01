package org.firstinspires.ftc.teamcode.roadRunner.drive;

public class baseAutonomousBlue {

    /*
    to left spike --> close (park as well)
    .strafeLeft(11)
                                .forward(13)
                                .back(13)
                                .lineToLinearHeading(new Pose2d(44, 42))
                                .strafeLeft(15)
                                .forward(15)


    */

    /*
    to left spike --> far side (park as well)
     .splineTo(new Vector2d(-34, 30), Math.toRadians(0))
                                .strafeLeft(30)
                                .forward(72)
                                .strafeRight(32)
                                .forward(8)
                                .strafeRight(18)
                                .forward(15)
    */

    /*
    to right spike --> close side (park as well)
    .splineTo(new Vector2d(10, 30), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(45, 29))
                                .strafeLeft(30)
                                .forward(15)
    */

    /*
    to right spike --> far side (park as well)
    .strafeRight(11)
                                .forward(12)
                                .back(15)
                                .strafeLeft(75)
                                .splineTo(new Vector2d(34, 58), Math.toRadians(90))
                                .strafeRight(16)
                                .forward(15)
                                .strafeRight(30)
                                .forward(10)
    */


    /*
    to middle spike --> close side (park as well)
    .forward(24)
                                .back(5)
                                .lineToLinearHeading(new Pose2d(46, 35))
                                .strafeLeft(25)
                                .forward(13)
    */

    /*
    to middle spike --> far side (park as well)
    .forward(22)
                                .back(24)
                                .strafeLeft(65)
                                .splineTo(new Vector2d(34, 60), Math.toRadians(90))
                                .strafeRight(25)
                                .forward(15)
                                .strafeRight(23)
                                .forward(15)
    */


}
