/*
package org.firstinspires.ftc.teamcode.AutoTest.Roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class RightSideAuto extends LinearOpMode {

    public static double highbar_x_coordinate = -33;
    public static double highbar_y_coordinate = 0;
    /*
    public static double specimen_pickup_x_coordinate = ;
    public static double specimen_pickup_y_coordinate = ;
    public static double first_forward =;
    public static double first_strafe =;
    public static double second_forward =;
    public static double second_strafe =;
    public static double first_back =;

     */
    /*
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RobotActionConfig value = new RobotActionConfig();
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(first_forward)
                //.addDisplacementMarker(() -> {})
                .waitSeconds(3)
                .strafeLeft(first_strafe)
                .back(second_forward)
                .lineToLinearHeading(new Pose2d(0,second_strafe,Math.toRadians(180)))
                .back(first_back)
                .lineToConstantHeading(new Vector2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate,highbar_y_coordinate,Math.toRadians(0)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(180)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate,highbar_y_coordinate,Math.toRadians(0)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(180)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }

     */


