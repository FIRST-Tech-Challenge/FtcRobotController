package com.acmerobotics.roadrunner.samples;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "drive")
public class AutoJunction extends LinearOpMode {
    public static double STARTING_OFFSET_FROM_CENTER = 4;
    public static double TILE_WIDTH = 24;
    public static double X_START_OFFSET = -33;
    public static double Y_START_OFFSET = -60;
    public static double H_START = 90; // Degrees
    public static double INTERIM_DELAY = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(X_START_OFFSET, Y_START_OFFSET, Math.toRadians(H_START));

        drive.setPoseEstimate(startPose);

        waitForStart();

        TrajectorySequence trajSeq;

        if (isStopRequested()) return;

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(STARTING_OFFSET_FROM_CENTER + TILE_WIDTH * 2)
                // .turn(Math.toRadians(-90))
                .build();
        drive.followTrajectorySequence(trajSeq);
    }
}
