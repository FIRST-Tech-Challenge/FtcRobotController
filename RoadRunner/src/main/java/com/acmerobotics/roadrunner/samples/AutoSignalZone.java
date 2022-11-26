package com.acmerobotics.roadrunner.samples;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "drive")
public class AutoSignalZone extends LinearOpMode {
    public static double TILE_WIDTH = 24;
    public static double X_START_OFFSET = -33;
    public static double Y_START_OFFSET = -60;
    public static double INTERIM_DELAY = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(X_START_OFFSET, Y_START_OFFSET, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        TrajectorySequence trajSeq;

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(TILE_WIDTH)
                    .strafeLeft(TILE_WIDTH)
                    .waitSeconds(INTERIM_DELAY)
                    .strafeRight(TILE_WIDTH)
                    .back(TILE_WIDTH)
                    .waitSeconds(INTERIM_DELAY)
                    .forward(TILE_WIDTH)
                    .waitSeconds(INTERIM_DELAY)
                    .back(TILE_WIDTH)
                    .waitSeconds(INTERIM_DELAY)
                    .forward(TILE_WIDTH)
                    .strafeRight(TILE_WIDTH)
                    .waitSeconds(INTERIM_DELAY)
                    .strafeLeft(TILE_WIDTH)
                    .back(TILE_WIDTH)
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
