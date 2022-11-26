package com.acmerobotics.roadrunner.samples;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(group = "drive")
public class AutoSplineSignalZone extends LinearOpMode {
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

        /*
         * The set reversed calls tell the motion planning at the waypoint that we are going to move
         * in the opposite general direction, which removes a jerking hiccup out of the path while
         * it figures out it is going to go in a different direction.
         */
        while (!isStopRequested()) {
            trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(false)
                    .splineToConstantHeading(new Vector2d(X_START_OFFSET - TILE_WIDTH, Y_START_OFFSET + TILE_WIDTH), 0)
                    .waitSeconds(2)
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(X_START_OFFSET, Y_START_OFFSET), 0)
                    .waitSeconds(2)
                    .setReversed(false)
                    .splineToConstantHeading(new Vector2d(X_START_OFFSET, Y_START_OFFSET + TILE_WIDTH), 0)
                    .waitSeconds(2)
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(X_START_OFFSET, Y_START_OFFSET), 0)
                    .waitSeconds(2)
                    .setReversed(false)
                    .splineToConstantHeading(new Vector2d(X_START_OFFSET + TILE_WIDTH, Y_START_OFFSET + TILE_WIDTH), 0)
                    .waitSeconds(2)
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(X_START_OFFSET, Y_START_OFFSET), 0)
                    .build();

            drive.followTrajectorySequence(trajSeq);
        }
    }
}
