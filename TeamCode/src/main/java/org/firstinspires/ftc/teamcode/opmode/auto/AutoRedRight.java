package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous(group="auto", name="Auto_RED_Backboard_side")
public class AutoRedRight extends LinearOpMode
{
    private static final double TILE = 24.0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(TILE/2, -2.5*TILE, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Center = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1.5*TILE))
                //.lineToConstantHeading(new Vector2d(TILE/2, -1.5*TILE))
                .addTemporalMarker(1, () -> {

                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-2*TILE, -1.5*TILE))
                .addTemporalMarker(1, () -> {

                })
                //.lineToLinearHeading(new Vector2d(-2*TILE, -1.5*TILE))
                .turn(Math.toRadians(90))
                .forward(TILE)
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(TILE*2, -2.5*TILE))
                .lineToConstantHeading(new Vector2d(TILE*2, -1.5*TILE))
                .addTemporalMarker(1, () -> {

                })
                .build();

        TrajectorySequence Right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1.8*TILE))
                .turn(Math.toRadians(-45))
                .lineToLinearHeading(new Pose2d(TILE/2, -1.5*TILE, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {

                })
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-2*TILE, -1.5*TILE))
                .addTemporalMarker(1, () -> {

                })
                //.lineToLinearHeading(new Vector2d(-2*TILE, -1.5*TILE))
                .turn(Math.toRadians(90))
                .forward(TILE)
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(TILE*2, -2.5*TILE))
                .lineToConstantHeading(new Vector2d(TILE*2, -1.2*TILE))
                .addTemporalMarker(1, () -> {

                })
                .build();

        TrajectorySequence Left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(TILE/2, -1.8*TILE))
                .turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(TILE/2, -1.5*TILE, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {

                })
                .lineToConstantHeading(new Vector2d(TILE*2, -2.5*TILE))
                .build();

        TeamElementPipeline.MarkerPosistion markerPosistion = TeamElementPipeline.MarkerPosistion.UNKNOWN;
        Vision.startStreaming(hardwareMap);
        markerPosistion = Vision.determineMarkerPosistion(4000);

        waitForStart();

        if (!isStopRequested()) return;

        switch (markerPosistion) {
            case CENTER:
            case UNKNOWN:
                drive.followTrajectorySequence(Center);
                break;
            case RIGHT:
                drive.followTrajectorySequence(Right);
                break;
            case LEFT:
                drive.followTrajectorySequence(Left);
                break;
        }
    }
}
