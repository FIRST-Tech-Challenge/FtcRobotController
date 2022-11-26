package com.acmerobotics.roadrunner.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.StandardTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("EncoderLeft", drive.getParallelEncoderPosition());
        // telemetry.addData("EncoderRight", drive.getEncoderPosition(StandardTrackingWheelLocalizer.EncoderLocation.RIGHT));
        // telemetry.addData("EncoderLateral", drive.getEncoderPosition(StandardTrackingWheelLocalizer.EncoderLocation.LATERAL));
        // telemetry.addData("LeftInches", drive.getEncoderInches(StandardTrackingWheelLocalizer.EncoderLocation.LEFT));
        // telemetry.addData("RightInches", drive.getEncoderInches(StandardTrackingWheelLocalizer.EncoderLocation.RIGHT));
        // telemetry.addData("LateralInches", drive.getEncoderInches(StandardTrackingWheelLocalizer.EncoderLocation.LATERAL));
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
