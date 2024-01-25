package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence BlueBottom = drive.trajectorySequenceBuilder(new Pose2d(-37.24, 67.47, Math.toRadians(269.24)))
                .lineToConstantHeading(new Vector2d(-36.59, 34))
                .lineToConstantHeading(new Vector2d(16.52, 34))
                .lineToSplineHeading(new Pose2d(44.37, 34, Math.toRadians(180.00)))
                .build();

        TrajectorySequence testing = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -25.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.00, 25.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(12.00, 25.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(12.00, -25.00), Math.toRadians(270.00))
                .build();
        drive.setPoseEstimate(BlueBottom.start());

        waitForStart();

        drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);

        drive.followTrajectorySequence(BlueBottom);
    }
}
