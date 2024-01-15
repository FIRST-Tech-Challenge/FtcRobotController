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
        TrajectorySequence testing = drive.trajectorySequenceBuilder(new Pose2d(-37.03, 47.39, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-36.81, -35.73), Math.toRadians(-89.85))
                .splineTo(new Vector2d(16.30, -38.32), Math.toRadians(-2.79))
                .splineTo(new Vector2d(11.77, 47.39), Math.toRadians(94.91))
                .build();
        drive.setPoseEstimate(testing.start());

        waitForStart();

        drive.followTrajectorySequence(testing);
    }
}
