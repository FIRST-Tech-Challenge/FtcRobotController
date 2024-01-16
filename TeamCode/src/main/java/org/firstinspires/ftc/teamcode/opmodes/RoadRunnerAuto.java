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
        TrajectorySequence testing = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -25.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.00, 25.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(12.00, 25.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(12.00, -25.00), Math.toRadians(270.00))
                .build();
        drive.setPoseEstimate(testing.start());

        waitForStart();

        drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);

        drive.followTrajectorySequence(testing);
    }
}
