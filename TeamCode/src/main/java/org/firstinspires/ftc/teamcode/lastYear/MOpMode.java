package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class MOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(65)
                .build();

        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory.end())
                .back(65)
                .build();

        Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory.end())
                .strafeRight(5)
                .build();

        Trajectory myTrajectory4 = drive.trajectoryBuilder(myTrajectory.end())
                .strafeLeft(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(myTrajectory2);
        drive.followTrajectory(myTrajectory3);
        drive.followTrajectory(myTrajectory4);
    }
}