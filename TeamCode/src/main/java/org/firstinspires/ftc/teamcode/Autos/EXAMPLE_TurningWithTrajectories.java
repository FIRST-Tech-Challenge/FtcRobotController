package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class EXAMPLE_TurningWithTrajectories extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        bot.setPoseEstimate(startPose);

            Trajectory forward = bot.trajectoryBuilder(startPose)
                    .forward(20)
                    .build();

            Trajectory left = bot.trajectoryBuilder(forward.end())
                    .strafeLeft(20)
                    .build();

            Trajectory back = bot.trajectoryBuilder(left.end())
                    .back(20)
                    .build();

            Trajectory right = bot.trajectoryBuilder(back.end())
                    .strafeRight(20)
                    .build();

            Trajectory forward2 = bot.trajectoryBuilder(right.end())
                    .forward(20)
                    .build();



        waitForStart();
            if (isStopRequested()) return;

            bot.followTrajectory(forward);
            bot.turn(Math.toRadians(90)); // turns are not actually a part of the trajectoryBuilder core
            bot.followTrajectory(left);
            bot.turn(Math.toRadians(90));
            bot.followTrajectory(back);
            bot.turn(Math.toRadians(90));
            bot.followTrajectory(right);

    }

}
