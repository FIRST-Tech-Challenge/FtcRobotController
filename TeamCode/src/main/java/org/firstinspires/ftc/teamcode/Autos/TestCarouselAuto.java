package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;


@Autonomous (name = "TCauto")
public class TestCarouselAuto extends LinearOpMode {
    @Override
    public void runOpMode(){

        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);


        TrajectoryVelocityConstraint STVC = new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 5;
            }
        };



        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        bot.trajectoryBuilder(startPose);

        Trajectory forward = bot.trajectoryBuilder(startPose)
                .forward(15)
                .build();

        Trajectory moveCaro = bot.trajectoryBuilder(forward.end())
                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(40)))
                .build();

        Trajectory strafeSetup = bot.trajectoryBuilder(moveCaro.end())
                .lineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(90)))
                .build();

        Trajectory strafeCollect = bot.trajectoryBuilder(strafeSetup.end())
                .strafeRight(15)

                .build();

        Trajectory moveHub = bot.trajectoryBuilder(strafeCollect.end())
                .lineToLinearHeading(new Pose2d(-12.3, -44, Math.toRadians(90)))
                .build();

        Trajectory movePark = bot.trajectoryBuilder(moveHub.end())
                .lineToLinearHeading(new Pose2d(-60, -34, Math.toRadians(180)))
                .build();

        bot.followTrajectory(forward);
        bot.followTrajectory(moveCaro);
        bot.followTrajectory(strafeSetup);
        bot.followTrajectory(strafeCollect);
        bot.followTrajectory(moveHub);
        bot.followTrajectory(movePark);

    }

}
