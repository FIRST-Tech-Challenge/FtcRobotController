package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        //the bot.trajectoryBuilder() actually returns a special class that allows us to method chain, and
        //when we call the .build() method, it turns that special object into an actual trajectory

        //separated two actions (strafe and then forward) into different trajectories
        Trajectory strafeTrajectory = bot.trajectoryBuilder(startPose)
                .strafeRight(10) //10 inches
                .build();

        Trajectory forwardTrajectory = bot.trajectoryBuilder(strafeTrajectory.end())
                .forward(5)
                .build();

        //added spline between two actions in the same trajectory + a turn
        Trajectory smoothTraj = bot.trajectoryBuilder(new Pose2d(0,0,0)) //added new pose with 90 angle to simulate turn
                .forward(10)
                .splineTo(new Vector2d(15,5), 0)
                .build();

        Trajectory goForward = bot.trajectoryBuilder(smoothTraj.end().plus(new Pose2d(0,0, Math.toRadians(90))), false)
                .forward(5)
                .build();

        //Reverse Spline
        Trajectory reverseSpline = bot.trajectoryBuilder(new Pose2d(), true) // you can also type Math.toRadians(180) in place of true, does the same thing
                        .splineTo(new Vector2d(10,10), 0)
                        .build();


        waitForStart();

        if(isStopRequested()) return;

        //These tell the bot to begin following the path, where each line is executed one by one

        /*\
        bot.followTrajectory(forwardTrajectory);
        bot.followTrajectory(reverseSpline);

        bot.followTrajectory(smoothTraj);
        bot.turn(Math.toRadians(90));
        bot.followTrajectory(goForward);
         \*/

        bot.followTrajectory(strafeTrajectory);
    }
}