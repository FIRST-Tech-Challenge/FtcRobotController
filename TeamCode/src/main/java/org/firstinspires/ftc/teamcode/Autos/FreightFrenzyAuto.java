package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class FreightFrenzyAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        int stepIncrement = 0;
        int numCycles = 3;

        Pose2d startPose = new Pose2d(-12, -59, Math.toRadians(90)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        //the bot.trajectoryBuilder() actually returns a special class that allows us to method chain, and
        //when we call the .build() method, it turns that special object into an actual trajectory

        Trajectory forwardToHub = bot.trajectoryBuilder(startPose)
                .forward(15)
                .build();

        Trajectory hubToWarehouse = bot.trajectoryBuilder(forwardToHub.end())
                .lineToLinearHeading(new Pose2d(10,-63.5, Math.toRadians(180)))
                .back(25 + stepIncrement)
                //.wait() <- isn't working for some reason?
                .forward(25 + stepIncrement)
                .build();

        Trajectory warehouseToHub = bot.trajectoryBuilder(hubToWarehouse.end())
                .lineToLinearHeading(new Pose2d(0,-42, Math.toRadians(125)))
                .build();

        Trajectory hubToParking = bot.trajectoryBuilder(warehouseToHub.end(), true)
                .splineToSplineHeading(new Pose2d(-59,-35), Math.toRadians(180))
                .build();

        /*
        Trajectory forwardTrajectory = bot.trajectoryBuilder(strafeTrajectory.end())
                .forward(5)
                .build();

        Trajectory smoothTraj = bot.trajectoryBuilder(new Pose2d(0,0,0)) //added new pose with 90 angle to simulate turn
                .forward(10)
                .splineTo(new Vector2d(15,5), 0)
                .build();

        Trajectory goForward = bot.trajectoryBuilder(smoothTraj.end().plus(new Pose2d(0,0, Math.toRadians(90))), false)
                .forward(5)
                .build();
         */



        waitForStart();

        if(isStopRequested()) return;

        //These tell the bot to begin following the path, where each line is executed one by one

        /*\
        bot.followTrajectory(strafeTrajectory);



        bot.followTrajectory(reverseSpline);
        bot.followTrajectory(forwardTrajectory);
        bot.followTrajectory(smoothTraj);
        bot.turn(Math.toRadians(90));
        bot.followTrajectory(goForward);
         \*/

        // Rather than use a for loop, we should copy and paste the same trajectory sequence and adjust accordingly.
        // Using a for loop will create minor inconsistencies since our encoders are not perfect (error increases in magnitude each iteration)
        for(int i = 0; i < numCycles; i++) {
            bot.followTrajectory(forwardToHub);
            //should be pauses in between trajectories but idk how exactly to do that
            bot.followTrajectory(hubToWarehouse);
            bot.followTrajectory(warehouseToHub);
        }
        bot.followTrajectory(hubToParking);

    }
}