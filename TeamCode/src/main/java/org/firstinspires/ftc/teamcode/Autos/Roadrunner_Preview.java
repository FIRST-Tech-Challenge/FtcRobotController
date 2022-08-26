package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

@Autonomous
public class Roadrunner_Preview extends LinearOpMode {
        @Override
        public void runOpMode(){
            SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

            int stepIncrement = 0;
            int numCycles = 3;

            // Creating and setting a start position
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90)); // x, y, heading (angle in radians)
            bot.setPoseEstimate(startPose);

            // Building trajectories:
            Trajectory forwardTest = bot.trajectoryBuilder(startPose)
                .forward(10)
                .build();




            waitForStart();
            if(isStopRequested()) return;



            bot.followTrajectory(forwardTest);
        }

}
