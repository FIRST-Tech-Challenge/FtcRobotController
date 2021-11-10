package org.firstinspires.ftc.teamcode.drive.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;
@Disabled

@Autonomous
public class RedA extends LinearOpMode {
       @Override
    public void runOpMode() {

           MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

           Pose2d startPose = new Pose2d(-62,-55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);

           Trajectory lineToA = drive.trajectoryBuilder(startPose)
               .splineTo(new Vector2d( 8,-55), Math.toRadians(0))
               .build();
           Trajectory aToGoal = drive.trajectoryBuilder(lineToA.end(), true )
                   .splineTo(new Vector2d(-33, -22), Math.toRadians(180))
                   .build();
           Trajectory goalToA = drive.trajectoryBuilder(aToGoal.end())
                   .splineTo(new Vector2d(8, -35), Math.toRadians(90))
                   .build();
           Trajectory aLine = drive.trajectoryBuilder(goalToA.end())
                   .splineTo(new Vector2d(8, -6), Math.toRadians(90))
                   .build();
           Trajectory aLine2 = drive.trajectoryBuilder(aLine.end())
                   .splineTo(new Vector2d( 8, -6 ), Math.toRadians(0))
                   .build();








           waitForStart();

           if(isStopRequested()) return;
           drive.grabGoal();
           sleep(500);
           drive.followTrajectory(lineToA);
           drive.releaseGoal();
           sleep(1000);
           drive.deployArm();
           drive.followTrajectory(aToGoal);           //Go back for second wobble goal
           drive.grabGoal(); //grab wobble goal
           drive.followTrajectory(goalToA);
           drive.releaseGoal();
           sleep(1000);
           drive.followTrajectory(aLine);
           drive.followTrajectory(aLine2);
           drive.retractArm();


           //Shoot preloaded rings

       }
}
