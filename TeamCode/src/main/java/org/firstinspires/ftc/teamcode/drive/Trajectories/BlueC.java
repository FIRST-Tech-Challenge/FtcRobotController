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
public class BlueC extends LinearOpMode {
       @Override
    public void runOpMode() {
           MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

           Pose2d startPose = new Pose2d(-62,55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);

           Trajectory targetZoneC = drive.trajectoryBuilder(startPose)
               .splineTo(new Vector2d(60, 50), Math.toRadians(0))
               .build();


              Trajectory cToGoal = drive.trajectoryBuilder(targetZoneC.end(),true)
                      .splineTo(new Vector2d(-29,26), Math.toRadians(-180))
                      .build();

              Trajectory goalToC = drive.trajectoryBuilder(cToGoal.end())
                      .splineTo(new Vector2d(60,50), Math.toRadians(0))
                      .build();
              Trajectory dropOffGoal = drive.trajectoryBuilder(goalToC.end())
                      .splineTo(new Vector2d(60,50), Math.toRadians(-180))
                      .build();
              Trajectory cToLine = drive.trajectoryBuilder(dropOffGoal.end(),true)
                      .splineTo(new Vector2d(5,50), Math.toRadians(-180))
                      .build();



           waitForStart();

           if(isStopRequested()) return;

           drive.followTrajectory(targetZoneC);

           drive.followTrajectory(cToGoal);
           drive.followTrajectory(goalToC);
           drive.followTrajectory(dropOffGoal);
           drive.followTrajectory(cToLine);



       }
}
