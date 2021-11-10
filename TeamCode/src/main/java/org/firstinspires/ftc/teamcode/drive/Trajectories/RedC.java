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
public class RedC extends LinearOpMode {
       @Override
    public void runOpMode() {
           MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

           Pose2d startPose = new Pose2d(-62,-55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);

           Trajectory targetZoneC = drive.trajectoryBuilder(startPose)
               .splineTo(new Vector2d(55, -51), Math.toRadians(0))
               .build();


              Trajectory cToGoal = drive.trajectoryBuilder(targetZoneC.end(),true)
                      .splineTo(new Vector2d(-33,-22), Math.toRadians(180))
                      .build();

              Trajectory goalToC = drive.trajectoryBuilder(cToGoal.end())
                      .splineTo(new Vector2d(48,-36), Math.toRadians(90))
                      .build();

              Trajectory cToLine = drive.trajectoryBuilder(goalToC.end())
                      .splineTo(new Vector2d(12,12), Math.toRadians(180))
                      .addTemporalMarker(0.5,()-> {
                          drive.retractArm();
                          sleep(1000);
                          drive.arm.setPower(0);
                      })
                      .build();



             /* Trajectory cToLine = drive.trajectoryBuilder(dropOffGoal.end(),true)
                      .splineTo(new Vector2d(5,50), Math.toRadians(-180))
                      .build();
            */


           waitForStart();

           if(isStopRequested()) return;

           drive.grabGoal();//grab goal,


           drive.followTrajectory(targetZoneC); //Drive to target zone C
           drive.releaseGoal();// release goal,
           sleep(1000);// wait 1 sec,
           drive.deployArm();// deploy arm,
           drive.followTrajectory(cToGoal);// drive to second goal,
           drive.grabGoal();// grab goal,
           sleep(1500);
           drive.followTrajectory(goalToC);// Drive back to C
           drive.releaseGoal();// release goal
           sleep(1000);

           drive.followTrajectory(cToLine);// drive to line + pick up arm

          //

       }
}
