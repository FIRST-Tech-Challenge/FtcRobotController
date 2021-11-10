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
public class SAMPLE extends LinearOpMode {
       @Override
    public void runOpMode() {
           MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

           Pose2d startPose = new Pose2d(-62,55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);

           Trajectory targetZoneA = drive.trajectoryBuilder(startPose)
               .splineTo(new Vector2d(10, 55), Math.toRadians(0))
               .build();

           Trajectory aToGoal = drive.trajectoryBuilder(targetZoneA.end(),true)
                   .splineTo(new Vector2d(-24,31), Math.toRadians(180))
                   .build();

           Trajectory goalToA = drive.trajectoryBuilder(aToGoal.end())
                   .splineTo(new Vector2d(10,50), Math.toRadians(0))
                   .build();



           waitForStart();

           if(isStopRequested()) return;

           drive.followTrajectory(targetZoneA);
           drive.releaseGoal();//Deploy Wobble Goal by setting servo to open
          drive.deployArm();
          sleep(1000);//Deploy Arm
              drive.arm.setPower(0);
           drive.followTrajectory(aToGoal);
           drive.grabGoal();//Grab Goal by setting servo to close
           drive.followTrajectory(goalToA);
           drive.releaseGoal();//Release Goal by setting servo to open

       }
}
