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
public class RedA2 extends LinearOpMode {
       @Override
    public void runOpMode() {

           MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

           Pose2d startPose = new Pose2d(-62,-55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);









           waitForStart();

           if(isStopRequested()) return;
           //Read the stack of rings
              //go to drop zone
              // drop wobble goal
              // go to start line
              //grab second wobble goal
              //go to parking line
              // shoot preloaded rings
              // go back to drop zone
              //drop wobble goal
              // drive to parking line






           //Shoot preloaded rings

       }
}
