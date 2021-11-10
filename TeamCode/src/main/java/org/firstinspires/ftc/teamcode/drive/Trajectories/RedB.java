package org.firstinspires.ftc.teamcode.drive.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;
@Disabled
@Autonomous
public class RedB extends LinearOpMode {
       @Override
    public void runOpMode() {
           MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);

           Pose2d startPose = new Pose2d(-62,-55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);


           Trajectory startToB = drive.trajectoryBuilder(new Pose2d(-62, -55))
                   .forward(90)
                   .build();
              Trajectory left = drive.trajectoryBuilder(startToB.end())
                      .strafeLeft(35)
                      .build();
           Trajectory bToGoal = drive.trajectoryBuilder(left.end(), true)
                   .splineTo(new Vector2d(-36, -24),Math.toRadians(180))
                   .build();


              Trajectory goalToB = drive.trajectoryBuilder(bToGoal.end())
                   .splineTo(new Vector2d(24, -25), Math.toRadians(90))
                   .build();

                  Trajectory bLine2 = drive.trajectoryBuilder(goalToB.end())
                   .splineTo(new Vector2d( 0, 0 ), Math.toRadians(180))
                   .build();








           waitForStart();

           if(isStopRequested()) return;
           drive.grabGoal();
           sleep(500);
           drive.followTrajectory(startToB);
           drive.followTrajectory(left);
           drive.releaseGoal();
           sleep(1000);
           drive.deployArm();
           drive.followTrajectory(bToGoal);//Go back for second wobble goal
           drive.grabGoal(); //grab wobble goal
           drive.followTrajectory(goalToB);
           drive.releaseGoal();
           sleep(1000);
           drive.followTrajectory(bLine2);
           drive.retractArm();
           sleep(1000);



           //Shoot preloaded rings

       }


}
