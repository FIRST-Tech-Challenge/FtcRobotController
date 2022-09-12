package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class EXAMPLE_SlowerAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        // This spline will be at 20 inches per second
        Trajectory spline1 = bot.trajectoryBuilder(new Pose2d(0,0), false)
            .splineTo(
            new Vector2d(30, 30), Math.toRadians(90),
            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();


         sleep(2000); //1,000 milliseconds in a second :/

        // In contrast, this spline will be at normal speed
        Trajectory spline2 = bot.trajectoryBuilder(new Pose2d(0,0), false)
            .splineTo(
            new Vector2d(50, 50), Math.toRadians(90)
            )
            .build();

        waitForStart();
        if(isStopRequested()) return;

        bot.followTrajectory(spline1);
        bot.followTrajectory(spline2);
    }

}
