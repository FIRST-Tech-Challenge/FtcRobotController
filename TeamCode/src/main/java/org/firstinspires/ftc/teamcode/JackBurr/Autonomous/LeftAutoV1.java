package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class LeftAutoV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Pick SampleMecanumDrive for dashboard and RRMecanumDrive for no dashboard
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Pose2d startPose = new Pose2d(60, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(40, -15), Math.toRadians(135))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(0, -15), Math.toRadians(-60))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(50)
                .build();



        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
    }
}
