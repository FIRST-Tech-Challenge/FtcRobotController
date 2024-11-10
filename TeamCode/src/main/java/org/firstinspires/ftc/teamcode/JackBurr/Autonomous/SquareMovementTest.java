package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class SquareMovementTest extends OpMode {
    public SampleMecanumDrive drive;
    public Trajectory forward;
    public Trajectory backward;
    public Trajectory left;
    public Trajectory right;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        forward = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();
        //Need to add new trajectories with .waitSeconds(0.2) after each movement
        right = drive.trajectoryBuilder(forward.end())
                .strafeRight(30)
                .build();
        backward = drive.trajectoryBuilder(right.end())
                .back(30)
                .build();
        left = drive.trajectoryBuilder(backward.end())
                .strafeLeft(30)
                .build();

    }

    @Override
    public void start() {
        drive.followTrajectory(forward);
        drive.followTrajectory(right);
        drive.followTrajectory(backward);
        drive.followTrajectory(left);
    }

    @Override
    public void loop(){

    }
}
