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

        forward = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();
        backward = drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .build();
        left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(10)
                .build();
        right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
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
