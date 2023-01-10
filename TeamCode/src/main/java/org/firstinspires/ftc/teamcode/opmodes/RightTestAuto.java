package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class RightTestAuto extends LinearOpMode {
    private SampleMecanumDrive drive;
    private Pose2d startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose2d(in(91), in(-155), rad(90));
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(cmPose(87.25, -22, 45), rad(10))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(cmVector(87.25, -50), rad(270))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), false)
                .splineTo(cmVector(87.25, -22), rad(45))
                .build();

        waitForStart();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
    }

    private Vector2d cmVector(double x, double y) {
        return new Vector2d(in(x), in(y));
    }

    private Pose2d cmPose(double x, double y, double headingDeg) {
        return new Pose2d(in(x), in(y), rad(headingDeg));
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}
