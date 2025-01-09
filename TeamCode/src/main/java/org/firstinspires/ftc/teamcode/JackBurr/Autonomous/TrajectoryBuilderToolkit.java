package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Callable;

public class TrajectoryBuilderToolkit {
    public HardwareMap hardwareMap;
    public SampleMecanumDrive drive;
    public Pose2d latestPose;

    public void init(HardwareMap hardwareMap, Pose2d startPose){
        this.hardwareMap = hardwareMap;
        this.drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        latestPose = startPose;
    }

    public Trajectory moveForward(double inches){
        Trajectory trajectory = drive.trajectoryBuilder(latestPose)
                .forward(inches)
                .build();
        drive.followTrajectory(trajectory);
        latestPose = trajectory.end();
        return trajectory;
    }

    public Trajectory moveBackward(double inches){
        Trajectory trajectory = drive.trajectoryBuilder(latestPose)
                .back(inches)
                .build();
        drive.followTrajectory(trajectory);
        latestPose = trajectory.end();
        return trajectory;
    }

    public Trajectory strafeLeft(double inches){
        Trajectory trajectory = drive.trajectoryBuilder(latestPose)
                .strafeLeft(inches)
                .build();
        drive.followTrajectory(trajectory);
        latestPose = trajectory.end();
        return trajectory;
    }

    public Trajectory strafeRight(double inches){
        Trajectory trajectory = drive.trajectoryBuilder(latestPose)
                .strafeRight(inches)
                .build();
        drive.followTrajectory(trajectory);
        latestPose = trajectory.end();
        return trajectory;
    }

    public Trajectory strafeTo(double x, double y){
        Trajectory trajectory = drive.trajectoryBuilder(latestPose)
                .strafeTo(new Vector2d(x,y))
                .build();
        drive.followTrajectory(trajectory);
        latestPose = trajectory.end();
        return trajectory;
    }

    public Trajectory lineTo(double x, double y){
        Trajectory trajectory = drive.trajectoryBuilder(latestPose)
                .lineTo(new Vector2d(x,y))
                .build();
        drive.followTrajectory(trajectory);
        latestPose = trajectory.end();
        return trajectory;
    }

    public Trajectory splineTo(double x, double y, double angleDegrees){
        Trajectory trajectory = drive.trajectoryBuilder(latestPose)
                .splineTo(new Vector2d(x,y), Math.toRadians(angleDegrees))
                .build();
        drive.followTrajectory(trajectory);
        latestPose = trajectory.end();
        return trajectory;
    }

    public Trajectory splineToConstantHeading(double x, double y, double angle){
        Trajectory trajectory = drive.trajectoryBuilder(latestPose)
                .splineToConstantHeading(new Vector2d(x,y), Math.toRadians(angle))
                .build();
        drive.followTrajectory(trajectory);
        latestPose = trajectory.end();
        return trajectory;
    }

    public Pose2d turnLeft(double degrees){
        drive.turn(Math.toRadians(degrees));
        latestPose = latestPose.plus(new Pose2d(0, 0, Math.toRadians(degrees)));
        return latestPose;
    }
    public Pose2d turnRight(double degrees){
        drive.turn(Math.toRadians(-degrees));
        latestPose = latestPose.plus(new Pose2d(0, 0, Math.toRadians(-degrees)));
        return latestPose;
    }
}
