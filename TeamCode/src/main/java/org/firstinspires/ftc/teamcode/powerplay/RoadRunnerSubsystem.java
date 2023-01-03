package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;

public class RoadRunnerSubsystem extends SubsystemBase {

    protected final SampleMecanumDrive drive;
    protected Trajectory t1;
    protected Trajectory t2;
    protected Trajectory t3;

    public RoadRunnerSubsystem(SampleMecanumDrive drive) {
        this.drive = drive;

        t1 = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();
        t2 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(70,30, Math.toRadians(90)))
                .build();
        t3 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(0,0, Math.toRadians(0)))
                .build();
    }

    public void runT1(){
        drive.followTrajectory(t1);
    }

    public void runT2(){
        drive.followTrajectory(t2);
    }

    public void runT3(){
        drive.followTrajectory(t3);
    }

}