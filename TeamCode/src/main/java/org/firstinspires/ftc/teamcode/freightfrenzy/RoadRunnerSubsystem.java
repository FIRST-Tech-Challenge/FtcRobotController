package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;

public class RoadRunnerSubsystem extends SubsystemBase {

    protected SampleMecanumDrive driveR;
    protected Trajectory t1;
    protected Trajectory t2;
    protected Trajectory t3;
    protected Trajectory t4;

    public RoadRunnerSubsystem(SampleMecanumDrive drive) {

        driveR = drive;

        Pose2d homePose = new Pose2d(70.25, -70.625, Math.toRadians(0));
        drive.setPoseEstimate(homePose);

        Pose2d midPose = new Pose2d(70.25, 0, Math.toRadians(0));
        drive.setPoseEstimate(homePose);

        Pose2d grabbingPose = new Pose2d(141.25, 0, Math.toRadians(90));
        drive.setPoseEstimate(homePose);

        Pose2d releasingPose = new Pose2d(46.75, 0, Math.toRadians(-90));
        drive.setPoseEstimate(homePose);


        t1 = drive.trajectoryBuilder(homePose)
                .lineToLinearHeading(midPose)
                .build();
        t2 = drive.trajectoryBuilder(midPose)
                .lineToLinearHeading(releasingPose)
                .build();
        t3 = drive.trajectoryBuilder(releasingPose)
                .lineToLinearHeading(grabbingPose)
                .build();
        t4 = drive.trajectoryBuilder(grabbingPose)
                .lineToLinearHeading(releasingPose)
                .build();

    }

    public void runT1(){
        driveR.followTrajectory(t1);
    }

    public void runT2(){
        driveR.followTrajectory(t2);
    }

    public void runT3(){
        driveR.followTrajectory(t3);
    }

    public void runT4(){
        driveR.followTrajectory(t4);
    }

}