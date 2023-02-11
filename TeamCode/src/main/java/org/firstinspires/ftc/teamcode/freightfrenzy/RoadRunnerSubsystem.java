package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;

public class RoadRunnerSubsystem extends SubsystemBase {

    protected SampleMecanumDrive driveR;
    protected Trajectory ToMid;
    protected Trajectory gr;
    protected Trajectory rg;
    protected Trajectory mr;
    protected Trajectory hm;
    protected Trajectory p1;
    protected Trajectory p2;
    protected Trajectory p3;

    int robotFB = 13;
    double dposition = 9.19;

    protected Pose2d homePose = new Pose2d(35, -70.625, Math.toRadians(90));
    protected Pose2d midPose = new Pose2d(35, -12, Math.toRadians(90));
    protected Pose2d grabbingPose = new Pose2d(71.25 - robotFB, -12, Math.toRadians(180));
    protected Pose2d releasingPose = new Pose2d(24 + dposition, 0 - dposition, Math.toRadians(135));
    protected Pose2d parking1 = new Pose2d(12, -12, Math.toRadians(90));
    protected Pose2d parking2 = new Pose2d(36, -12, Math.toRadians(90));
    protected Pose2d parking3 = new Pose2d(60, -12, Math.toRadians(90));
    protected  Pose2d currentPose;

    public RoadRunnerSubsystem(SampleMecanumDrive drive) {

        driveR = drive;

        currentPose = drive.getPoseEstimate();

        hm = drive.trajectoryBuilder(homePose)
                .lineToLinearHeading(midPose)
                .build();

        mr = drive.trajectoryBuilder(midPose)
                .lineToLinearHeading(releasingPose)
                .build();

        rg = drive.trajectoryBuilder(releasingPose)
                .lineToLinearHeading(grabbingPose)
                .build();

        gr = drive.trajectoryBuilder(grabbingPose)
                .lineToLinearHeading(releasingPose)
                .build();

        ToMid = drive.trajectoryBuilder(currentPose)
                .lineToLinearHeading(midPose)
                .build();

        p1 = drive.trajectoryBuilder(midPose)
                .lineToLinearHeading(midPose)
                .lineToLinearHeading(parking2)
                .lineToLinearHeading(parking1)
                .build();

        p2 = drive.trajectoryBuilder(midPose)
                .lineToLinearHeading(parking2)
                .build();

        p3 = drive.trajectoryBuilder(midPose)
                .lineToLinearHeading(midPose)
                .lineToLinearHeading(parking2)
                .lineToLinearHeading(parking3)
                .build();
    }

    public void runHM(){
        driveR.followTrajectory(hm);
    }

    public void runTOMID(){
        driveR.followTrajectory(ToMid);
    }

    public void runMR(){
        driveR.followTrajectory(mr);
    }

    public void runRG(){
        driveR.followTrajectory(rg);
    }

    public void runGR(){
        driveR.followTrajectory(gr);
    }

    public void runP1(){
        driveR.followTrajectory(p1);
    }

    public void runP2(){
        driveR.followTrajectory(p2);
    }

    public void runP3(){
        driveR.followTrajectory(p3);
    }

}