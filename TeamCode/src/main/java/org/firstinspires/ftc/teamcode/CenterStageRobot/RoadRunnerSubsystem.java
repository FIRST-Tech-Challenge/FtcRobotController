package org.firstinspires.ftc.teamcode.CenterStageRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.inventors.ftc.robotbase.MecanumDrivePPV2;
import org.inventors.ftc.trajectorysequence.TrajectorySequence;

public class RoadRunnerSubsystem extends SubsystemBase {
    protected MecanumDrivePPV2 driveR;
    protected TrajectorySequence test;
    protected Pose2d homePose = new Pose2d();

    public RoadRunnerSubsystem(MecanumDrivePPV2 drive, boolean Inverted)
    {

        driveR = drive;
        driveR.setPoseEstimate(homePose);

        test = driveR.trajectorySequenceBuilder(homePose)
                .turn(Math.toRadians(-45))
                .build();
    }
    public void runTEST(){
        driveR.followTrajectorySequence(test);
    }
}