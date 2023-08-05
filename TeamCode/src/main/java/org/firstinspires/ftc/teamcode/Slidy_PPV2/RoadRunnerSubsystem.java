package org.firstinspires.ftc.teamcode.Slidy_PPV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.inventors.ftc.robotbase.SampleDriveConstants;
import org.inventors.ftc.robotbase.MecanumDrivePPV2;
import org.inventors.ftc.trajectorysequence.TrajectorySequence;

public class RoadRunnerSubsystem extends SubsystemBase {
    protected MecanumDrivePPV2 driveR;
    protected Trajectory sm, smPlanB, smPlanC;
    protected TrajectorySequence test, Scoring;
    protected Trajectory hs, hsPlanB, hsPlanC;
    protected Trajectory gsPlanB, sgPlanB;
    protected Trajectory planBGrabbingToPlanCScoring, planAGrabbingToPlanCScoring;
    protected Trajectory p1, p3;
    protected TrajectorySequence hs2;
    static final double ErrorY = 2.5, ErrorX = 0;
    int inverted = 1, robotFB = 13 ;
    double dposition = 15.9;

    protected Pose2d homePose = new Pose2d(inverted *35 + ErrorX ,-72 + 4.4 + (13.2/2) + ErrorY, Math.toRadians(inverted *90));
    protected Pose2d midPose = new Pose2d(inverted *35, -12, Math.toRadians(inverted * 90));
    protected Pose2d planAMidPose = new Pose2d(inverted * 35, -20, Math.toRadians(inverted *-13.25));
    protected Pose2d midPose2 = new Pose2d(inverted *35, -55, Math.toRadians(inverted *0));
    protected Pose2d scoringPose = new Pose2d(inverted *24 + 21.80769231, 0 - 5.538461538, Math.toRadians(inverted *-17.0));
    protected Pose2d parking1 = new Pose2d(12, -12, Math.toRadians(inverted * 90));
    protected Pose2d parking3 = new Pose2d(60, -12, Math.toRadians(inverted * 90));
    public RoadRunnerSubsystem(MecanumDrivePPV2 drive, boolean Inverted)
    {
        if (Inverted) inverted = -1;

        driveR = drive;
        driveR.setPoseEstimate(homePose);

        test = driveR.trajectorySequenceBuilder(homePose)
                .turn(Math.toRadians(-45))
                .build();

        hs = driveR.trajectoryBuilder(homePose)
                .forward(1)
                .splineToConstantHeading(midPose2.vec(), Math.toRadians(90))
                .splineToSplineHeading(planAMidPose, Math.toRadians(90))
                .splineToConstantHeading(scoringPose.vec(), Math.toRadians(90),
                        MecanumDrivePPV2.getVelocityConstraint(10, SampleDriveConstants.MAX_ANG_VEL, SampleDriveConstants.TRACK_WIDTH),
                        MecanumDrivePPV2.getAccelerationConstraint(SampleDriveConstants.MAX_ACCEL))
                .build();

        hs2 = driveR.trajectorySequenceBuilder(homePose)
                .forward(53)
                .back(10)
                .turn(Math.toRadians(-107))
                .strafeLeft(3)
                .splineToConstantHeading(scoringPose.vec(), Math.toRadians(90),
                        MecanumDrivePPV2.getVelocityConstraint(30, SampleDriveConstants.MAX_ANG_VEL, SampleDriveConstants.TRACK_WIDTH),
                        MecanumDrivePPV2.getAccelerationConstraint(SampleDriveConstants.MAX_ACCEL))
                .build();

        sm = driveR.trajectoryBuilder(scoringPose)
                .strafeRight(1)
                .splineToSplineHeading(midPose, Math.toRadians(-90))
                .build();

        p1 = driveR.trajectoryBuilder(scoringPose)
                .strafeRight(1)
                .splineToSplineHeading(parking1, Math.toRadians(180))
                .build();

        p3 = driveR.trajectoryBuilder(scoringPose)
                .strafeRight(1)
                .splineToSplineHeading(parking3, Math.toRadians(0))
                .build();

    }
    public void runTEST(){
        driveR.followTrajectorySequence(test);
    }

    public void runHS(){
        driveR.followTrajectory(hs);
    }

    public void runHS2(){ driveR.followTrajectorySequence(hs2); }

    public void runTOMID(){
        driveR.followTrajectory(sm);
    }

    public void runP1(){
        driveR.followTrajectory(p1);
    }

    public void runP3(){
        driveR.followTrajectory(p3);
    }

}