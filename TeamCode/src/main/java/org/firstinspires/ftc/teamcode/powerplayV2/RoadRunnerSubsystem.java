package org.firstinspires.ftc.teamcode.powerplayV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.myroadrunner.trajectorysequence.TrajectorySequence;

public class RoadRunnerSubsystem extends SubsystemBase {
    protected SampleMecanumDrive driveR;
    protected Trajectory sm;
    protected Trajectory smPlanB;
    protected Trajectory smPlanC;
    protected Trajectory test;
    protected Trajectory Scoring;
    protected Trajectory hs;
    protected TrajectorySequence hs2;
    protected Trajectory hsPlanB;
    protected Trajectory gsPlanB;
    protected Trajectory sgPlanB;
    protected Trajectory hsPlanC;
    protected Trajectory planBGrabbingToPlanCScoring;
    protected Trajectory planAGrabbingToPlanCScoring;
    protected Trajectory p1;
    protected Trajectory p3;
    static final double ErrorY = 2.5;
    static final double ErrorX = 0;

    int inverted = 1;
    int robotFB = 13 ;
    double dposition = 15.9;

//    double slidderSize = 13.58;
//    double frontSlidderSize = 21.80;
//
//    double dBackPosition = 18.79;
//    double dFrontPosition = 24.60;

    protected Pose2d homePose = new Pose2d(inverted *35 + ErrorX ,-72 + 4.4 + (13.2/2) + ErrorY, Math.toRadians(inverted *90));
    protected Pose2d midPose = new Pose2d(inverted *35, -12, Math.toRadians(inverted * 90));
    protected Pose2d planAMidPose = new Pose2d(inverted * 35, -20, Math.toRadians(inverted *-13.25));
    protected Pose2d midPose2 = new Pose2d(inverted *35, -55, Math.toRadians(inverted *0));
    //   protected Pose2d grabbingPose = new Pose2d(71.25 - robotFB, -12, Math.toRadians(180));
    protected Pose2d scoringPose = new Pose2d(inverted *24 + 21.80769231, 0 - 5.538461538, Math.toRadians(inverted *-17.0));
    protected Pose2d grabbingPosePlanB = new Pose2d(inverted *71.25 - robotFB, -12, Math.toRadians(inverted *0));
    protected Pose2d scoringPosePlanB = new Pose2d(inverted * 0 + dposition , -23.62 + dposition, Math.toRadians(inverted *45));
    protected Pose2d grabbingToReleasingPosePlanB = new Pose2d(inverted *28, -14, Math.toRadians(inverted *45));
    protected Pose2d scoringPosePlanC = new Pose2d(inverted *-24 - 21.80769231, 0 - 5.538461538, Math.toRadians(inverted *194.25));
    protected Pose2d midPosePlanC = new Pose2d(inverted *23, -12, Math.toRadians(inverted *180));
    protected Pose2d midPosePlanC2 = new Pose2d(inverted *-20, -12, Math.toRadians(inverted *194.25));
    protected Pose2d parking1 = new Pose2d(12, -12, Math.toRadians(inverted * 90));
    protected Pose2d parking3 = new Pose2d(60, -12, Math.toRadians(inverted * 90));
    public RoadRunnerSubsystem(SampleMecanumDrive drive, HardwareMap hardwareMap, boolean Inverted) {

        if (Inverted) {
            inverted = -1;
        }

//        ClawSubsystem claw = new ClawSubsystem(hardwareMap);
        driveR = drive;
        driveR.setPoseEstimate(homePose);

        test = driveR.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        hs = driveR.trajectoryBuilder(homePose)
                .forward(1)
                .splineToConstantHeading(midPose2.vec(), Math.toRadians(90))
                .splineToSplineHeading(planAMidPose, Math.toRadians(90))
                .splineToConstantHeading(scoringPose.vec(), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        hs2 = driveR.trajectorySequenceBuilder(homePose)
                .forward(53)
                .back(10)
                .turn(Math.toRadians(-107))
                .strafeLeft(3)
                .splineToConstantHeading(scoringPose.vec(), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        planAGrabbingToPlanCScoring = driveR.trajectoryBuilder(scoringPose)
                .strafeRight(1)
                .splineToSplineHeading(midPosePlanC, Math.toRadians(180))
                .forward(30)
                .splineToSplineHeading(midPosePlanC2, Math.toRadians(180))
                .splineToConstantHeading(scoringPosePlanC.vec(), Math.toRadians(90))
                .build();

        hsPlanB = driveR.trajectoryBuilder(homePose)
                .forward(15)
                .splineToConstantHeading(midPose2.vec(), Math.toRadians(90))
                .splineToSplineHeading(midPose, Math.toRadians(90))
                .splineToConstantHeading(grabbingPosePlanB.vec(), Math.toRadians(0))
                .build();

        gsPlanB = driveR.trajectoryBuilder(grabbingPosePlanB)
                .back(2)
                .splineToSplineHeading(grabbingToReleasingPosePlanB, Math.toRadians(180))
                .splineToConstantHeading(scoringPosePlanB.vec(), Math.toRadians(180))
                .build();

        sgPlanB = driveR.trajectoryBuilder(scoringPosePlanB)
                .strafeRight(1)
                .splineToSplineHeading(grabbingPosePlanB, Math.toRadians(0))
                .build();

        hsPlanC = driveR.trajectoryBuilder(homePose)
                .forward(15)
                .splineToConstantHeading(midPose2.vec(), Math.toRadians(90))
                .splineToConstantHeading(midPose.vec(), Math.toRadians(90))
                .splineToSplineHeading(midPosePlanC, Math.toRadians(180))
                .forward(30)
                .splineToSplineHeading(midPosePlanC2, Math.toRadians(180))
                .splineToConstantHeading(scoringPosePlanC.vec(), Math.toRadians(90))
                .build();

        planBGrabbingToPlanCScoring = driveR.trajectoryBuilder(grabbingPosePlanB)
                .back(60)
                .splineToSplineHeading(scoringPosePlanC, Math.toRadians(135))
                .build();

        //        Scoring = driveR.trajectoryBuilder(scoringPose)
//                .addDisplacementMarker(() -> {
//                    (new InstantCommand(claw::grab, claw)).schedule();
//                })
//                .build();

        sm = driveR.trajectoryBuilder(scoringPose)
                .strafeRight(1)
                .splineToSplineHeading(midPose, Math.toRadians(-90))
                .build();

        smPlanB = driveR.trajectoryBuilder(scoringPosePlanB)
                .lineToLinearHeading(midPose)
                .build();

        smPlanC = driveR.trajectoryBuilder(scoringPosePlanC)
                .strafeLeft(1)
                .splineToSplineHeading(midPosePlanC2, Math.toRadians(0))
                .splineToSplineHeading(midPosePlanC, Math.toRadians(0))
                .splineToConstantHeading(midPose.vec(), Math.toRadians(-90))
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
        driveR.followTrajectory(test);
    }

    public void runHS(){
        driveR.followTrajectory(hs);
    }

    public void runHS2(){ driveR.followTrajectorySequence(hs2); }

    public void runTOMID(){
        driveR.followTrajectory(sm);
    }

    public void runHSPLANB(){
        driveR.followTrajectory(hsPlanB);
    }

    public void runGSPLANB(){
        driveR.followTrajectory(gsPlanB);
    }

    public void runSGPLANB(){ driveR.followTrajectory(sgPlanB); }

    public void runSMPLANB(){ driveR.followTrajectory(smPlanB); }

//    public void runSCORING(){
//        driveR.followTrajectory(Scoring);
//    }

    public void runP1(){
        driveR.followTrajectory(p1);
    }

    public void runP3(){
        driveR.followTrajectory(p3);
    }

}