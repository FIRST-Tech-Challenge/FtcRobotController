package org.firstinspires.ftc.teamcode.commandBased.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commandBased.AutoConstants;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.AutoOpMode;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class LeftTest extends AutoOpMode {

    public static TrajectorySequence firstMoveToPole;
    public static TrajectorySequence firstMoveToStack;

    public static TrajectorySequence medFromStack;
    public static TrajectorySequence stackFromMed;

    public static TrajectorySequence parkLeft;
    public static TrajectorySequence parkMid;
    public static TrajectorySequence parkRight;

    public static Pose2d startPose = new Pose2d(35, 65, Math.toRadians(-90));

    public static double xOffset;
    public static double yOffset;

    @Override
    public void initialize() {
        super.initialize();


    }

    @Override
    public void run() {
        super.run();
    }

    protected void instantiateFirstTrajectories() {
        firstMoveToPole = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(AutoConstants.INITIAL_FIRST_MED_LEFT)
                .splineToSplineHeading(AutoConstants.INITIAL_SECOND_MED_LEFT)
                .build();

        firstMoveToStack = drive.trajectorySequenceBuilder(firstMoveToPole.end())
                .lineToLinearHeading(AutoConstants.INITIAL_THIRD_MED_LEFT)
                .splineToLinearHeading(AutoConstants.INITIAL_FOURTH_MED_LEFT)
                .build();
    }

    protected void instantiateCyclingTrajectories() {
        medFromStack = drive.trajectorySequenceBuilder(firstMoveToStack.end())
                .lineToLinearHeading(AutoConstants.MED_FIRST)
                .splineToSplineHeading(AutoConstants.MED_SECOND)
                .build();
    }
}
