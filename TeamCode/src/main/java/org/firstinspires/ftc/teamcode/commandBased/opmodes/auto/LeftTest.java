package org.firstinspires.ftc.teamcode.commandBased.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBased.AutoConstants;
import org.firstinspires.ftc.teamcode.commandBased.classes.enums.Stack;
import org.firstinspires.ftc.teamcode.commandBased.commands._auto.CycleConeMed;
import org.firstinspires.ftc.teamcode.commandBased.commands._auto.InitialMoveMed;
import org.firstinspires.ftc.teamcode.commandBased.commands._auto.InitialMoveStack;
import org.firstinspires.ftc.teamcode.commandBased.commands._auto.ParkIdle;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.AutoOpMode;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.rr.util.DashboardUtil;

@Autonomous
public class LeftTest extends AutoOpMode {

    public static ParkIdle parkIdle;

    public static TrajectorySequence firstMoveToPole;
    public static TrajectorySequence firstMoveToStack;

    public static TrajectorySequence medFromStack;
    public static TrajectorySequence stackFromMed;

    public static TrajectorySequence parkLeft;
    public static TrajectorySequence parkMid;
    public static TrajectorySequence parkRight;

    public static boolean playInit;


    @Override
    public void initialize() {
        super.initialize();

        instantiateTrajectories();

        updateCurrentTag();
    }

    @Override
    public void run() {
        super.run();

        DashboardUtil.drawRobot(fieldOverlay, rrDrive.getPoseEstimate());

        if (!playInit) {
            determinePathFromTag();
            switch (tagPos) {
                case LEFT:
                case NULL:
                    parkIdle = new ParkIdle(subsystems, parkLeft);
                    break;
                case MIDDLE:
                    parkIdle = new ParkIdle(subsystems, parkMid);
                    break;
                case RIGHT:
                    parkIdle = new ParkIdle(subsystems, parkRight);
                    break;
            }

            schedule(new SequentialCommandGroup(
                    new InitialMoveMed(subsystems, firstMoveToPole),
                    new InitialMoveStack(subsystems, firstMoveToStack),

                    new CycleConeMed(subsystems, medFromStack, stackFromMed, Stack.Cone.FOURTH),
                    new CycleConeMed(subsystems, medFromStack, stackFromMed, Stack.Cone.THIRD),

                    parkIdle
            ));

            playInit = true;
        }
    }



    protected void instantiateTrajectories() {
        instantiateFirstTrajectories();
        instantiateCyclingTrajectories();
        instantiateParkingTrajectories();
    }

    protected void instantiateFirstTrajectories() {
        firstMoveToPole = drive.trajectorySequenceBuilder(AutoConstants.START_POSE_LEFT)
                .splineToSplineHeading(AutoConstants.INITIAL_SCORE_FIRST_MED_LEFT)
                .splineToSplineHeading(AutoConstants.INITIAL_SCORE_SECOND_MED_LEFT)
                .build();

        firstMoveToStack = drive.trajectorySequenceBuilder(firstMoveToPole.end())
                .lineToLinearHeading(AutoConstants.INITIAL_STACK_FIRST_MED_LEFT)
                .splineToLinearHeading(AutoConstants.INITIAL_STACK_SECOND_MED_LEFT)
                .build();
    }

    protected void instantiateCyclingTrajectories() {
        medFromStack = drive.trajectorySequenceBuilder(AutoConstants.STACK_POSE_LEFT)
                .lineToLinearHeading(AutoConstants.MED_FIRST)
                .splineToSplineHeading(AutoConstants.MED_SECOND)
                .build();

        stackFromMed = drive.trajectorySequenceBuilder(AutoConstants.MED_POSE_LEFT)
                .lineToLinearHeading(AutoConstants.STACK_FIRST)
                .splineToLinearHeading(AutoConstants.STACK_SECOND)
                .build();
    }

    protected void instantiateParkingTrajectories() {
        parkLeft = drive.trajectorySequenceBuilder(AutoConstants.MED_POSE_LEFT)
                .lineToLinearHeading(AutoConstants.PARK_L_FIRST_LEFT)
                .splineToLinearHeading(AutoConstants.PARK_L_SECOND_LEFT)
                .build();

        parkMid = drive.trajectorySequenceBuilder(AutoConstants.MED_POSE_LEFT)
                .lineToLinearHeading(AutoConstants.PARK_M_FIRST_LEFT)
                .build();

        parkRight = drive.trajectorySequenceBuilder(AutoConstants.MED_POSE_LEFT)
                .lineToLinearHeading(AutoConstants.PARK_R_FIRST_LEFT)
                .splineToLinearHeading(AutoConstants.PARK_R_SECOND_LEFT)
                .build();
    }
}
