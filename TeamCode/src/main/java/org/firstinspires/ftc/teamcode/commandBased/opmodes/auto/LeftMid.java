package org.firstinspires.ftc.teamcode.commandBased.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBased.classes.enums.Stack;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.CycleConeMid;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.InitialMoveMid;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.general.ParkIdle;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.parts.InitialMoveMidMid;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.parts.ScoreConeMid;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.AutoOpMode;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.Positions;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;
import static org.firstinspires.ftc.teamcode.commandBased.AutoLConstants.*;

@Autonomous
public class LeftMid extends AutoOpMode {

    public static ParkIdle parkIdle;

    public static TrajectorySequence firstMoveToPole;
    public static TrajectorySequence firstMoveToStack;

    public static TrajectorySequence medFromStack;
    public static TrajectorySequence stackFromMed;

    public static TrajectorySequence parkLeft;
    public static TrajectorySequence parkMid;
    public static TrajectorySequence parkRight;

    public static boolean playInit;
    public static double voltageOffset;
    public static double voltage = 0;


    @Override
    public void initialize() {
        super.initialize();

        instantiateTrajectories();

        drive.setPoseEstimate(START_POSE);

        voltage = getVoltage();

        initSchedule();

        playInit = false;

        updateCurrentTag();
    }

    @Override
    public void run() {
        super.run();

        tad("voltage offset", voltageOffset);

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

            pathSchedule();

            playInit = true;
        }
    }



    protected void initSchedule() {
        schedule(
                new MoveRotatorToPosition(rotatorSS, ROTATOR_FRONT)
        );
    }

    protected void pathSchedule() {
        schedule(
                new SequentialCommandGroup(
                        new InitialMoveMidMid(subsystems, firstMoveToPole),

//                        new InstantCommand(this::driftAccommodation),
//                        new CycleConeMid(subsystems, medFromStack, stackFromMed, Stack.Cone.FOURTH),
//
//                        new InstantCommand(this::driftAccommodation),
//                        new CycleConeMid(subsystems, medFromStack, stackFromMed, Stack.Cone.THIRD),
//
//                        new InstantCommand(this::driftAccommodation),
//                        new ScoreConeMid(subsystems, medFromStack),
                        parkIdle
//                        new InstantCommand(this::savePositions)
                ));
    }

    protected void driftAccommodation() {
        Pose2d current = drive.getPoseEstimate();
        Pose2d newPose;
        if (voltage > 13.5) {
            newPose = current.plus(new Pose2d(X_DRIFT_14, Y_DRIFT_14, 0));
            voltageOffset = 14;
        } else if (voltage > 13) {
            newPose = current.plus(new Pose2d(X_DRIFT_13_5, Y_DRIFT_13_5, 0));
            voltageOffset = 13.5;
        } else if (voltage > 12.75) {
            newPose = current.plus(new Pose2d(X_DRIFT_13, Y_DRIFT_13, 0));
            voltageOffset = 13;
        } else if (voltage > 12.5) {
            newPose = current.plus(new Pose2d(X_DRIFT_12_75, Y_DRIFT_12_75, 0));
            voltageOffset = 12.75;
        } else {
            newPose = current.plus(new Pose2d(X_DRIFT_12_5, Y_DRIFT_12_5, 0));
            voltageOffset = 12.5;
        }

        drive.setPoseEstimate(newPose);
    }

    protected void savePositions() {
        Positions.elePosition = elevatorSS.getElePos();
        Positions.armPosition = armSS.getArmPos();
    }

    protected void instantiateTrajectories() {
        instantiateFirstTrajectories();
        instantiateCyclingTrajectories();
        instantiateParkingTrajectories();
    }

    protected void instantiateFirstTrajectories() {
        firstMoveToPole = drive.trajectorySequenceBuilder(START_POSE)
                .splineToSplineHeading(INITIAL_SCORE_FIRST_MED)
                .splineToSplineHeading(INITIAL_SCORE_SECOND_MED)
                .build();

        firstMoveToStack = drive.trajectorySequenceBuilder(firstMoveToPole.end())
                .lineToLinearHeading(INITIAL_STACK_FIRST_MED)
                .splineToLinearHeading(INITIAL_STACK_SECOND_MED)
                .build();
    }

    protected void instantiateCyclingTrajectories() {
        medFromStack = drive.trajectorySequenceBuilder(STACK_POSE)
                .lineToLinearHeading(MED_FIRST)
                .splineToLinearHeading(MED_SECOND)
                .build();

        stackFromMed = drive.trajectorySequenceBuilder(MED_POSE)
                .lineToLinearHeading(STACK_MED_FIRST)
                .splineToLinearHeading(STACK_MED_SECOND)
                .build();
    }

    protected void instantiateParkingTrajectories() {
        parkLeft = drive.trajectorySequenceBuilder(MED_POSE)
                .lineToLinearHeading(PARK_L_FIRST)
                .splineToLinearHeading(PARK_L_SECOND)
                .build();

        parkMid = drive.trajectorySequenceBuilder(MED_POSE)
                .lineToLinearHeading(PARK_M_FIRST)
                .build();

        parkRight = drive.trajectorySequenceBuilder(MED_POSE)
                .lineToLinearHeading(PARK_R_FIRST)
                .splineToLinearHeading(PARK_R_SECOND)
                .build();
    }
}
