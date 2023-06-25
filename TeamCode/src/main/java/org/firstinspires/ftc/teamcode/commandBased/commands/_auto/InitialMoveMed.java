package org.firstinspires.ftc.teamcode.commandBased.commands._auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.ScoreCone;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.ScoreToIdle;
import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsyncCommand;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.AutoDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class InitialMoveMed extends SequentialCommandGroup {

    public InitialMoveMed(
            AutoDrivetrainSubsystem drive,
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake,
            TrajectorySequence medTraj,
            TrajectorySequence stackTraj
    ) {
        addCommands(
                new ParallelCommandGroup(
                        new LiftMoveRotateArm(
                                ele,
                                arm,
                                rot,
                                Constants.ARM_ANGLE_BACK,
                                Constants.ELE_HIGH,
                                Constants.ROTATOR_BACK
                        ),
                        new FollowTrajectorySequenceAsyncCommand(
                                drive, medTraj
                        )
                ),
                new ScoreCone(arm, rot, intake),
                new ParallelCommandGroup(
                        new MoveElevatorToPosition(ele, Constants.ELE_STACK)
                )

        );
    }
}
