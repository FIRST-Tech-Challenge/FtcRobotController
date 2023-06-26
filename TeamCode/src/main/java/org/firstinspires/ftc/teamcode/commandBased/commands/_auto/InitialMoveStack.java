package org.firstinspires.ftc.teamcode.commandBased.commands._auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.classes.enums.Stack;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.GrabConeStack;
import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.AutoSubsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class InitialMoveStack extends SequentialCommandGroup {

    public InitialMoveStack(
            AutoSubsystems subsystems,
            TrajectorySequence traj
    ) {
        addCommands(
                new ParallelCommandGroup(
                        new MoveElevatorToPosition(subsystems.getEle(), Constants.ELE_STACK),
                        new FollowTrajectorySequenceAsync(subsystems.getDrive(), traj)
                ),
                new GrabConeStack(
                        subsystems.getEle(),
                        subsystems.getArm(),
                        subsystems.getRot(),
                        subsystems.getIntake(),
                        Stack.Cone.FIFTH
                )
        );
    }
}
