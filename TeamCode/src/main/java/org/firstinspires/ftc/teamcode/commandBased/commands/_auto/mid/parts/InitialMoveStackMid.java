package org.firstinspires.ftc.teamcode.commandBased.commands._auto.mid.parts;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.classes.enums.Stack;
import org.firstinspires.ftc.teamcode.commandBased.commands._auto.general.GrabConeStackAuto;
import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

public class InitialMoveStackMid extends SequentialCommandGroup {

    public InitialMoveStackMid(
            Subsystems subsystems,
            TrajectorySequence traj
    ) {
        addCommands(
                new ParallelCommandGroup(
                        new MoveElevatorToPosition(subsystems.getEle(), ELE_STACK),
                        new FollowTrajectorySequenceAsync(subsystems.rrDrive(), traj)
                ),
                new GrabConeStackAuto(
                        subsystems.getEle(),
                        subsystems.getArm(),
                        subsystems.getRot(),
                        subsystems.getIntake(),
                        Stack.Cone.FIFTH
                )
        );
    }
}
