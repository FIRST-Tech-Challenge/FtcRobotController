package org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.parts;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.ELE_STACK;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.classes.enums.Stack;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.general.GrabConeStackAuto;
import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class GrabConeMid extends SequentialCommandGroup {

    public GrabConeMid(
            Subsystems subsystems,
            TrajectorySequence traj,
            Stack.Cone coneNumber
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
                        coneNumber
                )
        );
    }
}
