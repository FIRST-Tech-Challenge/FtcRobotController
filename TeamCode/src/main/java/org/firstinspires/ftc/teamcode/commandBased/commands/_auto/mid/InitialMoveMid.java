package org.firstinspires.ftc.teamcode.commandBased.commands._auto.mid;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.commands._auto.mid.parts.InitialMoveMidMid;
import org.firstinspires.ftc.teamcode.commandBased.commands._auto.mid.parts.InitialMoveStackMid;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class InitialMoveMid extends SequentialCommandGroup {

    public InitialMoveMid(
        Subsystems subsystems,
        TrajectorySequence midTraj,
        TrajectorySequence stackTraj
    ) {
        addCommands(
                new InitialMoveMidMid(subsystems, midTraj),
                new InitialMoveStackMid(subsystems, stackTraj)
        );
    }
}
