package org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.classes.enums.Stack;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.parts.GrabConeMid;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.parts.ScoreConeMid;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class CycleConeMid extends SequentialCommandGroup {

    /**
     * @param subsystems group of subsystems
     * @param medTraj trajectory from the stack to the medium pole
     * @param stackTraj trajectory from the medium pole to the stack
     * @param coneNumber number cone from the stack (top = 5, bottom = 1)
     *
     * Starts at stack with a cone, ends at stack with a cone
     */
    public CycleConeMid(
            Subsystems subsystems,
            TrajectorySequence medTraj,
            TrajectorySequence stackTraj,
            Stack.Cone coneNumber
    ) {
        addCommands(
                new ScoreConeMid(subsystems, medTraj),
                new GrabConeMid(subsystems, stackTraj, coneNumber)
        );
    }
}
