package org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.general;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class ParkIdle extends SequentialCommandGroup {

    public ParkIdle(
            Subsystems subsystems,
            TrajectorySequence parkTraj
    ) {
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new ComeFinished(
                                        subsystems.getEle(),
                                        subsystems.getArm(),
                                        subsystems.getRot()
                                )
                        ),
                        new FollowTrajectorySequenceAsync(subsystems.rrDrive(), parkTraj)
                )
        );
    }
}
