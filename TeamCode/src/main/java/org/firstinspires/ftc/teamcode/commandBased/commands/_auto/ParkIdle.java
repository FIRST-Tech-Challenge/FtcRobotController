package org.firstinspires.ftc.teamcode.commandBased.commands._auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.commands._groups.ComeFinished;
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
                        new ComeFinished(
                                subsystems.getEle(),
                                subsystems.getArm(),
                                subsystems.getRot()
                        ),
                        new FollowTrajectorySequenceAsync(subsystems.rrDrive(), parkTraj)
                )
        );
    }
}
