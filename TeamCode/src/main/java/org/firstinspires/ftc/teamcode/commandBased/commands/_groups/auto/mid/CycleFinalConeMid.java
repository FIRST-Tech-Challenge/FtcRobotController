package org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.ScoreCone;
import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.*;

public class CycleFinalConeMid extends SequentialCommandGroup {


    public CycleFinalConeMid(
            Subsystems subsystems,
            TrajectorySequence medTraj
    ) {
        addCommands(
                new ParallelCommandGroup(
                        new LiftMoveRotateArm(
                                subsystems.getEle(),
                                subsystems.getArm(),
                                subsystems.getRot(),
                                ARM_ANGLE_BACK,
                                ELE_MID,
                                ROTATOR_BACK
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(75),
                                new FollowTrajectorySequenceAsync(subsystems.rrDrive(), medTraj)
                        )
                ),
                new ScoreCone(subsystems.getArm(), subsystems.getRot(), subsystems.getIntake()),
                new WaitCommand(30)
        );
    }


}
