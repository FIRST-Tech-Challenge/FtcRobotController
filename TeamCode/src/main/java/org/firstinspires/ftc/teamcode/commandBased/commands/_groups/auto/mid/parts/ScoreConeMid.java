package org.firstinspires.ftc.teamcode.commandBased.commands._groups.auto.mid.parts;

import static org.firstinspires.ftc.teamcode.commandBased.Constants.ARM_ANGLE_BACK;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.ELE_MID;
import static org.firstinspires.ftc.teamcode.commandBased.Constants.ROTATOR_BACK;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.tele.ScoreConeStack;
import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class ScoreConeMid extends SequentialCommandGroup {

    public ScoreConeMid(
            Subsystems subsystems,
            TrajectorySequence traj
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
                                new FollowTrajectorySequenceAsync(subsystems.rrDrive(), traj)
                        )
                ),
                new ScoreConeStack(subsystems.getArm(), subsystems.getRot(), subsystems.getIntake())
        );
    }
}
