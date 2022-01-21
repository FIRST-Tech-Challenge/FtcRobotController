package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectorySequenceCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final TrajectorySequence m_trajectorySequence;

    public FollowTrajectorySequenceCommand(DriveTrainSubsystem driveTrain, TrajectorySequence trajectorySequence) {
        m_driveTrain = driveTrain;
        m_trajectorySequence = trajectorySequence;
    }

    @Override
    public void initialize() {
        m_driveTrain.followTrajectorySequenceAsync(m_trajectorySequence);
    }

    @Override
    public boolean isFinished() {
        return m_driveTrain.isBusy();
    }
}
