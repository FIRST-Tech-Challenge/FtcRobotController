package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final double m_heading;

    public TurnCommand(DriveTrainSubsystem driveTrain, double heading) {
        m_driveTrain = driveTrain;
        m_heading = heading;
    }

    @Override
    public void initialize() {
        m_driveTrain.turnAsync(m_heading);
    }

    @Override
    public boolean isFinished() {
        return m_driveTrain.isBusy();
    }
}
