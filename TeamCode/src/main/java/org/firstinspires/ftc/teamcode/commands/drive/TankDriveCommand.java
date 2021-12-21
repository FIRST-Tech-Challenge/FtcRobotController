package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.drive.TankDrive;

import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {
    private final TankDrive m_drive;
    private final DoubleSupplier m_leftSupplier;
    private final DoubleSupplier m_rightSupplier;

    public TankDriveCommand(TankDrive drive, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
        m_drive = drive;
        m_leftSupplier = leftSupplier;
        m_rightSupplier = rightSupplier;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.tankDrive(m_leftSupplier.getAsDouble(), m_rightSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
