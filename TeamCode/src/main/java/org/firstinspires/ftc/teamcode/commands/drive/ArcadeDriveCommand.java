package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.drive.ArcadeDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDriveCommand extends CommandBase {
    private final ArcadeDrive m_drive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_spinSupplier;

    public ArcadeDriveCommand(ArcadeDrive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier spinSupplier) {
        m_drive = drive;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_spinSupplier = spinSupplier;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.arcadeDrive(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble(), m_spinSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
