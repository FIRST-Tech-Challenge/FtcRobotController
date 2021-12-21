package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.drive.HorizontalDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveRightCommand extends CommandBase {
    private final HorizontalDrive m_drive;
    private final DoubleSupplier m_powerSupplier;
    private final double m_power;

    public DriveRightCommand(HorizontalDrive drive, double power) {
        m_drive = drive;
        m_power = power;
        m_powerSupplier = null;

        addRequirements(m_drive);
    }

    public DriveRightCommand(HorizontalDrive drive, DoubleSupplier powerSupplier) {
        m_drive = drive;
        m_power = 0;
        m_powerSupplier = powerSupplier;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        if (m_powerSupplier != null) m_drive.driveLeft(m_powerSupplier.getAsDouble());
        else                         m_drive.driveLeft(m_power);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
