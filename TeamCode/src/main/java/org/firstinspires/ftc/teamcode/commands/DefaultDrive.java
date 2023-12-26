package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final DoubleSupplier m_axial, m_lateral, m_yaw;
    private final DriveSubsystem m_drive;

    MecanumDrive drive;

    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier axial,
                        DoubleSupplier lateral, DoubleSupplier yaw) {
        m_drive = subsystem;
        m_axial = axial;
        m_lateral = lateral;
        m_yaw = yaw;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_drive.moveRobot(
                m_axial.getAsDouble(),
                m_lateral.getAsDouble(),
                m_yaw.getAsDouble()
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
