package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDrive extends CommandBase {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotationSpeed;

    private final DoubleSupplier m_strafe;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param p_forward   The control input for driving forwards/backwards
     * @param p_rotationSpeed  The control input for turning
     */
    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier p_forward, DoubleSupplier p_rotationSpeed, DoubleSupplier p_strafe) {
        m_drive = subsystem;
        m_forward = p_forward;
        m_rotationSpeed = p_rotationSpeed;
        m_strafe = p_strafe;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_strafe.getAsDouble(), m_forward.getAsDouble(), m_rotationSpeed.getAsDouble());
    }

}