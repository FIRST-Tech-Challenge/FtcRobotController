package org.firstinspires.ftc.teamcode.commandBased.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class RobotCentric extends CommandBase {

    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier turnSpeed;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public RobotCentric(DrivetrainSubsystem drivetrainSubsystem,
                        DoubleSupplier strafeSpeed,
                        DoubleSupplier forwardSpeed,
                        DoubleSupplier turnSpeed)
    {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.turnSpeed = turnSpeed;
    }

    @Override
    public void execute() {
        double turn = m_drivetrainSubsystem.getTurnAmount(turnSpeed.getAsDouble());
        m_drivetrainSubsystem.robotCentric(
                strafeSpeed.getAsDouble(),
                forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble()
        );
    }
}
