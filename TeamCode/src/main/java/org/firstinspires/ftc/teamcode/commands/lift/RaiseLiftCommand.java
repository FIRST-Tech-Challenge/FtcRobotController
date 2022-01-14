package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import static org.commandftc.RobotUniversal.opMode;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseLiftCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem;
    private final DoubleSupplier m_supplier;

    public RaiseLiftCommand(LiftSubsystem liftSubsystem, DoubleSupplier supplier) {
        m_liftSubsystem = liftSubsystem;
        m_supplier = supplier;

        addRequirements(m_liftSubsystem);
    }

    @Override
    public void execute() {
        m_liftSubsystem.setPower(m_supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_liftSubsystem.stop();
    }
}
