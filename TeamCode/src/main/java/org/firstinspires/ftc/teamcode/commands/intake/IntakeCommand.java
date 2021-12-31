package org.firstinspires.ftc.teamcode.commands.intake;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final DoubleSupplier m_supplier;

    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier supplier) {
        m_intake = intake;
        m_supplier = supplier;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.intake(m_supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
