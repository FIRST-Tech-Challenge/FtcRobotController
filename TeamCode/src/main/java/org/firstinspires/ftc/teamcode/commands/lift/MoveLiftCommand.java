package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveLiftCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem;
    private final double m_power;
    private final double m_change;

    public MoveLiftCommand(LiftSubsystem liftSubsystem, double power, double change) {
        m_liftSubsystem = liftSubsystem;
        m_power = power;
        m_change = change;

        addRequirements(m_liftSubsystem);
    }

    @Override
    public void initialize() {
        m_liftSubsystem.setPower(m_power);
        m_liftSubsystem.setHeight(m_liftSubsystem.getHeight() + m_change);
    }

    @Override
    public boolean isFinished() {
        return !m_liftSubsystem.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        m_liftSubsystem.stop();
    }
}
