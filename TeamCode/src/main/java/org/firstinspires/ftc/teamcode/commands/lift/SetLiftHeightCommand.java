package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLiftHeightCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem;
    private final double m_height;
    private final double m_power;

    public SetLiftHeightCommand(LiftSubsystem liftSubsystem, double height, double power) {
        m_liftSubsystem = liftSubsystem;
        m_height = height;
        m_power = power;

        addRequirements(m_liftSubsystem);
    }

    @Override
    public void initialize() {
        m_liftSubsystem.setPower(m_power);
        m_liftSubsystem.setHeight(m_height);
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
