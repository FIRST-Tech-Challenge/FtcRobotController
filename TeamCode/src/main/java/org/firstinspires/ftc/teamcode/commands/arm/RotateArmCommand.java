package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateArmCommand extends CommandBase {
    private final ArmSubsystem m_ArmSubsystem;
    private final double m_power;
    private final double[] m_states;
    private int m_current_state;

    public RotateArmCommand(ArmSubsystem armSubsystem, double power, int defaultIndex, double[] states) {
        m_ArmSubsystem = armSubsystem;
        m_power = power;
        m_states = states;
        m_current_state = defaultIndex;

        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.setPower(m_power);
        updateState();
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setPower(0);
    }

    public void incState() {
        m_current_state = Math.min(m_current_state+1, m_states.length-1);
        updateState();
    }

    public void decState() {
        m_current_state = Math.max(m_current_state-1, 0);
        updateState();
    }

    public void setState(int state) {
        m_current_state = state;
        updateState();
    }

    public int getState() {
        return m_current_state;
    }

    private void updateState() {
        m_ArmSubsystem.setAngle(m_states[m_current_state]);
    }
}
