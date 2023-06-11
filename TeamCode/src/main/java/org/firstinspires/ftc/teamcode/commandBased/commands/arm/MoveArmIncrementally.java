package org.firstinspires.ftc.teamcode.commandBased.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;

public class MoveArmIncrementally extends CommandBase {

    private double amount;
    private ArmSubsystem m_armSubsystem;

    public MoveArmIncrementally(ArmSubsystem armSubsystem, double amount) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.amount = amount;
    }

    @Override
    public void initialize() {
        //m_armSubsystem.setArmAngle(amount);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
