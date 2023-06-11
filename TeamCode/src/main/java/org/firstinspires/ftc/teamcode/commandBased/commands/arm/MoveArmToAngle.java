package org.firstinspires.ftc.teamcode.commandBased.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;

public class MoveArmToAngle extends CommandBase {

    private final double angle;
    private final ArmSubsystem m_armSubsystem;

    public MoveArmToAngle(ArmSubsystem armSubsystem, double angle) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setArmAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
