package org.firstinspires.ftc.teamcode.commandBased.commands.rotator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class MoveRotatorToAngle extends CommandBase {

    private final double angle;
    private final RotatorSubsystem m_rotatorSubsystem;

    public MoveRotatorToAngle(RotatorSubsystem rotatorSubsystem, double angle) {
        m_rotatorSubsystem = rotatorSubsystem;
        addRequirements(m_rotatorSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        m_rotatorSubsystem.setRotation(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
