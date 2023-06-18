package org.firstinspires.ftc.teamcode.commandBased.commands.rotator;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class MoveRotatorToPosition extends TriggerCommandBase {

    private final double pos;
    private final RotatorSubsystem m_rotatorSubsystem;

    public MoveRotatorToPosition(RotatorSubsystem rotatorSubsystem, double pos) {
        m_rotatorSubsystem = rotatorSubsystem;
        addRequirements(m_rotatorSubsystem);
        this.pos = pos;
    }

    @Override
    public void initialize() {
        m_rotatorSubsystem.setPosition(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean isTriggered() {
        return false;
    }
}
