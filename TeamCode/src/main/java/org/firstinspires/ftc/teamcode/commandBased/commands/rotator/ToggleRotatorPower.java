package org.firstinspires.ftc.teamcode.commandBased.commands.rotator;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class ToggleRotatorPower extends TriggerCommandBase {

    private final boolean enabled;
    private final RotatorSubsystem m_rotatorSubsystem;

    public ToggleRotatorPower(RotatorSubsystem rotatorSubsystem, boolean enabled) {
        m_rotatorSubsystem = rotatorSubsystem;
        addRequirements(rotatorSubsystem);
        this.enabled = enabled;
    }

    @Override
    public void initialize() {
        if (enabled) {
            m_rotatorSubsystem.enable();
        } else {
            m_rotatorSubsystem.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public boolean isTriggered() {
        return true;
    }
}
