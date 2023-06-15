package org.firstinspires.ftc.teamcode.commandBased.commands.rotator;

import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class SetRotatorRange extends TriggerCommandBase {

    private final PwmControl.PwmRange range;
    private final RotatorSubsystem m_rotatorSubsystem;

    public SetRotatorRange(RotatorSubsystem rotatorSubsystem, PwmControl.PwmRange range) {
        m_rotatorSubsystem = rotatorSubsystem;
        addRequirements(m_rotatorSubsystem);
        this.range = range;
    }

    @Override
    public void initialize() {
        m_rotatorSubsystem.setPWMRange(range);
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
