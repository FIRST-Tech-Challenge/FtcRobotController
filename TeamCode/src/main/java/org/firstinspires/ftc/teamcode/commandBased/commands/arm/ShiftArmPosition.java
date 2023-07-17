package org.firstinspires.ftc.teamcode.commandBased.commands.arm;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;

public class ShiftArmPosition extends TriggerCommandBase {

    private final ArmSubsystem m_armSubsystem;
    private final double shift;

    public ShiftArmPosition(ArmSubsystem armSubsystem, double shift) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.shift = shift;
    }

    @Override
    public void initialize() {
        m_armSubsystem.addOffset(shift);
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
