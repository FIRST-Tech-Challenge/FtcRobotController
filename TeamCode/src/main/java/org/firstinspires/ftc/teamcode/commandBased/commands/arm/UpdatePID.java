package org.firstinspires.ftc.teamcode.commandBased.commands.arm;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;

public class UpdatePID extends TriggerCommandBase {

    private final ArmSubsystem m_armSubsystem;

    public UpdatePID(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.createNewController();
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
