package org.firstinspires.ftc.teamcode.commandBased.commands.elevator;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;

public class UpdateElevatorPID extends TriggerCommandBase {

    private final ElevatorSubsystem m_elevatorSubsystem;

    public UpdateElevatorPID(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.createNewController();
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
