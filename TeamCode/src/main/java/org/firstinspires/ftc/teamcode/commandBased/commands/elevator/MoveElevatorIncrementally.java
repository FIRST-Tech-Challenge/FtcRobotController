package org.firstinspires.ftc.teamcode.commandBased.commands.elevator;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;

public class MoveElevatorIncrementally extends TriggerCommandBase {

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double amount;

    public MoveElevatorIncrementally(ElevatorSubsystem elevatorSubsystem, double amount) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
        this.amount = amount;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.moveProfileTarget(amount);
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
