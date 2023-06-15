package org.firstinspires.ftc.teamcode.commandBased.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.classes.command.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;

public class MoveElevatorToPosition extends TriggerCommandBase {

    private final double target;
    private final ElevatorSubsystem m_elevatorSubsystem;

    public MoveElevatorToPosition(ElevatorSubsystem elevatorSubsystem, double target) {
        m_elevatorSubsystem = elevatorSubsystem;
        addRequirements(m_elevatorSubsystem);
        this.target = target;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setProfileTarget(target);
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.isFinished();
    }

    @Override
    public boolean isTriggered() {
        return false;
    }
}
