package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class HoldElevator extends CommandBase {

    private ElevatorSubsystem m_elevatorSubsystem;

    public HoldElevator(ElevatorSubsystem subsystem) {
        this.m_elevatorSubsystem = subsystem;
        addRequirements(m_elevatorSubsystem);
    }
}
