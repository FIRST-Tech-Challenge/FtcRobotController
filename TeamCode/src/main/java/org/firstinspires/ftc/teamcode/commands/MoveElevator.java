package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class MoveElevator extends CommandBase {

    private ElevatorSubsystem m_elevatorSubsystem;
    private double target;

    public MoveElevator(ElevatorSubsystem subsystem, double target) {
        this.m_elevatorSubsystem = subsystem;
        addRequirements(m_elevatorSubsystem);
        this.target = target;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setHeight(target);
    }

//    public boolean isFinished() {
//        return true;
//    }
}
