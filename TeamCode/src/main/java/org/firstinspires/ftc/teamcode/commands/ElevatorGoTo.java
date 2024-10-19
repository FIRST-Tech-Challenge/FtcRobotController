package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorGoTo extends CommandBase {
    private Elevator elevator;
    private double target;

    public ElevatorGoTo(Elevator elevator, double target) {
        this.elevator = elevator;
        this.target = target;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        this.elevator.setTarget(target);
    }

    @Override
    public void execute() {
       elevator.goToPos();
    }

    @Override
    public boolean isFinished() {
        return elevator.atTarget();
    }
}
