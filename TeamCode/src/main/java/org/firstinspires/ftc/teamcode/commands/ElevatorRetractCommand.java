package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorRetractCommand extends CommandBase {
    private Elevator elevator;

    public ElevatorRetractCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.extend();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.brake();
    }
}
