package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorExtendCommand extends CommandBase {
    private Elevator elevator;

    public ElevatorExtendCommand(Elevator elevator) {
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
