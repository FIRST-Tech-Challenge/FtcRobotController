package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.CommandBase;

public class ElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final ElevatorSubsystem.Level targetLevel;

    public ElevatorCommand(ElevatorSubsystem elevator, ElevatorSubsystem.Level levelPicked){
        this.elevator = elevator;
        targetLevel = levelPicked;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setAuto();
        elevator.setLevel(targetLevel);
    }

    @Override
    public void execute() {
        elevator.run();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return elevator.atTargetLevel();
    }
}
