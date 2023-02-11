package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.CommandBase;

public class ElevatorManualCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final int direction;
    private static final int step = 20;

    public ElevatorManualCommand(ElevatorSubsystem elevator, int direction){
        this.elevator = elevator;
        this.direction = direction;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setManual();
    }

    @Override
    public void execute() {
        elevator.setPower(0.4*direction);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
