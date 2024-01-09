package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorManualCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier elevate_speed;

    public ElevatorManualCommand(ElevatorSubsystem elevator, DoubleSupplier elevate_speed){
        this.elevator = elevator;
        this.elevate_speed = elevate_speed;

        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setManual();
    }

    @Override
    public void execute() {
        elevator.setPower(elevate_speed.getAsDouble() * elevator.MAX_SPEED);
        elevator.run();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setAuto();
        elevator.run();
    }
}
