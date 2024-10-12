package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.function.DoubleSupplier;

public class ManualElevatorCommand extends CommandBase {
    private Elevator elevator;
    private DoubleSupplier joystickVal;
    private Telemetry telemetry;

    public ManualElevatorCommand(Elevator elevator, DoubleSupplier val, Telemetry tele) {
        this.elevator = elevator;
        this.joystickVal = val;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        this.elevator.manualControl(this.joystickVal.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
