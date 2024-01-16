package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorManualCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier elevate_speed;
    private Telemetry telemetry;

    public ElevatorManualCommand(ElevatorSubsystem elevator, DoubleSupplier elevate_speed, Telemetry telemetry){
        this.elevator = elevator;
        this.elevate_speed = elevate_speed;
        this.telemetry = telemetry;

        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setManual();
        telemetry.addData("init", "");
    }

    @Override
    public void execute() {
        telemetry.addData("exec", "");
        elevator.setPower(elevate_speed.getAsDouble() * elevator.MAX_SPEED + 0.05);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setAuto();
        elevator.run();
        telemetry.addData("end", "");
    }
}
