package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorManualCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier elevate_speed;
    private Telemetry telemetry;

    public final double kS = 230, kG = 260, kV = 1.0, kA = 0.0;

    ElevatorFeedforward feedforward;

    public double feedForwardValue = 0.0;

    public double max_ticks_per_second = 0;

    public ElevatorManualCommand(ElevatorSubsystem elevator, DoubleSupplier elevate_speed){
        this.elevator = elevator;
        this.elevate_speed = elevate_speed;

        max_ticks_per_second = elevator.leftMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND;

        feedforward = new ElevatorFeedforward(
                kS, kG, kV, kA
        );

        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setManual();
    }

    @Override
    public void execute() {
        feedForwardValue = feedforward.calculate(
                0.9 * elevate_speed.getAsDouble() * max_ticks_per_second
        );

        elevator.setPower((feedForwardValue / max_ticks_per_second) * elevator.MAX_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setAuto();
        elevator.run();
    }
}
