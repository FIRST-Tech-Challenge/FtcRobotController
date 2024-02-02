package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.function.DoubleSupplier;

public class ZeroWristCommand extends CommandBase {

    WristSubsystem subsystem;
    DoubleSupplier frontwardSupplier, backwardSupplier;

    public ZeroWristCommand(WristSubsystem subsystem, DoubleSupplier frontwardSupplier, DoubleSupplier backwardSupplier) {
        super();
        this.subsystem = subsystem;
        this.frontwardSupplier = frontwardSupplier;
        this.backwardSupplier = backwardSupplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        // Move forward
        double sign = Math.signum(subsystem.getServoPosition());
        if (Math.abs(subsystem.getServoPosition()) > 0.05) {
            subsystem.moveWrist(sign, 0);
        }
    }

    @Override
    public boolean isFinished() {
        boolean buttonPressed = frontwardSupplier.getAsDouble() != 0 || backwardSupplier.getAsDouble() != 0;
        if (buttonPressed) {
            return true;
        }
        if (Math.abs(subsystem.getServoPosition()) > 0.05) { // magic number
            // It has reached its destination
            return true;
        }
        return false;
    }
}