package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.function.DoubleSupplier;

public class MoveWristCommand extends CommandBase {

    private DoubleSupplier frontward, backward;
    private WristSubsystem subsystem;

    public MoveWristCommand(WristSubsystem subsystem, DoubleSupplier frontward, DoubleSupplier backward) {
        this.subsystem = subsystem;
        this.frontward = frontward;
        this.backward = backward;
        addRequirements(subsystem);
    }

    public MoveWristCommand(WristSubsystem subsystem, DoubleSupplier directionSupplier) {
        this(subsystem, directionSupplier, () -> 0);
    }

    @Override
    public void execute() {
        subsystem.moveWrist(frontward.getAsDouble(), backward.getAsDouble());
    }
}
