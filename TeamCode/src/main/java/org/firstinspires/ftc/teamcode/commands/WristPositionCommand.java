package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Sets the position of the wrist to be at either the board or the ground.
 * Zero is positioned on the ground.
 */
public class WristPositionCommand extends CommandBase {

    WristSubsystem subsystem;
    MoveWristCommand defaultCommand;
    boolean zero;

    public WristPositionCommand(WristSubsystem subsystem, boolean zero, MoveWristCommand defaultCommand) {
        super();
        this.subsystem = subsystem;
        this.zero = zero;
        this.defaultCommand = defaultCommand;
        addRequirements(subsystem);
    }

    private int getTargetPos() {
        if (zero) {
            return 0;
        }
        return 1;
    }

    @Override
    public void execute() {
        int position = getTargetPos();
        subsystem.setWristPosition(position);
    }

    @Override
    public boolean isFinished() {
        if (((int) subsystem.getPosition()) == getTargetPos()) {
            return true;
        }
        if (defaultCommand.frontward.getAsDouble() != 0 || defaultCommand.backward.getAsDouble() != 0) {
            return true;
        }
        return false;
    }
}