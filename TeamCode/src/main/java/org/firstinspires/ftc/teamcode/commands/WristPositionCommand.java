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

    public static int getBoardTargetPosition() {
        // TODO: tune this
        return 52;
    }

    private int getTargetPos() {
        if (zero) {
            return 0; // Slightly off from straight down
        }
        return getBoardTargetPosition();
    }

    @Override
    public void execute() {
        int position = getTargetPos();
        subsystem.setWristPosition(position);
        for (int i = 0; i < 10; i++) {
            // Flood the console
            System.out.println(position + " : CURRENT IS " + subsystem.getPosition());
        }
    }

    @Override
    public boolean isFinished() {
        /* (((int) subsystem.getPosition()) == getTargetPos()) {
            return true;
        }*/
        System.out.println("DATA: "+subsystem.isBusy()+", "+subsystem.getPosition()+", "+getTargetPos());
        if (!subsystem.isBusy() || ((int) subsystem.getPosition()) == getTargetPos()) {
            subsystem.getWrist().setVelocity(0);
            subsystem.getWrist().setPower(0);
            return true;
        }
        if (defaultCommand.frontward.getAsDouble() != 0 || defaultCommand.backward.getAsDouble() != 0) {
            return true;
        }
        return false;
    }
}