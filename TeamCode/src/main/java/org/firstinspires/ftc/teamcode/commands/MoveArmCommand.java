package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.Objects;

public class MoveArmCommand extends CommandBase {

    ArmSubsystem subsystem;
    ArmSubsystem.Direction direction;

    /**
     *
     * @param subsystem The arm subsystem
     * @param direction The direction of the command. If null, the command will act as a halt command
     */
    public MoveArmCommand(ArmSubsystem subsystem, ArmSubsystem.Direction direction) {
        Objects.requireNonNull(subsystem);
        this.subsystem = subsystem;
        this.direction = direction;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(direction == null) {
            subsystem.haltArm();
        } else {
            subsystem.manualMoveArm(direction, 1f);
        }
    }
}
