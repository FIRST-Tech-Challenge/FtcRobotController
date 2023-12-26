package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase {

    ArmSubsystem subsystem;
    ArmSubsystem.Direction direction;

    public MoveArmCommand(ArmSubsystem subsystem, ArmSubsystem.Direction direction) {
        this.subsystem = subsystem;
        this.direction = direction;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.manualMoveArm(direction, 1f);
    }
}
