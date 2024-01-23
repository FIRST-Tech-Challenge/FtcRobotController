package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.turnOn();
    }
}
