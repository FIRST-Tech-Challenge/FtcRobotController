package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ToggleIntakeCommand extends CommandBase {

    IntakeSubsystem intakeSubsystem;

    public ToggleIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        super.initialize();
        intakeSubsystem.toggleIntakeState();
    }
}
