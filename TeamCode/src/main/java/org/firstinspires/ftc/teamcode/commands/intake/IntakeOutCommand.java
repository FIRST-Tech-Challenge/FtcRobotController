package org.firstinspires.ftc.teamcode.commands.intake;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends Command {
    public IntakeSubsystem subsystem;
    public IntakeOutCommand(IntakeSubsystem s) {
        subsystem = s;
        this.addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.extake();
    }
}
