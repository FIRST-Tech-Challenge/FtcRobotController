package org.firstinspires.ftc.teamcode.robotcode;

import com.technototes.library.command.Command;

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
