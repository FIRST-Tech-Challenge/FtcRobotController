package org.firstinspires.ftc.teamcode.robotcode;

import com.technototes.library.command.Command;

public class IntakeInCommand extends Command {
    public IntakeSubsystem subsystem;
    public IntakeInCommand(IntakeSubsystem s) {
        subsystem=s;
        this.addRequirements(subsystem);

    }

    @Override
    public void execute() {
        subsystem.intake();
    }

}
