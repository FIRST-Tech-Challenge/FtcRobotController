package org.firstinspires.ftc.sixteen750.command.Claw;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.sixteen750.subsystem.ClawSubsystem;

public class ClawCloseCommand implements Command {
    private ClawSubsystem subsystem;
    public ClawCloseCommand(ClawSubsystem s) {
        subsystem = s;
        addRequirements(s);
    }

    @Override
    public void execute() {
        subsystem.close();

    }
}
