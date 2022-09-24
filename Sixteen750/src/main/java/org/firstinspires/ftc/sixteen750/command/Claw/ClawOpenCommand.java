package org.firstinspires.ftc.sixteen750.command.Claw;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.sixteen750.subsystem.ClawSubsystem;

public class ClawOpenCommand implements Command {
    private ClawSubsystem subsystem;

    public ClawOpenCommand(ClawSubsystem s) {
        this.subsystem = s;
        addRequirements(this.subsystem); //Keeps robot from breaking
    }

    @Override
    public void execute() {
        this.subsystem.open();

    }
}
