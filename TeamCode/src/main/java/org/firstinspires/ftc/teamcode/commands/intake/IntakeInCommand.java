package org.firstinspires.ftc.teamcode.commands.intake;

import com.technototes.library.command.Command;
import com.technototes.library.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

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
