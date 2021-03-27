package org.firstinspires.ftc.teamcode.commands.intake;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends Command {
    public IntakeSubsystem subsystem;
    public IntakeStopCommand(IntakeSubsystem s) {
        subsystem=s;
        //this.addRequirements(subsystem);

    }
    @Override
    public void execute() {
        subsystem.stop();
    }

}
