package org.firstinspires.ftc.teamcode.commands.index;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;

public class IndexSendToShooterCommand extends Command {
    public IndexSubsystem subsystem;
    public IndexSendToShooterCommand(IndexSubsystem s){
        subsystem = s;
        this.addRequirements(subsystem);
    }
    @Override
    public void execute() {
        subsystem.sendToShooter();
    }
}
