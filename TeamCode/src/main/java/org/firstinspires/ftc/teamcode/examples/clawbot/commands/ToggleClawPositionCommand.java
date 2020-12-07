package org.firstinspires.ftc.teamcode.examples.clawbot.commands;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.examples.clawbot.subsystems.ClawSubsystem;

public class ToggleClawPositionCommand extends Command {
    public ClawSubsystem subsystem;
    public ToggleClawPositionCommand(ClawSubsystem s){
        addRequirements(s);
        subsystem = s;
    }
    @Override
    public void execute() {
        subsystem.setClawPosition(subsystem.position.invert());
        //subsystem.setClawPosition(subsystem.position == ClawSubsystem.ClawPosition.OPEN ? ClawSubsystem.ClawPosition.CLOSED : ClawSubsystem.ClawPosition.OPEN);
    }
}
