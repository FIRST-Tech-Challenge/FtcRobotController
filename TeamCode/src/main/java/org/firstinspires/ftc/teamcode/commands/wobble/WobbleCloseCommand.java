package org.firstinspires.ftc.teamcode.commands.wobble;

import com.technototes.library.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class WobbleCloseCommand extends WaitCommand {
    public WobbleSubsystem subsystem;
    public WobbleCloseCommand(WobbleSubsystem s){
        //COOLDOWN
        super(1);
        subsystem = s;
        //this.addRequirements(subsystem);
    }

    @Override
    public void init() {
        subsystem.setClawPosition(WobbleSubsystem.ClawPosition.CLOSED);
    }
}

