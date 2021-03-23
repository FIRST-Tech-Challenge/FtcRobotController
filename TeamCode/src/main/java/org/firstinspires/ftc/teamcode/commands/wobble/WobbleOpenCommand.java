package org.firstinspires.ftc.teamcode.commands.wobble;

import com.technototes.library.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class WobbleOpenCommand extends WaitCommand {
    public WobbleSubsystem subsystem;
    public WobbleOpenCommand(WobbleSubsystem s){
        //COOLDOWN
        super(0.25);
        subsystem = s;
        this.addRequirements(subsystem);
    }

    @Override
    public void init() {
        subsystem.setClawPosition(WobbleSubsystem.ClawPosition.OPEN);
    }
}

