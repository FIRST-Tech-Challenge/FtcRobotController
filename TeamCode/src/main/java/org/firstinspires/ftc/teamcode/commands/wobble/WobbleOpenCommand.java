package org.firstinspires.ftc.teamcode.commands.wobble;

import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class WobbleOpenCommand extends WaitCommand {
    public WobbleSubsystem subsystem;
    public WobbleOpenCommand(WobbleSubsystem s){
        //COOLDOWN
        super(1);
        subsystem = s;
        this.addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setClawPosition(WobbleSubsystem.ClawPosition.OPEN);
    }
}

