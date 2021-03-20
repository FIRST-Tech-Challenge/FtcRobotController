package org.firstinspires.ftc.teamcode.commands.wobble;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class WobbleRaiseCommand extends WaitCommand {
    public WobbleSubsystem subsystem;

    public WobbleRaiseCommand(WobbleSubsystem w){
        //COOLDOWN
        super(1);
        addRequirements(w);
        subsystem = w;
    }

    @Override
    public void execute() {
        subsystem.setArmPosition(WobbleSubsystem.ArmPosition.RAISED);
    }

}
