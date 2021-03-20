package org.firstinspires.ftc.teamcode.commands.wobble;

import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class WobbleLowerCommand extends WaitCommand {
    public WobbleSubsystem subsystem;

    public WobbleLowerCommand(WobbleSubsystem w){
        //COOLDOWN
        super(1);
        addRequirements(w);
        subsystem = w;
    }

    @Override
    public void execute() {
        subsystem.setArmPosition(WobbleSubsystem.ArmPosition.LOWERED);
    }

}
