package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

public class OpenClaw extends CommandBase {

    private Claw claw;

    public OpenClaw(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute(){
        this.claw.open();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
