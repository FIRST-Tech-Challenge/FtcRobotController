package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

public class CloseClaw extends CommandBase {

    private Claw claw;

    public CloseClaw(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute(){
        this.claw.close();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
