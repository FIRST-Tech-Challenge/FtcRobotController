package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subbys.ShooterSubby;

public class ShooterCom extends CommandBase {
    private ShooterSubby subby;
    public ShooterCom(ShooterSubby sub){
        subby = sub;
        addRequirements(subby);
    }

    @Override
    public void initialize(){
        subby.release();
    }
    public void end(boolean interrupted){
        subby.bringBack();
    }
}
