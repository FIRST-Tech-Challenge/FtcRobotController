package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.WristSub;

/**
 * This command is dedicated to a command that controls the wrist for the Tele-op mode
 */

public class MoveWristRight extends CommandBase {

    private final WristSub wristSub;

    /**
     * This command deals with the wrist in teleop.
     *
     * @param wristSubParam The wrist sub to be imported
     */

    public MoveWristRight(WristSub wristSubParam){
        this.wristSub = wristSubParam;
        addRequirements(this.wristSub);
    }

    @Override
   public void execute(){
        //telemetry.addData("Wrist Position R",this.wristSub.getPosition());
        double wristPosition = this.wristSub.getPosition();
        if (wristPosition == Constants.WristConstants.wristLeft){
            this.wristSub.setPosition(Constants.WristConstants.wristCenter);
        } else if(wristPosition == Constants.WristConstants.wristCenter){
            this.wristSub.setPosition(Constants.WristConstants.wristRight);
        }
    }
}