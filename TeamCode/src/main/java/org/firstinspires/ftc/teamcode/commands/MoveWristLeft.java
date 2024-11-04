package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.WristSub;
/**

 * This command is dedicated to a command that controls the wrist for the Tele-op mode
 */

public class MoveWristLeft extends CommandBase {

    private final WristSub wristSub;
    private Telemetry telemetry;
    /**
     * This command deals with the wrist in teleop.
     *
     * @param wristSubParam The wrist sub to be imported
     */

    public MoveWristLeft(WristSub wristSubParam, Telemetry telemetryParam) {
        this.wristSub = wristSubParam;
        this.telemetry = telemetryParam;
        addRequirements(this.wristSub);
    }

    @Override
   public void execute(){
      telemetry.addData("Wrist Position",this.wristSub.getPosition());
      double wristPosition =this.wristSub.getPosition();
        if (wristPosition == Constants.WristConstants.wristRight){
            this.wristSub.setPosition(Constants.WristConstants.wristCenter);
            telemetry.addData("Wrist Position2",this.wristSub.getPosition());
        } else if(wristPosition == Constants.WristConstants.wristCenter){
           this.wristSub.setPosition(Constants.WristConstants.wristLeft);
            telemetry.addData("Wrist Position3",this.wristSub.getPosition());
        }
    }
}