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
    private final Telemetry telemetry;
    /**
     * This command deals with the wrist in teleop.
     *
     * @param wristSubParam The wrist sub to be imported
     */

    public MoveWristLeft(WristSub wristSubParam, Telemetry telemetry) {
        this.wristSub = wristSubParam;
        this.telemetry = telemetry;
        addRequirements(this.wristSub);
    }

    @Override
   public void execute(){
      double wristPosition = wristSub.getPosition();

//        if ((wristPosition != Constants.WristConstants.wristRight) && (wristPosition != Constants.WristConstants.wristCenter) && (wristPosition != Constants.WristConstants.wristLeft)) {
//            this.wristSub.setPosition(Constants.WristConstants.wristCenter);
//        }

        if (wristPosition == Constants.WristConstants.wristRight){
            wristSub.setPosition(Constants.WristConstants.wristCenter);
            telemetry.addData("Wrist moved to center", wristPosition);
        } else if (wristPosition == Constants.WristConstants.wristCenter){
           wristSub.setPosition(Constants.WristConstants.wristLeft);
            telemetry.addData("Wrist moved to left", wristPosition);


        }
    }
}