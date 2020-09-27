package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class WaitForeverAction implements Action {


    /**
     * Function called when the action is first executed
     *
     * @param hardware
     */
    @Override
    public void init(RobotHardware hardware) {

    }

    /**
     * Function that is called for every iteration of the OpMode controllerLoop
     *
     * @param hardware
     * @return Return true when the action is complete.
     */
    @Override
    public boolean doAction(RobotHardware hardware) {
        return false;
    }

    @Override
    public Double progress() {
        return 0.0;
    }

    @Override
    public String progressString() {
        return "Will never complete!";
    }
}
