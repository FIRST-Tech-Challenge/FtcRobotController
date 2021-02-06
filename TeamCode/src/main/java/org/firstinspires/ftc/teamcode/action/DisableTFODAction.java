package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class DisableTFODAction implements Action {

    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        if (hardware.tfod != null) {
            hardware.tfod.deactivate();
        }
        return true;
    }

    @Override
    public Object getActionResult() {
        return null;
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }
}
