package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class StopDriveAction implements Action {


    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        hardware.omniDrive.stopDrive();
        return true;
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
