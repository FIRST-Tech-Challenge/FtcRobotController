package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class EnableCollectorAction implements Action {

    boolean enabled;

    public EnableCollectorAction(boolean enabled) {
        this.enabled = enabled;
    }


    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        if (hardware instanceof UltimateGoalHardware) {
            UltimateGoalHardware ultimateGoalHardware = (UltimateGoalHardware) hardware;
            ultimateGoalHardware.collector.setPower(enabled ? 1 : 0);
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
