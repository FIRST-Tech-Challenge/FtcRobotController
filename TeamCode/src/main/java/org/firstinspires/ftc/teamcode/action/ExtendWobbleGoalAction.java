package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class ExtendWobbleGoalAction implements Action {

    boolean extend;

    public ExtendWobbleGoalAction(boolean extend) {
        this.extend = extend;
    }


    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        if (hardware instanceof UltimateGoalHardware) {
            UltimateGoalHardware ultimateGoalHardware = (UltimateGoalHardware) hardware;
            ultimateGoalHardware.extendWobbleGoal = extend;
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
