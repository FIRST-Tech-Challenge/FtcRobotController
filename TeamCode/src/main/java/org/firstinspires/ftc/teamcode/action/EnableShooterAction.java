package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class EnableShooterAction implements Action {

    boolean enabled;

    public EnableShooterAction(boolean enabled) {
        this.enabled = enabled;
    }


    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        if (hardware instanceof UltimateGoalHardware) {
            UltimateGoalHardware ultimateGoalHardware = (UltimateGoalHardware) hardware;
            ultimateGoalHardware.setShooterEnabled(this.enabled);
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
