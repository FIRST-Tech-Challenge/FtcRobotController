package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

import java.util.List;

public class BulkExecuteAction implements Action {

    List<Action> actions;

    public BulkExecuteAction(List<Action> actions) {
        this.actions = actions;
    }

    @Override
    public void init(RobotHardware hardware) {
        for (Action action : actions) {
            action.init(hardware);
        }
    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        boolean done = true;
        String notDone = "";
        for (Action action : actions) {
            if (!action.doAction(hardware)) {
                done = false;
                notDone += action.getClass().getSimpleName() + ", ";
            }
        }
        hardware.telemetry.addData("not done", notDone);
        return done;
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
