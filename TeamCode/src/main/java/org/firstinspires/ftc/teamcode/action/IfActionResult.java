package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class IfActionResult implements Action {

    Action compareAction;
    Object other;
    Action trueAction;
    Action falseAction;
    private Action actionToExecute;

    private int actionNumber = 1;

    public IfActionResult(Action action, Object other, Action trueAction, Action falseAction) {
        this.other = other;
        this.compareAction = action;
        this.trueAction = trueAction;
        this.falseAction = falseAction;
    }

    public void init(RobotHardware hardware) {
        if (compareAction.getActionResult().equals(other)) {
            actionToExecute = trueAction;
        } else {
            actionToExecute = falseAction;
        }

        if (actionToExecute != null) {
            actionToExecute.init(hardware);
        }
    }

    public boolean doAction(RobotHardware hardware) {
        if (actionToExecute == null) {
            return true;
        }

        hardware.telemetry.addData("IF Result", compareAction.getActionResult().toString());
        hardware.telemetry.addData("IF Executing", actionToExecute.getClass().getSimpleName());

        return actionToExecute.doAction(hardware);
    }

    @Override
    public Object getActionResult() {
        return null;
    }

    @Override
    public Double progress()
    {
        if (actionToExecute != null) {
            return actionToExecute.progress();
        }
        return null;
    }

    @Override
    public String progressString() {
        if (actionToExecute != null) {
            return actionToExecute.progressString();
        }
        return null;
    }
}
