package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.ActionExecutor;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class ExecuteSequenceAction implements Action {

    ActionSequence sequence;
    ActionExecutor executor;

    public ExecuteSequenceAction(ActionSequence sequence) {
        this.sequence = sequence;
    }

    @Override
    public void init(RobotHardware hardware) {
        this.executor = new ActionExecutor(hardware, sequence);
    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        return this.executor.loop();
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }

    @Override
    public Object getActionResult() {
        return null;
    }
}
