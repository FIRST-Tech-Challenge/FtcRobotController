package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.ActionExecutor;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class ExecuteSequenceAction implements Action {

    ActionSequence sequence;
    ActionExecutor executor;
    int times = 1;
    int times_looped = 0;

    public ExecuteSequenceAction(ActionSequence sequence) {
        this.sequence = sequence;
    }
    public ExecuteSequenceAction(ActionSequence sequence, int times) {
        this.sequence = sequence;
        this.times = times;
    }

    @Override
    public void init(RobotHardware hardware) {
        this.executor = new ActionExecutor(hardware, sequence);
    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        if (times_looped == times) {
            return true;
        } else {
            if (this.executor.loop()) {
                times_looped++;
                executor.init();
            }
        }
        return false;
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
