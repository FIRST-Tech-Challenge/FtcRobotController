package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class ExecuteSequenceAction implements Action {

    ActionSequence sequence;

    public ExecuteSequenceAction(ActionSequence sequence) {
        this.sequence = sequence;
    }

    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
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
}
