package org.firstinspires.ftc.teamcode.NewStuff.actions;

public class DoneStateAction extends Action {

    public DoneStateAction() {
        isDone = true;
    }

    @Override
    protected boolean checkDoneCondition() {
        return true;
    }

    @Override
    protected void update() {
        return;
    }
}
