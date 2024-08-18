package org.firstinspires.ftc.teamcode.NewStuff;

public class DoneStateAction extends Action {

    DoneStateAction() {
        isDone = true;
    }

    @Override
    boolean checkDoneCondition() {
        return true;
    }

    @Override
    void update() {
        return;
    }
}
