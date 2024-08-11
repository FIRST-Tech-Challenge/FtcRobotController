package org.firstinspires.ftc.teamcode.NewStuff;

public class DoneStateAction extends Action {
    @Override
    boolean updateIsDone() {
        return true;
    }

    @Override
    void update() {
        return;
    }

    boolean getIsDone() {
        return true;
    }

    void setDependentAction(Action newAction) {
        return;
    }

    Action getDependentAction() {
        return this;
    }

    void updateCheckDone() {
        return;
    }
}
