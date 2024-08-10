package org.firstinspires.ftc.teamcode.NewStuff;

public class DoneStateAction extends Action {
    @Override
    boolean updateIsDone() {
        return true;
    }

    @Override
    boolean getIsDone() {
        return true;
    }

    @Override
    void setDependentAction(Action newAction) {
        return;
    }

    @Override
    Action getDependentAction() {
        return this;
    }

    @Override
    void update() {
        return;
    }
}
