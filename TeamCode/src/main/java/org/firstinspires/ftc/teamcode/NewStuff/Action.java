package org.firstinspires.ftc.teamcode.NewStuff;

import com.kalipsorobotics.fresh.MarkerDetector;

public abstract class Action {

    Action dependentAction;
    DoneStateAction doneStateAction = new DoneStateAction();
    boolean isDone = false;

    boolean getIsDone() {
        return isDone;
    }
    void setDependentAction(Action newAction) {
        this.dependentAction = newAction;
    }
    Action getDependentAction() {
        return this.dependentAction;
    }
    public void updateCheckDone() {
        if (isDone) { return; } //if i'm done never update
        if (!dependentAction.getIsDone()) { return; } //if dependent action is not done never update

        update();

        updateIsDone();
    }

    boolean updateIsDone() {
        isDone = checkDoneCondition();
        return isDone;
    }

    abstract boolean checkDoneCondition();
    abstract void update();
}
