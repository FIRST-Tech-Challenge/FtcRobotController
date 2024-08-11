package org.firstinspires.ftc.teamcode.NewStuff;

public abstract class Action {

    Action dependentAction;
    DoneStateAction doneStateAction;
    boolean isDone;

    boolean getIsDone() {
        return isDone;
    }
    void setDependentAction(Action newAction) {
        this.dependentAction = newAction;
    }
    Action getDependentAction() {
        return this.dependentAction;
    }
    void updateCheckDone() {
        if (isDone) { return; } //if i'm done never update
        if (!dependentAction.getIsDone()) { return; } //if dependent action is not done never update

        update();

        updateIsDone();
    }
    abstract boolean updateIsDone();
    abstract void update();
}
