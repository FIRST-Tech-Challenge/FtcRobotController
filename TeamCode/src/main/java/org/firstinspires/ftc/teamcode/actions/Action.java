package org.firstinspires.ftc.teamcode.actions;

public abstract class Action {

    public Action dependentAction;
    boolean isDone = false;
    public boolean hasStarted = false;

    public boolean getIsDone() {
        return isDone;
    }
    public void setDependentAction(Action newAction) {
        this.dependentAction = newAction;
    }
    Action getDependentAction() {
        return this.dependentAction;
    }

    //updates the action
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

    //what condition the action needs to fulfill in order to be done
    public abstract boolean checkDoneCondition();
    //motor power, etc
    public abstract void update();
}
