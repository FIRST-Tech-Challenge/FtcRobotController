package com.kalipsorobotics.actions;

import android.util.Log;

public abstract class Action {

    protected Action dependentAction;
    boolean isDone = false;
    protected boolean hasStarted = false;

    public boolean getIsDone() {
        return isDone;
    }
    public boolean getHasStarted() {
        return hasStarted;
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

    private boolean updateIsDone() {
        isDone = checkDoneCondition();
        return isDone;
    }

    //what condition the action needs to fulfill in order to be done
    public abstract boolean checkDoneCondition();
    //motor power, etc
    public void update() {
        if (isDone) {
            Log.d("in super", "action done");
            return;
        }
    }
}
