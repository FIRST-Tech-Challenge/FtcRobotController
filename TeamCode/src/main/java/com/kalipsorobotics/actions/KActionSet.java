package com.kalipsorobotics.actions;

import android.util.Log;

import java.util.ArrayList;
import java.util.Collections;

public class KActionSet extends Action {

    ArrayList<Action> actions = new ArrayList<>();

    public void addAction(Action... actions) {
        Collections.addAll(this.actions, actions);
    }

    @Override
    public void update() {
        for (Action a : actions) {
            if (a != null && a.dependentActionsDone()) {
                Log.d("action set log", "executing " + a);
                a.updateCheckDone();
            }
        }
    }

    @Override
    public boolean checkDoneCondition() {
        for (Action a : actions) {
            if (a != null && !a.getIsDone()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public boolean updateCheckDone(){
        if (isDone) {
            Log.d("action set log", "done for " + name);
            return true;
        }

        update();
        return updateIsDone();
    }

    @Override
    protected boolean updateIsDone() {
        if(isDone) {
            return isDone;
        }
        isDone = checkDoneCondition();
        return isDone;
    }

    public void printWithDependentActions() {
        Log.d("action dependencies", "Start Action Set");
        super.printWithDependentActions();

        for (Action a : actions) {
            if (a != null) {
                a.printWithDependentActions();
            }
        }
        Log.d("action dependencies", "End Action Set");

    }

    public void clear() {
        actions.clear();
    }

}
