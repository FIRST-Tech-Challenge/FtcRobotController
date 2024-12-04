package com.kalipsorobotics.actions;

import android.util.Log;

import java.util.ArrayList;
import java.util.Collections;

public class KActionSet extends Action {

    ArrayList<Action> actions = new ArrayList<>();

    public void addAction(Action... actions) {
        Collections.addAll(this.actions, actions);
    }

    public void update() {
        if (isDone) {
            return;
        }
        for (Action a : actions) {
            if (a.dependantActionsDone()) {
                Log.d("action set log", "executing " + a);
                a.updateCheckDone();
            }
        }
    }

    @Override
    public boolean checkDoneCondition() {
        for (Action a : actions) {
            if (!a.checkDoneCondition()) {
                return false;
            }
        }
        isDone = true;
        return isDone;
    }
    public void printWithDependantActions() {
        Log.d("action dependancies", "Start Action Set");
        super.printWithDependantActions();

        for (Action a : actions) {
            a.printWithDependantActions();
        }
        Log.d("action dependancies", "End Action Set");

    }

    public void clear() {
        actions.clear();
    }



}
