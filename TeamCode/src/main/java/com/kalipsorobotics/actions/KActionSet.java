package com.kalipsorobotics.actions;

import android.util.Log;

import java.util.ArrayList;

public class KActionSet extends Action{

    ArrayList<Action> actions = new ArrayList<>();

    public void addAction(Action... actions) {
        for (Action a : actions ) {
            this.actions.add(a);
        }
    }

    public void update() {
        if (isDone) {
            return;
        }
        for (Action a : actions) {
            if (a.dependantActionsDone()) {
                Log.d("action set log", "executing " + a.toString());
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
        super.printWithDependantActions();
        for (Action a : actions) {
            a.printWithDependantActions();
        }
    }
}
