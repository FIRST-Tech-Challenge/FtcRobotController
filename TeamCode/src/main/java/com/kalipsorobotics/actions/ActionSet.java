package com.kalipsorobotics.actions;

import android.util.Log;

import java.util.ArrayList;

public class ActionSet extends Action {

    private ArrayList<Action> actions;
    public ActionSet(Action dependentAction) {
        actions = new ArrayList<Action>();
        this.dependentAction = dependentAction;
    }

    public ActionSet() {
        actions = new ArrayList<Action>();
        this.dependentAction = new DoneStateAction();
    }

    public void scheduleSequential(Action action) {
        if (actions.isEmpty()) {
            action.setDependentAction(new DoneStateAction());
            actions.add(action);
            Log.d("parallelaction", "scheduled empty sequential");
        } else {
            Action recentAction = actions.get(actions.size()-1);
            action.setDependentAction(recentAction);
            actions.add(action);
            Log.d("parallelaction", "scheduled sequential");
        }
    }

    public void scheduleParallel(Action action) {
        if (actions.isEmpty()) {
            action.setDependentAction(new DoneStateAction());
            actions.add(action);
            Log.d("parallelaction", "scheduled empty parallel");
        } else {
            Action recentAction = actions.get(actions.size()-1);
            action.setDependentAction(recentAction.getDependentAction());
            actions.add(action);
            Log.d("parallelaction", "scheduled parallel");
        }
    }

    @Override
    public boolean checkDoneCondition() {
        for(int i = 0; i < actions.size(); i++) {
            if (!actions.get(i).getIsDone()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void update() {
        for(int i = 0; i < actions.size(); i++) {
            actions.get(i).updateCheckDone();
            Log.d("parallelaction", "updating action #" + i);
        }
    }

}
