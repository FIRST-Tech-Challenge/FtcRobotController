package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import java.util.ArrayList;

public class ActionSet extends Action {

    private ArrayList<Action> actions;
    Action dependentAction;
    DoneStateAction doneStateAction = new DoneStateAction();

    boolean isDone = false;

    public ActionSet(Action dependentAction) {
        actions = new ArrayList<Action>();
        this.dependentAction = dependentAction;
    }

    public ActionSet() {
        actions = new ArrayList<Action>();
        this.dependentAction = doneStateAction;
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
    boolean updateIsDone() {
        for(int i = 0; i < actions.size(); i++) {
            if (!actions.get(i).getIsDone()) {
                isDone = false;
                return isDone;
            }
        }
        isDone = true;
        return isDone;
    }

    @Override
    boolean getIsDone() {
        return isDone;
    }

    @Override
    void setDependentAction(Action newAction) {
        this.dependentAction = newAction;
    }

    @Override
    Action getDependentAction() {
        return this.dependentAction;
    }

    public void update() {
        if (isDone) { return; } //if i'm done never update
        if (!dependentAction.getIsDone()) { return; } //if dependent action is not done never update

        Log.d("parallelaction", "should run");
        for(int i = 0; i < actions.size(); i++) {
            actions.get(i).update();
            Log.d("parallelaction", "updating action #" + i);
        }

        updateIsDone();
    }
}
