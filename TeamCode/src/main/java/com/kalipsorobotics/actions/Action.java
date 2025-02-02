package com.kalipsorobotics.actions;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;

public abstract class Action {

    protected ArrayList<Action> dependentActions = new ArrayList<>();
    protected boolean hasStarted = false;
    protected boolean isDone = false;

    protected String name;

    protected Telemetry telemetry;
    public boolean getIsDone() {
        return isDone;
    }

    public void setIsDone(boolean isDone) {
        this.isDone = isDone;
    }

    public boolean getHasStarted() {
        return hasStarted;
    }

    public void setDependentActions(Action... actions) {
        if (dependentActions == null) {
            dependentActions = new ArrayList<>();
        }
        dependentActions.clear();
        Collections.addAll(dependentActions, actions);

    }

    public void setDependentActions(ArrayList<Action> actions) {
        if (dependentActions == null) {
            dependentActions = new ArrayList<>();
        }
        dependentActions.clear();
        for (Action a : actions) {
            dependentActions.add(a);
        }

    }

    ArrayList<Action> getDependentActions() {
        return this.dependentActions;
    }

    //updates the action
    public boolean updateCheckDone() {
        if (telemetry != null) {
            int value = isDone ? 3 : (hasStarted ? 2:1);
            telemetry.addData("action_"+this.name, value);
        }
        if (isDone) {
            return true;
        } //if done never update

        for (Action action : dependentActions) {
            if (action != null && !action.getIsDone()) {
                return false;
            } //if dependent action is not done never update
            //dont start if dependant action not finished
        }

        update();


        return updateIsDone();

    }

    public boolean dependentActionsDone() {
        for (Action a : dependentActions) {
            if (a != null && !a.getIsDone()) {
                return false;
            }
        }
        return true;
    }

    protected boolean updateIsDone() {
        isDone = checkDoneCondition();
        return isDone;
    }

    protected abstract boolean checkDoneCondition();

    //motor power, etc
    protected abstract void update();

    @Override
    public String toString() {
        return "Action {" +
                "name='" + name + '\'' + "status=" + isDone +
                '}';
    }


    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.addData("action_"+this.getName(), 0);
    }

    public void printWithDependentActions() {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(this.name);
        stringBuilder.append("--->");
        for (Action a : dependentActions) {
            if (a == null) {
                stringBuilder.append("null");
            } else {
                stringBuilder.append(a);
            }
        }
        Log.d("action dependencies",  stringBuilder.toString());
    }
}
