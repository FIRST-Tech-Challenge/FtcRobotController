package org.firstinspires.ftc.teamcode.aim.action;

import java.util.ArrayList;
import java.util.List;
import java.util.Iterator;

public class ParallelAction extends Action {
    private List<Action> actions;

    // Constructor
    public ParallelAction(String name) {
        super(name);
        this.actions = new ArrayList<>();
    }

    // Method to add an AutoAction to the list
    public void addAction(Action action) {
        actions.add(action);
    }

    // The run method which calls 'run()' on each item in the list
    @Override
    public boolean run() {
        if (actions.isEmpty()) {
            return true;
        }
        for (Iterator<Action> it = actions.iterator(); it.hasNext();) {
            Action act = it.next();
            if (act.run()) {
                it.remove();
            }
        }
        if (actions.isEmpty()) {
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        String s = super.getName();
        for (Iterator<Action> it = actions.iterator(); it.hasNext();) {
            Action act = it.next();
            s += "\n  " + act.toString();
        }
        return s;
    }
}
