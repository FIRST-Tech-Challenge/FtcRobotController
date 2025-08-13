package org.firstinspires.ftc.teamcode.aim.action;

import java.util.ArrayList;
import java.util.List;

public class ParallelAction implements Action {
    private List<Action> actions;

    // Constructor
    public ParallelAction() {
        this.actions = new ArrayList<>();
    }

    // Method to add an AutoAction to the list
    public void addAction(Action action) {
        actions.add(action);
    }

    // The run method which calls 'run()' on each item in the list
    public boolean run() {
        for (Action action : actions) {
            boolean done = action.run();
            if (!done) {
                return false;
            }
        }
        return true;
    }
}
