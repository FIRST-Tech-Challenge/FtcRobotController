package org.firstinspires.ftc.teamcode.aim.action;

import java.util.ArrayList;
import java.util.List;

public class SeqAction extends Action {
    // A list to hold AutoAction items
    private List<Action> actions;

    // Constructor
    public SeqAction(String name) {
        super(name);
        this.actions = new ArrayList<>();
    }

    // Method to add an AutoAction to the list
    public void addAction(Action action) {
        actions.add(action);
    }

    @Override
    // The run method which calls 'run()' on each item in the list
    public boolean run() {
        if (actions.isEmpty()) {
            return true;
        }
        Action firstAction = actions.get(0);
        boolean done = firstAction.run();
        if (!done) {
            return false;
        }
        actions.remove(0);
        if (actions.isEmpty()) {
            return true;
        }
        return false;
    }
}
