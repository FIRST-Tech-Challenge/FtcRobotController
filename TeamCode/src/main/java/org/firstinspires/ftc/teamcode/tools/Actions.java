package org.firstinspires.ftc.teamcode.tools;

import java.util.ArrayList;

public class Actions {
    int currentActionIndex; // Index to keep track of the current action
    //ArrayList<Action> actions; // List of actions to be performed during the transition
    ArrayList<Action> actions;

    public Actions(ActionBuilder builder){
        this.actions = builder.getList();
        currentActionIndex = 0;
    }

    public void performAll (){
        while (!areActionsComplete()){
            // do nothing
        }
    }
    public boolean areActionsComplete() {
        // Iterate through all actions to see if they are complete
        for (; currentActionIndex < actions.size(); currentActionIndex++) {
            if (!(actions.get(currentActionIndex).evaluate())) {
                return false;
            }
        }
        currentActionIndex = 0; // Reset the action index
        return true; // All actions are complete
    }

    public void update(){
        areActionsComplete();
    }
}
