package org.firstinspires.ftc.teamcode;


// This is what an action is: a function with no input parameters which returns
// true/false when evaluate.
interface ActionFunction {
    Boolean evaluate();
}

public class Action {
    // Message is for debugging purpose, function is what is being evaluated here.
    Action(String msg, ActionFunction function) {
        this.message = msg;
        this.action = function;
    }

    Boolean performAction() {
        System.err.println("  Evaluate action " + message);
        return action.evaluate();
    }

    private ActionFunction action;
    private String message;
}
