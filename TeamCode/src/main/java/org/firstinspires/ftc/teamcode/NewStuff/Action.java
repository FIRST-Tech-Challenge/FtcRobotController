package org.firstinspires.ftc.teamcode.NewStuff;

public abstract class Action {
    abstract boolean updateIsDone();
    abstract boolean getIsDone();
    abstract void update();
    abstract void setDependentAction(Action newAction);
    abstract Action getDependentAction();
}
