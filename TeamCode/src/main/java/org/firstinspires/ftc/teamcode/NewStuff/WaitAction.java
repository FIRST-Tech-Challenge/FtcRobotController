package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction extends Action{

    double waitTimeSeconds;
    Action dependentAction;
    DoneStateAction doneStateAction = new DoneStateAction();
    boolean isDone = false;
    boolean timerStarted = false;
    ElapsedTime elapsedTime;

    public WaitAction(Action dependentAction, double targetWaitTimeSeconds) {
        this.dependentAction = dependentAction;
        this.waitTimeSeconds = targetWaitTimeSeconds;
    }

    public WaitAction(double targetWaitTimeSeconds) {
        this.dependentAction = doneStateAction;
        this.waitTimeSeconds = targetWaitTimeSeconds;
    }

    @Override
    boolean updateIsDone() {
        if (timerStarted) {
            if (elapsedTime.seconds() >= waitTimeSeconds) {
                isDone = true;
                return isDone;
            }
        }

        isDone = false;
        return isDone;
    }

    @Override
    void update() {
        if(!timerStarted) {
            elapsedTime = new ElapsedTime();
            timerStarted = true;
        }
    }

    boolean getIsDone() {
        return isDone;
    }

    void setDependentAction(Action newAction) {
        this.dependentAction = newAction;
    }

    Action getDependentAction() {
        return this.dependentAction;
    }

    void updateCheckDone() {
        if (isDone) { return; } //if i'm done never update
        if (!dependentAction.getIsDone()) { return; } //if dependent action is not done never update

        update();

        updateIsDone();
    }
}
