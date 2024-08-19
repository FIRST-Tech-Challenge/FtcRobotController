package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction extends Action{

    double waitTimeSeconds;
    ElapsedTime elapsedTime;

    public WaitAction(Action dependentAction, double targetWaitTimeSeconds) {
        this.dependentAction = dependentAction;
        this.waitTimeSeconds = targetWaitTimeSeconds;
    }

    public WaitAction(double targetWaitTimeSeconds) {
        this.dependentAction = new DoneStateAction();
        this.waitTimeSeconds = targetWaitTimeSeconds;
    }

    @Override
    boolean checkDoneCondition() {
        if (hasStarted) {
            if (elapsedTime.seconds() >= waitTimeSeconds) {
                return true;
            }
        }

        return false;
    }

    @Override
    void update() {
        if(!hasStarted) {
            elapsedTime = new ElapsedTime();
            hasStarted = true;
        }
    }
}
