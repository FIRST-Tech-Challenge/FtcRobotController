package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction extends Action {

    double waitTimeSeconds;
    ElapsedTime elapsedTime;

    public WaitAction(double targetWaitTimeSeconds) {
        this.dependentAction = new DoneStateAction();
        this.waitTimeSeconds = targetWaitTimeSeconds;
    }

    @Override
    public boolean checkDoneCondition() {
        if (hasStarted) {
            if (elapsedTime.seconds() >= waitTimeSeconds) {
                return true;
            }
        }

        return false;
    }

    @Override
    public void update() {
        if(!hasStarted) {
            elapsedTime = new ElapsedTime();
            hasStarted = true;
        }
    }
}
