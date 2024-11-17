package com.kalipsorobotics.actions;

import android.util.Log;

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
//            boolean done = elapsedTime.seconds() >= waitTimeSeconds;
            Log.d("waitaction", "elapsed time " + elapsedTime.seconds());
            if(elapsedTime.seconds() >= waitTimeSeconds) {
                return true;
            } else {
                return false;
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
