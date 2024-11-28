package com.kalipsorobotics.actions;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction extends Action {

    double waitTimeMS;
    ElapsedTime elapsedTime;

    public WaitAction(double targetWaitTimeMS) {
        this.dependentAction = new DoneStateAction();
        this.waitTimeMS = targetWaitTimeMS;
    }

    @Override
    public boolean checkDoneCondition() {
        if (hasStarted) {
//            boolean done = elapsedTime.seconds() >= waitTimeSeconds;
            Log.d("waitaction", "elapsed time " + elapsedTime.seconds());
            if(elapsedTime.milliseconds() >= waitTimeMS) {
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
