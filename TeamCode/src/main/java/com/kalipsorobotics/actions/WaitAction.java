package com.kalipsorobotics.actions;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction extends Action {

    double waitTimeMS;
    ElapsedTime elapsedTime;

    public WaitAction(double targetWaitTimeMS) {
        this.dependentActions.add(new DoneStateAction());
        this.waitTimeMS = targetWaitTimeMS;
    }

    @Override
    public boolean checkDoneCondition() {
        if (hasStarted) {
//            boolean done = elapsedTime.seconds() >= waitTimeSeconds;
            Log.d("waitaction", "elapsed time " + elapsedTime.seconds());
            return elapsedTime.milliseconds() >= waitTimeMS;
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
