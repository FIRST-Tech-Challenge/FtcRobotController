package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.math.CalculateTickPer;

public class CheckLSPastHeightMM extends Action {

    MoveLSAction moveLSAction;

    private final double checkingLimitMM;

    public CheckLSPastHeightMM(MoveLSAction moveLSAction, double checkingLimitMM) {
        this.moveLSAction = moveLSAction;
        this.checkingLimitMM = checkingLimitMM;
    }

    public boolean checkIfPastHeight() {
        Log.d("checkPastHeight",
                "numba  " + moveLSAction.linearSlide1.getCurrentPosition() + "  checking litmita  " + checkingLimitMM );
        return (Math.abs(CalculateTickPer.ticksToMmLS(moveLSAction.linearSlide1.getCurrentPosition())) > Math.abs(checkingLimitMM  - CalculateTickPer.ticksToMmLS(MoveLSAction.ERROR_TOLERANCE_TICKS) - 25));
    }

    @Override
    protected boolean checkDoneCondition() {
        if (isDone) {
            Log.d("checkPastHeight", "im in da hoooole");
            return isDone;
        }
        Log.d("checkPastHeight",
                "he put meh in da hole  ---> " + (moveLSAction.getHasStarted() && checkIfPastHeight()));
        setIsDone(moveLSAction.getHasStarted() && checkIfPastHeight());
        return isDone;
    }

    @Override
    protected void update() {

    }

}
