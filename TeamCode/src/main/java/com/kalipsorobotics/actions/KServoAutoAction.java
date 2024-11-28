package com.kalipsorobotics.actions;

import com.kalipsorobotics.utilities.KServo;

public class KServoAutoAction extends Action {

    protected KServo kServo;
    protected double targetPos;

    public KServoAutoAction(KServo kServo, double targetPos) {
        this.kServo = kServo;
        this.targetPos = targetPos;
    }

    @Override
    public void update() {
        super.update();
        if (isDone) {
            return;
        }

        kServo.setTargetPosition(targetPos);

        if (Math.abs(kServo.getTargetPosition() - kServo.getCurrentPosition()) < 0.01) {
            isDone = true;
        }

    }

    @Override
    public boolean checkDoneCondition() {
        return isDone;
    }

}
