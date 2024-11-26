package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeClawAutoAction extends Action {
    Servo outtakeClawServo;
    WaitAction wait = new WaitAction(0.15);

    public static enum ClawPosition {
        OPEN, CLOSE;
    }

    ClawPosition position;

    public OuttakeClawAutoAction(Outtake outtake, ClawPosition position) {
        this.outtakeClawServo = outtake.outtakeClawServo;
        this.dependentAction = new DoneStateAction();
        this.position = position;
    }

    @Override
    public boolean checkDoneCondition() {
        if (hasStarted) {
            wait.updateCheckDone();
            if(wait.getIsDone()) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        if (!hasStarted) {
            if (position == ClawPosition.OPEN) {
                outtakeClawServo.setPosition(0.65);
            } else
                outtakeClawServo.setPosition(0.45);
            }

        hasStarted = true;

    }
}
