package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakePivotAutoAction extends Action {
    Servo outtakePivotServo;
    WaitAction wait = new WaitAction(0.15);

    public static enum Position {
        IN, SPECIMEN, BASKET;
    }

    Position position;

    public OuttakePivotAutoAction(Outtake outtake, Position position) {
        this.outtakePivotServo = outtake.outtakePivotServo;
        this.dependentAction = new DoneStateAction();
        this.position = position;
    }

    @Override
    public boolean checkDoneCondition() {
        if (hasStarted) {
            wait.updateCheckDone();
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        if (!hasStarted) {
            if (position == Position.IN) {
                outtakePivotServo.setPosition(0.256);
            } else if (position == Position.SPECIMEN){
                outtakePivotServo.setPosition(0.675);
            } else {
                outtakePivotServo.setPosition(0.85);
            }

            hasStarted = true;
        }
    }
}
