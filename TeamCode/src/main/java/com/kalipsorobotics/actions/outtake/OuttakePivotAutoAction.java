package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakePivotAutoAction extends Action {

    KServo kServo;

    public static enum Position {
        IN, SPECIMEN, BASKET;
    }


    Servo outtakePivotServo;
    MoveLSAction moveLSUp;
    WaitAction wait = new WaitAction(15);
    Position position;

    public OuttakePivotAutoAction(Outtake outtake, Position position) {
        this.outtakePivotServo = outtake.outtakePivotServo;
        this.dependentAction = new DoneStateAction();
        this.position = position;
        //TODO PUT DA ACTUAL STUFF IN
        kServo = new KServo(outtakePivotServo, 0, 0, 0, false);
    }

    @Override
    public boolean checkDoneCondition() {
        if (hasStarted) {
            wait.updateCheckDone();
            if(wait.getIsDone()) {
                Log.d("pivottimer", "done");
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
            if (position == Position.IN) {
                outtakePivotServo.setPosition(0.925);
            } else if (position == Position.SPECIMEN){
                outtakePivotServo.setPosition(0.0);
            } else { //position BASKET
                outtakePivotServo.setPosition(0.0);
            }

            hasStarted = true;
        }
    }
}
