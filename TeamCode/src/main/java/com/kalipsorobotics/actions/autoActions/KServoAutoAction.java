package com.kalipsorobotics.actions.autoActions;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
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

        Log.d("servo_action", "execute or something  " + targetPos + "currentTime  " + kServo.getTime() +
                "port number  " + kServo.getPortNumber());
        kServo.setTargetPosition(targetPos);

//        if (Math.abs(kServo.getTargetPosition() - kServo.getCurrentPosition()) < 0.01) {
//            Log.d("servo_action", "done" + targetPos + "currentPos  " + kServo.getCurrentPosition() +
//                    "port number  " + kServo.getPortNumber());
//            isDone = true;
//        }

        isDone = kServo.isDone();
        if (isDone) {
            Log.d("servo_action", "done" + targetPos + "current time  " + kServo.getTime() +
                    "port number  " + kServo.getPortNumber());
        }

    }

    @Override
    public boolean checkDoneCondition() {
        return isDone;
    }



}
