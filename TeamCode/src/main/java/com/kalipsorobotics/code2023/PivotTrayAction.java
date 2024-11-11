package com.kalipsorobotics.code2023;

import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.modules.Outtake;

class PivotTrayAction extends Action {

    Servo trayPivot;
    Outtake outtake;

    double targetPos;

    public static double TRAY_INTAKE_POS = 0.3;
    public static double TRAY_OUTTAKE_POS = 0;

    public PivotTrayAction(Action dependentAction, double targetPos, Outtake outtake) {
        this.dependentAction = dependentAction;
        this.targetPos = targetPos;
        this.outtake = outtake;
    }

    public PivotTrayAction(double targetPos, Outtake outtake) {
        this.dependentAction = new DoneStateAction();
        this.targetPos = targetPos;
        this.outtake = outtake;
    }

    @Override
    public boolean checkDoneCondition() {
        if(trayPivot.getPosition() == targetPos) {
            outtake.getOpModeUtilities().getOpMode().sleep(100);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        trayPivot.setPosition(targetPos);
    }
}
