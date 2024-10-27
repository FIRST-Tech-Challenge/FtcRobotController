package org.firstinspires.ftc.teamcode.NewStuff.actions.code2023;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewStuff.actions.Action;
import org.firstinspires.ftc.teamcode.NewStuff.actions.DoneStateAction;

public class PivotTrayAction extends Action {

    Servo trayPivot;
    Outtake2023 outtake;

    double targetPos;

    public static double TRAY_INTAKE_POS = 0.3;
    public static double TRAY_OUTTAKE_POS = 0;

    public PivotTrayAction(Action dependentAction, double targetPos, Outtake2023 outtake) {
        this.dependentAction = dependentAction;
        this.targetPos = targetPos;
        this.outtake = outtake;
        this.trayPivot = outtake.tray;
    }

    public PivotTrayAction(double targetPos, Outtake2023 outtake) {
        this.dependentAction = new DoneStateAction();
        this.targetPos = targetPos;
        this.outtake = outtake;
        this.trayPivot = outtake.tray;
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
