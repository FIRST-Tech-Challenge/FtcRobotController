package org.firstinspires.ftc.teamcode.code2023;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.DoneStateAction;
import org.firstinspires.ftc.teamcode.modules.Outtake2024;

public class PivotTrayAction extends Action {

    Servo trayPivot;
    Outtake2024 outtake;

    double targetPos;

    public static double TRAY_INTAKE_POS = 0.3;
    public static double TRAY_OUTTAKE_POS = 0;

    public PivotTrayAction(Action dependentAction, double targetPos, Outtake2024 outtake) {
        this.dependentAction = dependentAction;
        this.targetPos = targetPos;
        this.outtake = outtake;
    }

    public PivotTrayAction(double targetPos, Outtake2024 outtake) {
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
